#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <condition_variable>
#include <exception>
#include <functional>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "raisim/World.hpp"

namespace {
class Workers {
 public:
  explicit Workers(int count) {
    try {
      for (int id = 0; id < count; ++id) {
        threads_.emplace_back([this, id] {
          size_t generation = 0;
          std::unique_lock<std::mutex> lock(mutex_);
          for (;;) {
            ready_.wait(lock, [&] { return stop_ || generation_ != generation; });
            if (stop_) return;
            generation = generation_;
            lock.unlock();
            std::exception_ptr error;
            try {
              task_(id);
            } catch (...) {
              error = std::current_exception();
            }
            lock.lock();
            if (error && !error_) error_ = error;
            if (--remaining_ == 0) finished_.notify_one();
          }
        });
      }
    } catch (...) {
      Stop();
      throw;
    }
  }
  ~Workers() { Stop(); }
  Workers(const Workers&) = delete;
  Workers& operator=(const Workers&) = delete;

  void Run(std::function<void(int)> task) {
    std::unique_lock<std::mutex> lock(mutex_);
    task_ = std::move(task);
    error_ = nullptr;
    remaining_ = threads_.size();
    ++generation_;
    ready_.notify_all();
    finished_.wait(lock, [&] { return remaining_ == 0; });
    if (error_) std::rethrow_exception(error_);
  }

 private:
  void Stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    ready_.notify_all();
    for (auto& thread : threads_) thread.join();
  }
  std::mutex mutex_;
  std::condition_variable ready_, finished_;
  std::vector<std::thread> threads_;
  std::function<void(int)> task_;
  std::exception_ptr error_;
  size_t generation_ = 0, remaining_ = 0;
  bool stop_ = false;
};

struct alignas(64) Statistics {
  uint64_t contacts = 0, bilateral = 0, resets = 0, lifted = 0;
  double mimic = 0., force_checksum = 0.;
};

constexpr int kCycle = 2150;
uint32_t Bits(uint32_t world, uint32_t block, uint32_t channel) {
  uint32_t x = UINT32_C(20261117) ^ (world * UINT32_C(0x9e3779b9)) ^
      (block * UINT32_C(0x85ebca6b)) ^ (channel * UINT32_C(0xc2b2ae35));
  x = (x ^ (x >> 16)) * UINT32_C(0x7feb352d);
  x = (x ^ (x >> 15)) * UINT32_C(0x846ca68b);
  return x ^ (x >> 16);
}
double Uniform(int w, int block, int channel) {
  return double(Bits(w, block, channel) & UINT32_C(0xffffff)) / 16777216.;
}
struct Scene {
  raisim::World world;
  raisim::ArticulatedSystem* robot;
  raisim::SingleBodyObject* object;
  Eigen::VectorXd initial, target, velocity, force;
  std::array<size_t, 7> qi{}, vi{};
  std::array<size_t, 5> followers{};
  int period, delay;
  double height;
  size_t left, right;
  explicit Scene(const std::filesystem::path& assets, int w) {
    world.setTimeStep(.001);
    world.setSleepingEnabled(false);
    world.setDefaultMaterial(.8, 0., .001);
    world.addGround();
    auto* table = world.addBox(.58, .74, .04, 1., "table");
    table->setBodyType(raisim::BodyType::STATIC);
    table->setPosition(.43, 0., .30);
    robot = world.addArticulatedSystem((assets / "robotiq.urdf").string());
    if (robot->getDOF() != 12) {
      throw std::runtime_error("Expected 12 robot DOFs");
    }
    initial = Eigen::VectorXd::Zero(12);
    velocity = force = initial;
    auto kp = initial, kd = initial;
    const std::array<double, 7> p{2000., 3000., 2500., 600., 600., 300., 10.};
    const std::array<double, 7> d{60., 80., 70., 20., 20., 12., .2};
    const std::array<double, 7> home{
        .38344342446, 2.53791743213, -2.94081775313, 0.,
        .80445012192, .38344342446, 0.};
    for (size_t j = 0; j < 7; ++j) {
      const auto name = j < 6 ? "kinova_joint_" + std::to_string(j + 1)
                              : "robotiq_85_left_knuckle_joint";
      qi[j] = robot->getJoint(name).getIdxInGeneralizedCoordinate();
      vi[j] = robot->getGeneralizedVelocityIndex(name);
      initial[qi[j]] = home[j];
      kp[vi[j]] = p[j];
      kd[vi[j]] = d[j];
    }
    const std::array<std::string, 5> names{
        "robotiq_85_right_knuckle_joint",
        "robotiq_85_left_inner_knuckle_joint",
        "robotiq_85_right_inner_knuckle_joint",
        "robotiq_85_left_finger_tip_joint",
        "robotiq_85_right_finger_tip_joint"};
    for (size_t j = 0; j < 5; ++j) {
      followers[j] = robot->getJoint(names[j]).getIdxInGeneralizedCoordinate();
    }
    left = robot->getBodyIdx("robotiq_85_left_finger_tip_link");
    right = robot->getBodyIdx("robotiq_85_right_finger_tip_link");
    for (auto& body : robot->getCollisionBodies()) {
      body.setMaterial(body.localIdx == left || body.localIdx == right
                           ? "pad" : "arm");
    }
    const size_t bodies = robot->getBodyNames().size();
    for (size_t a = 0; a < bodies; ++a) {
      for (size_t b = a + 1; b < bodies; ++b) {
        robot->ignoreCollisionBetween(a, b);
      }
    }
    robot->updateSelfCollisionCache(world);
    robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    robot->setPdGains(kp, kd);
    const int kind = Bits(w, 0, 20) % 3;
    const double x = .036 + .008 * Uniform(w, 0, 21);
    const double y = .036 + .008 * Uniform(w, 0, 22);
    const double z = .036 + .008 * Uniform(w, 0, 23);
    if (kind == 0) {
      object = world.addBox(x, y, z, .1, "object");
    } else if (kind == 1) {
      object = world.addSphere(x / 2., .1, "object");
    } else {
      object = world.addCylinder(x / 2., z, .1, "object");
    }
    height = .32 + (kind == 1 ? x : z) / 2.;
    world.setMaterialPairProp("pad", "object", 1., 0., .001);
    world.setMaterialPairProp("arm", "object", .5, 0., .001);
    world.setMaterialPairProp("table", "object", .8, 0., .001);
    period = kCycle + 150 + Bits(w, 0, 30) % 301;
    delay = Bits(w, 0, 31) % 101;
    Reset(w, 0);
  }
  void Reset(int w, int epoch) {
    robot->setState(initial, Eigen::VectorXd::Zero(12));
    object->setPosition(.42 + .004 * (Uniform(w, epoch, 40) - .5),
                        -.18 + .004 * (Uniform(w, epoch, 41) - .5), height);
    const double yaw = 6.283185307179586 * Uniform(w, epoch, 42);
    object->setOrientation(std::cos(yaw / 2.), 0., 0., std::sin(yaw / 2.));
    object->setVelocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    target = initial;
    velocity.setZero();
    force.setZero();
    robot->setPdTarget(target, velocity);
    robot->setGeneralizedForce(force);
    world.resetContactSolverWarmStart();
    world.setWorldTime(0.);
  }
};
}
int main(int argc, char** argv) {
  try {
    std::filesystem::path assets;
    int worlds = 1, steps = 30000, repeats = 3, requested_threads = 30;
    bool dump = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "--assets DIR --worlds N --steps N --repeats N "
                     "--threads N (default 30, capped by worlds)\n";
        return 0;
      }
      if (arg == "--dump-workload") {
        dump = true;
        continue;
      }
      if (++i == argc) throw std::invalid_argument("Missing value");
      if (arg == "--assets") {
        assets = argv[i];
      } else if (arg == "--worlds" || arg == "--steps" || arg == "--repeats" ||
                 arg == "--threads") {
        const std::string value = argv[i];
        if (value.empty() ||
            value.find_first_not_of("0123456789") != std::string::npos) {
          throw std::invalid_argument("Expected positive integer");
        }
        const int n = std::stoi(value);
        if (arg == "--worlds") {
          worlds = n;
        } else if (arg == "--steps") {
          steps = n;
        } else if (arg == "--repeats") {
          repeats = n;
        } else {
          requested_threads = n;
        }
      } else {
        throw std::invalid_argument("Unknown argument");
      }
    }
    if ((!dump && assets.empty()) || worlds < 1 || steps < 10 ||
        steps % 10 || repeats < 1 || requested_threads < 1) {
      throw std::invalid_argument("Invalid counts/assets");
    }
    if (dump) {
      double checksum = 0.;
      uint64_t resets = 0;
      std::cout << std::setprecision(17) << "WORKLOAD {\"objects\":[";
      for (int w = 0; w < worlds; ++w) {
        std::cout << (w ? "," : "") << "[" << Bits(w, 0, 20) % 3;
        for (int c = 21; c < 24; ++c) {
          std::cout << "," << .036 + .008 * Uniform(w, 0, c);
        }
        std::cout << "]";
        resets += (steps / 10 - 1) / (kCycle + 150 + Bits(w, 0, 30) % 301);
        for (int b = 0; b < steps / 10; ++b) {
          checksum += .25 * (2. * Uniform(w, b, 50) - 1.);
        }
      }
      std::cout << "],\"force_checksum\":" << checksum << ",\"resets\":" << resets << "}\n";
      return 0;
    }
    std::ifstream file(assets / "trajectory.txt");
    std::vector<std::array<double, 14>> trajectory(kCycle);
    for (auto& row : trajectory) {
      for (auto& x : row) {
        if (!(file >> x) || !std::isfinite(x)) {
          throw std::runtime_error("Invalid trajectory");
        }
      }
    }
    double extra;
    if (file >> extra) throw std::runtime_error("Unexpected trajectory rows");
    if (const char* key = std::getenv("RAISIM_ACTIVATION_KEY")) {
      raisim::World::setActivationKey(key);
    }
    std::vector<std::unique_ptr<Scene>> scenes;
    for (int w = 0; w < worlds; ++w) {
      scenes.push_back(std::make_unique<Scene>(assets, w));
    }
    const int threads = std::min(worlds, requested_threads);
    std::vector<Statistics> statistics(worlds);
    Workers workers(threads);
    auto run = [&](int n, bool diagnostic) {
      workers.Run([&](int worker) {
        const int first = static_cast<int>(int64_t(worlds) * worker / threads);
        const int last = static_cast<int>(int64_t(worlds) * (worker + 1) / threads);
        for (int block = 0; block < n / 10; ++block) {
          for (int w = first; w < last; ++w) {
            auto& stats = statistics[w];
            auto& scene = *scenes[w];
            const int epoch = block / scene.period;
            const int age = block % scene.period;
            if (age == 0) {
              scene.Reset(w, epoch);
              if (diagnostic && block) ++stats.resets;
            }
            const int phase = std::clamp(age - scene.delay, 0, kCycle - 1);
            for (size_t j = 0; j < 7; ++j) {
              scene.target[scene.qi[j]] = trajectory[phase][j];
              scene.velocity[scene.vi[j]] = trajectory[phase][j + 7];
            }
            scene.force[scene.vi[6]] = .25 * (2. * Uniform(w, block, 50) - 1.);
            scene.robot->setPdTarget(scene.target, scene.velocity);
            scene.robot->setGeneralizedForce(scene.force);
            if (diagnostic) stats.force_checksum += scene.force[scene.vi[6]];
            for (int k = 0; k < 10; ++k) {
              scene.world.integrate();
              if (!diagnostic) continue;
              const auto& q = scene.robot->getGeneralizedCoordinate();
              if (!q.e().allFinite() ||
                  !scene.robot->getGeneralizedVelocity().e().allFinite() ||
                  !scene.object->getPosition().allFinite() ||
                  !scene.object->getLinearVelocity().allFinite() ||
                  !scene.object->getAngularVelocity().allFinite()) {
                throw std::runtime_error("Nonfinite trajectory");
              }
              const std::array<double, 5> signs{-1., 1., -1., -1., 1.};
              for (size_t j = 0; j < 5; ++j) {
                stats.mimic = std::max(stats.mimic, std::abs(q[scene.followers[j]] -
                                               signs[j] * q[scene.qi[6]]));
              }
              bool left = false, right = false;
              for (const auto& contact : scene.robot->getContacts()) {
                if (contact.skip() || contact.getPairObjectIndex() !=
                                          scene.object->getIndexInWorld()) {
                  continue;
                }
                ++stats.contacts;
                left |= contact.getlocalBodyIndex() == scene.left;
                right |= contact.getlocalBodyIndex() == scene.right;
              }
              stats.bilateral += left && right;
              stats.lifted += scene.object->getPosition()[2] > scene.height + .10;
            }
          }
        }
      });
    };
    run(100, false);
    std::vector<double> samples;
    for (int r = 0; r < repeats; ++r) {
      const auto start = std::chrono::steady_clock::now();
      run(steps, false);
      samples.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    }
    run(steps, true);
    uint64_t contacts = 0, bilateral = 0, resets = 0, lifted = 0;
    double mimic = 0., forceChecksum = 0.;
    for (const auto& stats : statistics) {
      contacts += stats.contacts;
      bilateral += stats.bilateral;
      resets += stats.resets;
      lifted += stats.lifted;
      mimic = std::max(mimic, stats.mimic);
      forceChecksum += stats.force_checksum;
    }
    auto sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    const double median = .5 * (sorted[(sorted.size() - 1) / 2] + sorted[sorted.size() / 2]);
    const bool valid = mimic < .01 && (steps < 21500 || (contacts > 0 && bilateral > 0 && lifted > 0));
    std::cout << std::setprecision(17)
              << "RESULT {\"backend\":\"raisim\",\"model\":\"robotiq\",\"worlds\":"
              << worlds << ",\"threads\":" << threads
              << ",\"requested_threads\":" << requested_threads
              << ",\"solver_substeps\":1"
              << ",\"integration_dt\":0.001,\"steps\":" << steps
              << ",\"aggregate_fps\":" << double(worlds) * steps / median
              << ",\"finite\":true,\"quality_pass\":" << (valid ? "true" : "false")
              << ",\"max_mimic_error_rad\":" << mimic
              << ",\"object_contact_samples\":" << contacts
              << ",\"bilateral_grasp_samples\":" << bilateral
              << ",\"lifted_samples\":" << lifted << ",\"resets\":" << resets
              << ",\"force_checksum\":" << forceChecksum << ",\"seconds\":[";
    for (size_t i = 0; i < samples.size(); ++i) {
      std::cout << (i ? "," : "") << samples[i];
    }
    std::cout << "]}\n";
    return valid ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
