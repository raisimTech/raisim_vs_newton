#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif
#include <iomanip>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "raisim/World.hpp"
#include "prices.hpp"
#include <filesystem>

namespace {
class Gate {
 public:
  explicit Gate(size_t count) : count_(count), remaining_(count) {}
  void wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    const size_t phase = phase_;
    if (--remaining_ == 0) {
      remaining_ = count_;
      ++phase_;
      condition_.notify_all();
    } else condition_.wait(lock, [&] { return phase != phase_; });
  }
 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  size_t count_, remaining_, phase_ = 0;
};

struct PointPair {
  size_t parent, child;
  raisim::Vec<3> p, c;
};
struct Anchor {
  std::string parent, child;
  raisim::Vec<3> initial;
};
struct Scene {
  std::unique_ptr<raisim::World> world;
  raisim::ArticulatedSystem* robot;
  Eigen::VectorXd q, v, targetQ, targetV;
  std::vector<size_t> actionQ, actionV;
  std::vector<double> nominal;
  std::vector<PointPair> points;
  uint64_t contactTotal = 0;
  double maxGap = 0.;
  bool finite = true;
};
}

uint32_t randomBits(uint32_t world, uint32_t block, uint32_t joint) {
  uint32_t x = UINT32_C(20261115) ^ (world * UINT32_C(0x9e3779b9)) ^
               (block * UINT32_C(0x85ebca6b)) ^ (joint * UINT32_C(0xc2b2ae35));
  x ^= x >> 16; x *= UINT32_C(0x7feb352d);
  x ^= x >> 15; x *= UINT32_C(0x846ca68b);
  return x ^ (x >> 16);
}
float actionNoise(uint32_t world, uint32_t block, uint32_t joint) {
  return float(randomBits(world, block, joint) & UINT32_C(0x00ffffff)) * 0x1p-23f - 1.f;
}

int main(int argc, char** argv) {
  try {
    std::string model = "anymal";
    int count = 1, threads = 1, steps = 5000, repeats = 3;
    float actionScale = 1.f;
    double cpuPrice = 0., gpuPrice = 0.;
    bool pin = false, dump = false;
    std::filesystem::path assets = std::filesystem::current_path() / "build" / "assets";
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "raisim_benchmark --model anymal|strandbeest --worlds N --threads N --steps N --repeats N\n"
                     "  --assets DIR --cpu-price USD --gpu-price USD --pin-threads --action-scale 0..1\n"
                     "Prices are prompted when omitted. Set RAISIM_ACTIVATION_KEY to your license path.\n";
        return 0;
      }
      if (arg == "--pin-threads") { pin = true; continue; }
      if (arg == "--dump-actions") { dump = true; continue; }
      if (++i >= argc) throw std::invalid_argument("missing argument for " + arg);
      const std::string value = argv[i];
      if (arg == "--model") model = value;
      else if (arg == "--worlds") count = std::stoi(value);
      else if (arg == "--threads") threads = std::stoi(value);
      else if (arg == "--steps") steps = std::stoi(value);
      else if (arg == "--repeats") repeats = std::stoi(value);
      else if (arg == "--assets") assets = value;
      else if (arg == "--cpu-price") cpuPrice = benchmark::parsePrice(value);
      else if (arg == "--gpu-price") gpuPrice = benchmark::parsePrice(value);
      else if (arg == "--action-scale") actionScale = std::stof(value);
      else throw std::invalid_argument("unknown option " + arg);
    }
    if ((model != "anymal" && model != "strandbeest") || count < 1 || threads < 1 ||
        threads > count || steps < 10 || steps % 10 || repeats < 1 ||
        !std::isfinite(actionScale) || actionScale < 0.f || actionScale > 1.f)
      throw std::invalid_argument("invalid model/counts; steps must be divisible by ten");
    if (dump) {
      const std::vector<float> nominal = model == "anymal" ?
          std::vector<float>{.03f,.4f,-.8f,-.03f,.4f,-.8f,.03f,-.4f,.8f,-.03f,-.4f,.8f} :
          std::vector<float>{float(2. * std::acos(-1.))};
      std::cout << std::setprecision(17) << "ACTIONS [";
      for (int w = 0; w < count; ++w) for (size_t j = 0; j < nominal.size(); ++j) {
        const float noise = actionScale * actionNoise(w, steps / 10 - 1, j);
        const float target = model == "anymal" ? nominal[j] + .15f * noise : nominal[j] * (1.f + .25f * noise);
        std::cout << ((w || j) ? "," : "") << target;
      }
      std::cout << "]\n";
      return 0;
    }
    if (cpuPrice == 0.) cpuPrice = benchmark::promptPrice("CPU");
    if (gpuPrice == 0.) gpuPrice = benchmark::promptPrice("GPU");
    if (const char* key = std::getenv("RAISIM_ACTIVATION_KEY")) raisim::World::setActivationKey(key);
    std::vector<Scene> scenes;
    std::vector<Anchor> anchors;
    if (model == "strandbeest") {
      std::ifstream f(assets / "strandbeest-anchors.txt");
      size_t n = 0; f >> n;
      if (n != 84) throw std::runtime_error("invalid anchor file");
      anchors.resize(n);
      for (auto& a : anchors) f >> a.parent >> a.child >> a.initial[0] >> a.initial[1] >> a.initial[2];
      if (!f) throw std::runtime_error("invalid anchor entries");
    }

    for (int i = 0; i < count; ++i) {
      Scene scene;
      scene.world = std::make_unique<raisim::World>();
      scene.world->setSleepingEnabled(false);
      scene.world->setTimeStep(model == "anymal" ? 0.002 : 0.001);
      scene.world->setDefaultMaterial(0.8, 0., 0.01, 0.8, 1e-3, 0., 0.);
      scene.world->addGround();
      scene.robot = model == "anymal" ?
          scene.world->addArticulatedSystem((assets / "anymal.urdf").string()) :
          scene.world->addUsdArticulatedSystem((assets / "strandbeest.usda").string());
      auto& robot = *scene.robot;
      auto q = robot.getGeneralizedCoordinate().e().eval();
      auto kp = Eigen::VectorXd::Zero(robot.getDOF()).eval();
      auto kd = kp;
      auto targetVelocity = kp;
      if (model == "anymal") {
        q.setZero();
        q[2] = 0.54;
        q[3] = 1.;
        const std::vector<std::pair<std::string, double>> pose = {
            {"LF_HAA", .03}, {"LF_HFE", .4}, {"LF_KFE", -.8},
            {"RF_HAA", -.03}, {"RF_HFE", .4}, {"RF_KFE", -.8},
            {"LH_HAA", .03}, {"LH_HFE", -.4}, {"LH_KFE", .8},
            {"RH_HAA", -.03}, {"RH_HFE", -.4}, {"RH_KFE", .8}};
        for (const auto& joint : pose) {
          const size_t d = robot.getGeneralizedVelocityIndex(joint.first);
          q[robot.getJoint(joint.first).getIdxInGeneralizedCoordinate()] = joint.second;
          kp[d] = 200.;
          kd[d] = 10.;
          scene.actionQ.push_back(robot.getJoint(joint.first).getIdxInGeneralizedCoordinate());
          scene.actionV.push_back(d);
          scene.nominal.push_back(joint.second);
        }
      } else {
        const size_t d = robot.getGeneralizedVelocityIndex("joint_crossbar_crank");
        kd[d] = 200.;
        targetVelocity[d] = 2. * std::acos(-1.);
        scene.actionV.push_back(d); scene.nominal.push_back(2. * std::acos(-1.));
      }
      robot.setState(q, Eigen::VectorXd::Zero(robot.getDOF()));
      robot.setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
      robot.setPdGains(kp, kd);
      robot.setPdTarget(q, targetVelocity);
      scene.targetQ = q; scene.targetV = targetVelocity;
      const size_t bodies = robot.getBodyNames().size();
      for (size_t a = 0; a < bodies; ++a)
        for (size_t b = a + 1; b < bodies; ++b) robot.ignoreCollisionBetween(a, b);
      robot.updateSelfCollisionCache(*scene.world);
      for (const auto& a : anchors) {
        PointPair point;
        point.parent = robot.getFrameIdxByLinkName(a.parent);
        point.child = robot.getFrameIdxByLinkName(a.child);
        raisim::Vec<3> p, c; raisim::Mat<3, 3> rp, rc;
        robot.getFramePosition(point.parent, p); robot.getFramePosition(point.child, c);
        robot.getFrameOrientation(point.parent, rp); robot.getFrameOrientation(point.child, rc);
        point.p.e() = rp.e().transpose() * (a.initial.e() - p.e());
        point.c.e() = rc.e().transpose() * (a.initial.e() - c.e());
        scene.points.push_back(point);
      }
      scene.q = q;
      scene.v = robot.getGeneralizedVelocity().e();
      scenes.push_back(std::move(scene));
    }
    if (model == "anymal") {
      for (int i = 0; i < 1000; ++i) scenes.front().world->integrate();
      const auto settledQ = scenes.front().robot->getGeneralizedCoordinate().e().eval();
      const auto settledV = scenes.front().robot->getGeneralizedVelocity().e().eval();
      for (auto& scene : scenes) {
        scene.robot->setState(settledQ, settledV);
        scene.world->resetContactSolverWarmStart();
        scene.world->setWorldTime(0.);
      }
    }
    std::vector<int> pinCpus;
#if defined(__linux__)
    if (pin) {
      cpu_set_t available;
      if (sched_getaffinity(0, sizeof(available), &available) != 0)
        throw std::runtime_error("Cannot read CPU affinity mask");
      for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu)
        if (CPU_ISSET(cpu, &available)) pinCpus.push_back(cpu);
      if (size_t(threads) > pinCpus.size())
        throw std::invalid_argument("Requested more pinned workers than available CPUs");
    }
#else
    if (pin) throw std::invalid_argument("CPU pinning requires Linux");
#endif
    Gate gate(size_t(threads + 1));
    Gate policyGate{size_t(threads)};
    std::vector<std::thread> workers;
    for (int id = 0; id < threads; ++id) {
      workers.emplace_back([&, id] {
        const int first = count * id / threads, last = count * (id + 1) / threads;
        auto run = [&](int n, bool actions, bool diagnostic) {
          for (int step = 0; step < n;) {
            const int blockSteps = std::min(10, n - step);
            for (int i = first; i < last; ++i) {
              auto& scene = scenes[i];
              if (actions) {
                for (size_t j = 0; j < scene.nominal.size(); ++j) {
                  const float noise = actionScale * actionNoise(uint32_t(i), uint32_t(step / 10), uint32_t(j));
                  const float target = model == "anymal" ?
                      float(scene.nominal[j]) + 0.15f * noise :
                      float(scene.nominal[j]) * (1.f + 0.25f * noise);
                  if (model == "anymal") scene.targetQ[scene.actionQ[j]] = double(target);
                  else scene.targetV[scene.actionV[j]] = double(target);
                }
                scene.robot->setPdTarget(scene.targetQ, scene.targetV);
              }
              for (int k = 0; k < blockSteps; ++k) {
                scene.world->integrate();
                if (diagnostic) {
                  for (const auto& contact : scene.robot->getContacts())
                    if (contact.getPairObjectIndex() == 0) ++scene.contactTotal;
                }
              }
              if (diagnostic) {
                scene.finite = scene.finite && scene.robot->getGeneralizedCoordinate().e().allFinite() &&
                               scene.robot->getGeneralizedVelocity().e().allFinite();
                for (const auto& point : scene.points) {
                  raisim::Vec<3> p, c; raisim::Mat<3, 3> rp, rc;
                  scene.robot->getFramePosition(point.parent, p); scene.robot->getFramePosition(point.child, c);
                  scene.robot->getFrameOrientation(point.parent, rp); scene.robot->getFrameOrientation(point.child, rc);
                  scene.maxGap = std::max(scene.maxGap, (p.e() + rp.e() * point.p.e() - c.e() - rc.e() * point.c.e()).norm());
                }
              }
            }
            if (actions && threads > 1) policyGate.wait();
            step += blockSteps;
          }
        };
        run(100, false, false);
        if (model == "anymal") for (int i = first; i < last; ++i) {
          scenes[i].q = scenes[i].robot->getGeneralizedCoordinate().e();
          scenes[i].v = scenes[i].robot->getGeneralizedVelocity().e();
        }
        gate.wait();
        for (int repeat = 0; repeat <= repeats; ++repeat) {
          for (int i = first; i < last; ++i) {
            scenes[i].robot->setState(scenes[i].q, scenes[i].v);
            scenes[i].world->resetContactSolverWarmStart();
            scenes[i].world->setWorldTime(0.);
          }
          gate.wait();
          gate.wait();
          run(steps, true, repeat == repeats);
          gate.wait();
        }
      });
      if (pin) {
#if defined(__linux__)
        if (id >= CPU_SETSIZE) std::terminate();
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(pinCpus[id], &mask);
        if (pthread_setaffinity_np(workers.back().native_handle(), sizeof(mask), &mask) != 0)
          std::terminate();
#else
        std::cerr << "thread affinity is only available on Linux" << std::endl;
        std::terminate();
#endif
      }
    }
    gate.wait();
    std::vector<double> samples;
    for (int repeat = 0; repeat <= repeats; ++repeat) {
      gate.wait();
      const auto begin = std::chrono::steady_clock::now();
      gate.wait();
      gate.wait();
      if (repeat < repeats) samples.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count());
    }
    for (auto& worker : workers) worker.join();
    uint64_t stateHash = UINT64_C(14695981039346656037);
    size_t contacts = 0;
    uint64_t trajectoryContacts = 0; double maxGap = 0.; bool finite = true;
    double height = 0., checksum = 0.;
    for (const auto& scene : scenes) {
      trajectoryContacts += scene.contactTotal; maxGap = std::max(maxGap, scene.maxGap); finite = finite && scene.finite;
      if (!scene.robot->getGeneralizedCoordinate().e().allFinite() ||
          !scene.robot->getGeneralizedVelocity().e().allFinite()) throw std::runtime_error("nonfinite state");

      for (const auto& contact : scene.robot->getContacts())
        if (contact.getPairObjectIndex() == 0) ++contacts;
      height += scene.robot->getGeneralizedCoordinate()[2];
      checksum += scene.robot->getGeneralizedCoordinate().e().sum();
      for (const auto* state : {&scene.robot->getGeneralizedCoordinate(), &scene.robot->getGeneralizedVelocity()}) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(state->data());
        for (size_t i = 0; i < state->size() * sizeof(double); ++i)
          stateHash = (stateHash ^ bytes[i]) * UINT64_C(1099511628211);
      }
    }
    auto ordered = samples;
    std::sort(ordered.begin(), ordered.end());
    const size_t middle = ordered.size() / 2;
    const double median = ordered.size() % 2 ? ordered[middle] :
        0.5 * (ordered[middle - 1] + ordered[middle]);
    double targetChecksum = 0.;
    for (size_t i = 0; i < scenes.size(); ++i) {
      Eigen::VectorXd p(scenes[i].robot->getGeneralizedCoordinateDim()), v(scenes[i].robot->getDOF()); scenes[i].robot->getPdTarget(p, v);
      for (size_t j = 0; j < scenes[i].nominal.size(); ++j)
        targetChecksum += model == "anymal" ? p[scenes[i].actionQ[j]] : v[scenes[i].actionV[j]];
    }
    std::cout << "ACTION_CHECKSUM " << std::setprecision(17) << targetChecksum << '\n';
    std::cout << std::setprecision(15) << "RESULT {\"model\":\"" << model << "\",\"worlds\":" << count
        << ",\"threads\":" << threads
        << ",\"steps\":" << steps
        << ",\"cpu_price_usd\":" << cpuPrice << ",\"gpu_price_usd\":" << gpuPrice
        << ",\"fps_per_usd\":" << count * double(steps) / median / cpuPrice
        << ",\"aggregate_fps\":" << count * double(steps) / median << ",\"contacts\":" << contacts
        << ",\"mean_contacts_per_world\":" << double(trajectoryContacts) / count / steps
        << ",\"max_loop_gap_m\":" << maxGap << ",\"finite\":" << (finite ? "true" : "false")
        << ",\"quality_pass\":" << (finite && maxGap < .001 ? "true" : "false")
        << ",\"action_seed\":20261115,\"action_decimation\":10,\"last_action_checksum\":" << targetChecksum
        << ",\"state_hash\":" << stateHash << ",\"mean_height\":" << height / count << ",\"checksum\":" << checksum << ",\"seconds\":[";
    for (size_t i = 0; i < samples.size(); ++i) std::cout << (i ? "," : "") << samples[i];
    std::cout << "]}" << std::endl;
    std::cout << "RaiSim: " << count * double(steps) / median << " FPS, "
              << count * double(steps) / median / cpuPrice << " FPS/USD\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
