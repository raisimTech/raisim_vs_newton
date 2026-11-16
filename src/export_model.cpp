#include <raisim/World.hpp>
#include <Eigen/Geometry>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::invalid_argument("Usage: export_model model.urdf link_names.txt");
    if (const char* key = std::getenv("RAISIM_ACTIVATION_KEY")) raisim::World::setActivationKey(key);
    raisim::World world;
    auto* robot = world.addArticulatedSystem(argv[1]);
    std::ifstream names(argv[2]);
    if (!names) throw std::runtime_error("Cannot open link name list");
    std::cout << "MODEL_KINEMATICS {" << std::setprecision(17);
    std::string name;
    bool first = true;
    while (names >> name) {
      auto frame = robot->getFrameIdxByLinkName(name);
      if (frame == size_t(-1) && name == robot->getBodyNames().front())
        frame = robot->getFrameIdxByName("ROOT");
      if (frame == size_t(-1)) throw std::runtime_error("Public SDK has no frame for link " + name);
      raisim::Vec<3> position;
      raisim::Mat<3, 3> orientation;
      robot->getFramePosition(frame, position);
      robot->getFrameOrientation(frame, orientation);
      Eigen::Quaterniond q(orientation.e());
      if (!first) std::cout << ',';
      first = false;
      std::cout << '"' << name << "\":[" << position[0] << ',' << position[1] << ',' << position[2]
                << ',' << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ']';
    }
    std::cout << "}\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
