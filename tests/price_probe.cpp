#include "prices.hpp"
#include <exception>
#include <iomanip>
#include <iostream>
int main() {
  try {
    const auto cpu = benchmark::promptPrice("CPU");
    const auto gpu = benchmark::promptPrice("GPU");
    std::cout << std::setprecision(17) << "PRICES " << cpu << ' ' << gpu << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
