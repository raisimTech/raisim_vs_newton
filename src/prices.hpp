#pragma once
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace benchmark {
inline double parsePrice(const std::string& text) {
  size_t end = 0;
  const double price = std::stod(text, &end);
  if (end != text.size() || !std::isfinite(price) || price <= 0.)
    throw std::invalid_argument("price must be a positive finite USD amount");
  return price;
}
inline double promptPrice(const std::string& component) {
  while (true) {
    std::cout << "How much did your " << component << " cost (USD)? " << std::flush;
    std::string text;
    if (!std::getline(std::cin, text))
      throw std::runtime_error("No price input; supply --cpu-price and --gpu-price for unattended runs");
    try { return parsePrice(text); }
    catch (const std::exception&) { std::cout << "Enter a positive finite number, such as 339.99.\n"; }
  }
}
}
