#include <exact-real/element.hpp>
#include <exact-real/module.hpp>
#include <exact-real/rational_field.hpp>
#include <exact-real/real_number.hpp>

#include <iostream>
#include <vector>

int main() {
  const auto module = exactreal::Module<exactreal::RationalField>::make(
      {exactreal::RealNumber::rational(1)});
  const exactreal::Element<exactreal::RationalField> malformed(module, {});
  std::cout << "module_rank=" << malformed.module()->rank()
            << " coefficient_count=" << malformed.coefficients().size()
            << " nonzero=" << static_cast<bool>(malformed) << '\n';
}
