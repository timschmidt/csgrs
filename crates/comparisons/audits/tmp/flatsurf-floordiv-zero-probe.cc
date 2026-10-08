#include <exact-real/element.hpp>
#include <exact-real/module.hpp>
#include <exact-real/rational_field.hpp>
#include <exact-real/real_number.hpp>

int main() {
  const auto module = exactreal::Module<exactreal::RationalField>::make(
      {exactreal::RealNumber::rational(1)});
  const auto one = module->gen(0);
  const auto zero = module->zero();
  return one.floordiv(zero).get_si();
}
