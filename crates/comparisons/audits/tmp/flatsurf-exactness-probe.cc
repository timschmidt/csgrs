#include <exact-real/arb.hpp>
#include <exact-real/element.hpp>
#include <exact-real/integer_ring.hpp>
#include <exact-real/module.hpp>
#include <exact-real/rational_field.hpp>
#include <exact-real/real_number.hpp>

#include <gmpxx.h>

#include <iostream>
#include <optional>
#include <stdexcept>

namespace {

void print_optional(const char* name, const std::optional<bool>& value) {
  std::cout << name << '=';
  if (!value) {
    std::cout << "unknown\n";
  } else {
    std::cout << (*value ? "true" : "false") << '\n';
  }
}

}  // namespace

int main() {
  using exactreal::Arb;
  using exactreal::Element;
  using exactreal::IntegerRing;
  using exactreal::Module;
  using exactreal::RationalField;
  using exactreal::RealNumber;

  const mpq_class half(1, 2);
  const Arb half_ball(half, 64);
  print_optional("half_eq_half", half_ball == half);
  print_optional("half_ne_half", half_ball != half);

  const Arb one(1);
  const mpq_class two(2);
  print_optional("one_eq_two", one == two);
  print_optional("one_ne_two", one != two);

  std::cout << "floor_neg_half=" << RationalField::floor(mpq_class(-1, 2))
            << " expected=-1\n";

  const auto source = Module<RationalField>::make({RealNumber::rational(1)});
  const auto empty = Module<RationalField>::make({});
  auto promoted = source->gen(0);
  std::cout << "promote_before_nonzero=" << static_cast<bool>(promoted) << '\n';
  promoted.promote(empty);
  std::cout << "promote_after_nonzero=" << static_cast<bool>(promoted)
            << " expected=1\n";

  const auto negative_unit =
      Module<IntegerRing>::make({RealNumber::rational(mpq_class(-1))});
  try {
    const auto unit = negative_unit->one();
    std::cout << "negative_unit_one=" << unit << " expected=1\n";
  } catch (const std::exception& error) {
    std::cout << "negative_unit_one=threw:" << error.what()
              << " expected=1\n";
  }
}
