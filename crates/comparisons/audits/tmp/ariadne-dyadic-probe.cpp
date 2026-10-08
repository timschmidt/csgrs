#include <iostream>
#include <utility>

#include "numeric/dyadic.hpp"
#include "numeric/rational.hpp"

using namespace Ariadne;

int main(int argc, char**) {
    if (argc > 1) {
        ScientificWriter writer;
        std::cout << writer(Dyadic(0)) << '\n';
        return 0;
    }

    mpf_t raw;
    mpf_init_set_ui(raw, 7u);
    Dyadic from_raw(raw);
    mpf_clear(raw);
    std::cout << "raw_7=" << from_raw.get_d() << '\n';

    Integer exact = pow(Integer(2), 70000u) + 1;
    Dyadic truncated(exact);
    std::cout << "70001_bit_integer_preserved="
              << (Rational(truncated) == Rational(exact)) << '\n';

    std::cout << "neg([1,3])=" << neg(DyadicBounds(1, 3)) << '\n';

    Integer wide = pow(Integer(2), 1000u) - 1;
    Dyadic source(wide);
    Dyadic moved(std::move(source));
    Dyadic replacement(wide);
    source = replacement;
    std::cout << "moved_from_precision=" << mpf_get_prec(source._mpf) << '\n';
    std::cout << "moved_from_reassignment_preserved="
              << (Rational(source) == Rational(wide)) << '\n';
}
