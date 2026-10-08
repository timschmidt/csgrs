#include <iostream>
#include <sstream>
#include <string>

#include "numeric/dyadic.hpp"
#include "numeric/floatdp.hpp"
#include "numeric/floatmp.hpp"
#include "numeric/integer.hpp"
#include "numeric/rational.hpp"

using namespace Ariadne;

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "mp";
    if (mode == "dp-huge") {
        Integer z = pow(Integer(2), 2000u);
        FloatDP f(Dyadic(z), FloatDP::ROUND_DOWNWARD, dp);
        std::cout << f << '\n';
        return 0;
    }

    Integer z = pow(Integer(2), 70000u) + 1;
    FloatMP exact(z, FloatMP::ROUND_TO_NEAREST, mp(70002));
    Rational converted = static_cast<Rational>(exact);
    std::cout << std::boolalpha
              << "mp_to_rational_preserved=" << (converted == Rational(z)) << '\n';

    FloatMP nan = FloatMP::nan(mp(53));
    FloatMP zero(0, mp(53));
    std::cout << "nan_eq_zero=" << (nan == zero)
              << " cmp_nan_zero=" << static_cast<int>(cmp(nan, zero)) << '\n';

    FloatMP large(TwoExp(10000), mp(100));
    FloatMP small(TwoExp(-10000), mp(100));
    String large_literal = large.literal();
    String small_literal = small.literal();
    std::cout << "large_literal_length=" << large_literal.size()
              << " large_suffix=" << large_literal.substr(large_literal.size() - 8) << '\n'
              << "small_literal_length=" << small_literal.size()
              << " small_suffix=" << small_literal.substr(small_literal.size() - 8) << '\n';

    FloatDP zero_dp(cast_exact(0.0), dp);
    FloatDP next_zero = next(up, zero_dp);
    std::cout << "next_zero=" << std::hexfloat << next_zero.get_d()
              << " true_next=" << std::nextafter(0.0, 1.0) << '\n';
}
