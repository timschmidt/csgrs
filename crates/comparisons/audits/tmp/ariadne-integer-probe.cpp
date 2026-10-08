#include <cstdint>
#include <iostream>
#include <limits>

#include "numeric/integer.hpp"
#include "utility/string.hpp"

using namespace Ariadne;

int main() {
    auto show = [](char const* name, Integer const& z) {
        std::cout << name << '=' << z << '\n';
    };

    show("Int32(-3)", Integer(Int32(-3)));
    show("int64_min", Integer(std::numeric_limits<std::int64_t>::min()));
    show("uint64_max", Integer(std::numeric_limits<std::uint64_t>::max()));
    std::cout << "log2floor(0)=" << log2floor(Natural(0u)) << '\n';
    show("invalid_string", Integer(String("not-an-integer")));
    for (auto [a,b] : {std::pair{-7,3}, std::pair{7,-3}, std::pair{-7,-3}}) {
        Integer za(a), zb(b);
        std::cout << "quot/rem " << a << '/' << b << ": q=" << quot(za,zb)
                  << " r=" << rem(za,zb)
                  << " reconstruct=" << quot(za,zb)*zb+rem(za,zb) << '\n';
    }
}
