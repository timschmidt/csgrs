#include "config.hpp"

#include "algebra/vector.hpp"
#include "numeric/numeric.hpp"
#include "symbolic/constant.hpp"
#include "symbolic/expression.hpp"
#include "symbolic/templates.hpp"
#include "symbolic/valuation.hpp"
#include "symbolic/variable.hpp"

#include <array>
#include <iostream>
#include <string>

using namespace Ariadne;

namespace {

struct SumThree {
    int operator()(int a, int b, int c) const { return a + b + c; }
};

struct Increment {
    int operator()(int value) const { return value + 1; }
};

Sequence<int> make_dangling_iteration() {
    Symbolic<Iterate, Increment> iteration{Iterate(), Increment()};
    int start = 10;
    return iteration(start);
}

void overwrite_stack() {
    volatile unsigned char bytes[16384]{};
    for (std::size_t i = 0; i != 16384; ++i) {
        bytes[i] = static_cast<unsigned char>(i);
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) { return 64; }
    std::string const which(argv[1]);

    RealVariable x("x"), y("y");

    if (which == "opposite") {
        std::cout
            << "lt-swapped=" << opposite(x < y, y < x) << " expected=0\n"
            << "le-swapped=" << opposite(x <= y, y <= x) << " expected=0\n"
            << "ge-le-same=" << opposite(x >= y, x <= y) << " expected=0\n"
            << "lt-ge-complement=" << opposite(x < y, x >= y) << " expected=1\n";
        return 0;
    }

    if (which == "simplify-div-zero") {
        RealExpression original = x / x;
        RealExpression reduced = simplify(original);
        Map<Identifier, Real> at_zero;
        at_zero[x.name()] = Real(0);
        Real original_value = evaluate(original, at_zero);
        Real reduced_value = evaluate(reduced, at_zero);
        std::cout << "original-expression=" << original
                  << " reduced-expression=" << reduced << '\n';
        std::cout << "original-at-zero=" << original_value.compute_get(Effort(8), dp)
                  << " reduced-at-zero=" << reduced_value.compute_get(Effort(8), dp)
                  << '\n';
        return 0;
    }

    if (which == "simplify-log-exp-domain") {
        RealExpression original = exp(log(x));
        RealExpression reduced = simplify(original);
        Map<Identifier, Real> at_minus_one;
        at_minus_one[x.name()] = Real(-1);
        std::cout << "original-expression=" << original
                  << " reduced-expression=" << reduced << '\n';
        try {
            std::cout << "original-at-minus-one="
                      << evaluate(original, at_minus_one).compute_get(Effort(8), dp)
                      << '\n';
        } catch (std::exception const& error) {
            std::cout << "original-at-minus-one-threw=" << error.what() << '\n';
        }
        std::cout << "reduced-at-minus-one="
                  << evaluate(reduced, at_minus_one).compute_get(Effort(8), dp)
                  << '\n';
        return 0;
    }

    if (which == "vector-mismatch") {
        RealVectorExpression lhs({x, y});
        RealVectorExpression rhs({x});
        RealVectorExpression sum = add(lhs, rhs);
        std::cout << "reported-size=" << sum.size() << " lhs=2 rhs=1\n";
        std::cout << "second-component=" << simplify(sum[1]) << '\n';
        return 0;
    }

    if (which == "component-constant") {
        RealVectorExpression vector({x, RealExpression(1)});
        std::cout << "component-1-constant-in-x="
                  << is_constant_in(vector[1], Set<RealVariable>({x}))
                  << " expected=1\n";
        return 0;
    }

    if (which == "category") {
        std::cout << "primed=" << VariableCategory::PRIMED << " expected=PRIMED\n"
                  << "dotted=" << VariableCategory::DOTTED << " expected=DOTTED\n";
        return 0;
    }

    if (which == "constant-string") {
        Constant<String> value(String("payload"));
        std::cout << "name=" << value.name() << " value=" << value.value() << '\n';
        return 0;
    }

    if (which == "three-argument") {
        Symbolic<SumThree, int, int, int> value(SumThree(), 1, 2, 4);
        std::cout << "sum=" << static_cast<int>(value) << " expected=7\n";
        return 0;
    }

    if (which == "iterate") {
        Sequence<int> sequence = make_dangling_iteration();
        overwrite_stack();
        std::cout << "term-3=" << sequence[Natural(3u)] << " expected=13\n";
        return 0;
    }

    if (which == "before-pi") {
        RealExpression lhs(pi);
        RealExpression rhs(pi);
        std::cout << "before=" << before(lhs, rhs) << " expected=0\n";
        return 0;
    }

    return 65;
}
