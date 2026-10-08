#include "config.hpp"
#include "numeric/numeric.hpp"
#include "symbolic/space.hpp"
#include "symbolic/valuation.hpp"

int main() {
    Ariadne::Map<Ariadne::Identifier, Ariadne::String> strings;
    Ariadne::Map<Ariadne::Identifier, Ariadne::Real> values;
    Ariadne::HybridValuation<Ariadne::Real> valuation(strings, values);
    return 0;
}
