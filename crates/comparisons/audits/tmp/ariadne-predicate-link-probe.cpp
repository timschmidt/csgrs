#include "config.hpp"
#include "algebra/vector.hpp"
#include "numeric/numeric.hpp"
#include "function/function.hpp"
#include "symbolic/predicate.hpp"

int main() {
    Ariadne::DisjunctivePredicate predicate;
    return predicate.tautologous() ? 0 : 1;
}
