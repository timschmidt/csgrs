#include "config.hpp"
#include "function/formula.hpp"
#include "function/procedure.hpp"

using namespace Ariadne;

int main() {
    ApproximateFormula x=ApproximateFormula::coordinate(0);
    Vector<ApproximateFormula> formulas({x});
    Vector<ApproximateProcedure> procedure(1,formulas);
    Vector<FloatDPApproximation> arguments({FloatDPApproximation(7,dp)});
    auto result=evaluate(procedure,arguments);
    return result.size()!=1;
}
