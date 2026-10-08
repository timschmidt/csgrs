#include "config.hpp"

#include "algebra/sweeper.hpp"
#include "algebra/algebra.hpp"
#include "function/affine.hpp"
#include "function/affine_model.hpp"
#include "function/affine_model.tpl.hpp"
#include "function/chebyshev_polynomial.hpp"
#include "function/formula.hpp"
#include "function/function.hpp"
#include "function/polynomial.hpp"
#include "function/procedure.hpp"
#include "function/taylor_function.hpp"
#include "function/taylor_model.hpp"

#include <iostream>
#include <string>

using namespace Ariadne;

int main(int argc, char** argv) {
    if (argc != 2) { return 64; }
    const std::string which(argv[1]);

    if (which == "affine") {
        using X = FloatDPApproximation;
        Affine<X> a(X(5,dp), {X(3,dp)});
        Vector<X> x({X(2,dp)});
        std::cout << "value=" << a.evaluate(x) << " expected=11\n";
        return 0;
    }

    if (which == "affine-model-mul") {
        using A = AffineModel<ApproximateTag,FloatDP>;
        A a(FloatDPApproximation(2,dp), {FloatDPApproximation(3,dp)});
        A b(FloatDPApproximation(4,dp), {FloatDPApproximation(5,dp)});
        A c = a*b;
        std::cout << "value=" << c.value() << " gradient=" << c.gradient(0)
                  << " expected-value=8 expected-gradient=22\n";
        return 0;
    }

    if (which == "procedure-duplicate") {
        ApproximateFormula x = ApproximateFormula::coordinate(0);
        Vector<ApproximateFormula> fs({x,x});
        Vector<ApproximateProcedure> p(1,fs);
        Vector<FloatDPApproximation> v({FloatDPApproximation(7,dp)});
        List<FloatDPApproximation> t(p._instructions.size(),v.zero_element());
        execute(t,p,v);
        Vector<FloatDPApproximation> r(p.result_size(),v.zero_element());
        for (SizeType i=0; i!=r.size(); ++i) { r[i]=std::move(t[p._results[i]]); }
        std::cout << "results=" << r << " expected=[7,7]\n";
        return 0;
    }

    if (which == "procedure-from-scalar") {
        ApproximateFormula x = ApproximateFormula::coordinate(0);
        ApproximateProcedure p(1,x);
        Vector<ApproximateProcedure> v(p);
        std::cout << "argument-size=" << v.argument_size() << " expected=1\n";
        return 0;
    }

    if (which == "polynomial-partial-constant") {
        MultivariatePolynomial<FloatDPApproximation> p(1,dp);
        p[MultiIndex({0})] = FloatDPApproximation(3,dp);
        auto q = partial_evaluate(p,0,FloatDPApproximation(2,dp));
        std::cout << "result=" << q << "\n";
        return 0;
    }

    if (which == "chebyshev-add") {
        using C = UnivariateChebyshevPolynomial<FloatDPApproximation>;
        C x = C::coordinate(dp);
        C y = x + FloatDPApproximation(3,dp);
        std::cout << "value=" << y(FloatDPApproximation(2,dp)) << " expected=5\n";
        return 0;
    }

    if (which == "chebyshev-mul") {
        using C = UnivariateChebyshevPolynomial<FloatDPApproximation>;
        C p = C::basis(0,dp) + C::basis(1,dp);
        C q = p*p;
        std::cout << "result=" << q << "\n";
        return 0;
    }

    if (which == "function-join") {
        ValidatedScalarMultivariateFunction s = ValidatedScalarMultivariateFunction::constant(2,1);
        ValidatedVectorMultivariateFunction v = ValidatedVectorMultivariateFunction::identity(2);
        auto j = join(s,v);
        std::cout << "result-size=" << j.result_size() << " expected=3\n";
        return 0;
    }

    if (which == "function-alias") {
        EffectiveVectorMultivariateFunction id = EffectiveVectorMultivariateFunction::identity(2);
        EffectiveVectorMultivariateFunction copy = id;
        copy[0] = copy[1];
        Vector<FloatDPApproximation> x({FloatDPApproximation(2,dp),FloatDPApproximation(3,dp)});
        std::cout << "original=" << id(x) << " copy=" << copy(x)
                  << " expected-original=[2,3]\n";
        return 0;
    }

    if (which == "scaled-sub") {
        ThresholdSweeper<FloatDP> swp(dp,1e-8);
        ExactBoxType dom={{-1,+1}};
        auto a=ValidatedVectorMultivariateTaylorFunctionModelDP::identity(dom,swp);
        auto b=a;
        a-=b;
        Vector<FloatDPBounds> x({FloatDPBounds(0.5_x,dp)});
        std::cout << "value=" << a(x) << " expected=[0]\n";
        return 0;
    }

    if (which == "taylor-negative-power") {
        ThresholdSweeper<FloatDP> swp(dp,1e-8);
        auto x=ValidatedTaylorModelDP::constant(1,FloatDPBounds(2,dp),swp);
        auto y=pow(x,-1);
        Vector<FloatDPBounds> p({FloatDPBounds(0,dp)});
        std::cout << "value=" << evaluate(y,p) << " expected=[0.5,0.5]\n";
        return 0;
    }

    if (which == "sweep-bounds") {
        ThresholdSweeper<FloatDP> swp(dp,FloatDP(1,dp));
        ValidatedBoundsTaylorModelDP tm(1,swp);
        tm.set_gradient(0,FloatDPBounds(0.5_x,dp));
        tm.sweep();
        std::cout << "terms=" << tm.number_of_terms() << " error=" << tm.error()
                  << " expected-terms=0 expected-error-at-least=0.5\n";
        return 0;
    }

    if (which == "sweep-interval") {
        ThresholdSweeper<FloatDP> swp(dp,FloatDP(1,dp));
        ValidatedIntervalTaylorModelDP tm(1,swp);
        tm.set_gradient(0,FloatDPUpperInterval(FloatDPBounds(0.5_x,dp)));
        tm.sweep();
        std::cout << "terms=" << tm.number_of_terms() << " error=" << tm.error()
                  << " expected-terms=0 expected-error-at-least=0.5\n";
        return 0;
    }

    if (which == "taylor-derivative-error") {
        ThresholdSweeper<FloatDP> swp(dp,1e-8);
        auto tm=ValidatedTaylorModelDP::unit_ball(1,swp);
        auto dtm=derivative(tm,0);
        std::cout << "input-error=" << tm.error() << " derivative-error=" << dtm.error()
                  << " expected-no-finite-derivative-enclosure\n";
        return 0;
    }

    if (which == "scaled-same-mismatch") {
        ThresholdSweeper<FloatDP> swp(dp,1e-8);
        ExactBoxType dom={{-1,+1}};
        auto one=ValidatedVectorMultivariateTaylorFunctionModelDP::identity(dom,swp);
        auto two=join(one,one);
        std::cout << "same=" << same(two,one) << " expected=false\n";
        return 0;
    }

    if (which == "patch-plus-constant") {
        ThresholdSweeper<FloatDP> swp(dp,1e-8);
        ExactBoxType dom={{-1,+1}};
        ValidatedScalarMultivariateTaylorFunctionModelDP patch=
            ValidatedScalarMultivariateTaylorFunctionModelDP::coordinate(dom,0,swp);
        ValidatedScalarMultivariateFunction function=patch;
        auto result=function+ValidatedNumber(1);
        Vector<FloatDPBounds> x({FloatDPBounds(0,dp)});
        std::cout << "value=" << result(x) << " expected=[1,1]\n";
        return 0;
    }

    return 65;
}
