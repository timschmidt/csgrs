/***************************************************************************
 *            symbolic/function_expression.hpp
 *
 *  Copyright  2008-20  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This file is part of Ariadne.
 *
 *  Ariadne is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ariadne is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.
 */

/*! \file symbolic/function_expression.hpp
 *  \brief Expressions created as wrappers around functions
 */

#ifndef ARIADNE_FUNCTION_EXPRESSION_HPP
#define ARIADNE_FUNCTION_EXPRESSION_HPP

#include "symbolic/expression.hpp"
#include "symbolic/expression_set.hpp"
#include "symbolic/space.hpp"

#include "function/function.hpp"
#include "function/formula.hpp"
#include "function/function_model.hpp"
#include "function/taylor_function.hpp"

namespace Ariadne {

template<class P, class SIG> class FunctionExpression;
/*
template<class P, class D> using ScalarFunctionExpression = FunctionExpression<P,D,IntervalDomainType>;
template<class P, class D> using VectorFunctionExpression = FunctionExpression<P,D,BoxDomainType>;
template<class P> using ScalarMultivariateFunctionExpression = FunctionExpression<P,BoxDomainType,IntervalDomainType>;
template<class P> using VectorMultivariateFunctionExpression = FunctionExpression<P,BoxDomainType,BoxDomainType>;

using ValidatedScalarFunctionExpression = FunctionExpression<ValidatedTag,BoxDomainType,IntervalDomainType>;
using ValidatedVectorFunctionExpression = FunctionExpression<ValidatedTag,BoxDomainType,BoxDomainType>;
*/

template<class P> using ScalarMultivariateFunctionExpression = FunctionExpression<P,RealScalar(RealVector)>;
template<class P> using VectorMultivariateFunctionExpression = FunctionExpression<P,RealVector(RealVector)>;

using VariableIntervalDomainType = VariableInterval<IntervalDomainType::UpperBoundType>;
using VariablesBoxDomainType = VariablesBox<IntervalDomainType>;

inline BoxDomainType preimage(Projection const& prj, BoxDomainType const& bx) {
    BoxDomainType dbx(prj.argument_size(),IntervalDomainType(-infty,+infty));
    for(SizeType i=0; i!=prj.result_size(); ++i) {
        IntervalDomainType& dbxj=dbx[prj.index(i)]; dbxj=intersection(dbxj,bx[i]); }
    return dbx;
}

VariableIntervalDomainType _make_domain(RealVariable const& arg, IntervalDomainType const& dom) {
    return VariableIntervalDomainType(arg,dom);
}

VariablesBoxDomainType _make_domain(Vector<RealVariable> const& args, BoxDomainType const& dom) {
    Map<RealVariable,FloatDPExactInterval> domain;
    for(SizeType i=0; i!=args.size(); ++i) { domain[args[i]]=dom[i]; }
    return domain;
}

template<class P,class RES> class FunctionExpression<P,RES(RealVariable)> {
    using ARG=RealVariable;
    using SIG=RES(ARG);
    typedef BoxDomainType D;
  public:
    template<class Y> using Argument = typename SignatureTraits<SIG>::template ArgumentType<Y>;
    template<class Y> using Result = typename SignatureTraits<SIG>::template ResultType<Y>;
  private:
    Argument<RealVariable> _vars;
    Function<P,SIG> _f;
  public:
    FunctionExpression(Function<P,SIG> const& f, Argument<RealVariable> const& vars) : _vars(vars), _f(f) {
        ARIADNE_PRECONDITION(_f.argument_size()==_vars.size()); }
    Function<P,SIG> const& function() const { return this->_f; }
    Argument<RealVariable> const& variables() const { return this->_vars; }
    Set<RealVariable> arguments() const { Set<RealVariable> args; for(SizeType i=0; i!=_vars.size(); ++i) { args.insert(_vars[i]); } return args; }
    auto domain() const -> decltype(_make_domain(_vars,_f.domain())) { return _make_domain(_vars,_f.domain()); }
    template<class PP> friend OutputStream& operator<<(OutputStream& os, FunctionExpression<PP,SIG> const& sfe);
};

template<class P> OutputStream& operator<<(OutputStream& os, ScalarMultivariateFunctionExpression<P> const& e) {
    os << e._f;
    for(SizeType i=0; i!=e._vars.size(); ++i) {
        os << (i==0?'[':',') << "x[" << i << "]=" << e._vars[i];
    } return os << "]";
}

template<class P> FunctionExpression<P,RealVector(RealVector)> FunctionFacade<P,RealVector(RealVector)>::operator() (Vector<RealVariable> const& vars) const {
    return FunctionExpression<P,RealVector(RealVector)>(static_cast<Function<P,RealVector(RealVector)>const&>(*this),vars);
}

template<class P> ScalarMultivariateFunctionExpression<P> evaluate(ScalarMultivariateFunction<P> const& f, Vector<RealVariable> const& vars) {
    return ScalarMultivariateFunctionExpression<P>(f,vars);
}

template<class F> TaylorModel<ValidatedTag,F> compose(const TaylorModel<ValidatedTag,F>& x, Projection const& prj) {
    TaylorModel<ValidatedTag,F> r(prj.argument_size(),x.sweeper());
    r.expansion().reserve(x.number_of_nonzeros());
    MultiIndex ra(r.argument_size());
    for(auto xiter=x.begin(); xiter!=x.end(); ++xiter) {
        UniformConstReference<MultiIndex> xa=xiter->index();
        UniformConstReference<F> xc=xiter->coefficient();
        for(SizeType i=0; i!=r.argument_size(); ++i) {
            ra[i]=xa[prj.index(i)];
        }
        r.expansion().append(ra,xc);
    }
    r.cleanup();
    return r;
}

template<class M> ScaledFunctionPatch<M> compose(ScaledFunctionPatch<M> const& f, Projection const& prj) {
    auto f_dom=f.domain();
    BoxDomainType dom=preimage(prj,f_dom);

    bool has_strict_subdomain = false;
    for(SizeType i=0; i!=prj.result_size(); ++i) {
        if(not same(f_dom[i],dom[prj.index(i)])) { has_strict_subdomain=true; }
    }
    if (has_strict_subdomain) {
        Vector<ScaledFunctionPatch<M>> id=f.create_coordinates(dom);
        return compose(f,prj(id));
    } else {
        return ScaledFunctionPatch<M>(dom,compose(f.model(),prj));
    }
}

template<class P, class PR, class PRE> ScalarFunctionModel<P,PR,PRE> compose(ScalarFunctionModel<P,PR,PRE> const& f, Projection const& prj) {
    auto fpp = std::dynamic_pointer_cast<const ValidatedScalarMultivariateTaylorFunctionModelDP>(f.managed_pointer());
    if(fpp) {
        return compose(ValidatedScalarMultivariateTaylorFunctionModelDP(fpp),prj);
    }
    auto f_dom=f.domain();
    BoxDomainType dom=preimage(prj,f_dom);
    Vector<ScalarFunctionModel<P,PR,PRE>> id=f.create_coordinates(dom);
    Vector<ScalarFunctionModel<P,PR,PRE>> pid=prj(id);
    VectorFunctionModel<P,PR,PRE> vpid(pid.array());
    return compose(f,vpid);
}

template<class P> ScalarFunction<P,BoxDomainType> compose(ScalarFunction<P,BoxDomainType> const& f, Projection const& prj) {
    auto fmp = std::dynamic_pointer_cast<const typename ScalarMultivariateFunctionModelDP<P>::Interface>(f.managed_pointer());
    if(fmp) {
        return compose(ScalarMultivariateFunctionModelDP<P>(fmp),prj);
    }
    auto f_dom=f.domain();
    BoxDomainType dom=preimage(prj,f_dom);
    Vector<ScalarFunction<P,BoxDomainType>> id(ScalarFunction<P,BoxDomainType>::coordinates(dom));
    VectorFunction<P,BoxDomainType> pid=prj(id);
    return compose(f,pid);
}

template<class P> ScalarFunction<P,EuclideanDomain> compose(ScalarFunction<P,EuclideanDomain> const& f, Projection const& prj) {
    auto f_dom=f.domain();
    BoxDomainType dom=preimage(prj,f_dom);
    Vector<ScalarFunction<P,EuclideanDomain>> id(ScalarFunction<P,EuclideanDomain>::coordinates(dom));
    VectorFunction<P,EuclideanDomain> pid=prj(id);
    return compose(f,pid);
}

RealScalarMultivariateFunction compose(RealScalarMultivariateFunction const& f, Projection const& prj) {
    EuclideanDomain dom(prj.argument_size());
    Vector<RealScalarMultivariateFunction> id(RealScalarMultivariateFunction::coordinates(dom));
    RealVectorMultivariateFunction pid=prj(id);
    return compose(f,pid);
}


template<class P> ScalarMultivariateFunction<P> make_function(RealSpace args, ScalarMultivariateFunctionExpression<P> const& expr) {
    ARIADNE_PRECONDITION(subset(expr.arguments(),args.variables()));
    auto expr_vars=expr.variables();
    auto expr_func=expr.function();

    Array<SizeType> indices(expr_vars.size());
    for(SizeType i=0; i!=expr_vars.size(); ++i) {
        indices[i]=args.index(expr_vars[i]);
    }
    Projection projection(args.size(),indices);

    return compose(expr_func,projection);
}

} // namespace Ariadne

#endif /* ARIADNE_FUNCTION_EXPRESSION_HPP */
