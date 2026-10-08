/***************************************************************************
 *            function/function.cpp
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

#include <typeinfo>

#include "numeric/numeric.hpp"

#include "numeric/operators.hpp"
#include "algebra/differential.hpp"
#include "algebra/algebra.hpp"
#include "function/taylor_model.hpp"

#include "function/formula.hpp"
#include "function/formula.tpl.hpp"

#include "function/function.hpp"

#include "function/function_mixin.hpp"
#include "function/function_mixin.tpl.hpp"

#include "function/function_patch.hpp"
#include "function/function_model.hpp"

#include "function/symbolic_function.hpp"

#include "symbolic/expression.hpp"


namespace Ariadne {

//template<class T> inline String class_name() { return "Unknown"; }

template<class T> inline StringType str(const T& t) {
    StringStream ss; ss << t; return ss.str(); }

// Templated conversions dynamically checked at runtime
template<class R, class A> R const& checked_same(A const& a) {
    if constexpr(Same<R,A>) { return a; }
    else { ARIADNE_THROW(std::runtime_error,"checked_same<R,A> with R="<<class_name<R>()<<", A="<<class_name<A>(),"argument "<<a<<" does not have the same type as result."); } }
template<class R, class A> R checked_convert(A&& a) {
    if constexpr(Convertible<A,R>) { return a; }
    else { ARIADNE_THROW(std::runtime_error,"checked_convert<R,A> with R="<<class_name<R>()<<", A="<<class_name<A>(),"argument "<<a<<" is not convertible to result."); } }
template<class R, class A> R checked_construct(A const& a) {
    if constexpr(Constructible<R,A>) { return R(a); }
    else { ARIADNE_THROW(std::runtime_error,"checked_construct<R,A> with R="<<class_name<R>()<<", A="<<class_name<A>(),"argument "<<a<<" is not explicitly convertible to result."); } }

//------------------------ Formula Function ----------------------------------//

template<class P, class Y> ScalarUnivariateFunction<P> make_formula_function(RealDomain dom, Scalar<Formula<Y>> const& e) {
    return ScalarUnivariateFunction<P>(ScalarUnivariateFormulaFunction<Y>(e));
}

template<class P, class Y> VectorUnivariateFunction<P> make_formula_function(RealDomain dom, Vector<Formula<Y>> const& e) {
    return VectorUnivariateFunction<P>(VectorUnivariateFormulaFunction<Y>(e));
}

template<class P, class Y> ScalarMultivariateFunction<P> make_formula_function(EuclideanDomain dom, Scalar<Formula<Y>> const& e) {
    return ScalarMultivariateFunction<P>(ScalarFormulaFunction<Y>(dom.dimension(),e));
}

template<class P, class Y> VectorMultivariateFunction<P> make_formula_function(EuclideanDomain dom, Vector<Formula<Y>> const& e) {
    return VectorMultivariateFunction<P>(VectorFormulaFunction<Y>(dom.dimension(),e));
}

//------------------------ Function ----------------------------------//

namespace {

template<class P, class D> ScalarFunction<P,ElementKind<D>> make_zero_function(SizeOne rs, D dom) {
    return FunctionConstructors<P>::zero(dom); }
template<class P, class D> VectorFunction<P,ElementKind<D>> make_zero_function(SizeType rs, D dom) {
    return  FunctionConstructors<P>::zeros(rs,dom); }
}

template<class P, class SIG> Function<P,SIG>::Function() : Function(nullptr) {
}

template<class P, class SIG> Function<P,SIG>::Function(DomainType dom)
    : Function(ResultSizeType(),dom) { }

template<class P, class SIG> Function<P,SIG>::Function(ResultSizeType rs, ArgumentSizeType as)
    : Function(rs,DomainType(as)) { }

template<class P, class SIG> Function<P,SIG>::Function(ResultSizeType rs, DomainType dom)
    : Function(make_zero_function<P,D>(rs,dom)) { }

template<class P, class SIG> Function<P,SIG>::Function(ResultSizeType rs, ScalarFunction<P,ARG> sf)
    : Function(Vector<ScalarFunction<P,ARG>>(SizeType(rs),sf)) {
}

template<class P, class SIG> Function<P,SIG>::Function(InitializerList<ScalarFunction<P,ARG>> const& lsf)
    : Function(Vector<ScalarFunction<P,ARG>>(lsf)) {
}

template<class P, class SIG> Function<P,SIG>::Function(List<ScalarFunction<P,ARG>> const& lsf)
    : Function(Vector<ScalarFunction<P,ARG>>(lsf)) {
}

template<class P, class SIG> Function<P,SIG>::Function(DomainType dom, Result<Formula<Y>>const& e)
    : Function(make_formula_function<P>(dom,e)) { }

template<class P, class SIG> Function<P,SIG>::Function(typename SignatureTraits<SIG>::ArgumentSpaceType const& spc, Result<RealExpression> const& e)
    : Function(make_function(spc,e)) { }

template<class P, class SIG> struct MakeVectorFunction;
template<class P, class... ARGS> struct MakeVectorFunction<P,Real(ARGS...)> {
    ScalarFunction<P,ARGS...> create(Vector<ScalarFunction<P,ARGS...>> const& lsf) {
        ARIADNE_FAIL_MSG("Cannot construct scalar function from list."); }
};
/*
template<class P> struct MakeVectorFunction<P,RealVector(Real)> {
    VectorUnivariateFunction<P> create(Vector<ScalarUnivariateFunction<P>> const& lsf) {
        ARIADNE_FAIL_MSG("Cannot construct multivariate function from list of univariate functions."); }
};
*/
template<class P> struct MakeVectorFunction<P,RealVector(Real)> {
    VectorUnivariateFunction<P> create(Vector<ScalarUnivariateFunction<P>> const& lsf) {
        return VectorUnivariateFunction<P>(VectorOfScalarFunction<P,RealScalar>(lsf));
    }
};
template<class P> struct MakeVectorFunction<P,RealVector(RealVector)> {
    VectorMultivariateFunction<P> create(Vector<ScalarMultivariateFunction<P>> const& lsf) {
        if constexpr (Same<P,ValidatedTag>) {
            if (lsf.size()==1) {
                auto fmptr = dynamic_cast<ValidatedScalarMultivariateFunctionModelDP::Interface const*>(lsf[0].raw_pointer());
                if (fmptr) {
                    auto fm = ValidatedScalarMultivariateFunctionModelDP(*fmptr);
                    auto vfm = ValidatedVectorMultivariateFunctionModelDP(List<ValidatedScalarMultivariateFunctionModelDP>(1u,fm));
                    return VectorMultivariateFunction<P>(vfm);
                }
            }
        }
        return VectorMultivariateFunction<P>(VectorOfScalarFunction<P,RealVector>(lsf));
    }
};


template<class P, class SIG, class... ARGS> inline decltype(auto) make_vector_function(Vector<ScalarFunction<P,ARGS...>> const& lsf) {
    return MakeVectorFunction<P,SIG>().create(lsf);
}

template<class P, class SIG> Function<P,SIG>::Function(Vector<ScalarFunction<P,ARG>> const& vsf)
    : Function<P,SIG>(make_vector_function<P,SIG>(vsf)) {
}


//------------------------ Function Constructors -----------------------------------//

template<class P> ScalarMultivariateFunction<P> FunctionConstructors<P>::zero(VectorDomainType dom) {
    return ConstantFunction<Y,RealVector>(dom, Y(0));
}


template<class P> ScalarMultivariateFunction<P> FunctionConstructors<P>::constant(VectorDomainType dom, NumericType c) {
    return ConstantFunction<Y,RealVector>(dom, c);
}

template<class P> ScalarMultivariateFunction<P> FunctionConstructors<P>::coordinate(VectorDomainType dom, SizeType j) {
    return CoordinateFunction<P,RealVector>(dom, j);
}

template<class P> List<ScalarMultivariateFunction<P>> FunctionConstructors<P>::coordinates(VectorDomainType dom) {
    List<ScalarMultivariateFunction<P>> r; r.reserve(dom.dimension());
    for(SizeType j=0; j!=dom.dimension(); ++j) { r.append(coordinate(dom,j)); }
    return r;
}

template<class P> VectorMultivariateFunction<P> FunctionConstructors<P>::zeros(SizeType rs, VectorDomainType dom) {
    return VectorMultivariateFunction<P>(VectorOfScalarMultivariateFunction<P>(rs,zero(dom)));
}

template<class P> VectorMultivariateFunction<P> FunctionConstructors<P>::identity(VectorDomainType dom) {
    SizeType n=dom.dimension();
    ScalarMultivariateFunction<P> z=ScalarMultivariateFunction<P>::zero(dom);
    VectorOfScalarMultivariateFunction<P> res = VectorOfScalarMultivariateFunction<P>(n,z);
    for(SizeType i=0; i!=n; ++i) {
        res._vec[i]=ScalarMultivariateFunction<P>::coordinate(dom,i);
    }
    return VectorMultivariateFunction<P>(res);
}


template<class P> ScalarUnivariateFunction<P> FunctionConstructors<P>::zero(ScalarDomainType dom) {
    return ConstantUnivariateFunction<Y>(dom, Y(0));
}

template<class P> ScalarUnivariateFunction<P> FunctionConstructors<P>::constant(ScalarDomainType dom, NumericType c) {
    return ConstantUnivariateFunction<Y>(dom,c);
}

template<class P> ScalarUnivariateFunction<P> FunctionConstructors<P>::coordinate(ScalarDomainType dom, IndexZero as) {
    return CoordinateFunction<P,Real>(dom,as);
}

template<class P> ScalarUnivariateFunction<P> FunctionConstructors<P>::coordinate(ScalarDomainType dom) {
    return coordinate(dom,IndexZero());
}

template<class P> VectorUnivariateFunction<P> FunctionConstructors<P>::zeros(SizeType rs, ScalarDomainType dom) {
    return constant(dom,Vector<Y>(rs,0));
}


template<class P> ScalarUnivariateFunction<P> FunctionConstructors<P>::identity(ScalarDomainType dom) {
    return coordinate(dom,IndexZero());
}


template<class P> ScalarUnivariateFunction<P> FunctionConstructors<P>::zero() {
    return zero(RealDomain());
}

template<class P> ScalarUnivariateFunction<P> FunctionConstructors<P>::constant(NumericType c) {
    return constant(RealDomain(),c);
}

template<class P> ScalarUnivariateFunction<P> FunctionConstructors<P>::coordinate() {
    return coordinate(RealDomain());
}

template<class P> VectorUnivariateFunction<P> FunctionConstructors<P>::zeros(SizeType rs) {
    return zeros(rs,RealDomain());
}


template<class P> ScalarUnivariateFunction<P> FunctionConstructors<P>::identity() {
    return identity(RealDomain());
}


template<class P> ScalarMultivariateFunction<P> FunctionConstructors<P>::zero(SizeType as) {
    return ScalarMultivariateFunction<P>(ScalarFormulaFunction<Y>(as,Formula<Y>::zero()));
}

template<class P> ScalarMultivariateFunction<P> FunctionConstructors<P>::constant(SizeType as, NumericType c) {
    return ScalarMultivariateFunction<P>(ScalarFormulaFunction<Y>(as,Formula<Y>::constant(c)));
}

template<class P> ScalarMultivariateFunction<P> FunctionConstructors<P>::coordinate(SizeType as, SizeType j) {
    ARIADNE_ASSERT(j<as);
    return ScalarMultivariateFunction<P>(ScalarFormulaFunction<Y>(as,Formula<Y>::coordinate(j)));
}

template<class P> VectorMultivariateFunction<P> FunctionConstructors<P>::zeros(SizeType rs, SizeType as) {
    VectorOfScalarMultivariateFunction<P>res = VectorOfScalarMultivariateFunction<P>(rs,as);
    for(SizeType i=0; i!=rs; ++i) {
        res._vec[i]=ScalarMultivariateFunction<P>::zero(as);
    }
    return VectorMultivariateFunction<P>(res);
}

template<class P> VectorMultivariateFunction<P> FunctionConstructors<P>::constant(SizeType as, Vector<NumericType> c) {
    SizeType rs=c.size();
    VectorOfScalarFunction<P,RealVector>res = VectorOfScalarFunction<P,RealVector>(rs,as);
    for(SizeType i=0; i!=rs; ++i) {
        res._vec[i]=ScalarMultivariateFunction<P>::constant(as,c[i]);
    }
    return VectorMultivariateFunction<P>(res);
}

template<class P> List<ScalarMultivariateFunction<P>> FunctionConstructors<P>::coordinates(SizeType as) {
    List<ScalarMultivariateFunction<P>> r; r.reserve(as);
    for(SizeType j=0; j!=as; ++j) { r.append(coordinate(as,j)); }
    return r;
}

template<class P> VectorMultivariateFunction<P> FunctionConstructors<P>::identity(SizeType n) {
    ScalarMultivariateFunction<P> z=ScalarMultivariateFunction<P>::zero(n);
    VectorOfScalarFunction<P,RealVector>res = VectorOfScalarFunction<P,RealVector>(n,n);
    for(SizeType i=0; i!=n; ++i) {
        res._vec[i]=ScalarMultivariateFunction<P>::coordinate(n,i);
    }
    return VectorMultivariateFunction<P>(res);
}

template<class P> VectorMultivariateFunction<P> FunctionConstructors<P>::constant(VectorDomainType dom, Vector<NumericType> c) {
    SizeType n=c.size();
    ScalarMultivariateFunction<P> z=ScalarMultivariateFunction<P>::zero(dom);
    VectorOfScalarFunction<P,RealVector> res = VectorOfScalarFunction<P,RealVector>(n,z);
    for(SizeType i=0; i!=n; ++i) {
        res._vec[i]=ScalarMultivariateFunction<P>::constant(dom,c[i]);
    }
    return VectorMultivariateFunction<P>(res);
}

template<class P> VectorUnivariateFunction<P> FunctionConstructors<P>::constant(ScalarDomainType dom, Vector<NumericType> c) {
    SizeType n=c.size();
    ScalarUnivariateFunction<P> z=ScalarUnivariateFunction<P>::zero(dom);
    VectorOfScalarUnivariateFunction<P>res = VectorOfScalarUnivariateFunction<P>(n,z);
    for(SizeType i=0; i!=n; ++i) {
        res._vec[i]=ScalarUnivariateFunction<P>::constant(dom,c[i]);
    }
    return VectorUnivariateFunction<P>(res);
}

template<class P> VectorUnivariateFunction<P> FunctionConstructors<P>::constant(Vector<NumericType> c) {
    return constant(RealDomain(),c);
}


template class FunctionConstructors<ApproximateTag>;
template class FunctionConstructors<ValidatedTag>;
template class FunctionConstructors<EffectiveTag>;

//------------------------ Converting to and from Expression classes to Function classes via Formula -----------------------------------//

template<class X> class Expression;
template<class X> class Variable;
template<class X> class Space;

SizeType dimension(const Space<Real>& spc);


inline EffectiveScalarUnivariateFunction make_formula_function(SizeOne as, const Formula<EffectiveNumber>& fm) {
    return EffectiveScalarUnivariateFunction(RealDomain(),fm); }
inline EffectiveVectorUnivariateFunction make_formula_function(SizeOne as, const Vector<Formula<EffectiveNumber>>& fm) {
    return EffectiveVectorUnivariateFunction(RealDomain(),fm); }
inline EffectiveScalarMultivariateFunction make_formula_function(SizeType as, const Formula<EffectiveNumber>& fm) {
    return EffectiveScalarMultivariateFunction(EuclideanDomain(as),fm); }
inline EffectiveVectorMultivariateFunction make_formula_function(SizeType as, const Vector<Formula<EffectiveNumber>>& fm) {
    return EffectiveVectorMultivariateFunction(EuclideanDomain(as),fm); }

EffectiveScalarUnivariateFunction make_function(const Variable<Real>& var, const Expression<Real>& expr) {
    return make_formula_function(SizeOne(),make_formula(expr,var)); }
EffectiveVectorUnivariateFunction make_function(const Variable<Real>& var, const Vector<Expression<Real>>& expr) {
    return make_formula_function(SizeOne(),make_formula(expr,var)); }
EffectiveScalarMultivariateFunction make_function(const Space<Real>& spc, const Expression<Real>& expr) {
    return make_formula_function(dimension(spc),make_formula(expr,spc)); }
EffectiveVectorMultivariateFunction make_function(const Space<Real>& spc, const Vector<Expression<Real>>& expr) {
    return make_formula_function(dimension(spc),make_formula(expr,spc)); }

template<class P, class R, class T, class... AS> Function<P,R(AS...)> make_composed_function(const Function<P,R(T)>& f, const Function<P,T(AS...)>& g) {
    ARIADNE_ASSERT(f.argument_size()==g.result_size());
    return Function<P,R(AS...)>(ComposedFunction<P,R,T,AS...>(f,g)); }

Formula<Real> make_formula(const EffectiveScalarMultivariateFunction& f) {
    Vector<Formula<Real>> x(f.argument_size());
    for(SizeType i=0; i!=x.size(); ++i) {
        x[i]=Formula<Real>::coordinate(i);
    }
    return f(x);
}

Vector<Formula<Real>> make_formula(const EffectiveVectorMultivariateFunction& f) {
    const VectorMultivariateFunctionInterface<EffectiveTag>& fi=f;
    const VectorFormulaFunction<Real>* ff;
    const VectorOfScalarMultivariateFunction<EffectiveTag>* vf;
    if( (vf=dynamic_cast<const VectorOfScalarMultivariateFunction<EffectiveTag>*>(&fi)) ) {
        Vector< Formula<Real> > r(vf->result_size());
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=make_formula((*vf)[i]); }
        return r;
    } else if( (ff=dynamic_cast<const VectorFormulaFunction<Real>*>(&fi)) ) {
        return ff->formulae();
    } else {
        ARIADNE_FAIL_MSG("Cannot compute formula for function "<<f);
    }
}

//------------------------ Scalar Function ----------------------------------//

//------------------------ Vector Function ----------------------------------//

template<class P, class SIG> auto Function<P,SIG>::get(SizeType i) const -> ScalarFunction<P,ARG> {
    ARIADNE_ASSERT(i<this->result_size());
    if constexpr (Same<ResultSizeType,SizeType>) {
        const VectorOfFunctionInterface<P,ARG>* vfp = dynamic_cast<const VectorOfFunctionInterface<P,ARG>*>(this->raw_pointer());
        return ScalarFunction<P,ARG>(SharedPointer<ScalarFunctionInterface<P,ARG>>(vfp->_get(i)));
    } else {
        ARIADNE_ASSERT((Same<ResultSizeType,SizeType>));
        std::abort();
    }
}

template<class P, class SIG> Void Function<P,SIG>::set(SizeType i, ScalarFunction<P,ARG> sf) {
    ARIADNE_ASSERT(i<this->result_size());
    if constexpr (Same<ResultSizeType,SizeType>) {
        const VectorOfScalarFunction<P,ARG>* cvf =
            dynamic_cast<const FunctionWrapper<VectorOfScalarFunction<P,ARG>,P,SIG>*>(this->_ptr.operator->());
        if (cvf==nullptr) {
            ARIADNE_THROW(std::runtime_error,
                          "Function<P,RealVector(ARGS...)>::set(SizeType i, Function<P,RealScalar(ARGS...)>)",
                          "Cannot assign to component of vector-valued function "<<(*this)<<" as it is not a VectorOfScalarFunction.");
        }
        VectorOfScalarFunction<P,ARG>& vf = const_cast<VectorOfScalarFunction<P,ARG>&>(*cvf);
        vf[i]=sf;
    } else {
        ARIADNE_ASSERT((Same<ResultSizeType,SizeType>));
        std::abort();
    }
}


//------------------------ Instantiate functions -----------------------------------//

template class Function<ApproximateTag,RealScalar(RealScalar)>;
template class Function<ApproximateTag,RealVector(RealScalar)>;
template class Function<ApproximateTag,RealScalar(RealVector)>;
template class Function<ApproximateTag,RealVector(RealVector)>;

template class Function<ValidatedTag,RealScalar(RealScalar)>;
template class Function<ValidatedTag,RealVector(RealScalar)>;
template class Function<ValidatedTag,RealScalar(RealVector)>;
template class Function<ValidatedTag,RealVector(RealVector)>;

template class Function<EffectiveTag,RealScalar(RealScalar)>;
template class Function<EffectiveTag,RealVector(RealScalar)>;
template class Function<EffectiveTag,RealScalar(RealVector)>;
template class Function<EffectiveTag,RealVector(RealVector)>;





//------------------------ Vector function operators -------------------------------//

EffectiveVectorMultivariateFunction operator*(const EffectiveScalarMultivariateFunction& f, const Vector<EffectiveNumber>& e) {
    for(SizeType i=0; i!=e.size(); ++i) { ARIADNE_ASSERT(decide(e[i]==EffectiveNumber(0)) || decide(e[i]==EffectiveNumber(1))); }
    VectorMultivariateFunction<EffectiveTag> r(e.size(),f.domain());
    for(SizeType i=0; i!=e.size(); ++i) {
        if(decide(e[i]==EffectiveNumber(1))) { r.set(i,f); }
    }
    return r;
}

EffectiveVectorMultivariateFunction operator+(const EffectiveVectorMultivariateFunction& f) {
    EffectiveVectorMultivariateFunction r(f.result_size(),f.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,+f[i]);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator-(const EffectiveVectorMultivariateFunction& f) {
    EffectiveVectorMultivariateFunction r(f.result_size(),f.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,-f[i]);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator+(const EffectiveVectorMultivariateFunction& f1, const EffectiveVectorMultivariateFunction& f2) {
    ARIADNE_ASSERT(f1.result_size()==f2.result_size());
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    EffectiveVectorMultivariateFunction r(f1.result_size(),f1.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,f1[i]+f2[i]);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator-(const EffectiveVectorMultivariateFunction& f1, const EffectiveVectorMultivariateFunction& f2) {
    ARIADNE_ASSERT(f1.result_size()==f2.result_size());
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    EffectiveVectorMultivariateFunction r(f1.result_size(),f1.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,f1[i]-f2[i]);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator*(const EffectiveScalarMultivariateFunction& sf, const EffectiveVectorMultivariateFunction& vf) {
    ARIADNE_ASSERT(sf.argument_size()==vf.argument_size());
    EffectiveVectorMultivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,sf*vf[i]);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator*(const EffectiveVectorMultivariateFunction& vf, const EffectiveScalarMultivariateFunction& sf) {
    ARIADNE_ASSERT(vf.argument_size()==sf.argument_size());
    EffectiveVectorMultivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,vf[i]*sf);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator/(const EffectiveVectorMultivariateFunction& vf, const EffectiveScalarMultivariateFunction& sf) {
    ARIADNE_ASSERT(vf.argument_size()==sf.argument_size());
    EffectiveVectorMultivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,vf[i]/sf);
    }
    return r;
}


EffectiveVectorMultivariateFunction operator+(const Vector<EffectiveNumber>& vc, const EffectiveVectorMultivariateFunction& vf) {
    ARIADNE_ASSERT(vc.size()==vf.result_size());
    EffectiveVectorMultivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,vc[i]+vf[i]);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator+(const EffectiveVectorMultivariateFunction& vf, const Vector<EffectiveNumber>& vc) {
    ARIADNE_ASSERT(vf.result_size()==vc.size());
    EffectiveVectorMultivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,vf[i]+vc[i]);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator-(const Vector<EffectiveNumber>& vc, const EffectiveVectorMultivariateFunction& vf) {
    ARIADNE_ASSERT(vc.size()==vf.result_size());
    EffectiveVectorMultivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,vc[i]-vf[i]);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator-(const EffectiveVectorMultivariateFunction& vf, const Vector<EffectiveNumber>& vc) {
    ARIADNE_ASSERT(vf.result_size()==vc.size());
    EffectiveVectorMultivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,vf[i]-vc[i]);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator*(const EffectiveNumber& c, const EffectiveVectorMultivariateFunction& vf) {
    EffectiveVectorMultivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,c*vf[i]);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator*(const EffectiveVectorMultivariateFunction& vf, const EffectiveNumber& c) {
    EffectiveVectorMultivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,vf[i]*c);
    }
    return r;
}

EffectiveVectorMultivariateFunction operator/(const EffectiveVectorMultivariateFunction& vf, const EffectiveNumber& c) {
    EffectiveVectorMultivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,vf[i]/c);
    }
    return r;
}



EffectiveVectorMultivariateFunction join(const EffectiveScalarMultivariateFunction& f1, const EffectiveScalarMultivariateFunction& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    EffectiveVectorMultivariateFunction r(2,f1.domain());
    r.set(0,f1);
    r.set(1,f2);
    return r;
}

EffectiveVectorMultivariateFunction join(const EffectiveVectorMultivariateFunction& f1, const EffectiveScalarMultivariateFunction& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    EffectiveVectorMultivariateFunction r(f1.result_size()+1u,f1.domain());
    for(SizeType i=0u; i!=f1.result_size(); ++i) {
        r.set(i,f1.get(i));
    }
    r.set(f1.result_size(),f2);
    return r;
}

EffectiveVectorMultivariateFunction join(const EffectiveScalarMultivariateFunction& f1, const EffectiveVectorMultivariateFunction& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    EffectiveVectorMultivariateFunction r(f2.result_size()+1u,f1.domain());
    r.set(0u,f1);
    for(SizeType i=0u; i!=f2.result_size(); ++i) {
        r.set(i+1u,f2.get(i));
    }
    return r;
}

EffectiveVectorMultivariateFunction join(const EffectiveVectorMultivariateFunction& f1, const EffectiveVectorMultivariateFunction& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    EffectiveVectorMultivariateFunction r(f1.result_size()+f2.result_size(),f1.domain());
    for(SizeType i=0u; i!=f1.result_size(); ++i) {
        r.set(i,f1.get(i));
    }
    for(SizeType i=0u; i!=f2.result_size(); ++i) {
        r.set(i+f1.result_size(),f2.get(i));
    }
    return r;
}


EffectiveScalarMultivariateFunction embed(SizeType as1, const EffectiveScalarMultivariateFunction& f2, SizeType as3) {
    typedef EuclideanDomain D; typedef RealDomain C;
    return EffectiveScalarMultivariateFunction(EmbeddedFunction<EffectiveTag,D,D,D,C>(as1,f2,as3));
}

EffectiveVectorMultivariateFunction embed(SizeType as1, const EffectiveVectorMultivariateFunction& f2, SizeType as3) {
    typedef EuclideanDomain D; typedef EuclideanDomain C;
    return EffectiveVectorMultivariateFunction(EmbeddedFunction<EffectiveTag,D,D,D,C>(as1,f2,as3));
}

EffectiveScalarUnivariateFunction compose(const EffectiveScalarUnivariateFunction& f, const EffectiveScalarUnivariateFunction& g) {
    return make_composed_function(f,g);
}

EffectiveScalarUnivariateFunction compose(const EffectiveScalarMultivariateFunction& f, const EffectiveVectorUnivariateFunction& g) {
    return make_composed_function(f,g);
}

EffectiveVectorUnivariateFunction compose(const EffectiveVectorUnivariateFunction& f, const EffectiveScalarUnivariateFunction& g) {
    return make_composed_function(f,g);
}

EffectiveVectorUnivariateFunction compose(const EffectiveVectorMultivariateFunction& f, const EffectiveVectorUnivariateFunction& g) {
    return make_composed_function(f,g);
}

EffectiveScalarMultivariateFunction compose(const EffectiveScalarUnivariateFunction& f, const EffectiveScalarMultivariateFunction& g) {
    return make_composed_function(f,g);
}

EffectiveScalarMultivariateFunction compose(const EffectiveScalarMultivariateFunction& f, const EffectiveVectorMultivariateFunction& g) {
    return make_composed_function(f,g);
}

EffectiveVectorMultivariateFunction compose(const EffectiveVectorUnivariateFunction& f, const EffectiveScalarMultivariateFunction& g) {
    return make_composed_function(f,g);
}

EffectiveVectorMultivariateFunction compose(const EffectiveVectorMultivariateFunction& f, const EffectiveVectorMultivariateFunction& g) {
    return make_composed_function(f,g);
}

EffectiveVectorMultivariateFunction derivatives(const EffectiveScalarMultivariateFunction& f) {
    return VectorOfScalarFunction(Vector<EffectiveScalarMultivariateFunction>(f.argument_size(),[&f](SizeType i){return f.derivative(i);}));
}

EffectiveScalarMultivariateFunction lie_derivative(const EffectiveScalarMultivariateFunction& g, const EffectiveVectorMultivariateFunction& f) {
    ARIADNE_ASSERT_MSG(g.argument_size()==f.result_size(),"f="<<f<<", g="<<g);
    ARIADNE_ASSERT_MSG(f.result_size()==f.argument_size(),"f="<<f<<", g="<<g);
    ARIADNE_ASSERT_MSG(f.result_size()>0,"f="<<f<<", g="<<g);

    try {
        EffectiveScalarMultivariateFunction r=g.derivative(0)*f[0];
        for(SizeType i=1; i!=g.argument_size(); ++i) {
            r=r+g.derivative(i)*f[i];
        }
        return r;
    }
    catch(...) {
        ARIADNE_FAIL_MSG("Failed to compute Lie derivative of "<<g<<" under vector field "<<f);
    }
}

EffectiveVectorMultivariateFunction lie_derivative(const EffectiveVectorMultivariateFunction& g, const EffectiveVectorMultivariateFunction& f) {
    ARIADNE_ASSERT_MSG(g.argument_size()==f.result_size(),"f="<<f<<", g="<<g);
    ARIADNE_ASSERT_MSG(f.result_size()==f.argument_size(),"f="<<f<<", g="<<g);
    ARIADNE_ASSERT_MSG(f.result_size()>0,"f="<<f<<", g="<<g);

    try {
        EffectiveVectorMultivariateFunction r(g.result_size(),g.domain());
        for(SizeType i=0; i!=g.result_size(); ++i) { r[i]=lie_derivative(g[i],f); }
        return r;
    }
    catch(...) {
        ARIADNE_FAIL_MSG("Failed to compute Lie derivative of vector function "<<g<<" under vector field "<<f);
    }
}



//------------------------ Effective univariate function operators -------------------------------//

EffectiveVectorUnivariateFunction operator+(const EffectiveVectorUnivariateFunction& f1, const EffectiveVectorUnivariateFunction& f2) {
    ARIADNE_ASSERT(f1.result_size()==f2.result_size());
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    EffectiveVectorUnivariateFunction r(f1.result_size(),f1.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,f1[i]+f2[i]);
    }
    return r;
}

EffectiveVectorUnivariateFunction operator*(const EffectiveVectorUnivariateFunction& vf, const EffectiveScalarUnivariateFunction& sf) {
    ARIADNE_ASSERT(vf.argument_size()==sf.argument_size());
    EffectiveVectorUnivariateFunction r(vf.result_size(),vf.domain());
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r.set(i,vf[i]*sf);
    }
    return r;
}


//------------------------ Validated function operators -------------------------------//

ValidatedVectorMultivariateFunction operator+(ValidatedVectorMultivariateFunction const& f1, ValidatedVectorMultivariateFunction const& f2) {
    auto f1ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    auto f2ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f1ptr && f2ptr) {
        return ValidatedVectorMultivariateFunctionPatch(*f1ptr) + ValidatedVectorMultivariateFunctionPatch(*f2ptr);
    } else if(f1ptr) {
        return ValidatedVectorMultivariateFunctionPatch(*f1ptr) + f2.reference();
    } else if(f2ptr) {
        return f1.reference() + ValidatedVectorMultivariateFunctionPatch(*f2ptr);
    } else {
        VectorOfScalarMultivariateFunction<ValidatedTag> r(f1.result_size(),ValidatedScalarMultivariateFunction(f1.argument_size()));
        for(SizeType i=0; i!=r.result_size(); ++i) {
            r[i]=f1[i]+f2[i];
        }
        return r;
    }
}

ValidatedVectorMultivariateFunction operator-(ValidatedVectorMultivariateFunction const& f1, ValidatedVectorMultivariateFunction const& f2) {
    auto f1ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    auto f2ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f1ptr && f2ptr) {
        return ValidatedVectorMultivariateFunctionPatch(*f1ptr) - ValidatedVectorMultivariateFunctionPatch(*f2ptr);
    } else if(f1ptr) {
        return ValidatedVectorMultivariateFunctionPatch(*f1ptr) - f2.reference();
    } else if(f2ptr) {
        return f1.reference() - ValidatedVectorMultivariateFunctionPatch(*f2ptr);
    } else {
        VectorOfScalarMultivariateFunction<ValidatedTag> r(f1.result_size(),ValidatedScalarMultivariateFunction(f1.argument_size()));
        for(SizeType i=0; i!=r.result_size(); ++i) {
            r[i]=f1[i]-f2[i];
        }
        return r;
    }
}

ValidatedVectorMultivariateFunction operator*(ValidatedScalarMultivariateFunction const& f1, ValidatedVectorMultivariateFunction const& f2) {
    auto f1ptr=std::dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    auto f2ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f1ptr && f2ptr) {
        return ValidatedScalarMultivariateFunction(*f1ptr) * ValidatedVectorMultivariateFunction(*f2ptr);
    } else if(f1ptr) {
        return ValidatedScalarMultivariateFunction(*f1ptr) * f2.reference();
    } else if(f2ptr) {
        return f1.reference() * ValidatedVectorMultivariateFunction(*f2ptr);
    } else {
        VectorOfScalarMultivariateFunction<ValidatedTag> r(f2.result_size(),ValidatedScalarMultivariateFunction(f2.argument_size()));
        for(SizeType i=0; i!=r.result_size(); ++i) {
            r[i]=f1*f2[i];
        }
        return r;
    }
}

ValidatedVectorMultivariateFunction operator*(ValidatedVectorMultivariateFunction const& f1, ValidatedScalarMultivariateFunction const& f2) {
    auto f1ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    auto f2ptr=std::dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f1ptr && f2ptr) {
        return ValidatedVectorMultivariateFunctionPatch(*f1ptr) * ValidatedScalarMultivariateFunctionPatch(*f2ptr);
    } else if(f1ptr) {
        return ValidatedVectorMultivariateFunctionPatch(*f1ptr) * f2.reference();
    } else if(f2ptr) {
        return f1.reference() * ValidatedScalarMultivariateFunctionPatch(*f2ptr);
    } else {
        VectorOfScalarMultivariateFunction<ValidatedTag> r(f1.result_size(),ValidatedScalarMultivariateFunction(f1.argument_size()));
        for(SizeType i=0; i!=r.result_size(); ++i) {
            r[i]=f1[i]*f2;
        }
        return r;
    }
}

ValidatedVectorMultivariateFunction operator/(ValidatedVectorMultivariateFunction const& f1, ValidatedScalarMultivariateFunction const& f2) {
    auto f1ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    auto f2ptr=std::dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f1ptr && f2ptr) {
        return ValidatedVectorMultivariateFunctionPatch(*f1ptr) / ValidatedScalarMultivariateFunctionPatch(*f2ptr);
    } else if(f1ptr) {
        return ValidatedVectorMultivariateFunctionPatch(*f1ptr) / f2.reference();
    } else if(f2ptr) {
        return f1.reference() / ValidatedScalarMultivariateFunctionPatch(*f2ptr);
    } else {
        VectorOfScalarMultivariateFunction<ValidatedTag> r(f1.result_size(),ValidatedScalarMultivariateFunction(f1.argument_size()));
        for(SizeType i=0; i!=r.result_size(); ++i) {
            r[i]=f1[i]/f2;
        }
        return r;
    }
}

template<class P, class R, class T, class... AS> inline
Function<P,R(AS...)> _validated_compose(const Function<P,R(T)>& f, const Function<P,T(AS...)>& g) {
    ARIADNE_ASSERT(f.argument_size()==g.result_size());
    typedef DoublePrecision PR;

    auto gp=std::dynamic_pointer_cast<typename FunctionModel<P,T(AS...),PR>::Interface const>(g.managed_pointer());
    if(gp) {
        return compose(f,FunctionModel<P,T(AS...),PR>(gp->_clone()));
    } else {
        return make_composed_function(f,g);
    }
}

ValidatedScalarUnivariateFunction compose(const ValidatedScalarUnivariateFunction& f, const ValidatedScalarUnivariateFunction& g) {
    return _validated_compose(f,g);
}

ValidatedScalarUnivariateFunction compose(const ValidatedScalarMultivariateFunction& f, const ValidatedVectorUnivariateFunction& g) {
    return _validated_compose(f,g);
}

ValidatedVectorUnivariateFunction compose(const ValidatedVectorUnivariateFunction& f, const ValidatedScalarUnivariateFunction& g) {
    return _validated_compose(f,g);
}

ValidatedVectorUnivariateFunction compose(const ValidatedVectorMultivariateFunction& f, const ValidatedVectorUnivariateFunction& g) {
    return _validated_compose(f,g);
}

ValidatedScalarMultivariateFunction compose(const ValidatedScalarUnivariateFunction& f, const ValidatedScalarMultivariateFunction& g) {
    return _validated_compose(f,g);
}

ValidatedScalarMultivariateFunction compose(const ValidatedScalarMultivariateFunction& f, const ValidatedVectorMultivariateFunction& g) {
    return _validated_compose(f,g);
}

ValidatedVectorMultivariateFunction compose(const ValidatedVectorUnivariateFunction& f, const ValidatedScalarMultivariateFunction& g) {
    return _validated_compose(f,g);
}

ValidatedVectorMultivariateFunction compose(const ValidatedVectorMultivariateFunction& f, const ValidatedVectorMultivariateFunction& g) {
    return _validated_compose(f,g);
}

ValidatedVectorMultivariateFunction join(ValidatedVectorMultivariateFunction const& f1, const ValidatedVectorMultivariateFunction& f2) {
    auto f1ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    auto f2ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f1ptr && f2ptr) {
        ValidatedVectorMultivariateFunctionPatch f1ptch(f1ptr); ValidatedVectorMultivariateFunctionPatch f2ptch(f2ptr); return join(f1ptch,f2ptch);
    } else if(f1ptr) {
        ValidatedVectorMultivariateFunctionPatch f1ptch(f1ptr); ValidatedVectorMultivariateFunctionPatch f2ptch=factory(f1ptch).create(f2); return join(f1ptch,f2ptch);
    } else if(f2ptr) {
        ValidatedVectorMultivariateFunctionPatch f2ptch(f2ptr); ValidatedVectorMultivariateFunctionPatch f1ptch=factory(f2ptch).create(f1); return join(f1ptch,f2ptch);
    } else {
        typedef ValidatedVectorMultivariateFunction::DomainType DomainType;
        typedef ValidatedVectorMultivariateFunction::CodomainType CodomainType;
        return ValidatedVectorMultivariateFunction(JoinedFunction<ValidatedTag,DomainType,CodomainType,CodomainType>(f1,f2));
    }
}

ValidatedVectorMultivariateFunction join(ValidatedVectorMultivariateFunction const& f1, const ValidatedScalarMultivariateFunction& f2) {
    auto f1ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    auto f2ptr=std::dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f1ptr && f2ptr) {
        ValidatedVectorMultivariateFunctionPatch f1ptch(f1ptr); ValidatedScalarMultivariateFunctionPatch f2ptch(f2ptr); return join(f1ptch,f2ptch);
    } else if(f1ptr) {
        ValidatedVectorMultivariateFunctionPatch f1ptch(f1ptr); ValidatedScalarMultivariateFunctionPatch f2ptch=factory(f1ptch).create(f2); return join(f1ptch,f2ptch);
    } else if(f2ptr) {
        ValidatedScalarMultivariateFunctionPatch f2ptch(f2ptr); ValidatedVectorMultivariateFunctionPatch f1ptch=factory(f2ptch).create(f1); return join(f1ptch,f2ptch);
    } else {
        VectorOfScalarMultivariateFunction<ValidatedTag> r(f1.result_size()+1u,f1.domain());
        for(SizeType i=0; i!=f1.result_size(); ++i) { r[i]=f1[i]; }
        r[f1.result_size()]=f2;
        return r;
    }
}

ValidatedVectorMultivariateFunction join(ValidatedScalarMultivariateFunction const& f1, const ValidatedVectorMultivariateFunction& f2) {
    auto f1ptr=std::dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    auto f2ptr=std::dynamic_pointer_cast<ValidatedVectorMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f1ptr && f2ptr) {
        ValidatedScalarMultivariateFunctionPatch f1ptch(f1ptr); ValidatedVectorMultivariateFunctionPatch f2ptch(f2ptr); return join(f1ptch,f2ptch);
    } else if(f1ptr) {
        ValidatedScalarMultivariateFunctionPatch f1ptch(f1ptr); ValidatedVectorMultivariateFunctionPatch f2ptch=factory(f1ptch).create(f2); return join(f1ptch,f2ptch);
    } else if(f2ptr) {
        ValidatedVectorMultivariateFunctionPatch f2ptch(f2ptr); ValidatedScalarMultivariateFunctionPatch f1ptch=factory(f2ptch).create(f1); return join(f1ptch,f2ptch);
    } else {
        VectorOfScalarMultivariateFunction<ValidatedTag> r(f1.result_size()+1u,f1.domain());
        r[0u]=f1;
        for(SizeType i=0; i!=f1.result_size(); ++i) { r[i+1]=f2[i]; }
        return r;
    }
}

ValidatedVectorMultivariateFunction join(ValidatedScalarMultivariateFunction const& f1, const ValidatedScalarMultivariateFunction& f2) {
    auto f1ptr=std::dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    auto f2ptr=std::dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f1ptr && f2ptr) {
        ValidatedScalarMultivariateFunctionPatch f1ptch(f1ptr); ValidatedScalarMultivariateFunctionPatch f2ptch(f2ptr); return join(f1ptch,f2ptch);
    } else if(f1ptr) {
        ValidatedScalarMultivariateFunctionPatch f1ptch(f1ptr); ValidatedScalarMultivariateFunctionPatch f2ptch=factory(f1ptch).create(f2); return join(f1ptch,f2ptch);
    } else if(f2ptr) {
        ValidatedScalarMultivariateFunctionPatch f2ptch(f2ptr); ValidatedScalarMultivariateFunctionPatch f1ptch=factory(f2ptch).create(f1); return join(f1ptch,f2ptch);
    } else {
        VectorOfScalarMultivariateFunction<ValidatedTag> r(2u,f1.domain());
        r[0u]=f1;
        r[1u]=f2;
        return r;
    }
}


FloatDPUpperInterval evaluate_range(ValidatedScalarMultivariateFunction const& f, const Vector<FloatDPUpperInterval>& x) {
    return static_cast<FloatDPUpperInterval>(f(reinterpret_cast<Vector<FloatDPBounds>const&>(x)));
}

Vector<FloatDPUpperInterval> evaluate_range(ValidatedVectorMultivariateFunction const& f, const Vector<FloatDPUpperInterval>& x) {
    return static_cast<Vector<FloatDPUpperInterval>>(f(reinterpret_cast<Vector<FloatDPBounds>const&>(x)));
}

Vector<Differential<FloatDPUpperInterval>> derivative_range(ValidatedVectorMultivariateFunction const& f,
                                                            const Vector<Differential<FloatDPUpperInterval>>& x) {
    return static_cast<Vector<Differential<FloatDPUpperInterval>>>(
        f(reinterpret_cast<Vector<Differential<FloatDPBounds>>const&>(x)));
}

Covector<FloatDPUpperInterval> gradient_range(ValidatedScalarMultivariateFunction const& f,
                                              const Vector<FloatDPUpperInterval>& x) {
    return static_cast<Covector<FloatDPUpperInterval>>(gradient(f,reinterpret_cast<Vector<FloatDPBounds>const&>(x)));
}

Matrix<FloatDPUpperInterval> jacobian_range(ValidatedVectorMultivariateFunction const& f, const Vector<FloatDPUpperInterval>& x) {
    return static_cast<Matrix<FloatDPUpperInterval>>(jacobian(f,reinterpret_cast<Vector<FloatDPBounds>const&>(x)));
}

//------------------------ Validated univariate function operators -------------------------------//

// FIXME: Also support function models
ValidatedVectorUnivariateFunction operator+(ValidatedVectorUnivariateFunction const& f1, ValidatedVectorUnivariateFunction const& f2) {
    VectorOfScalarUnivariateFunction<ValidatedTag> r(f1.result_size(),ValidatedScalarUnivariateFunction(f1.argument_size()));
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r[i]=f1[i]+f2[i];
    }
    return r;
}

// FIXME: Also support function models
ValidatedVectorUnivariateFunction operator*(ValidatedVectorUnivariateFunction const& f1, ValidatedScalarUnivariateFunction const& f2) {
    VectorOfScalarUnivariateFunction<ValidatedTag> r(f1.result_size(),ValidatedScalarUnivariateFunction(f1.argument_size()));
    for(SizeType i=0; i!=r.result_size(); ++i) {
        r[i]=f1[i]*f2;
    }
    return r;
}


//------------------------ Function operators -------------------------------//

template<class P> VectorMultivariateFunction<P> join(const ScalarMultivariateFunction<P>& f1, const ScalarMultivariateFunction<P>& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    VectorMultivariateFunction<P> r(2,f1.domain());
    r.set(0,f1);
    r.set(1,f2);
    return r;
}

template<class P> VectorMultivariateFunction<P> join(const VectorMultivariateFunction<P>& f1, const ScalarMultivariateFunction<P>& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    VectorMultivariateFunction<P> r(f1.result_size()+1u,f1.domain());
    for(SizeType i=0u; i!=f1.result_size(); ++i) {
        r.set(i,f1.get(i));
    }
    r.set(f1.result_size(),f2);
    return r;
}

template<class P> VectorMultivariateFunction<P> join(const ScalarMultivariateFunction<P>& f1, const VectorMultivariateFunction<P>& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    VectorMultivariateFunction<P> r(f2.result_size()+1u,f1.domain());
    r.set(0u,f1);
    for(SizeType i=0u; i!=f2.result_size(); ++i) {
        r.set(i+1u,f2.get(i));
    }
    return r;
}

template<class P> VectorMultivariateFunction<P> join(const VectorMultivariateFunction<P>& f1, const VectorMultivariateFunction<P>& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    VectorMultivariateFunction<P> r(f1.result_size()+f2.result_size(),f1.domain());
    for(SizeType i=0u; i!=f1.result_size(); ++i) {
        r.set(i,f1.get(i));
    }
    for(SizeType i=0u; i!=f2.result_size(); ++i) {
        r.set(i+f1.result_size(),f2.get(i));
    }
    return r;
}

ApproximateVectorMultivariateFunction join(const ApproximateScalarMultivariateFunction& f1, const ApproximateScalarMultivariateFunction& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    ApproximateVectorMultivariateFunction r(2,f1.domain());
    r.set(0,f1);
    r.set(1,f2);
    return r;
}

ApproximateVectorMultivariateFunction join(const ApproximateVectorMultivariateFunction& f1, const ApproximateScalarMultivariateFunction& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    ApproximateVectorMultivariateFunction r(f1.result_size()+1u,f1.domain());
    for(SizeType i=0u; i!=f1.result_size(); ++i) {
        r.set(i,f1.get(i));
    }
    r.set(f1.result_size(),f2);
    return r;
}

ApproximateVectorMultivariateFunction join(const ApproximateScalarMultivariateFunction& f1, const ApproximateVectorMultivariateFunction& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    ApproximateVectorMultivariateFunction r(f2.result_size()+1u,f1.domain());
    r.set(0u,f1);
    for(SizeType i=0u; i!=f2.result_size(); ++i) {
        r.set(i+1u,f2.get(i));
    }
    return r;
}

ApproximateVectorMultivariateFunction join(const ApproximateVectorMultivariateFunction& f1, const ApproximateVectorMultivariateFunction& f2) {
    ARIADNE_ASSERT(f1.argument_size()==f2.argument_size());
    ApproximateVectorMultivariateFunction r(f1.result_size()+f2.result_size(),f1.domain());
    for(SizeType i=0u; i!=f1.result_size(); ++i) {
        r.set(i,f1.get(i));
    }
    for(SizeType i=0u; i!=f2.result_size(); ++i) {
        r.set(i+f1.result_size(),f2.get(i));
    }
    return r;
}


ApproximateScalarUnivariateFunction compose(const ApproximateScalarUnivariateFunction& f, const ApproximateScalarUnivariateFunction& g) {
    return make_composed_function(f,g); }
ApproximateScalarUnivariateFunction compose(const ApproximateScalarMultivariateFunction& f, const ApproximateVectorUnivariateFunction& g) {
    return make_composed_function(f,g); }
ApproximateVectorUnivariateFunction compose(const ApproximateVectorUnivariateFunction& f, const ApproximateScalarUnivariateFunction& g) {
    return make_composed_function(f,g); }
ApproximateVectorUnivariateFunction compose(const ApproximateVectorMultivariateFunction& f, const ApproximateVectorUnivariateFunction& g) {
    return make_composed_function(f,g); }
ApproximateScalarMultivariateFunction compose(const ApproximateScalarUnivariateFunction& f, const ApproximateScalarMultivariateFunction& g) {
    return make_composed_function(f,g); }
ApproximateScalarMultivariateFunction compose(const ApproximateScalarMultivariateFunction& f, const ApproximateVectorMultivariateFunction& g) {
    return make_composed_function(f,g); }
ApproximateVectorMultivariateFunction compose(const ApproximateVectorUnivariateFunction& f, const ApproximateScalarMultivariateFunction& g) {
    return make_composed_function(f,g); }
ApproximateVectorMultivariateFunction compose(const ApproximateVectorMultivariateFunction& f, const ApproximateVectorMultivariateFunction& g) {
    return make_composed_function(f,g); }


using std::dynamic_pointer_cast;



template<> EffectiveScalarMultivariateFunction AlgebraOperations<EffectiveScalarMultivariateFunction,EffectiveNumber>::apply(UnaryElementaryOperator op, EffectiveScalarMultivariateFunction const& f) {
    auto e=dynamic_pointer_cast<const EffectiveScalarFormulaFunction>(f.managed_pointer());
    if(e) { return make_formula_function(e->argument_size(),op(e->formula())); }
    else { return EffectiveScalarMultivariateFunction(UnaryMultivariateFunction<EffectiveTag>(op,f)); }
}
template<> EffectiveScalarMultivariateFunction AlgebraOperations<EffectiveScalarMultivariateFunction,EffectiveNumber>::apply(BinaryElementaryOperator op, EffectiveScalarMultivariateFunction const& f1, EffectiveScalarMultivariateFunction const& f2) {
    auto e1=dynamic_pointer_cast<const EffectiveScalarFormulaFunction>(f1.managed_pointer());
    auto e2=dynamic_pointer_cast<const EffectiveScalarFormulaFunction>(f2.managed_pointer());
    if(e1 && e2 && e1->argument_size()==e2->argument_size()) {
        return make_formula_function(e1->argument_size(),op(e1->formula(),e2->formula()));
    }
    else { return EffectiveScalarMultivariateFunction(BinaryMultivariateFunction<EffectiveTag>(op,f1,f2)); }
}
template<> EffectiveScalarMultivariateFunction AlgebraOperations<EffectiveScalarMultivariateFunction,EffectiveNumber>::apply(BinaryElementaryOperator op, EffectiveScalarMultivariateFunction const& f1, EffectiveNumber const& c2) {
    auto e1=dynamic_pointer_cast<const EffectiveScalarFormulaFunction>(f1.managed_pointer());
    if(e1) { return make_formula_function(e1->argument_size(),op(e1->formula(),c2)); }
    else { return op(f1,EffectiveScalarMultivariateFunction::constant(f1.argument_size(),c2)); }
}
template<> EffectiveScalarMultivariateFunction AlgebraOperations<EffectiveScalarMultivariateFunction,EffectiveNumber>::apply(BinaryElementaryOperator op, EffectiveNumber const& c1, EffectiveScalarMultivariateFunction const& f2) {
    auto e2=dynamic_pointer_cast<const EffectiveScalarFormulaFunction>(f2.managed_pointer());
    if(e2) { return make_formula_function(e2->argument_size(),op(c1,e2->formula())); }
    else { return op(EffectiveScalarMultivariateFunction::constant(f2.argument_size(),c1),f2); }
}
template<> EffectiveScalarMultivariateFunction AlgebraOperations<EffectiveScalarMultivariateFunction,EffectiveNumber>::apply(GradedElementaryOperator op, EffectiveScalarMultivariateFunction const& f, Int n) {
    return EffectiveScalarMultivariateFunction(GradedMultivariateFunction<EffectiveTag>(op,f,n));
}



template<> ValidatedScalarMultivariateFunction AlgebraOperations<ValidatedScalarMultivariateFunction,ValidatedNumber>::apply(UnaryElementaryOperator op, ValidatedScalarMultivariateFunction const& f) {
    auto fptr=dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f.managed_pointer());
    if(fptr) { ValidatedScalarMultivariateFunctionPatch fptch=fptr; return op(fptch); }
    else { return ValidatedScalarMultivariateFunction(UnaryMultivariateFunction<ValidatedTag>(op,f)); }
}

template<> ValidatedScalarMultivariateFunction AlgebraOperations<ValidatedScalarMultivariateFunction,ValidatedNumber>::apply(BinaryElementaryOperator op, ValidatedScalarMultivariateFunction const& f1, ValidatedScalarMultivariateFunction const& f2) {
    auto f1ptr=dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    auto f2ptr=dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f1ptr && f2ptr) { ValidatedScalarMultivariateFunctionPatch f1ptch(f1ptr); ValidatedScalarMultivariateFunctionPatch f2ptch(f2ptr); return op(f1ptch,f2ptch); }
    else if(f1ptr) { ValidatedScalarMultivariateFunctionPatch f1ptch(f1ptr); return op(f1ptch,factory(f1ptch).create(f2)); }
    else if(f2ptr) { ValidatedScalarMultivariateFunctionPatch f2ptch(f2ptr); return op(factory(f2ptch).create(f1),f2ptch); }
    else { return ValidatedScalarMultivariateFunction(BinaryMultivariateFunction<ValidatedTag>(op,f1,f2)); }
}

template<> ValidatedScalarMultivariateFunction AlgebraOperations<ValidatedScalarMultivariateFunction,ValidatedNumber>::apply(BinaryElementaryOperator op, ValidatedScalarMultivariateFunction const& f1, ValidatedNumber const& c2) {
    auto f1ptr=dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f1.managed_pointer());
    if(f1ptr) { ValidatedScalarMultivariateFunctionPatch f1ptch=f1ptr; return op(f1,c2); }
    else { return ValidatedScalarMultivariateFunction(BinaryMultivariateFunction<ValidatedTag>(op,f1,f1.create_constant(c2))); }
}

template<> ValidatedScalarMultivariateFunction AlgebraOperations<ValidatedScalarMultivariateFunction,ValidatedNumber>::apply(BinaryElementaryOperator op, ValidatedNumber const& c1, ValidatedScalarMultivariateFunction const& f2) {
    auto f2ptr=dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f2.managed_pointer());
    if(f2ptr) { ValidatedScalarMultivariateFunctionPatch f2ptch=f2ptr; return op(c1,f2ptch); }
    else { return ValidatedScalarMultivariateFunction(BinaryMultivariateFunction<ValidatedTag>(op,f2.create_constant(c1),f2)); }
}

template<> ValidatedScalarMultivariateFunction AlgebraOperations<ValidatedScalarMultivariateFunction,ValidatedNumber>::apply(GradedElementaryOperator op, ValidatedScalarMultivariateFunction const& f, Int n) {
    auto fptr=dynamic_pointer_cast<ValidatedScalarMultivariateFunctionPatch::Interface const>(f.managed_pointer());
    if(fptr) { ValidatedScalarMultivariateFunctionPatch fptch=fptr; return op(fptch,n); }
    else { return ValidatedScalarMultivariateFunction(GradedMultivariateFunction<ValidatedTag>(op,f,n)); }
}

template<> ApproximateScalarMultivariateFunction AlgebraOperations<ApproximateScalarMultivariateFunction,ApproximateNumber>::apply(UnaryElementaryOperator op, ApproximateScalarMultivariateFunction const& f) {
    return ApproximateScalarMultivariateFunction(UnaryMultivariateFunction<ApproximateTag>(op,f));
}
template<> ApproximateScalarMultivariateFunction AlgebraOperations<ApproximateScalarMultivariateFunction,ApproximateNumber>::apply(BinaryElementaryOperator op, ApproximateScalarMultivariateFunction const& f1, ApproximateScalarMultivariateFunction const& f2) {
    return ApproximateScalarMultivariateFunction(BinaryMultivariateFunction<ApproximateTag>(op,f1,f2));
}
template<> ApproximateScalarMultivariateFunction AlgebraOperations<ApproximateScalarMultivariateFunction,ApproximateNumber>::apply(BinaryElementaryOperator op, ApproximateScalarMultivariateFunction const& f1, Number<ApproximateTag> const& c2) {
    return ApproximateScalarMultivariateFunction(BinaryMultivariateFunction<ApproximateTag>(op,f1,f1.create_constant(c2)));
}
template<> ApproximateScalarMultivariateFunction AlgebraOperations<ApproximateScalarMultivariateFunction,ApproximateNumber>::apply(BinaryElementaryOperator op, Number<ApproximateTag> const& c1, ApproximateScalarMultivariateFunction const& f2) {
    return ApproximateScalarMultivariateFunction(BinaryMultivariateFunction<ApproximateTag>(op,f2.create_constant(c1),f2));
}
template<> ApproximateScalarMultivariateFunction AlgebraOperations<ApproximateScalarMultivariateFunction,ApproximateNumber>::apply(GradedElementaryOperator op, ApproximateScalarMultivariateFunction const& f, Int n) {
    return ApproximateScalarMultivariateFunction(GradedMultivariateFunction<ApproximateTag>(op,f,n));
}


template<class P, class... ARGS> ScalarFunction<P,ARGS...> AlgebraOperations<ScalarFunction<P,ARGS...>,Number<P>>::apply(UnaryElementaryOperator op, ScalarFunction<P,ARGS...> const& f) {
    return ScalarFunction<P,ARGS...>(UnaryFunction<P,ARGS...>(op,f)); }
template<class P, class... ARGS> ScalarFunction<P,ARGS...> AlgebraOperations<ScalarFunction<P,ARGS...>,Number<P>>::apply(BinaryElementaryOperator op, ScalarFunction<P,ARGS...> const& f1, ScalarFunction<P,ARGS...> const& f2) {
    return ScalarFunction<P,ARGS...>(BinaryFunction<P,ARGS...>(op,f1,f2)); }
template<class P, class... ARGS> ScalarFunction<P,ARGS...> AlgebraOperations<ScalarFunction<P,ARGS...>,Number<P>>::apply(BinaryElementaryOperator op, ScalarFunction<P,ARGS...> const& f1, Number<P> const& c2) {
    return ScalarFunction<P,ARGS...>(BinaryFunction<P,ARGS...>(op,f1,f1.create_constant(c2))); }
template<class P, class... ARGS> ScalarFunction<P,ARGS...> AlgebraOperations<ScalarFunction<P,ARGS...>,Number<P>>::apply(BinaryElementaryOperator op, Number<P> const& c1, ScalarFunction<P,ARGS...> const& f2) {
    return ScalarFunction<P,ARGS...>(BinaryFunction<P,ARGS...>(op,f2.create_constant(c1),f2)); }
template<class P, class... ARGS> ScalarFunction<P,ARGS...> AlgebraOperations<ScalarFunction<P,ARGS...>,Number<P>>::apply(GradedElementaryOperator op, ScalarFunction<P,ARGS...> const& f, Int n) {
    return ScalarFunction<P,ARGS...>(GradedFunction<P,ARGS...>(op,f,n)); }


template struct AlgebraOperations<ScalarMultivariateFunction<ApproximateTag>,Number<ApproximateTag>>;
template struct AlgebraOperations<ScalarMultivariateFunction<ValidatedTag>,Number<ValidatedTag>>;
template struct AlgebraOperations<ScalarMultivariateFunction<EffectiveTag>,Number<EffectiveTag>>;

template struct AlgebraOperations<ScalarUnivariateFunction<ApproximateTag>,Number<ApproximateTag>>;
template struct AlgebraOperations<ScalarUnivariateFunction<ValidatedTag>,Number<ValidatedTag>>;
template struct AlgebraOperations<ScalarUnivariateFunction<EffectiveTag>,Number<EffectiveTag>>;


} // namespace Ariadne
