/***************************************************************************
 *            inclusion_integrator.cpp
 *
 *  Copyright  2008-18  Luca Geretti, Pieter Collins, Sanja Zivanovic
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

#include "function/taylor_function.hpp"
#include "solvers/integrator.hpp"
#include "solvers/bounder.hpp"
#include "algebra/expansion.inl.hpp"
#include "inclusion_integrator.hpp"

namespace Ariadne {

template<class I> decltype(auto) inline norm(Box<I> const& bx) { return norm(cast_vector(bx)); }

namespace {
static const typename ErrorType::PrecisionType pr=typename ErrorType::PrecisionType();

template<class M> inline VectorScaledFunctionPatch<M> compose(const ValidatedVectorMultivariateFunctionPatch& g, const VectorScaledFunctionPatch<M>& f) {
    return compose(cast_unrestricted(g),f);
}

} // namespace

template<> ErrorType wstar_multiplier<ZeroApproximation>() { return ErrorType(0u,pr); }
template<> ErrorType wstar_multiplier<ConstantApproximation>() { return ErrorType(1u,pr); }
template<> ErrorType wstar_multiplier<AffineApproximation>() { return ErrorType(5u,pr)/3u; }
template<> ErrorType wstar_multiplier<SinusoidalApproximation>() { return ErrorType(1.3645_dec,pr); }
template<> ErrorType wstar_multiplier<PiecewiseApproximation>() { return ErrorType(1.25_x,pr); }

ErrorType vstar(BoxDomainType const& inputs) {
    ErrorType result(0u,pr);
    for (auto i : range(0,inputs.size())) {
        result = max(result,cast_positive(cast_exact(abs(inputs[i]).upper_bound())));
    }
    return result;
}

ErrorType zeroparam_worstcase_error(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h);
ErrorType oneparam_worstcase_error(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h);
template<class R> ErrorType twoparam_worstcase_error(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h);
ErrorType zeroparam_component_error(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h, SizeType j);
ErrorType oneparam_component_error(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h, SizeType j);
template<class R> ErrorType twoparam_component_error(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h, SizeType j);

template<class A, class R> ErrorType worstcase_error(ErrorConstants const& n, BoxDomainType const& inputs, PositiveFloatDP const& h) {
    if constexpr (Same<A,ZeroApproximation>) { return zeroparam_worstcase_error(n, vstar(inputs), wstar<A>(inputs), h); }
    else if constexpr (Same<A,ConstantApproximation>) { return oneparam_worstcase_error(n, vstar(inputs), wstar<A>(inputs), h); }
    else { return twoparam_worstcase_error<R>(n, vstar(inputs), wstar<A>(inputs), h); }
}

template<class A, class R> ErrorType component_error(ErrorConstants const& n, BoxDomainType const& inputs, PositiveFloatDP const& h, SizeType j) {
    if constexpr (Same<A,ZeroApproximation>) { return zeroparam_component_error(n, vstar(inputs), wstar<A>(inputs), h, j); }
    else if constexpr (Same<A,ConstantApproximation>) { return oneparam_component_error(n, vstar(inputs), wstar<A>(inputs), h, j); }
    else { return twoparam_component_error<R>(n, vstar(inputs), wstar<A>(inputs), h, j); }
}

ErrorConstants::ErrorConstants(ErrorType const& K_, Vector<ErrorType> const& Kj_, ErrorType const& pK_, ErrorType const& pKv_, ErrorType const& pKw_, Vector<ErrorType> const& pKj_, Vector<ErrorType> const& pKjv_, Vector<ErrorType> const& pKjw_,
                               ErrorType const& L_, Vector<ErrorType> const& Lj_, ErrorType const& pL_, ErrorType const& pLv_, ErrorType const& pLw_, Vector<ErrorType> const& pLj_, Vector<ErrorType> const& pLjv_, Vector<ErrorType> const& pLjw_,
                               ErrorType const& H_, Vector<ErrorType> const& Hj_, ErrorType const& pH_, ErrorType const& pHv_, ErrorType const& pHw_, Vector<ErrorType> const& pHj_, Vector<ErrorType> const& pHjv_, Vector<ErrorType> const& pHjw_,
                               FloatDPUpperBound const& Lambda_, ErrorType const& expLambda_, ErrorType const& expL_)
 : K(K_), Kj(Kj_), pK(pK_), pKv(pKv_), pKw(pKw_), pKj(pKj_), pKjv(pKjv_), pKjw(pKjw_),
   L(L_), Lj(Lj_), pL(pL_), pLv(pLv_), pLw(pLw_), pLj(pLj_), pLjv(pLjv_), pLjw(pLjw_),
   H(H_), Hj(Hj_), pH(pH_), pHv(pHv_), pHw(pHw_), pHj(pHj_), pHjv(pHjv_), pHjw(pHjw_),
   Lambda(Lambda_), expLambda(expLambda_), expL(expL_) {
    _dimension = Kj.size();
    assert(Kj.size() == _dimension and pKj.size() == _dimension and pKjv.size() == _dimension and pKjw.size() == _dimension and
           Lj.size() == _dimension and pLj.size() == _dimension and pLjv.size() == _dimension and pLjw.size() == _dimension and
           Hj.size() == _dimension and pHj.size() == _dimension and pHjv.size() == _dimension and pHjw.size() == _dimension);
}

auto ErrorConstants::values() const
    -> Tuple<ErrorType,Vector<ErrorType>,ErrorType,ErrorType,ErrorType,Vector<ErrorType>,Vector<ErrorType>,Vector<ErrorType>,
      ErrorType,Vector<ErrorType>,ErrorType,ErrorType,ErrorType,Vector<ErrorType>,Vector<ErrorType>,Vector<ErrorType>,
      ErrorType,Vector<ErrorType>,ErrorType,ErrorType,ErrorType,Vector<ErrorType>,Vector<ErrorType>,Vector<ErrorType>,
      FloatDPUpperBound,ErrorType,ErrorType>
{
    return std::tie(this->K,this->Kj,this->pK,this->pKv,this->pKw,this->pKj,this->pKjv,this->pKjw,this->L,this->Lj,this->pL,this->pLv,this->pLw,this->pLj,this->pLjv,this->pLjw,this->H,this->Hj,this->pH,this->pHv,this->pHw,this->pHj,this->pHjv,this->pHjw,this->Lambda,this->expLambda,this->expL);
}

template<class A> ErrorConstants
compute_constants(EffectiveVectorMultivariateFunction const& noise_independent_component, Vector<EffectiveVectorMultivariateFunction> const& input_derivatives, BoxDomainType const& inputs, PositiveFloatDP const& h, UpperBoxType const& B) {

    auto n = noise_independent_component.result_size();
    auto m = input_derivatives.size();
    ErrorType ze(pr);
    ErrorType K=ze, pK=ze, pKv=ze, pKw=ze, L=ze, pL=ze, pLv=ze, pLw=ze, H=ze, pH=ze, pHv=ze, pHw=ze;
    Vector<ErrorType> Kj(n,pr), pKj(n,pr), pKjv(n,pr), pKjw(n,pr), Lj(n,pr), pLj(n,pr), pLjv(n,pr), pLjw(n,pr), Hj(n,pr), pHj(n,pr), pHjv(n,pr), pHjw(n,pr);
    FloatDPUpperBound Lambda(-infty,double_precision);

    auto Df=noise_independent_component.differential(cast_singleton(B),2);
    for (auto j : range(n)) {
        auto Df_j=Df[j].expansion();
        ErrorType K_j=ze, L_j=ze, H_j=ze; FloatDPUpperBound Lambda_j=ze;
        for (auto ac : Df_j) {
            UniformReference<MultiIndex> a=ac.index();
            UniformReference<FloatDPBounds> c=ac.coefficient();
            if (a.degree()==0) {
                K_j = mag(c);
            } else if (a.degree()==1) {
                L_j += mag(c);
                if (a[j]==1) { Lambda_j += c.upper(); }
                else { Lambda_j += mag(c); }
            } else {
                assert(a.degree()==2);
                H_j += mag(c);
            }
        }
        K=max(K,K_j); L=max(L,L_j); H=max(H,H_j); Lambda=max(Lambda,Lambda_j);
        Kj[j] = K_j;
        Lj[j] = L_j;
        Hj[j] = H_j;
    }

    Matrix<ErrorType> pK_ij(m,n,pr), pL_ij(m,n,pr), pH_ij(m,n,pr);

    for (auto i : range(m)) {
        auto Diff_g_i=input_derivatives[i].differential(cast_singleton(B),2);
        ErrorType pK_i=ze, pL_i=ze, pH_i=ze;
        for (auto j : range(n)) {
            auto Diff_g_ij=Diff_g_i[j].expansion();
            ErrorType g_ij=ze, Dg_ij=ze, D2g_ij=ze;
            for (auto ac : Diff_g_ij) {
                UniformReference<MultiIndex> a=ac.index();
                FloatDPBounds const& c=ac.coefficient();
                auto val = mag(c);
                if (a.degree()==0) {
                    g_ij = val;
                    pK_ij[i][j] = val;
                } else if (a.degree()==1) {
                    Dg_ij += val;
                    pL_ij[i][j] = max(pL_ij[i][j],ErrorType(val));
                } else {
                    assert(a.degree()==2);
                    D2g_ij += val;
                    pH_ij[i][j] = max(pH_ij[i][j],ErrorType(val));
                }
            }
            pK_i=max(pK_i,g_ij); pL_i=max(pL_i,Dg_ij); pH_i=max(pH_i,D2g_ij);
        }
        pK+=pK_i; pL+=pL_i; pH+=pH_i;
    }

    for (auto j : range(n)) {
        for (auto i : range(m)) {
            ErrorType Vi(abs(inputs[i]).upper_bound());
            ErrorType Wi(Vi*wstar_multiplier<A>());
            pKjv[j] += Vi*pK_ij[i][j]; pLjv[j] += Vi*pL_ij[i][j]; pHjv[j] += Vi*pH_ij[i][j];
            pKjw[j] += Wi*pK_ij[i][j]; pLjw[j] += Wi*pL_ij[i][j]; pHjw[j] += Wi*pH_ij[i][j];
        }
        pKv = max(pKv,pKjv[j]); pLv = max(pLv,pLjv[j]); pHv = max(pHv,pHjv[j]);
        pKw = max(pKw,pKjw[j]); pLw = max(pLw,pLjw[j]); pHw = max(pHw,pHjw[j]);
    }

    ErrorType expLambda = (possibly(Lambda>0)) ? ErrorType(dexp(Lambda*h)) : ErrorType(1u,pr);
    ErrorType expL = cast_positive(exp(L*h));

    return ErrorConstants(K, Kj, pK, pKv, pKw, pKj, pKjv, pKjw,
                          L, Lj, pL, pLv, pLw, pLj, pLjv, pLjw,
                          H, Hj, pH, pHv, pHw, pHj, pHjv, pHjw,
                          Lambda, expLambda, expL);
}

ErrorType zeroparam_worstcase_error(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h) {
    return min(n.pK*n.expLambda*h, (n.K*2u+n.pK)*h); }
ErrorType zeroparam_component_error(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h, SizeType j) {
    return min(n.pKv*n.expL*h, min( (n.Kj[j]*2u+n.pKjv[j])*h, n.pKv*n.expLambda*h));
}

ErrorType oneparam_worstcase_error(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h) {
    return pow(h,2u)*((n.K+n.pKv)*n.pLv + n.pKv*(n.L+n.pLw)*n.expLambda)*2u; }
ErrorType oneparam_component_error(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h, SizeType j) {
    return pow(h,2u)*(n.pLjv[j]*(n.K+n.pKv)/2u + (n.Lj[j]+n.pLjw[j])*(n.pKv+n.pKw)*cast_positive(cast_exact(n.expLambda-1u))/(cast_positive(cast_exact(n.Lambda))*h) ); }

template<> ErrorType twoparam_worstcase_error<AffineInputs>(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h) {
    return (n.pLv*n.pKv + n.pLw*n.pKw +
            h*(n.pKv+n.pKw)*((n.pHw*2u + n.H)*(n.K+n.pKw)+sqr(n.L)+(n.L*3u+n.pLw*2u)*n.pLw)*n.expLambda +
            (n.K+n.pKv)*h/6u*((n.H*(n.pKv+n.pKw)+n.L*(n.pLv+n.pLw))*3u+((n.pHv+n.pHw)*n.K+n.L*(n.pLv+n.pLw))*4u)
           )/cast_positive(+1u-h*n.L/2u-h*n.pLw)*pow(h,2u)/4u; }
template<> ErrorType twoparam_worstcase_error<AdditiveInputs>(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h) {
    return (n.H*(n.K+n.pKv)/2u +
            (pow(n.L,2u)+n.H*(n.K+n.pKv))*n.expLambda
           )/cast_positive(+1u-h*n.L/2u)*(n.pKv+n.pKw)*pow(h,3u)/4u; }
template<> ErrorType twoparam_worstcase_error<SingularInput>(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h) {
    return ((n.pKv+n.pKw)*((n.pHw*2u+n.H)*(n.K+n.pKw)+sqr(n.L)+(n.L*3u+2u*n.pLw)*n.pLw)*n.expLambda +
            (n.K+n.pKv)/6u*(((n.H*(n.pKv+n.pKw)+n.L*(n.pLv+n.pLw))*3u +((n.pHv+n.pHw)*n.K+n.L*(n.pLv+n.pLw))*4u) +
                           (n.pHv*n.pKv+sqr(n.pLv)+n.pHw*n.pKw+sqr(n.pLw))*8u)
           )*pow(h,3u)/4u/cast_positive(+1u-h*n.L/2u-h*n.pLw); }
template<> ErrorType twoparam_worstcase_error<DualInputs>(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h) {
    return twoparam_worstcase_error<AffineInputs>(n, v, w, h); }

template<> ErrorType twoparam_component_error<AffineInputs>(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h, SizeType j) {
    return (
            pow(h,2u) * (n.pKv*n.pLjv[j] + n.pKw*n.pLjw[j]) +
            pow(h,3u) * (((
               n.Hj[j]*(n.pKv+n.pKw) + n.Lj[j]*(n.pLv+n.pLw))*(n.K+n.pKv)/8u + ((n.pHjv[j] + n.pHjw[j])*n.K*(n.K+n.pKv) + n.L*(n.pLjv[j] + n.pLjw[j])*(n.K + n.pKw))/6u) +
               (n.pKv + n.pKw) * ( n.Lj[j]*n.pLw + n.Lj[j]*n.pL + n.Hj[j]*(n.K+n.pKw) ) * psi0(n.Lambda*h) +
               (n.pKv + n.pKw) * ( n.pLjv[j]*n.L + n.pLv*n.pLjv[j] +n.pHjw[j]*(n.K+n.pKw) ) * psi1(n.Lambda*h)  )
            ) / cast_positive(1u - h * (n.Lj[j]/2u+w*n.pLj[j]) );
}
template<> ErrorType twoparam_component_error<AdditiveInputs>(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h, SizeType j) {
    return pow(h,3u)*(v+w)*(n.Hj[j]*(n.K+v)/8u + (n.Lj[j]*n.L+n.Hj[j]*(n.K+w))*psi0(n.Lambda*h)
           ) / cast_positive(+1u-h*n.Lj[j]/2u); }
template<> ErrorType twoparam_component_error<SingularInput>(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h, SizeType j) {
    return pow(h,3u)*(((n.pHj[j]*n.pK*(n.K+n.pKv)+n.pL*n.pLj[j]*(n.K+n.pKw))/6u +
                          (n.Hj[j]*(n.pKv+n.pKw) + n.Lj[j]*(n.pLv+n.pLw))*(n.K+n.pKv)/8u +
                          ((n.pHjv[j] + n.pHjw[j])*n.K*(n.K+n.pKv) + n.L*(n.pLjv[j] + n.pLjw[j])*(n.K + n.pKw))/6u) +
            (n.pKv + n.pKw) * ( n.Lj[j]*n.pLw + n.Lj[j]*n.pL + n.Hj[j]*(n.K+n.pKw) ) * psi0(n.Lambda*h) +
            (n.pKv + n.pKw) * ( n.pLjv[j]*n.L + n.pLv*n.pLjv[j] +n.pHjw[j]*(n.K+n.pKw) ) * psi1(n.Lambda*h)
           )/cast_positive(1u-h*n.Lj[j]/2u-h*w*n.pLj[j]); }
template<> ErrorType twoparam_component_error<DualInputs>(ErrorConstants const& n, ErrorType const& v, ErrorType const& w, PositiveFloatDP const& h, SizeType j) {
    return twoparam_component_error<AffineInputs>(n, v, w, h, j); }

template<class A, class R> Vector<ErrorType> ApproximationErrorProcessor<A,R>::process(ErrorConstants const& n, PositiveFloatDP const& h) const {

    Vector<ErrorType> result_worstcase(n.dimension(),worstcase_error<A,R>(n,_inputs,h));

    Vector<ErrorType> result_componentwise(n.dimension(),n.precision());
    for (auto j: range(n.dimension()))
        result_componentwise[j] = component_error<A,R>(n,_inputs,h,j);

    Vector<ErrorType> result(n.dimension(),n.precision());
    for (auto j: range(n.dimension())) {
        result[j] = min(result_worstcase[j],result_componentwise[j]);
    }

    return result;
}

template<class A, class R> Vector<ErrorType> ApproximationErrorProcessor<A,R>::process(PositiveFloatDP const& h, UpperBoxType const& B) const {
    CONCLOG_SCOPE_CREATE;
    ErrorConstants norms = compute_constants<A>(Ariadne::noise_independent_component(_f, _inputs.size()),
                                             Ariadne::input_derivatives(_f, _inputs.size()), _inputs, h, B);
    CONCLOG_PRINTLN("norms: " << norms);
    Set<Nat> input_idx;
    for (SizeType i : range(_f.result_size(),_f.result_size()+_inputs.size())) { input_idx.insert(i); }
    if (is_additive_in(_f,input_idx))
        norms.pK=mag(norm(_inputs));
    return process(norms,h);
}

InclusionIntegrator
InclusionIntegratorFactory::create(EffectiveVectorMultivariateFunction const& f, BoxDomainType const& inputs, InputApproximation const& approximation) const {
    if (approximation.handles(ZeroApproximation())) return InclusionIntegrator(new InclusionIntegratorImpl<ZeroApproximation>(f, inputs, _integrator));
    if (approximation.handles(ConstantApproximation())) return InclusionIntegrator(new InclusionIntegratorImpl<ConstantApproximation>(f, inputs, _integrator));
    if (approximation.handles(AffineApproximation())) return InclusionIntegrator(new InclusionIntegratorImpl<AffineApproximation>(f, inputs, _integrator));
    if (approximation.handles(SinusoidalApproximation())) return InclusionIntegrator(new InclusionIntegratorImpl<SinusoidalApproximation>(f, inputs, _integrator));
    if (approximation.handles(PiecewiseApproximation())) return InclusionIntegrator(new InclusionIntegratorImpl<PiecewiseApproximation>(f, inputs, _integrator));
    ARIADNE_FAIL_MSG("Unhandled input approximation " << approximation);
}


template<class A> Bool
InclusionIntegratorImpl<A>::operator==(const InclusionIntegratorInterface& rhs) const {
    return instance_of<InclusionIntegratorImpl<A>>(&rhs);
}

template<class A> Bool
InclusionIntegratorImpl<A>::operator<(const InclusionIntegratorInterface& rhs) const {
    return this->index() < rhs.index();
}

template<class A> List<ValidatedVectorMultivariateFunctionPatch>
InclusionIntegratorImpl<A>::reach(BoxDomainType const& domx, ValidatedVectorMultivariateFunctionPatch const& evolve_function, UpperBoxType const& B, TimeStepType const& t, StepSizeType const& h) const {
    CONCLOG_SCOPE_CREATE;

    TimeStepType new_t = lower_bound(t+h);

    Interval<TimeStepType> domt(t,new_t);

    auto e=this->compute_errors(h,B);
    CONCLOG_PRINTLN("approximation errors:"<<e);
    auto doma = this->build_parameter_domain(_inputs);

    auto w = build_w_functions<A>(domt,doma,_f.result_size(),_inputs.size());
    CONCLOG_PRINTLN("w:"<<w);
    auto Fw = substitute_v_with_w(_f, w);
    CONCLOG_PRINTLN("Fw:"<<Fw);
    auto phi = this->_integrator->flow_step(Fw,domx,domt,doma,B);
    add_errors(phi,e);

    List<ValidatedVectorMultivariateFunctionPatch> result;
    result.append(this->build_reach_function(evolve_function, phi, t, new_t));
    return result;
}

template<class A> Vector<EffectiveScalarMultivariateFunction> InclusionIntegratorImpl<A>::build_secondhalf_piecewise_w_functions(Interval<TimeStepType> const& domt, BoxDomainType const& doma, SizeType n, SizeType m) const {
    auto zero = EffectiveScalarMultivariateFunction::zero(n+1+2*m);
    auto one = EffectiveScalarMultivariateFunction::constant(n+1+2*m,1_z);

    auto result = Vector<EffectiveScalarMultivariateFunction>(m,n+1+2*m);
    for (auto i : range(m)) {
        auto Vi = ExactNumber(doma[i].upper_bound());
        auto p0 = EffectiveScalarMultivariateFunction::coordinate(n+1+2*m,n+1+i);
        auto p1 = EffectiveScalarMultivariateFunction::coordinate(n+1+2*m,n+1+m+i);
        result[i] = (definitely (doma[i].upper_bound() == 0.0_exact) ? zero : p0+(one-p0*p0/Vi/Vi)*p1);
    }
    return result;
}

template<class A> ValidatedVectorMultivariateFunctionPatch InclusionIntegratorImpl<A>::build_secondhalf_piecewise_reach_function(
        ValidatedVectorMultivariateFunctionPatch const& evolve_function, ValidatedVectorMultivariateFunctionPatch const& Phi, TimeStepType const& t,
        TimeStepType const& new_t) const {

    // Evolve function is e(x,a,b) at s; Flow is phi(x,h,b)
    // Want (x,t,a,b):->phi(e(x,a,b),t-s,b))

    auto swp = dynamic_cast<ValidatedVectorMultivariateTaylorFunctionModelDP const&>(Phi.reference()).properties();

    SizeType n=evolve_function.result_size();
    SizeType b=Phi.argument_size()-(n+1);

    SizeType a=evolve_function.argument_size()-n-b;

    BoxDomainType X=evolve_function.domain()[range(0,n)];
    BoxDomainType PA=evolve_function.domain()[range(n,n+a)];
    BoxDomainType PB=Phi.domain()[range(n+1,n+1+b)];
    // TODO: Change to use FunctionPatch
    auto Tau=IntervalDomainType(t,new_t);
    BoxDomainType XTP = product(X,Tau,PA,PB);
    ValidatedVectorMultivariateTaylorFunctionModelDP xf=ValidatedVectorMultivariateTaylorFunctionModelDP::projection(XTP,range(0,n),swp);
    ValidatedScalarMultivariateTaylorFunctionModelDP tf=ValidatedScalarMultivariateTaylorFunctionModelDP::coordinate(XTP,n,swp);
    ValidatedVectorMultivariateTaylorFunctionModelDP af=ValidatedVectorMultivariateTaylorFunctionModelDP::projection(XTP,range(n+1,n+1+a),swp);
    ValidatedVectorMultivariateTaylorFunctionModelDP bf=ValidatedVectorMultivariateTaylorFunctionModelDP::projection(XTP,range(n+1+a,n+1+a+b),swp);

    ValidatedVectorMultivariateTaylorFunctionModelDP ef=compose(evolve_function,join(xf,af,bf));

    return compose(Phi,join(ef,tf,bf));
}

template<class A> ValidatedVectorMultivariateFunctionPatch InclusionIntegratorImpl<A>::build_reach_function(
        ValidatedVectorMultivariateFunctionPatch const& evolve_function, ValidatedVectorMultivariateFunctionPatch const& Phi, TimeStepType const& t,
        TimeStepType const& new_t) const {

    // Evolve function is e(x,a) at s; flow is phi(x,h,b)
    // Want (x,t,a,b):->phi(e(x,a),t-s,b))

    auto swp = dynamic_cast<ValidatedVectorMultivariateTaylorFunctionModelDP const&>(Phi.reference()).properties();

    SizeType n=evolve_function.result_size();

    SizeType a=evolve_function.argument_size()-n;
    SizeType b=Phi.argument_size()-(n+1);

    BoxDomainType X=evolve_function.domain()[range(0,n)];
    BoxDomainType PA=evolve_function.domain()[range(n,n+a)];
    BoxDomainType PB=Phi.domain()[range(n+1,n+1+b)];

    auto Tau=IntervalDomainType(t,new_t);
    BoxDomainType XTP = product(X,Tau,PA,PB);
    ValidatedVectorMultivariateTaylorFunctionModelDP xf(ValidatedVectorMultivariateTaylorFunctionModelDP::projection(XTP,range(0,n),swp));
    ValidatedVectorMultivariateTaylorFunctionModelDP tf(ValidatedScalarMultivariateTaylorFunctionModelDP::coordinate(XTP,n,swp));
    ValidatedVectorMultivariateTaylorFunctionModelDP af(ValidatedVectorMultivariateTaylorFunctionModelDP::projection(XTP,range(n+1,n+1+a),swp));
    ValidatedVectorMultivariateTaylorFunctionModelDP bf(ValidatedVectorMultivariateTaylorFunctionModelDP::projection(XTP,range(n+1+a,n+1+a+b),swp));

    ValidatedVectorMultivariateTaylorFunctionModelDP ef=compose(evolve_function,join(xf,af));

    return compose(Phi,join(ef,tf,bf));
}

template<class A> ValidatedVectorMultivariateFunctionPatch InclusionIntegratorImpl<A>::evolve(ValidatedVectorMultivariateFunctionPatch const& reach_function, TimeStepType const& t) const {
    return partial_evaluate(reach_function,reach_function.result_size(),t);
}

Void add_errors(ValidatedVectorMultivariateFunctionPatch& phi, Vector<ErrorType> const& e) {
    assert(phi.result_size()==e.size());
    ValidatedVectorMultivariateTaylorFunctionModelDP& tphi = dynamic_cast<ValidatedVectorMultivariateTaylorFunctionModelDP&>(phi.reference());
    for (auto i : range(e.size())) {
        tphi[i].add_error(e[i]);
    }
}

EffectiveVectorMultivariateFunction substitute_v_with_w(EffectiveVectorMultivariateFunction const& F, Vector<EffectiveScalarMultivariateFunction> const& w) {

    auto n = F.result_size();
    auto m = w.size();
    auto p = w[0].argument_size();

    auto coordinates = EffectiveVectorMultivariateFunction::coordinates(p);

    auto substitution = EffectiveVectorMultivariateFunction::zeros(n+m,p);
    for (auto i : range(n)) {
        substitution.set(i,coordinates[i]);
    }
    for (auto i : range(m)) {
        substitution.set(n+i,w[i]);
    }

    return compose(F,substitution);
}

template<class A> BoxDomainType InclusionIntegratorImpl<A>::build_parameter_domain(BoxDomainType const& V) const {
    BoxDomainType result(0u);
    for (SizeType i=0; i<this->_num_params_per_input; ++i)
        result = product(result,V);
    return result;
}

template<> Vector<EffectiveScalarMultivariateFunction> build_w_functions<ZeroApproximation>(Interval<TimeStepType> const& domt, BoxDomainType const& doma, SizeType n, SizeType m) {
    auto result = Vector<EffectiveScalarMultivariateFunction>(m,n+1);
    for (auto i : range(0,m))
        result[i] = EffectiveScalarMultivariateFunction::zero(n+1);
    return result;
}


template<> Vector<EffectiveScalarMultivariateFunction> build_w_functions<ConstantApproximation>(Interval<TimeStepType> const& domt, BoxDomainType const& doma, SizeType n, SizeType m) {
    auto result = Vector<EffectiveScalarMultivariateFunction>(m,n+1+m);
    for (auto i : range(0,m))
        result[i] = EffectiveScalarMultivariateFunction::coordinate(n+1+m,n+1+i);
    return result;
}


template<> Vector<EffectiveScalarMultivariateFunction> build_w_functions<AffineApproximation>(Interval<TimeStepType> const& domt, BoxDomainType const& doma, SizeType n, SizeType m) {
    auto zero = EffectiveScalarMultivariateFunction::zero(n+1+2*m);
    auto one = EffectiveScalarMultivariateFunction::constant(n+1+2*m,1_z);
    auto three = EffectiveScalarMultivariateFunction::constant(n+1+2*m,3_z);
    auto t = EffectiveScalarMultivariateFunction::coordinate(n+1+2*m,n);
    auto tk = EffectiveScalarMultivariateFunction::constant(n+1+2*m,domt.lower_bound());
    auto hc = EffectiveScalarMultivariateFunction::constant(n+1+2*m,domt.width());

    auto result = Vector<EffectiveScalarMultivariateFunction>(m,n+1+2*m);
    for (auto i : range(m)) {
        auto Vi = ExactNumber(doma[i].upper_bound());
        auto p0 = EffectiveScalarMultivariateFunction::coordinate(n+1+2*m,n+1+i);
        auto p1 = EffectiveScalarMultivariateFunction::coordinate(n+1+2*m,n+1+m+i);
        result[i] = (definitely (doma[i].upper_bound() == 0.0_exact) ? zero : p0+three*(one-p0*p0/Vi/Vi)*p1*(t-tk-hc/2)/hc);
    }
    return result;
}


template<> Vector<EffectiveScalarMultivariateFunction> build_w_functions<SinusoidalApproximation>(Interval<TimeStepType> const& domt, BoxDomainType const& doma, SizeType n, SizeType m) {
    auto zero = EffectiveScalarMultivariateFunction::zero(n+1+2*m);
    auto one = EffectiveScalarMultivariateFunction::constant(n+1+2*m,1_z);
    auto pgamma = EffectiveScalarMultivariateFunction::constant(n+1+2*m,1.1464_dec);
    auto gamma = EffectiveScalarMultivariateFunction::constant(n+1+2*m,4.162586_dec);
    auto t = EffectiveScalarMultivariateFunction::coordinate(n+1+2*m,n);
    auto tk = EffectiveScalarMultivariateFunction::constant(n+1+2*m,domt.lower_bound());
    auto hc = EffectiveScalarMultivariateFunction::constant(n+1+2*m,domt.width());

    auto result = Vector<EffectiveScalarMultivariateFunction>(m,n+1+2*m);
    for (auto i : range(m)) {
        auto Vi = ExactNumber(doma[i].upper_bound());
        auto p0 = EffectiveScalarMultivariateFunction::coordinate(n+1+2*m,n+1+i);
        auto p1 = EffectiveScalarMultivariateFunction::coordinate(n+1+2*m,n+1+m+i);
        result[i] = (definitely (doma[i].upper_bound() == 0.0_exact) ? zero : p0+(one-p0*p0/Vi/Vi)*pgamma*p1*sin((t-tk-hc/2)*gamma/hc));
    }
    return result;
}


template<> Vector<EffectiveScalarMultivariateFunction> build_w_functions<PiecewiseApproximation>(Interval<TimeStepType> const& domt, BoxDomainType const& doma, SizeType n, SizeType m) {
    auto zero = EffectiveScalarMultivariateFunction::zero(n+1+2*m);
    auto one = EffectiveScalarMultivariateFunction::constant(n+1+2*m,1_z);

    auto result = Vector<EffectiveScalarMultivariateFunction>(m,n+1+2*m);
    for (auto i : range(m)) {
        auto Vi = ExactNumber(doma[i].upper_bound());
        auto p0 = EffectiveScalarMultivariateFunction::coordinate(n+1+2*m,n+1+i);
        auto p1 = EffectiveScalarMultivariateFunction::coordinate(n+1+2*m,n+1+m+i);
        result[i] = (definitely (doma[i].upper_bound() == 0.0_exact) ? zero : p0-(one-p0*p0/Vi/Vi)*p1);
    }
    return result;
}

template<> List<ValidatedVectorMultivariateFunctionPatch>
InclusionIntegratorImpl<PiecewiseApproximation>::reach(BoxDomainType const& domx, ValidatedVectorMultivariateFunctionPatch const& evolve_function, UpperBoxType const& B, TimeStepType const& t, StepSizeType const& h) const {
    CONCLOG_SCOPE_CREATE;

    List<ValidatedVectorMultivariateFunctionPatch> result;

    auto n = _f.result_size();
    auto m = _inputs.size();

    TimeStepType intermediate_t = lower_bound(t+hlf(h));
    TimeStepType new_t = lower_bound(t+h);

    Interval<TimeStepType> domt_first(t,intermediate_t);
    Interval<TimeStepType> domt_second(intermediate_t,new_t);
    auto doma = this->build_parameter_domain(_inputs);

    auto e=this->compute_errors(h,B);
    CONCLOG_PRINTLN("approximation errors:"<<e);

    auto w_hlf = build_w_functions<PiecewiseApproximation>(domt_first,doma,n,m);
    CONCLOG_PRINTLN_AT(1,"w_hlf:"<<w_hlf);
    auto Fw_hlf = substitute_v_with_w(_f, w_hlf);
    CONCLOG_PRINTLN_AT(1,"Fw_hlf:" << Fw_hlf);

    auto phi_hlf = this->_integrator->flow_step(Fw_hlf,domx,domt_first,doma,B);
    auto intermediate_reach=this->build_reach_function(evolve_function, phi_hlf, t, intermediate_t);
    auto intermediate_evolve=this->evolve(intermediate_reach,intermediate_t);

    auto domx_second = cast_exact_box(intermediate_evolve.range());

    auto w = this->build_secondhalf_piecewise_w_functions(domt_second,doma,n,m);
    CONCLOG_PRINTLN_AT(1,"w:"<<w);
    auto Fw = substitute_v_with_w(_f, w);
    CONCLOG_PRINTLN_AT(1,"Fw:"<<Fw);
    auto phi = this->_integrator->flow_step(Fw,domx_second,domt_second,doma,B);
    add_errors(phi,e);

    // The reach set is only the second part, but this is ok since we can't use this reach set except for construction of the evolve set
    return this->build_secondhalf_piecewise_reach_function(intermediate_evolve, phi, intermediate_t, new_t);
}

} // namespace Ariadne;
