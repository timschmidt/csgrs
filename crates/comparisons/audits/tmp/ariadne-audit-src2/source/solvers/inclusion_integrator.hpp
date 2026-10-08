/***************************************************************************
 *            inclusion_integrator.hpp
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

/*! \file inclusion_evolver.hpp
 *  \brief Evolver for differential inclusion dynamics.
 */

#ifndef ARIADNE_INCLUSION_INTEGRATOR_HPP
#define ARIADNE_INCLUSION_INTEGRATOR_HPP

#include "utility/typedefs.hpp"
#include "utility/attribute.hpp"
#include "algebra/sweeper.hpp"
#include "algebra/algebra.hpp"
#include "function/domain.hpp"
#include "function/function_patch.hpp"
#include "function/function_model.hpp"
#include "function/formula.hpp"
#include "function/symbolic_function.hpp"
#include "symbolic/expression_set.hpp"
#include "conclog/logging.hpp"
#include "solvers/integrator_interface.hpp"

using namespace ConcLog;

namespace Ariadne {

class Real;

using TimeStepType = Dyadic;

typedef FloatDPError ErrorType;

template<class C, class T> Bool instance_of(T* const obj) {
    return (dynamic_cast<const C*>(obj) != nullptr);
}

inline TimeStepType lower_bound(TimeStepType const& t) {
    return TimeStepType(FloatDPLowerBound(t,DoublePrecision()).raw()); }
inline TimeStepType lower_bound(Real const& t) {
    return TimeStepType(FloatDPLowerBound(t,DoublePrecision()).raw()); }

inline Vector<FloatDP> const& cast_exact(Vector<ErrorType> const& v) {
    return reinterpret_cast<Vector<FloatDP>const&>(v); }

Void add_errors(ValidatedVectorMultivariateFunctionPatch& phi, Vector<ErrorType> const& e);

EffectiveVectorMultivariateFunction substitute_v_with_w(EffectiveVectorMultivariateFunction const& F, Vector<EffectiveScalarMultivariateFunction> const& w);

template<class F1, class F2, class F3, class... FS> decltype(auto) product(F1 const& f1, F2 const& f2, F3 const& f3, FS const& ... fs) {
    return product(product(f1,f2),f3,fs...); }

template<class F1, class F2, class F3, class... FS> decltype(auto) join(F1 const& f1, F2 const& f2, F3 const& f3, FS const& ... fs) {
    return join(join(f1,f2),f3,fs...); }

//! \brief The function dexp(x)=(exp(x)-1)/x.
//! Note that the function is positive and monotone increasing.
template<class F> PositiveUpperBound<F> dexp(UpperBound<F> const& x) {
    if(x.raw()>=0) { return cast_positive(exp(x)-1)/cast_positive(cast_exact(x)); }
    else { return cast_positive(cast_exact(1-exp(x)))/cast_positive(-x); }
}

template<class F> PositiveLowerBound<F> dexp(LowerBound<F> const& x) {
    if(x.raw()>=0) { return cast_positive(exp(x)-1)/cast_positive(cast_exact(x)); }
    else { return cast_positive(cast_exact(1-exp(x)))/cast_positive(-x); }
}

template<class F> PositiveBounds<F> dexp(Bounds<F> const& x) {
    return PositiveBounds<F>(dexp(x.lower_bound()),dexp(x.upper_bound()));
}

// Compute (x*e^x/2-e^x+x/2+1)/x^3, which is a positive monotone function.
template<class F> PositiveBounds<F> psi0(F const& x) {
    return PositiveBounds<F>((x*exp(x)/2u-exp(x)+x/2u+1u)/pow(x,3u));
}
// Compute (x*e^x-e^x-x^2/2+1)/x^3, which is a positive monotone function.
template<class F> PositiveBounds<F> psi1(F const& x) {
    return PositiveBounds<F>(((x-1)*exp(x)+(1u-sqr(x)/2u))/pow(x,3u));
}

template<class F> PositiveUpperBound<F> psi0(UpperBound<F> const& x) {
    return psi0(cast_exact(x));
}

template<class F> PositiveUpperBound<F> psi1(UpperBound<F> const& x) {
    return psi1(cast_exact(x));
}

template<class F> PositiveUpperBound<F> psi0(PositiveUpperBound<F> const& xu) {
    PositiveLowerBound<F> xl=cast_exact(xu);
    return cast_positive(xu*exp(xu)/2u+xu/2u+1u-exp(xl))/pow(xl,3u);
}

template<class F> PositiveUpperBound<F> psi1(PositiveUpperBound<F> const& xu) {
    PositiveLowerBound<F> xl=cast_exact(xu);
    return cast_positive(xu*exp(xu)+1u-(exp(xl)+sqr(xl)/2u))/pow(xl,3u);
}

struct ErrorConstants {
    typedef decltype(declval<ErrorType>().precision()) PrecisionType;

    ErrorType K; // K
    Vector<ErrorType> Kj; // K[j]
    ErrorType pK; // K'
    ErrorType pKv; // K'v
    ErrorType pKw; // K'w
    Vector<ErrorType> pKj; // K'[j]
    Vector<ErrorType> pKjv; // K'v[j]
    Vector<ErrorType> pKjw; // K'w[j]
    ErrorType L; // L
    Vector<ErrorType> Lj; // L[j]
    ErrorType pL; // L'
    ErrorType pLv; // L'v
    ErrorType pLw; // L'w
    Vector<ErrorType> pLj; // L'[j]
    Vector<ErrorType> pLjv; // L'v[j]
    Vector<ErrorType> pLjw; // L'w[j]
    ErrorType H; // H
    Vector<ErrorType> Hj; // H[j]
    ErrorType pH; // H'
    ErrorType pHv; // H'v
    ErrorType pHw; // H'w
    Vector<ErrorType> pHj; // H'[j]
    Vector<ErrorType> pHjv; // H'v[j]
    Vector<ErrorType> pHjw; // H'w[j]
    FloatDPUpperBound Lambda; // Lambda
    ErrorType expLambda; // e^(Lambda*h - 1) / (Lambda*h)
    ErrorType expL; // e^(L*h)

    ErrorConstants(ErrorType const&, Vector<ErrorType> const&, ErrorType const&, ErrorType const&, ErrorType const&, Vector<ErrorType> const&, Vector<ErrorType> const&, Vector<ErrorType> const&,
                   ErrorType const&, Vector<ErrorType> const&, ErrorType const&, ErrorType const&, ErrorType const&, Vector<ErrorType> const&, Vector<ErrorType> const&, Vector<ErrorType> const&,
                   ErrorType const&, Vector<ErrorType> const&, ErrorType const&, ErrorType const&, ErrorType const&, Vector<ErrorType> const&, Vector<ErrorType> const&, Vector<ErrorType> const&, FloatDPUpperBound const&, ErrorType const&, ErrorType const&);
    Tuple<ErrorType,Vector<ErrorType>,ErrorType,ErrorType,ErrorType,Vector<ErrorType>,Vector<ErrorType>,Vector<ErrorType>,
          ErrorType,Vector<ErrorType>,ErrorType,ErrorType,ErrorType,Vector<ErrorType>,Vector<ErrorType>,Vector<ErrorType>,
          ErrorType,Vector<ErrorType>,ErrorType,ErrorType,ErrorType,Vector<ErrorType>,Vector<ErrorType>,Vector<ErrorType>,
            FloatDPUpperBound,ErrorType,ErrorType> values() const;

    SizeType dimension() const { return _dimension; }
    PrecisionType precision() const { return PrecisionType(); }
  private:
    SizeType _dimension;
};

inline std::ostream& operator << (std::ostream& os, const ErrorConstants& n) {
    os << "K=" << n.K << ", Kj=" << n.Kj << ", K'=" << n.pK << ", K'v=" << n.pKv << ", Kw'=" << n.pKw << ", Kj'=" << n.Kj << ", K'jv=" << n.pKjv << ", K'jw=" << n.pKjw <<
          ", L=" << n.L << ", Lj=" << n.Lj << ", L'=" << n.pL << ", L'v=" << n.pLv << ", L'w=" << n.pLw << ", Lj'=" << n.pLj << ", Lj'v=" << n.pLjv << ", Lj'w=" << n.pLjw <<
          ", H=" << n.H << ", Hj=" << n.Hj << ", H'=" << n.pH << ", H'v=" << n.pHv << ", H'w=" << n.pHw << ", Hj'=" << n.pHj << ", Hj'v=" << n.pHjv << ", Hj'w=" << n.pHjw <<
          ", Lambda=" << n.Lambda << ", expLambda=" << n.expLambda << ", expL=" << n.expL;
    return os;
}

ErrorConstants compute_constants(EffectiveVectorMultivariateFunction const&, Vector<EffectiveVectorMultivariateFunction> const&, BoxDomainType const&, PositiveFloatDP const&, UpperBoxType const&);

struct InputApproximationInterface {
    //! \brief Whether the reached set is rigorous or another approximation must be used
    virtual bool is_reach_rigorous() const = 0;
    virtual Void write(OutputStream& os) const = 0;
    virtual InputApproximationInterface* clone() const = 0;
    virtual ~InputApproximationInterface() = default;
    virtual Nat index() const = 0;
};

inline std::ostream& operator << (std::ostream& os, const InputApproximationInterface& a) { a.write(os); return os; }

struct ZeroApproximation : public InputApproximationInterface {
    virtual bool is_reach_rigorous() const override { return true; }
    virtual Void write(OutputStream& os) const override { os << "ZERO"; }
    virtual Nat index() const override { return 0; }
    virtual ZeroApproximation* clone() const override { return new ZeroApproximation(*this); }
};
struct ConstantApproximation : public InputApproximationInterface {
    virtual bool is_reach_rigorous() const override { return true; }
    virtual Void write(OutputStream& os) const override { os << "CONSTANT"; }
    virtual Nat index() const override { return 1; }
    virtual ConstantApproximation* clone() const override { return new ConstantApproximation(*this); }
};
struct AffineApproximation : public InputApproximationInterface {
    virtual bool is_reach_rigorous() const override { return false; }
    virtual Void write(OutputStream& os) const override { os << "AFFINE"; }
    virtual Nat index() const override { return 2; }
    virtual AffineApproximation* clone() const override { return new AffineApproximation(*this); }
};
struct SinusoidalApproximation : public InputApproximationInterface {
    virtual bool is_reach_rigorous() const override { return false; }
    virtual Void write(OutputStream& os) const override { os << "SINUSOIDAL"; }
    virtual Nat index() const override { return 3; }
    virtual SinusoidalApproximation* clone() const override { return new SinusoidalApproximation(*this); }
};
struct PiecewiseApproximation : public InputApproximationInterface {
    virtual bool is_reach_rigorous() const override { return false; }
    virtual Void write(OutputStream& os) const override { os << "PIECEWISE"; }
    virtual Nat index() const override { return 4; }
    virtual PiecewiseApproximation* clone() const override { return new PiecewiseApproximation(*this); }
};

class InputApproximation : public Handle<const InputApproximationInterface> {
  public:
    using Handle<const InputApproximationInterface>::Handle;
    bool is_reach_rigorous() const { return this->_ptr->is_reach_rigorous(); }
    template<class A> Bool handles(A const& a) const { return instance_of<A>(&*this->_ptr); }
};

template<class A> constexpr Nat const_num_params_per_input();
template<> constexpr Nat const_num_params_per_input<ZeroApproximation>() { return 0u; }
template<> constexpr Nat const_num_params_per_input<ConstantApproximation>() { return 1u; }
template<> constexpr Nat const_num_params_per_input<AffineApproximation>() { return 2u; }
template<> constexpr Nat const_num_params_per_input<SinusoidalApproximation>() { return 2u; }
template<> constexpr Nat const_num_params_per_input<PiecewiseApproximation>() { return 2u; }

enum class InputsRelationKind : std::uint8_t { AFFINE, SINGULAR, DUAL, ADDITIVE};

template<InputsRelationKind R> struct InputsRelationKindTrait {
    static InputsRelationKind kind() { return R; }
};

ErrorType vstar(BoxDomainType const& inputs);

template<class A> ErrorType wstar_multiplier();

template<class A> ErrorType wstar(BoxDomainType const& inputs) {
    typedef typename ErrorType::PrecisionType PR;
    PR pr;
    ErrorType result(0u,pr);
    for (auto i : range(0,inputs.size()))
        result = max(result,wstar_multiplier<A>()*ErrorType(abs(inputs[i]).upper_bound()));
    return result;
}

class AffineInputs : public InputsRelationKindTrait<InputsRelationKind::AFFINE> { };
class SingularInput : public InputsRelationKindTrait<InputsRelationKind::SINGULAR> { };
class DualInputs : public InputsRelationKindTrait<InputsRelationKind::DUAL> { };
class AdditiveInputs : public InputsRelationKindTrait<InputsRelationKind::ADDITIVE> { };

inline std::ostream& operator<<(std::ostream& os, const InputsRelationKind& kind) {
    switch (kind) {
        case InputsRelationKind::AFFINE: os << "AFFINE"; break;
        case InputsRelationKind::SINGULAR: os << "SINGULAR"; break;
        case InputsRelationKind::DUAL: os << "DUAL"; break;
        case InputsRelationKind::ADDITIVE: os << "ADDITIVE"; break;
        default: ARIADNE_FAIL_MSG("Unhandled InputsRoles for output streaming");
    }
    return os;
}

template<class A> Vector<EffectiveScalarMultivariateFunction> build_w_functions(Interval<TimeStepType> const& domt, BoxDomainType const& doma, SizeType n, SizeType m);

class InclusionIntegrator;
class InclusionIntegratorInterface;

class InclusionIntegratorFactory {
    SharedPointer<IntegratorInterface> _integrator;
public:
    InclusionIntegratorFactory(IntegratorInterface const& integrator) : _integrator(integrator.clone()) { }
    InclusionIntegrator create(EffectiveVectorMultivariateFunction const& f, BoxDomainType const& inputs, InputApproximation const& approximation) const;
    InclusionIntegratorFactory* clone() const { return new InclusionIntegratorFactory(*this); }
};

template<class A> class ApproximationErrorProcessorInterface {
public:
    virtual Vector<ErrorType> process(PositiveFloatDP const& h, UpperBoxType const& B) const = 0;
};

template<class A, class R>
class ApproximationErrorProcessor : public ApproximationErrorProcessorInterface<A> {
  public:
    ApproximationErrorProcessor(EffectiveVectorMultivariateFunction const& f, BoxDomainType const& inputs)
        : _f(f), _inputs(inputs) { }
    virtual Vector<ErrorType> process(PositiveFloatDP const& h, UpperBoxType const& B) const override;
  private:
    EffectiveVectorMultivariateFunction const& _f;
    BoxDomainType const& _inputs;
  private:
    Vector<ErrorType> process(ErrorConstants const& n, PositiveFloatDP const& h) const;
  public:
    virtual ~ApproximationErrorProcessor() = default;
};


template<class A>
class ApproximationErrorProcessorFactory {
    typedef ApproximationErrorProcessorInterface<A> Processor;
public:
    SharedPointer<Processor> create(EffectiveVectorMultivariateFunction const& f, BoxDomainType const& inputs) const {
        SizeType n = f.result_size();
        SizeType m = inputs.size();
        ARIADNE_ASSERT_MSG(f.argument_size()-n == m, "ApproximationErrorProcessorFactory was given an incompatible f argument space in respect to the inputs box");
        Set<Nat> input_idx;
        for (SizeType i : range(n,n+m)) { input_idx.insert(i); }
        if (is_additive_in(f,input_idx)) return SharedPointer<Processor>(new ApproximationErrorProcessor<A,AdditiveInputs>(f,inputs));
        else if (m == 1) return SharedPointer<Processor>(new ApproximationErrorProcessor<A,SingularInput>(f, inputs));
        else if (m == 2) return SharedPointer<Processor>(new ApproximationErrorProcessor<A,DualInputs>(f, inputs));
        else return SharedPointer<Processor>(new ApproximationErrorProcessor<A,AffineInputs>(f,inputs));
    }
};

class InclusionIntegratorInterface {
  public:
    virtual Void write(OutputStream& os) const = 0;
    virtual Bool operator==(const InclusionIntegratorInterface& rhs) const = 0;
    virtual Bool operator<(const InclusionIntegratorInterface& rhs) const = 0;
    virtual Nat index() const = 0;
    virtual Nat num_params_per_input() const = 0;
    virtual bool is_reach_rigorous() const = 0;
    virtual List<ValidatedVectorMultivariateFunctionPatch> reach(BoxDomainType const& D, ValidatedVectorMultivariateFunctionPatch const& evolve_function, UpperBoxType const& B, TimeStepType const& t, StepSizeType const& h) const = 0;
    virtual ValidatedVectorMultivariateFunctionPatch evolve(ValidatedVectorMultivariateFunctionPatch const& reach_function, TimeStepType const& t) const = 0;
    friend std::ostream& operator<<(std::ostream& os, const InclusionIntegratorInterface& approximator) { approximator.write(os); return os; }
    virtual ~InclusionIntegratorInterface() = default;
};


template<class A>
class InclusionIntegratorImpl : public InclusionIntegratorInterface {
    friend class InclusionIntegratorFactory;
  protected:
    EffectiveVectorMultivariateFunction const& _f;
    BoxDomainType const& _inputs;
    SharedPointer<IntegratorInterface> _integrator;
    SharedPointer<ApproximationErrorProcessorInterface<A>> _processor;
    InclusionIntegratorImpl(EffectiveVectorMultivariateFunction const& f, BoxDomainType const& inputs, SharedPointer<IntegratorInterface> const& integrator) :
        _f(f), _inputs(inputs), _integrator(integrator), _processor(ApproximationErrorProcessorFactory<A>().create(f,inputs)), _num_params_per_input(const_num_params_per_input<A>()) { }
  private:
    const Nat _num_params_per_input;
  public:
    virtual Void write(OutputStream& os) const override { os << A(); }

    virtual List<ValidatedVectorMultivariateFunctionPatch> reach(BoxDomainType const& D, ValidatedVectorMultivariateFunctionPatch const& evolve_function, UpperBoxType const& B, TimeStepType const& t, StepSizeType const& h) const override;
    virtual ValidatedVectorMultivariateFunctionPatch evolve(ValidatedVectorMultivariateFunctionPatch const& reach_function, TimeStepType const& t) const override;

    virtual Bool operator==(const InclusionIntegratorInterface& rhs) const override;
    virtual Bool operator<(const InclusionIntegratorInterface& rhs) const override;

    virtual Nat index() const override { return A().index(); }

    virtual bool is_reach_rigorous() const override { return A().is_reach_rigorous(); }

    virtual Nat num_params_per_input() const override { return _num_params_per_input; }

    virtual ~InclusionIntegratorImpl() = default;
  private:
    Vector<ErrorType> compute_errors(StepSizeType const& h, UpperBoxType const& B) const { return _processor->process(PositiveFloatDP(h,DoublePrecision()),B); };
    BoxDomainType build_parameter_domain(BoxDomainType const& V) const;
    ValidatedVectorMultivariateFunctionPatch build_reach_function(ValidatedVectorMultivariateFunctionPatch const& evolve_function, ValidatedVectorMultivariateFunctionPatch const& Phi, TimeStepType const& t, TimeStepType const& new_t) const;
    ValidatedVectorMultivariateFunctionPatch build_secondhalf_piecewise_reach_function(ValidatedVectorMultivariateFunctionPatch const& evolve_function, ValidatedVectorMultivariateFunctionPatch const& Phi, TimeStepType const& t, TimeStepType const& new_t) const;
    Vector<EffectiveScalarMultivariateFunction> build_secondhalf_piecewise_w_functions(Interval<TimeStepType> const& domt, BoxDomainType const& doma, SizeType n, SizeType m) const;
};

class InclusionIntegrator : public Handle<const InclusionIntegratorInterface> {
  public:
    using Handle<const InclusionIntegratorInterface>::Handle;
    virtual ~InclusionIntegrator() = default;
    template<class A> Bool handles(A const& a) const { return instance_of<A>(&*this->_ptr); }

    friend Bool operator==(const InclusionIntegrator& lhs, const InclusionIntegrator& rhs) {
        return lhs._ptr->operator==(*rhs._ptr); }
    friend Bool operator<(const InclusionIntegrator& lhs, const InclusionIntegrator& rhs) {
        return lhs._ptr->operator<(*rhs._ptr); }

    bool is_reach_rigorous() const { return _ptr->is_reach_rigorous(); }
    Nat index() const { return _ptr->index(); }
    Nat num_params_per_input() const { return _ptr->num_params_per_input(); }

    friend std::ostream& operator<<(std::ostream& os, const InclusionIntegrator& approximator) { os << *approximator._ptr; return os; }

    operator InclusionIntegratorInterface const& () const { return *_ptr; }
    InclusionIntegrator* clone() const { return new InclusionIntegrator(*this); }

    List<ValidatedVectorMultivariateFunctionPatch> reach(BoxDomainType const& D, ValidatedVectorMultivariateFunctionPatch const& evolve_function, UpperBoxType const& B, TimeStepType const& t, StepSizeType const& h) const { return _ptr->reach(D,evolve_function,B,t,h); }
    ValidatedVectorMultivariateFunctionPatch evolve(ValidatedVectorMultivariateFunctionPatch const& reach_function, TimeStepType const& t) const { return _ptr->evolve(reach_function,t); }
};


} // namespace Ariadne;

#endif // ARIADNE_INCLUSION_INTEGRATOR_HPP
