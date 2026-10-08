/***************************************************************************
 *            function/user_function.hpp
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

/*! \file function/user_function.hpp
 *  \brief Wrappers for user functions
 */

#ifndef ARIADNE_USER_FUNCTION_HPP
#define ARIADNE_USER_FUNCTION_HPP

#include <cstdarg>
#include <iosfwd>

#include "function/function_interface.hpp"
#include "function/function_mixin.hpp"

#include "utility/macros.hpp"
#include "utility/pointer.hpp"
#include "utility/container.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "function/taylor_model.hpp"
#include "algebra/differential.hpp"
#include "function/formula.hpp"

#include "function_mixin.hpp"

namespace Ariadne {

//! \related ScalarUserFunction
//! \brief Template structure containing scalar function meta-information.
template<SizeType AS, SizeType PS=0u, DegreeType SM=255u>
struct ScalarMultivariateFunctionData
{
    static SizeType argument_size() { return AS; }
    static SizeType parameter_size() { return PS; }
    static DegreeType smoothness() { return SM; }
};

//! \brief A wrapper for converting templated C++ functions to %Ariadne functions.
//!
//! Given a C++ class F with a (static) template method
//!   <code>template<class R, class A, class P> compute(R& r, const A& a, const P& p);</code>
//! the type <code>ScalarUserFunction<F></code> is an Ariadne function defined by \f$r=f(a)\f$.
//! The constructor for ScalarUserFunction<F> takes a Vector<ApproximateNumericType> or Vector<ValidatedNumericType> argument which is used for \a p.
//!
//! The class F must also define meta-data <c>argument_size(), parameter_size()
//! and smoothness()</c>. These are most easily defined by inheriting from the
//! <tt>ScalarMultivariateFunctionData<AS,PS,SM=SMOOTH></tt> class.
//!
//! The constant \a SMOOTH is used for an arbitrarily-differentiable function.
template<class F> class ScalarUserFunction
    : public EffectiveScalarMultivariateFunction
{
  private:
    class Representation
        : public ScalarMultivariateFunctionMixin< Representation, EffectiveTag >
    {
      private:
        Vector<EffectiveNumber> _p;
      public:
        Representation(const Vector<EffectiveNumber>& p) : _p(p) { }

        template<class R, class A> inline Void _compute(R& r, const A& a) const { F::compute(r,a,_p); }

        virtual Representation* clone() const { return new Representation(*this); }

        virtual SizeType argument_size() const { return F::argument_size(); }
        virtual SizeType parameter_size() const { return F::parameter_size(); }


        virtual EffectiveScalarMultivariateFunction derivative(SizeType j) const { ARIADNE_NOT_IMPLEMENTED; }

        virtual Covector<FloatDPApproximation> gradient(const Vector<FloatDPApproximation>& x) const {
            return this->evaluate(Differential<FloatDPApproximation>::variables(1u,x)).gradient(); }
        virtual Covector<FloatDPBounds> gradient(const Vector<FloatDPBounds>& x) const {
            return this->evaluate(Differential<FloatDPBounds>::variables(1u,x)).gradient(); }

         virtual OutputStream& repr(OutputStream& os) const  {
            return os << "USER"; }
       virtual OutputStream& _write(OutputStream& os) const  {
            return os << "ScalarUserFunction( argument_size="<<this->argument_size()<<" )"; }
    };
  public:
    ScalarUserFunction() : EffectiveScalarMultivariateFunction(new Representation(Vector<EffectiveNumber>(this->parameter_size()))) { }
    ScalarUserFunction(const Vector<Real>& p) : EffectiveScalarMultivariateFunction(new Representation(Vector<EffectiveNumber>(p))) { }
    ScalarUserFunction(const Vector<EffectiveNumber>& p) : EffectiveScalarMultivariateFunction(new Representation(p)) { }

    SizeType parameter_size() const { return F().parameter_size(); }
    //const Vector<ValidatedNumericType> parameters() const { return _p; }
};




//! \brief A convenience class defining the meta-data of an %Ariadne function.
template<SizeType RS, SizeType AS, SizeType PS=0u, DegreeType SM=255u>
class VectorMultivariateFunctionData
{
  public:
    //!
    static SizeType result_size() { return RS; }
    //!
    static SizeType argument_size() { return AS; }
    //!
    static SizeType parameter_size() { return PS; }
    //!
    static DegreeType smoothness() { return SM; }
};




//! \brief A wrapper for converting templated C++ functions to %Ariadne functions.
//!
//! Given a C++ class F with a (static) template method
//!   <code>template<class R, class A, class P> compute(R& r, const A& a, const P& p);</code>
//! the type <code>VectorUserFunction<F></code> is an Ariadne function defined by \f$r=f(a)\f$.
//! The constructor for VectorUserFunction<F> takes a Vector<EffectiveNumericType> argument which is used for \a p.
//!
//! The class F must also define meta-data <c>result_size(), argument_size(), parameter_size()
//! and smoothness()</c> as static functions. These are most easily defined by inheriting from the
//! <tt>VectorMultivariateFunctionData<RS,AS,PS,SM=SMOOTH></tt> class.
//!
//! The constant \a SMOOTH is used for an arbitrarily-differentiable function.
template<class F> class VectorUserFunction
    : public EffectiveVectorMultivariateFunction
{
    typedef DoublePrecision PR;
  private:
    class Representation
        : public VectorMultivariateFunctionMixin< Representation, EffectiveTag >
    {
      public:
        Representation(const Vector<EffectiveNumber>& p) : _p(p) { }

        virtual Representation* clone() const { return new Representation(*this); }

        virtual SizeType result_size() const { return F::result_size(); }
        virtual SizeType argument_size() const { return F::argument_size(); }
        virtual SizeType parameter_size() const { return F::parameter_size(); }

        template<class R, class A> inline Void _compute(R& r, const A& a) const { F::compute(r,a,_p); }

        virtual Matrix<FloatDPApproximation> jacobian(const Vector<FloatDPApproximation>& x) const {
            return Ariadne::jacobian(this->evaluate(Differential<FloatDPApproximation>::variables(1u,x))); }
        virtual Matrix<FloatDPBounds> jacobian(const Vector<FloatDPBounds>& x) const {
            return Ariadne::jacobian(this->evaluate(Differential<FloatDPBounds>::variables(1u,x))); }

        virtual EffectiveScalarMultivariateFunction::Interface* _get(SizeType i) const { ARIADNE_NOT_IMPLEMENTED; }
        virtual EffectiveScalarMultivariateFunction operator[](SizeType i) const { ARIADNE_NOT_IMPLEMENTED; }

        // TODO: Find a better way for writing functions which can handle transformations which may not have a
        // _write() method or operator<<.
        virtual OutputStream& _write(OutputStream& os) const  {
            return os << "VectorUserFunction( result_size="<<this->result_size()<<", argument_size="<<this->argument_size()<<" )"; }

        Vector<EffectiveNumber> _p;
    };
  public:
    VectorUserFunction(const Vector<EffectiveNumber>& p) : EffectiveVectorMultivariateFunction(new Representation(p)) { }
    const Vector<EffectiveNumber>& parameters() const { return dynamic_cast<const Representation*>(this->raw_pointer())->_p; }
};


} // namespace Ariadne

#endif /* ARIADNE_USER_FUNCTION_HPP */
