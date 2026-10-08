/***************************************************************************
 *            symbolic/expression.hpp
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

/*! \file symbolic/expression.hpp
 *  \brief Internal expressions
 */

#ifndef ARIADNE_EXPRESSION_HPP
#define ARIADNE_EXPRESSION_HPP

#include <cstdarg>
#include <iosfwd>
#include <iostream>

#include "utility/macros.hpp"
#include "utility/declarations.hpp"
#include "utility/pointer.hpp"
#include "utility/container.hpp"
#include "utility/writable.hpp"

#include "foundations/logical.decl.hpp"
#include "numeric/number.decl.hpp"

#include "numeric/operators.hpp"
#include "symbolic/constant.hpp"
#include "symbolic/variable.hpp"
#include "symbolic/valuation.hpp"
#include "symbolic/operations.hpp"

namespace Ariadne {

template<class T> struct DeclareExpressionOperations;
template<class X> struct ExpressionNode;

class OperationExpressionWriter;
class OperatorExpressionWriter;

//! \ingroup SymbolicModule
//! \brief A simple expression in named variables.
//! %Ariadne supports expressions of type Boolean, Kleenean, String, Integer and Real.
//!    \tparam T The type represented by the expression
//!
//! The independent variables are given string names, rather than an integer index.
//! Expressions in different variables may be combined; the argument variables of the resulting expression
//! are all variables occuring in all expression.
//! Expressions may be manipulated symbolically.
//!
//! \par \b Example
//! \snippet tutorials/symbolic_usage.cpp Expression_usage
//! \see Constant,  Variable,  Assignment
template<class T>
class Expression
    : public DeclareExpressionOperations<T>
{
    typedef SharedPointer<const ExpressionNode<T>> Pointer;
    static Writer<Expression<T>> _default_writer;
  public:
    static Void set_default_writer(Writer<Expression<T>> w) { _default_writer=w; }
    static Writer<Expression<T>> default_writer() { return _default_writer; }
  public:
    typedef Real NumericType;
    //! \brief The type represented by the expression.
    typedef T ValueType;
    typedef Constant<T> ConstantType;
    typedef Variable<T> VariableType;
  public:
    // Use template formulation to avoid ambiguity treating Expression(0) as a pointer construction.
    template<ConvertibleTo<SharedPointer<const ExpressionNode<T>>> P>
        explicit Expression(P const& eptr) : _root(eptr) { }
  public:
    //! \brief Default expression is a constant with default value.
    Expression();
    //! \brief Construct an expression from a numerical value.
    Expression(const T& c);
    template<ConvertibleTo<T> U> Expression(const U& c) : Expression(T(c)) { }
    //! \brief Construct an expression from a named constant.
    Expression(const Constant<T>& c);
    //! \brief Construct an expression from a variable.
    Expression(const Variable<T>& v);
    //! \brief Construct a constant expression from a value.
    static Expression<T> constant(const ValueType& c);
    //! \brief Construct a constant expression from a named variable value.
    static Expression<T> constant(const Constant<ValueType>& c);
    //! \brief Construct an expression from a name.
    static Expression<T> variable(const Identifier& c);

    //! \brief Create the zero element.
    Expression<T> create_zero() const { return Expression<T>::constant(T()); }
    //! \brief Create a constant element.
    Expression<T> create_constant(T const& t) const { return Expression<T>::constant(t); }
  public:
    Operator op() const;
    OperatorCode code() const;
    OperatorKind kind() const;
    const ValueType& val() const;
    const Identifier& var() const;
    const Expression<T>& arg() const;
    const Int& num() const;
    const Expression<T>& arg1() const;
    const Expression<T>& arg2() const;
    template<class A> const Expression<A>& cmp1(A* dummy=0) const;
    template<class A> const Expression<A>& cmp2(A* dummy=0) const;
    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, Expression<T> const& e) { return e._write(os); }
    //! \brief A writer for Expression objects using prefix notation.
    friend class OperationExpressionWriter;
    //! \brief A write for Expression objects using infix notation.
    friend class OperatorExpressionWriter;
  public:
    operator ElementaryAlgebra<Real>() const;
  public:
    //! \brief The variables needed to compute the expression.
    Set<UntypedVariable> arguments() const;
  public:
    SharedPointer<const ExpressionNode<T>> node_ptr() const { return _root; }
    const ExpressionNode<T>* node_raw_ptr() const { return _root.operator->(); }
    const ExpressionNode<T>& node_ref() const { return _root.operator*(); }
  private:
    OutputStream& _write(OutputStream& os) const;
  private:
    SharedPointer<const ExpressionNode<T>> _root;
};

template<class T> Bool same(Expression<T> const& e1, Expression<T> const& e2) { return identical(e1,e2); }

template<>
class Expression<Vector<Real>>
    : public DeclareExpressionOperations<Real>
    , public DeclareExpressionOperations<RealVector>
{
    typedef Vector<Real> T;
  public:
    // Use template formulation to avoid ambiguity treating Expression(0) as a pointer construction.
    template<ConvertibleTo<SharedPointer<const ExpressionNode<T>>> P>
        explicit Expression(P const& eptr) : _root(eptr) { }
  public:
    typedef Expression<Real> ScalarType;
    Expression(InitializerList<Expression<Real>>);
    Expression(Vector<Real>);
    Expression(Variables<Real>);
    Expression(Constant<Vector<Real>> c);
    Expression(Variable<Vector<Real>>);
    Expression(Vector<Expression<Real>>);
    Operator op() const;
    SizeType size() const;
    Expression<Real> get(SizeType i) const;
    Void set(SizeType i, Expression<Real>);
    Expression<Real> operator[](SizeType i) const { return this->get(i); }
    friend Expression<Vector<Real>> nul(Expression<Vector<Real>>);
    friend Expression<Vector<Real>> pos(Expression<Vector<Real>>);
    friend Expression<Vector<Real>> neg(Expression<Vector<Real>>);
    friend Expression<Vector<Real>> add(Expression<Vector<Real>>, Expression<Vector<Real>>);
    friend Expression<Vector<Real>> sub(Expression<Vector<Real>>, Expression<Vector<Real>>);
    friend Expression<Vector<Real>> mul(Expression<Real>, Expression<Vector<Real>>);
    friend Expression<Vector<Real>> mul(Expression<Vector<Real>>, Expression<Real>);
    friend Expression<Vector<Real>> div(Expression<Vector<Real>>, Expression<Real>);
    friend Expression<Vector<Real>> join(Expression<Vector<Real>>, Expression<Vector<Real>>);
    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, Expression<Vector<Real>> const& e) { return e._write(os); }
  public:
    //! \brief The variables needed to compute the expression.
    Set<UntypedVariable> arguments() const;
  public:
    SharedPointer<const ExpressionNode<T>> node_ptr() const { return _root; }
    const ExpressionNode<T>* node_raw_ptr() const { return _root.operator->(); }
    const ExpressionNode<T>& node_ref() const { return _root.operator*(); }
  private:
    OutputStream& _write(OutputStream& os) const;
  private:
    SharedPointer<const ExpressionNode<Vector<Real>>> _root;
};



//@{
//! \name Evaluation and related operations.
//! \related Expression

template<class T> LogicType<T> evaluate(const Expression<LogicType<T>>& e, const Valuation<T>& x);
template<class T> T evaluate(const Expression<T>& e, const Valuation<T>& x);
template<class T> Vector<T> evaluate(const Expression<Vector<T>>& e, const Valuation<T>& x);

//! \brief Evaluate expression \a e on argument \a x which is a map of variable identifiers to values of type \c A.
template<class T> LogicType<T> evaluate(const Expression<LogicType<T>>& e, const Map<Identifier,T>& x);
//! \brief Evaluate expression \a e on argument \a x which is a map of variable identifiers to values of type \c A.
template<class T> T evaluate(const Expression<T>& e, const Map<Identifier,T>& x);

Boolean evaluate(const Expression<Boolean>& e, const DiscreteValuation& q);
template<class X> X evaluate(const Expression<Real>& e, const ContinuousValuation<X>&);
template<class X> Kleenean evaluate(const Expression<Kleenean>&, const ContinuousValuation<X>&);

//! \brief Extract the arguments of expression \a e.
template<class T> Set<Identifier> arguments(const Expression<T>& e);

//! \brief Substitute all occurrences of variable \a v of type \c Y with constant value \a c.
template<class T, class Y> Expression<T> substitute(const Expression<T>& e, const Variable<Y>& v, const Y& c);
//! \brief Substitute all occurrences of variable \a v of type \c Y with expression value \a se.
template<class T, class Y> Expression<T> substitute(const Expression<T>& e, const List< Assignment< Variable<Y>,Expression<Y> > >& a);
template<class T, class Y> Vector<Expression<T>> substitute(const Vector<Expression<T>>& e, const List< Assignment< Variable<Y>,Expression<Y> > >& a);

//!@}

//!@{
//! \name Operations on expressions.
//! \related Expression

//! \brief Given \a sign when the predicate \a p is true.
Expression<Real> indicator(Expression<Kleenean> p, Sign sign=Sign::POSITIVE);

//! \brief The derivative of the expression \a e with respect to the variable \a v.
Expression<Real> derivative(const Expression<Real>& e, Variable<Real> v);
//!@}


//!@{
//! \name Testing the form of expressions.
//! \related Expression

//! \brief Returns \a true if the expression\a e is syntactically equal to the constant \a c.
template<class T> Bool is_constant(const Expression<T>& e, const SelfType<T>& c);

//! \brief Returns \a true if the expression \a e is syntactically equal to the variable with name \a vn.
template<class T> Bool is_variable(const Expression<T>& e, const Identifier& vn);

//! \brief Returns \a true if the expression \a e is syntactically equal to the variable \a v.
template<class T> Bool is_variable(const Expression<T>& e, const Variable<T>& v);

//! \brief Returns \a true if the expression \a e is syntactically constant in the variables \a vs.
template<class T> Bool is_constant_in(const Expression<T>& e, const Set<Variable<T>>& vs);
template<class T> Bool is_constant_in(const Expression<Vector<T>>& e, const Set<Variable<T>>& vs);
template<class T> Bool is_constant_in(const Vector<Expression<T>>& e, const Set<Variable<T>>& vs);
//! \brief Returns \a true if the expression \a e is syntactically affine in the variables \a vs.
Bool is_affine_in(const Expression<Real>& e, const Set<Variable<Real>>& vs);
//! \brief Returns \a true if the vector of expressions \a e is syntactically affine in the variables \a vs.
Bool is_affine_in(const Vector<Expression<Real>>& e, const Set<Variable<Real>>& vs);
//! \brief Returns \a true if the vector expression \a e is syntactically affine in the variables \a vs.
Bool is_affine_in(const Expression<Vector<Real>>& e, const Set<Variable<Real>>& vs);
//! \brief Returns \a true if the vector expression \a e is syntactically additive (possibly with multipliers) in the variables \a vs.
Bool is_additive_in(const Vector<Expression<Real>>& e, const Set<Variable<Real>>& vs);
Bool is_additive_in(const Expression<Real>& e, const Variable<Real>& v);
//! \brief Returns \a true if the expression \a e is syntactically polynomial in the variables \a vs.
Bool is_polynomial_in(const Vector<Expression<Real>>& e, const Set<Variable<Real>>& vs);
Bool is_polynomial_in(const Expression<Real>& e, const Set<Variable<Real>>& vs);

//! \brief Returns true if the expressions are mutual negations. <br>
//! Currently can only test for pairs of the form (a1<=a2; a1>=a2),  (a1<=a2; a2<=a1)
//! or (a1>=a2; a2>=a1).
Bool opposite(Expression<Kleenean> p, Expression<Kleenean> q);
//!@}

//!@{
//! \name Equality and ordering.
//! \related Expression

//! \brief Tests whether two expressions are identical.
template<class T> Bool identical(const Expression<T>& e1, const Expression<T>& e2);
//! \brief Check the ordering of two expressions \a e1 and \a e2, by identifying whether \a e1 precedes \a e2.
template<class T> Bool before(Expression<T> const& e1, Expression<T> const& e2);
//!@}

//!@{
//! \name Complexity checks and simplification.
//! \related Expression

//! \brief Count the number of nodes in the expression \a e.
template<class T> SizeType count_nodes(const Expression<T>& e);
//! \brief Count the number of distinct (i.e., having identical representation) nodes in the expression \a e.
template<class T> SizeType count_distinct_nodes(const Expression<T>& e);
//! \brief Count the number of distinct node pointers in the expression \a e.
template<class T> SizeType count_distinct_node_pointers(const Expression<T>& e);

//! \brief Simplify the expression \a e, such as by eliminating double negations \f$-(-e) \mapsto e\f$.
template<class T> Expression<T> simplify(const Expression<T>& e);
//! \brief Eliminate common subexpression in \a e by replacing identical nodes.
template<class T> Void eliminate_common_subexpressions(Expression<T>& e);
template<class T> Void eliminate_common_subexpressions(Vector<Expression<T>>& e);
//!@}




//!@{
//! \name Conversion to/from functions and formulae.
//! \related Expression

//! \brief Make a formula in terms of numbered coordinates from an expression in named variables.
Formula<EffectiveNumber> make_formula(const Expression<Real>& e, const Map<Identifier,SizeType>& v);
Formula<EffectiveNumber> make_formula(const Expression<Real>& e, const Space<Real>& spc);
Vector<Formula<EffectiveNumber>> make_formula(const Vector<Expression<Real>>& e, const Space<Real>& spc);
Formula<EffectiveNumber> make_formula(const Expression<Real>& e, const Variable<Real>& var);
Vector<Formula<EffectiveNumber>> make_formula(const Vector<Expression<Real>>& e, const Variable<Real>& spc);
Formula<EffectiveNumber> make_formula(const Expression<Real>& e, const List<Variable<Real>>& vars);
Formula<EffectiveNumber> make_formula(const Expression<Real>& out, const List<Assignment<Variable<Real>,Expression<Real>>>& aux, const Space<Real> spc);
Vector<Formula<EffectiveNumber>> make_formula(const Vector<Expression<Real>>& out, const List<Assignment<Variable<Real>,Expression<Real>>>& aux, const Space<Real> spc);

//! \brief Make a function on the real line given an expression in a single argument variable.
ScalarUnivariateFunction<EffectiveTag> make_function(const Variable<Real>& v, const Expression<Real>& e);
//! \brief Make a vector function on the real line given a vector of expressions in a single argument variable.
VectorUnivariateFunction<EffectiveTag> make_function(const Variable<Real>& v, const Vector<Expression<Real>>& e);
//! \brief Make a function on a Euclidean domain given an ordered list including all argument variables.
ScalarMultivariateFunction<EffectiveTag> make_function(const Space<Real>& s, const Expression<Real>& e);
//! \brief Make a function on a Euclidean domain given an ordered list including all argument variables.
VectorMultivariateFunction<EffectiveTag> make_function(const Space<Real>& s, const Vector<Expression<Real>>& e);

//! \brief Make a function on a Euclidean domain given an ordered list including all argument variables.
Expression<Real> make_expression(const ScalarMultivariateFunction<EffectiveTag>& f, const Space<Real>& s);

Expression<Real> make_expression(const Formula<Real>& f, const Space<Real>& s);
//!@}


//! \brief Infix notation for writing an Expression
class OperatorExpressionWriter {
  private: public:
    template<class T> OutputStream& _write(OutputStream& os, Expression<T> const& e) const;
  public:
    template<class T> WritableTemporary<Expression<T>,OperatorExpressionWriter> operator() (Expression<T> const& e) const {
        return WritableTemporary(*this,e); }
};

//! \brief Prefix notation for writing an Expression, using named operations instead of operator symbols.
class OperationExpressionWriter {
  private: public:
    template<class T> OutputStream& _write(OutputStream& os, Expression<T> const& e) const;
    template<class T> OutputStream& _write(OutputStream& os, Expression<Vector<T>> const& ve) const;
  public:
    template<class T> WritableTemporary<T,OperationExpressionWriter> operator() (Expression<T> const& e) const {
        return WritableTemporary(*this,e); }
    template<class T> WritableTemporary<Expression<Vector<T>>,OperationExpressionWriter> operator() (Expression<Vector<T>> const& ve) const {
        return WritableTemporary(*this,ve); }
};


} // namespace Ariadne

#endif /* ARIADNE_EXPRESSION_HPP */
