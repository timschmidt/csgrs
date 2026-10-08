/***************************************************************************
 *            expression.decl.hpp
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

/*! \file expression.decl.hpp
 *  \brief Declarations of expression classes and type aliases.
 */

#ifndef ARIADNE_EXPRESSION_DECL_HPP
#define ARIADNE_EXPRESSION_DECL_HPP

namespace Ariadne {

class String;
class Integer;
class Real;

template<class T> class Vector;

class Identifier;

template<class T> class Vector;
template<class T> class Matrix;
using RealVector = Vector<Real>;
using RealMatrix = Matrix<Real>;

template<class T> class Constant;
template<class T> class Variable;
template<class T> class Variables;
template<class T> class Space;
template<class T> class Expression;

template<class T> class LetVariable;
template<class T> class DottedVariable;
template<class T> class PrimedVariable;
template<class V,class E> class Assignment;

//!@{
//! \ingroup SymbolicModule
//! \relates Constant
//! \name Type synonyms
using StringConstant = Constant<String>; //!< <p/>
using IntegerConstant = Constant<Integer>; //!< <p/>
using RealConstant = Constant<Real>; //!< <p/>
//!@}

//!@{
//! \ingroup SymbolicModule
//! \relates Variable
//! \name Type synonyms
using BooleanVariable = Variable<Boolean>; //!< <p/>
using KleeneanVariable = Variable<Kleenean>; //!< <p/>
using StringVariable = Variable<String>; //!< <p/>
using IntegerVariable = Variable<Integer>; //!< <p/>
using RealVariable = Variable<Real>; //!< <p/>
//!@}

//!@{
//! \relates Variables
//! \name Type synonyms
using RealVariables = Variables<Real>; //!< <p/>
using RealVectorVariable = Variable<RealVector>; //!< <p/>
//!@}

//!@{
//! \relates LetVariable
//! \name Type synonyms
using LetStringVariable = LetVariable<String>; //!< <p/>
using LetIntegerVariable = LetVariable<Integer>; //!< <p/>
using LetRealVariable = LetVariable<Real>; //!< <p/>
//!@}

//!@{
//! \relates PrimedVariable
//! \name Type synonyms
using PrimedStringVariable = PrimedVariable<String>; //!< <p/>
using PrimedIntegerVariable = PrimedVariable<Integer>; //!< <p/>
using PrimedRealVariable = PrimedVariable<Real>; //!< <p/>
//!@}

//! \relates DottedVariable
//! \name Type synonyms
using DottedRealVariable = DottedVariable<Real>; //!< <p/>
//!@}

//!@{
//! \relates Expression
//! \name Type synonyms
using BooleanExpression = Expression<Boolean>; //!< <p/>
using KleeneanExpression = Expression<Kleenean>; //!< <p/>
using StringExpression = Expression<String>; //!< <p/>
using IntegerExpression = Expression<Integer>; //!< <p/>
using RealExpression = Expression<Real>; //!< <p/>
using RealExpressions = List<Expression<Real>>; //!< <p/>
using RealExpressionVector = Vector<Expression<Real>>; //!< <p/>
using RealVectorExpression = Expression<RealVector>;
using RealMatrixExpression = Expression<RealMatrix>;


using DiscretePredicate = Expression<Boolean>; //!< \brief A decidable predicate over discrete variables.
using ContinuousPredicate = Expression<Kleenean>; //!< \brief A quasidecidable predicate over continuous variables.
//!@}

//!@{
//! \relates Assignment
//! \name Type synonyms
using StringAssignment = Assignment<StringVariable,StringExpression>; //!< <p/>
using PrimedStringAssignment = Assignment<PrimedStringVariable,StringExpression>; //!< <p/>
using IntegerAssignment = Assignment<IntegerVariable,IntegerExpression>; //!< <p/>
using PrimedIntegerAssignment = Assignment<PrimedIntegerVariable,IntegerExpression>; //!< <p/>
using RealAssignment = Assignment<RealVariable,RealExpression>; //!< <p/>
using PrimedRealAssignment = Assignment<PrimedRealVariable,RealExpression>; //!< <p/>
using DottedRealAssignment = Assignment<DottedRealVariable,RealExpression>; //!< <p/>

using PrimedStringAssignments = List<PrimedStringAssignment>; //!< <p/>
using RealAssignments = List<RealAssignment>; //!< <p/>
using PrimedRealAssignments = List<PrimedRealAssignment>; //!< <p/>
using DottedRealAssignments = List<DottedRealAssignment>; //!< <p/>

using RealConstantAssignment = Assignment<RealVariable,Real>; //!< <p/>
//!@}

typedef Space<Real> RealSpace;

template<class T, class X=T> class Valuation;
//!@{
//! \relates Valuation
//! \name Type synonyms
using StringValuation = Valuation<String>; //!< <p/>
using IntegerValuation = Valuation<Integer>; //!< <p/>
//!@}

class DiscreteValuation;
template<class X> class ContinuousValuation;


template<class UB> class Interval;
using RealInterval = Interval<Real>;

template<class UB> class VariableInterval;
template<class UB> class VariableLowerInterval;
template<class UB> class VariableUpperInterval;
template<class IVL> class VariablesBox;

//!@{
//! \relates RealVariableInterval
//! \name Type synonyms
using RealVariableInterval = VariableInterval<Real>; //!< <p/>
using RealVariableLowerInterval = VariableLowerInterval<Real>; //!< <p/>
using RealVariableUpperInterval = VariableUpperInterval<Real>; //!< <p/>
using RealVariableIntervals = List<RealVariableInterval>; //!< <p/>
//!@}

//!@{
//! \relates VariablesBox
//! \name Type synonyms
using RealVariablesBox = VariablesBox<RealInterval>; //!< <p/>
//!@}

} // namespace Ariadne

#endif /* ARIADNE_EXPRESSION_DECL_HPP */
