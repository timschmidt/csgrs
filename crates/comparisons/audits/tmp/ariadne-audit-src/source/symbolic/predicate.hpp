/***************************************************************************
 *            symbolic/predicate.hpp
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

/*! \file symbolic/predicate.hpp
 *  \brief Predicates.
 */

#ifndef ARIADNE_PREDICATE_HPP
#define ARIADNE_PREDICATE_HPP

#include <cstdarg>
#include <iosfwd>
#include <iostream>
#include "function/function_interface.hpp"

#include "utility/macros.hpp"
#include "utility/pointer.hpp"

#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/differential.hpp"
#include "function/taylor_model.hpp"

namespace Ariadne {


class PredicateInterface {
  public:
    typedef std::size_t SizeType;

    virtual SizeType argument_size() const = 0;
    virtual Kleenean evaluate(const Vector<FloatDP>& x) const = 0;
    virtual Kleenean evaluate(const Vector<ValidatedNumericType>& x) const = 0;
};

class ExpressionPredicate
    : public PredicateInterface
{
    friend ExpressionPredicate operator!(const ExpressionPredicate&);
  public:
    ExpressionPredicate(const EffectiveScalarMultivariateFunction& expression)
        : _expression(expression), _sign(+1) { }
    const EffectiveScalarMultivariateFunction& expression() const { return _expression; }
    Int sign() const { return _sign; }

    Bool same(const ExpressionPredicate& ep2) const {
        const ExpressionPredicate& ep1=*this;
        return ep1._expression.pointer()==ep2._expression.pointer() && ep1._sign==ep2._sign; }
    Bool opposite(const ExpressionPredicate& ep2) const {
        const ExpressionPredicate& ep1=*this;
        return ep1._expression.pointer()==ep2._expression.pointer() && ep1._sign!=ep2._sign; }
    Bool operator<(const ExpressionPredicate& p) const {
        return (_expression.pointer()) < (const Void*)(p._expression.pointer()); }
    SizeType argument_size() const { return _expression.argument_size(); }
    Kleenean evaluate(const Vector<FloatDP>& x) const {
        FloatDP value=_expression.evaluate(x)*_sign;
        if(value<0) { return true; }
        else if(value>0) { return false; }
        else { return indeterminate; } }
    Kleenean evaluate(const Vector<ValidatedNumericType>& x) const {
        ExactIntervalType range=_expression.evaluate(x)*_sign;
        if(range.upper()<0) { return true; }
        else if(range.lower()>0) { return false; }
        else { return indeterminate; } }
  private:
    EffectiveScalarMultivariateFunction _expression;
    Int _sign;
};

//! \brief A predicate which is a disjunction of simple predicates.
//! An example of a disjunctive predicate is \f$p_1\vee p_2\vee p_3\f$.
class DisjunctivePredicate
    : public PredicateInterface
{
  public:
    SizeType size() const { return _predicates.size(); }
    const ExpressionPredicate& operator[](SizeType i) const { return _predicates[i]; }

    Bool vacuous() const { return _predicates.empty(); }
    Bool tautologous() const { return _tautology; }

    DisjunctivePredicate& operator|=(ExpressionPredicate p) {
        if(_tautology) { return *this; }
        for(SizeType i=0; i!=_predicates.size(); ++i) {
            if(p.same(_predicates[i])) { return *this; }
            if(p.opposite(_predicates[i])) { _predicates.clear(); _tautology=true; return *this; }
        }
        _predicates.push_back(p);
        return *this;
    }

    DisjunctivePredicate& operator|=(const DisjunctivePredicate& p) {
        for(SizeType i=0; i!=p.size(); ++i) { (*this) |= p[i]; } return *this; }

    virtual SizeType argument_size() const;
    virtual Kleenean evaluate(const Vector<FloatDP>& x) const;
    virtual Kleenean evaluate(const Vector<ValidatedNumericType>& x) const;

  private:
    std::vector<ExpressionPredicate> _predicates;
    Bool _tautology;
};

//! \brief A predicate in <em>conjunctive normal form</em>.
//!
//! An example of conjunctive normal form is \f$(p_1\vee p_2)\wedge(p_2\vee p_3\vee p_4)\f$.
class ConjunctiveNormalFormPredicate
    : public PredicateInterface
{
    friend ConjunctiveNormalFormPredicate
        operator&&(const ConjunctiveNormalFormPredicate& p1, const ConjunctiveNormalFormPredicate& p2);
    friend ConjunctiveNormalFormPredicate
        operator||(const ConjunctiveNormalFormPredicate& p1, const ConjunctiveNormalFormPredicate& p2);
  public:
    ConjunctiveNormalFormPredicate& operator&=(DisjunctivePredicate p) {
        _cnf.push_back(p); return *this; }
    ConjunctiveNormalFormPredicate& operator|=(DisjunctivePredicate p) {
        for(SizeType i=0; i!=_cnf.size(); ++i) { _cnf[i] |= p; } return *this; }

    virtual SizeType argument_size() const;
    virtual Kleenean evaluate(const Vector<FloatDP>& x) const;
    virtual Kleenean evaluate(const Vector<ValidatedNumericType>& x) const;
  private:
    std::vector<DisjunctivePredicate> _cnf;
};

ExpressionPredicate operator!(const ExpressionPredicate& p) {
    ExpressionPredicate res(p); res._sign=-p._sign; return res;
}



} // namespace Ariadne

#endif
