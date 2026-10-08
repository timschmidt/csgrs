/***************************************************************************
 *            foundations/logical.cpp
 *
 *  Copyright  2013-20  Pieter Collins
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

/*! \file foundations/logical.cpp
 *  \brief
 */

#include "utility/stdlib.hpp"
#include "utility/string.hpp"
#include "utility/macros.hpp"
#include "numeric/sequence.hpp"
#include "symbolic/templates.hpp"

#include "logical.hpp"
#include "numeric/integer.hpp"

namespace Ariadne {

namespace Detail {

inline LogicalValue check(LogicalValue l, Effort e) { return l; }
template<class OP, class ARG> decltype(auto) check(Symbolic<OP,ARG> const& s, Effort e) { return s._op(check(s._arg,e)); }
template<class OP, class ARG1, class ARG2> decltype(auto) check(Symbolic<OP,ARG1,ARG2> const& s, Effort e) { return s._op(check(s._arg1,e),check(s._arg2,e)); }

template<class L> class LogicalWrapper
    : public virtual LogicalInterface, public L
{
    using L::L;
  private:
    virtual LogicalInterface* _copy() const { return new LogicalWrapper<L>(*this); }
    virtual LogicalValue _check(Effort e) const { return check(static_cast<L const&>(*this),e); }
    virtual OutputStream& _write(OutputStream& os) const { return os << static_cast<L const&>(*this); }
};

template<> class LogicalWrapper<LogicalValue>
    : public virtual LogicalInterface
{
    LogicalValue _v;
  public:
    LogicalWrapper(LogicalValue v) : _v(v) { }
    operator LogicalValue() const { return this->_v; }
  private:
    virtual LogicalInterface* _copy() const { return new LogicalWrapper<LogicalValue>(*this); }
    virtual LogicalValue _check(Effort e) const { return this->_v; }
    virtual OutputStream& _write(OutputStream& os) const { return os << this->_v; }
};


class LogicalConstant : public LogicalWrapper<LogicalValue> {
    using LogicalWrapper<LogicalValue>::LogicalWrapper;
};

template<class OP, class... ARGS> class LogicalExpression : public LogicalWrapper<Symbolic<OP,ARGS...>> {
    using LogicalWrapper<Symbolic<OP,ARGS...>>::LogicalWrapper;
};


LogicalInterface* new_logical_pointer_from_value(LogicalValue v) {
    return new LogicalWrapper<LogicalValue>(v);
}

LogicalValue logical_value_from_pointer(LogicalInterface* ptr) {
    auto vlptr=dynamic_cast<LogicalWrapper<LogicalValue>*>(ptr);
    if (!vlptr) { throw std::runtime_error("logical_type_from_pointer: No conversion from abstract to concrete logical value"); }
    return *vlptr;
}

LogicalHandle LogicalHandle::constant(LogicalValue l) {
    return LogicalHandle(make_handle<const LogicalConstant>(l));
}

LogicalHandle operator&&(LogicalHandle l1, LogicalHandle l2) {
    return LogicalHandle(make_handle<const LogicalExpression<AndOp,LogicalHandle,LogicalHandle>>(AndOp(),l1,l2));
}

LogicalHandle operator||(LogicalHandle l1, LogicalHandle l2) {
    return LogicalHandle(make_handle<const LogicalExpression<OrOp,LogicalHandle,LogicalHandle>>(OrOp(),l1,l2));
}

LogicalHandle operator==(LogicalHandle l1, LogicalHandle l2) {
    return LogicalHandle(make_handle<const LogicalExpression<Equal,LogicalHandle,LogicalHandle>>(Equal(),l1,l2));
}

LogicalHandle operator^(LogicalHandle l1, LogicalHandle l2) {
    return LogicalHandle(make_handle<const LogicalExpression<XOrOp,LogicalHandle,LogicalHandle>>(XOrOp(),l1,l2));
}

LogicalHandle operator!(LogicalHandle l) {
    return LogicalHandle(make_handle<const LogicalExpression<NotOp,LogicalHandle>>(NotOp(),l));
}

LogicalHandle conjunction(LogicalHandle l1, LogicalHandle l2) {
    return LogicalHandle(make_handle<const LogicalExpression<AndOp,LogicalHandle,LogicalHandle>>(AndOp(),l1,l2));
}

LogicalHandle disjunction(LogicalHandle l1, LogicalHandle l2) {
    return LogicalHandle(make_handle<const LogicalExpression<OrOp,LogicalHandle,LogicalHandle>>(OrOp(),l1,l2));
}

LogicalHandle negation(LogicalHandle l) {
    return LogicalHandle(make_handle<const LogicalExpression<NotOp,LogicalHandle>>(NotOp(),l));
}

LogicalHandle equality(LogicalHandle l1, LogicalHandle l2) {
    return LogicalHandle(make_handle<const LogicalExpression<Equal,LogicalHandle,LogicalHandle>>(Equal(),l1,l2));
}


LogicalHandle exclusive(LogicalHandle l1, LogicalHandle l2) {
    return LogicalHandle(make_handle<const LogicalExpression<XOrOp,LogicalHandle,LogicalHandle>>(XOrOp(),l1,l2));
}


LogicalValue operator==(LogicalValue l1, LogicalValue l2) {
    switch (l1) {
        case LogicalValue::TRUE:
            return l2;
        case LogicalValue::LIKELY:
            switch (l2) { case LogicalValue::TRUE: return LogicalValue::LIKELY; case LogicalValue::FALSE: return LogicalValue::UNLIKELY; default: return l2; }
        case LogicalValue::INDETERMINATE:
            return LogicalValue::INDETERMINATE;
        case LogicalValue::UNLIKELY:
            switch (l2) { case LogicalValue::TRUE: return LogicalValue::UNLIKELY; case LogicalValue::FALSE: return LogicalValue::LIKELY; default: return not l2; }
        case LogicalValue::FALSE:
            return not l2;
        default:
            return LogicalValue::INDETERMINATE;
    }
}

OutputStream& operator<<(OutputStream& os, LogicalValue l) {
    switch(l) {
        case LogicalValue::TRUE: os << "true"; break;
        case LogicalValue::LIKELY: os << "likely";  break;
        case LogicalValue::INDETERMINATE: os << "indeterminate";  break;
        case LogicalValue::UNLIKELY: os << "unlikely"; break;
        case LogicalValue::FALSE: os << "false"; break;
        default: ARIADNE_FAIL_MSG("Unhandled LogicalValue for output streaming.");
    }
    return os;
}

template<> struct LogicalExpression<OrOp,Sequence<LowerKleenean>> : public LogicalInterface {
    Sequence<LowerKleenean> _seq;
  public:
    LogicalExpression(OrOp, Sequence<LowerKleenean> seq) : _seq(seq) { }
    LogicalInterface* _copy() const { return new LogicalExpression<OrOp,Sequence<LowerKleenean>>(*this); }
    LogicalValue _check(Effort eff) const {
        for(Natural k=0u; k!=eff.work(); ++k) {
            if ( definitely(_seq[k].check(eff)) ) { return LogicalValue::TRUE; }
        }
        return LogicalValue::INDETERMINATE;
    }
    OutputStream& _write(OutputStream& os) const {
        return os << "disjunction(" << _seq[0u] << "," << _seq[1u] << "," << _seq[2u] << ",...)";
    }
};


template<> struct LogicalExpression<AndOp,Sequence<UpperKleenean>> : public LogicalInterface {
    Sequence<UpperKleenean> _seq;
  public:
    LogicalExpression(AndOp, Sequence<UpperKleenean> seq) : _seq(seq) { }
    LogicalInterface* _copy() const { return new LogicalExpression<AndOp,Sequence<UpperKleenean>>(*this); }
    LogicalValue _check(Effort eff) const {
        for(Natural k=0u; k!=eff.work(); ++k) {
            if ( definitely(not _seq[k].check(eff)) ) { return LogicalValue::FALSE; }
        }
        return LogicalValue::INDETERMINATE;
    }
    OutputStream& _write(OutputStream& os) const {
        return os << "conjunction(" << _seq[0u] << "," << _seq[1u] << "," << _seq[2u] << ",...)";
    }
};

} // namespace Detail


LowerKleenean disjunction(Sequence<LowerKleenean> const& l) {
    return LowerKleenean(LogicalHandle(make_handle<const Detail::LogicalExpression<OrOp,Sequence<LowerKleenean>>>(OrOp(),l))) ;
}
UpperKleenean conjunction(Sequence<UpperKleenean> const& l) {
    return UpperKleenean(LogicalHandle(make_handle<const Detail::LogicalExpression<AndOp,Sequence<UpperKleenean>>>(AndOp(),l)));
}

Nat Effort::_default = 0u;

const Indeterminate indeterminate = Indeterminate();

Bool NondeterministicBoolean::_choose(LowerKleenean p1, LowerKleenean p2) {
    Effort eff(0u);
    while(true) {
        if(definitely(p1.check(eff))) { return true; }
        if(definitely(p2.check(eff))) { return false; }
        ++eff;
    }
}


template<> String class_name<ExactTag>() { return "Exact"; }
template<> String class_name<EffectiveTag>() { return "Effective"; }
template<> String class_name<ValidatedTag>() { return "Validated"; }
//template<> String class_name<UpperTag>() { return "Upper"; }
//template<> String class_name<LowerTag>() { return "Lower"; }
template<> String class_name<ApproximateTag>() { return "Approximate"; }

template<> String class_name<Bool>() { return "Bool"; }
template<> String class_name<Boolean>() { return "Boolean"; }
template<> String class_name<Sierpinskian>() { return "Sierpinskian"; }
template<> String class_name<NegatedSierpinskian>() { return "NegatedSierpinskian"; }
template<> String class_name<Kleenean>() { return "Kleenean"; }
template<> String class_name<LowerKleenean>() { return "LowerKleenean"; }
template<> String class_name<UpperKleenean>() { return "UpperKleenean"; }
template<> String class_name<ValidatedSierpinskian>() { return "ValidatedSierpinskian"; }
template<> String class_name<ValidatedNegatedSierpinskian>() { return "ValidatedNegatedSierpinskian"; }
template<> String class_name<ValidatedKleenean>() { return "ValidatedKleenean"; }
template<> String class_name<ValidatedLowerKleenean>() { return "ValidatedLowerKleenean"; }
template<> String class_name<ValidatedUpperKleenean>() { return "ValidatedUpperKleenean"; }
template<> String class_name<ApproximateKleenean>() { return "ApproximateKleenean"; }

} // namespace Ariadne

#include "utility/array.hpp"

namespace Ariadne {

SizeType nondeterministic_choose_index(Array<LowerKleenean> const& p) {
    Effort eff(0u);
    while(true) {
        for (SizeType i=0; i!=p.size(); ++i) {
            if(definitely(p[i].check(eff))) { return i; }
        }
        ++eff;
    }
}

} // namespace Ariadne
