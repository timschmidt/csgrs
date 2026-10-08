/***************************************************************************
 *            hybrid/hybrid_scalings.hpp
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

/*! \file hybrid/hybrid_scalings.hpp
 *  \brief Scalings for variables in hybrid spaces.
 */

#ifndef ARIADNE_HYBRID_SCALINGS_HPP
#define ARIADNE_HYBRID_SCALINGS_HPP

#include <iostream>
#include <map>

#include "utility/container.hpp"
#include "helper/stlio.hpp"

#include "symbolic/variable.hpp"

namespace Ariadne {

typedef void Void;
typedef std::ostream OutputStream;


//! \ingroup HybridModule
//! \brief A class which defines the state space grid to use in location \a loc given the continuous state variables \a spc.
class HybridScalingsInterface
{
  public:
    //!
    virtual ~HybridScalingsInterface() = default;
    virtual HybridScalingsInterface* clone() const = 0;
    virtual ExactDouble scaling(const DiscreteLocation& loc, const RealVariable& var) const = 0;
    virtual Void _write(OutputStream& os) const = 0;
};
inline OutputStream& operator<<(OutputStream& os, const HybridScalingsInterface& hsc) { hsc._write(os); return os; }


//! \ingroup HybridModule
//! \brief A class which defines the state space grid to use in location \a loc given the continuous state variables \a spc.
class HybridScalings
{
    std::shared_ptr<HybridScalingsInterface> _ptr;
  public:
    HybridScalings(const HybridScalingsInterface& ref) : _ptr(ref.clone()) { }
    operator const HybridScalingsInterface& () const { return *this->_ptr; }
    operator HybridScalingsInterface& () { return *this->_ptr; }
    ExactDouble scaling(const DiscreteLocation& loc, const RealVariable& var) const { return this->_ptr->scaling(loc,var); }
    Grid grid(const DiscreteLocation& loc, const RealSpace& spc) const;
  public:
    HybridScalings(const Map<Identifier,ExactDouble>& scalings);
    HybridScalings(const InitializerList<Pair<RealVariable,ApproximateDouble>>& scalings);
};

class SimpleHybridScalings
    : public HybridScalingsInterface
{
    ExactDouble _default_scaling;
    Map<Identifier,ExactDouble> _scalings;
  public:
    SimpleHybridScalings() : _default_scaling(1.0_x), _scalings() { }
    SimpleHybridScalings(ExactDouble default_scaling, const Map<Identifier,ExactDouble>& scalings)
        : _default_scaling(default_scaling), _scalings(scalings) { }
    SimpleHybridScalings(const InitializerList<Pair<RealVariable,ApproximateDouble>>& scalings);
    Void set_scaling(const RealVariable& var, ApproximateDouble scal) {
        ARIADNE_ASSERT(decide(scal>0)); _scalings[var.name()]=cast_exact(scal); }
    virtual SimpleHybridScalings* clone() const { return new SimpleHybridScalings(*this); }
    virtual ExactDouble scaling(const DiscreteLocation& loc, const RealVariable& var) const {
        return (this->_scalings.has_key(var.name())) ? this->_scalings[var.name()] : this->_default_scaling; }
    virtual Void _write(OutputStream& os) const { os << "HybridScalings( " << this->_scalings << " )"; }
};

inline Pair<RealVariable,ExactDouble> operator|(const RealVariable& var, ApproximateDouble scal) {
    return Pair<RealVariable,ExactDouble>(var,cast_exact(scal)); }

inline HybridScalings::HybridScalings(const Map<Identifier,ExactDouble>& scalings)
    : _ptr(new SimpleHybridScalings(1.0_x,scalings)) { }

inline HybridScalings::HybridScalings(const InitializerList<Pair<RealVariable,ApproximateDouble>>& scalings)
    : _ptr(new SimpleHybridScalings(scalings)) { }

inline SimpleHybridScalings::SimpleHybridScalings(const InitializerList<Pair<RealVariable,ApproximateDouble>>& scalings) {
    for(auto iter=scalings.begin(); iter!=scalings.end(); ++iter) {
        this->_scalings.insert(iter->first.name(),cast_exact(iter->second));
    }
}

}

#endif // ARIADNE_HYBRID_SCALINGS_HPP

