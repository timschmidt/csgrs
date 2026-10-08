/***************************************************************************
 *            geometry/geometry.hpp
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

/*! \file geometry/geometry.hpp
 *  \brief Geometric operations on abstract sets.
 */
#ifndef ARIADNE_GEOMETRY_HPP
#define ARIADNE_GEOMETRY_HPP

#include <vector>

#include "geometry/box.hpp"
#include "function/function_interface.hpp"

namespace Ariadne {

enum class SplitPart : char;


inline
SizeType irmax(const ExactBoxType& bx) {
    FloatDP dmax(0.0_x,dp);
    SizeType imax=0;
    for(SizeType i=0; i!=bx.size(); ++i) {
        FloatDP d=bx[i].width().upper().raw();
        if(d>dmax) {
            imax=i;
            dmax=d;
        }
    }
    return imax;
}

inline FloatDP mid(FloatDP l, FloatDP u) {
    return FloatDP(med(approx,l,u));
}

inline
ExactBoxType split(const ExactBoxType& bx, SizeType i, SplitPart lr) {
    ExactBoxType result(bx);
    ExactIntervalType& ivl=result[i];
    const FloatDP& l=ivl.lower_bound();
    const FloatDP& u=ivl.upper_bound();
    FloatDP c=mid(l,u);
    if(lr==SplitPart::MIDDLE) {
        ivl.set_bounds(mid(l,c),mid(c,u));
    } else {
        if(lr==SplitPart::LOWER) { ivl.set_upper_bound(c); }
        else { ivl.set_lower_bound(c); }
    }
    return result;
}

inline
Pair<ExactBoxType,ExactBoxType> split(const ExactBoxType& bx, SizeType i)
{
    Pair<ExactBoxType,ExactBoxType> result(bx,bx);
    FloatDP c=mid(bx[i].lower_bound(),bx[i].upper_bound());
    result.first[i].set_upper_bound(c);
    result.second[i].set_lower_bound(c);
    return result;
}

inline
ExactBoxType split(const ExactBoxType& bx, SplitPart lr)
{
    SizeType i=irmax(bx);
    return split(bx,i,lr);
}

inline
Pair<ExactBoxType,ExactBoxType> split(const ExactBoxType& bx) {
    return split(bx,irmax(bx));
}


template<class F>
ValidatedKleenean
image_separated(const ExactBoxType& d, const F& f, const ExactBoxType& b, const RawFloatDP& eps)
{

    ExactBoxType fd=f.evaluate(d);
    ExactBoxType fc=f.evaluate(ExactBoxType(midpoint(d)));

    //cout << "called with " << d << ", having fd=" << fd << " and fc=" << fc << endl;
    if(disjoint(fd,b)) {
        //cout << "evaluation is disjoint\n";
        return true;
    } else if(definitely(inside(fc,b))) {
        //cout << "evaluation of the midpoint is inside\n";
        return false;
    } else if(d.radius().upper().raw()<eps) {
        //cout << "radius limit reached\n";
        return indeterminate;
    } else {
        SizeType i=irmax(d);
        //cout << "splitting\n";
        return separated(split(d,i,SplitPart::LOWER),f,b,eps) && separated(split(d,i,SplitPart::UPPER),f,b,eps);
    }
}


template<class F>
ValidatedKleenean
image_inside(const ExactBoxType& d, const F& f, const ExactBoxType& b, const RawFloatDP& eps)
{

    ExactBoxType fd=f.evaluate(d);
    ExactBoxType fc=f.evaluate(ExactBoxType(midpoint(d)));

    //cout << "called with " << d << ", having fd=" << fd << " and fc=" << fc << endl;
    if(disjoint(fc,b)) {
        //cout << "evaluation of the midpoint is disjoint\n";
        return false;
    } else if(definitely(inside(fd,b))) {
        //cout << "evaluation is definitely inside\n";
        return true;
    } else if(d.radius().upper().raw()<eps) {
        //cout << "radius limit reached\n";
        return indeterminate;
    } else {
        SizeType i=irmax(d);
        //cout << "splitting\n";
        return inside(split(d,i,SplitPart::LOWER),f,b,eps) && inside(split(d,i,SplitPart::UPPER),f,b,eps);
    }
}

template<class DS>
DS remove_subsets(const DS& ls)
{
    DS result;
    for(SizeType i=0; i!=ls.size(); ++i) {
        for(SizeType j=0; j!=ls.size(); ++j) {
            if(inside(ls[i],ls[j])) {
                break;
            }
        }
        result.adjoin(ls[i]);
    }
}

template<class DS>
DS remove_supersets(const DS& ls)
{
    DS result;
    for(SizeType i=0; i!=ls.size(); ++i) {
        for(SizeType j=0; j!=ls.size(); ++j) {
            if(inside(ls[j],ls[i])) {
                break;
            }
        }
        result.adjoin(ls[i]);
    }
}


//! \brief Tests if \a ls overlaps \a rs, to a tolerance of \a eps.
template<class T> ValidatedKleenean overlap(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const FloatDP& eps);

//! \brief Tests if \a ls is a inside of \a rs, to a tolerance of \a eps.
template<class T> ValidatedKleenean inside(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const FloatDP& eps);

//! \brief Tests if \a ls is disjoint from \a rs, to a tolerance of \a eps.
template<class T> ValidatedKleenean separated(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const FloatDP& eps);


//! \brief Tests if \a ovs overlaps \a ops, to a tolerance of \a eps.
template<class T> ValidatedLowerKleenean overlap(const OvertSetInterface<EffectiveTag,T>& ovs, const OpenSetInterface<EffectiveTag,T>& ops, const FloatDP& eps);

//! \brief Tests if \a cps is a inside of \a ops, to a tolerance of \a eps.
template<class T> ValidatedLowerKleenean inside(const CompactSetInterface<EffectiveTag,T>& cps, const OpenSetInterface<EffectiveTag,T>& ops, const FloatDP& eps);

//! \brief Tests if \a cps is disjoint from \a cls, to a tolerance of \a eps.
template<class T> ValidatedLowerKleenean separated(const CompactSetInterface<EffectiveTag,T>& cps, const ClosedSetInterface<EffectiveTag,T>& cls, const FloatDP& eps);




//! \brief Tests if the intersection of \a ls and \a bx overlaps \a rs, to a tolerance of \a eps.
template<class T> ValidatedKleenean overlap(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const ExactBoxType& bx, const FloatDP& eps);

//! \brief Tests if the intersection of \a ls and \a bx is a inside of \a rs, to a tolerance of \a eps.
template<class T> ValidatedKleenean inside(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const ExactBoxType& bx, const FloatDP& eps);

//! \brief Tests if the intersection of \a ls and \a bx is a inside of \a rs, to a tolerance of \a eps.
template<class T> ValidatedKleenean separated(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const ExactBoxType& bx, const FloatDP& eps);


//! \brief Tests if the intersection of \a ls and \a bx overlaps \a rs, to a tolerance of \a eps.
template<class T> ValidatedLowerKleenean intersection_overlap(const OvertSetInterface<EffectiveTag,T>& ls, const OpenSetInterface<EffectiveTag,T>& rs, const ExactBoxType& bx, const FloatDP& eps);

//! \brief Tests if the intersection of \a ls and \a bx is a inside of \a rs, to a tolerance of \a eps.
template<class T> ValidatedSierpinskian intersection_inside(const ClosedSetInterface<EffectiveTag,T>& ls, const OpenSetInterface<EffectiveTag,T>& rs, const ExactBoxType& bx, const FloatDP& eps);

//! \brief Tests if the intersection of \a ls and \a bx is a inside of \a rs, to a tolerance of \a eps.
template<class T> ValidatedSierpinskian intersection_separated(const ClosedSetInterface<EffectiveTag,T>& ls, const ClosedSetInterface<EffectiveTag,T>& rs, const ExactBoxType& bx, const FloatDP& eps);


} // namespace Ariadne

#endif // ARIADNE_GEOMETRY_HPP
