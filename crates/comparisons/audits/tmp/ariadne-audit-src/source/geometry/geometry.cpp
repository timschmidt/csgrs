/***************************************************************************
 *            geometry/geometry.cpp
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

#include "numeric/numeric.hpp"
#include "config.hpp"

#include "geometry/geometry.hpp"
#include "utility/tuple.hpp"

namespace Ariadne {


template<class T> ValidatedKleenean
separated(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const FloatDP& eps)
{
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    BasicSetType bb=cast_exact_box(ls.bounding_box());
    if(definitely(bb.is_empty())) { return true; }
    return separated(ls,rs,bb,eps);
}


template<class T> ValidatedKleenean
overlap(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const FloatDP& eps)
{
    ExactBoxType bb=cast_exact_box(ls.bounding_box());
    if(definitely(bb.is_empty())) { return false; }
    return overlap(ls,rs,bb,eps);
}


template<class T> ValidatedKleenean
inside(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const FloatDP& eps)
{
    ExactBoxType bb=cast_exact_box(ls.bounding_box());
    if(definitely(bb.is_empty())) { return true; }
    return inside(ls,rs,bb,eps);
}


template<class T> ValidatedKleenean
overlap(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const ExactBoxType& bx, const FloatDP& eps)
{
    if(definitely(ls.separated(bx))) {
        return false;
    }
    if(definitely(rs.separated(bx))) {
        return false;
    }
    else if(definitely(rs.covers(bx))) {
        return true;
    }
    else if(definitely(bx.radius()<cast_exact(eps))) {
        return indeterminate;
    } else {
        ExactBoxType bx1,bx2;
        make_lpair(bx1,bx2)=split(bx);
        if(definitely(ls.separated(bx1))) {
            return overlap(ls,rs,bx2,eps);
        } else if(definitely(ls.separated(bx2))) {
            return overlap(ls,rs,bx1,eps);
        } else {
            return overlap(ls,rs,bx1,eps) || overlap(ls,rs,bx2,eps);
        }
    }
}


template<class T> ValidatedKleenean
inside(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const ExactBoxType& bx, const FloatDP& eps)
{
    if(definitely(ls.separated(bx) || rs.separated(bx))) {
        return true;
    } else if(decide(bx.radius()<cast_exact(eps))) {
        return indeterminate;
    } else {
        ExactBoxType bx1,bx2;
        make_lpair(bx1,bx2)=split(bx);
        if(definitely(ls.separated(bx1))) {
            return inside(ls,rs,bx2,eps);
        } else if(definitely(ls.separated(bx2))) {
            return inside(ls,rs,bx1,eps);
        } else {
            return inside(ls,rs,bx1,eps) && inside(ls,rs,bx2,eps);
        }
    }
}


template<class T> ValidatedKleenean
separated(const LocatedSetInterface<EffectiveTag,T>& ls, const RegularSetInterface<EffectiveTag,T>& rs, const ExactBoxType& bx, const FloatDP& eps)
{
    if(definitely(ls.separated(bx) || rs.separated(bx))) {
        return true;
    } else if(definitely(bx.radius()<cast_exact(eps))) {
        return indeterminate;
    } else {
        ExactBoxType bx1,bx2;
        make_lpair(bx1,bx2)=split(bx);
        if(definitely(ls.separated(bx1))) {
            return separated(ls,rs,bx2,eps);
        } else if(definitely(ls.separated(bx2))) {
            return separated(ls,rs,bx1,eps);
        } else {
            return separated(ls,rs,bx1,eps) && separated(ls,rs,bx2,eps);
        }
    }
}




template<class T> ValidatedKleenean
overlap(const OvertSetInterface<EffectiveTag,T>& ovs, const OpenSetInterface<EffectiveTag,T>& ops, const ExactBoxType& bx, const FloatDP& eps)
{
    if(definitely(ovs.overlaps(bx))) {
        if(definitely(ops.covers(bx))) {
            return true;
        } else if(decide(bx.radius()<cast_exact(eps))) {
            return indeterminate;
        } else {
            ExactBoxType bx1,bx2;
            make_lpair(bx1,bx2)=split(bx);
            if(definitely(overlap(ovs,ops,bx1,eps))) {
                return true;
            } else {
                return overlap(ovs,ops,bx2,eps);
            }
        }
    } else {
        return indeterminate;
    }
}


template<class T> ValidatedKleenean
inside(const ClosedSetInterface<EffectiveTag,T>& cls, const OpenSetInterface<EffectiveTag,T>& ops, const ExactBoxType& bx, const FloatDP& eps)
{
    if(definitely(cls.separated(bx) || ops.covers(bx))) {
        return true;
    } else if(decide(bx.radius()<cast_exact(eps))) {
        return indeterminate;
    } else {
        ExactBoxType bx1,bx2;
        make_lpair(bx1,bx2)=split(bx);
        if(definitely(inside(cls,ops,bx1,eps))) {
            return inside(cls,ops,bx2,eps);
        } else {
            return indeterminate;
        }
    }
}


template<class T> ValidatedKleenean
separated(const ClosedSetInterface<EffectiveTag,T>& cls1, const ClosedSetInterface<EffectiveTag,T>& cls2, const ExactBoxType& bx, const FloatDP& eps)
{
    if(definitely(cls1.separated(bx) || cls2.separated(bx))) {
        return true;
    } else if(decide(bx.radius()<cast_exact(eps))) {
        return indeterminate;
    } else {
        ExactBoxType bx1,bx2;
        make_lpair(bx1,bx2)=split(bx);
        if(definitely(separated(cls1,cls2,bx1,eps))) {
            return separated(cls1,cls2,bx2,eps);
        } else {
            return indeterminate;
        }
    }
}


template ValidatedKleenean overlap(const LocatedSetInterface<EffectiveTag,RealVector>& ls, const RegularSetInterface<EffectiveTag,RealVector>& rs, const FloatDP& eps);
template ValidatedKleenean inside(const LocatedSetInterface<EffectiveTag,RealVector>& ls, const RegularSetInterface<EffectiveTag,RealVector>& rs, const FloatDP& eps);
template ValidatedKleenean separated(const LocatedSetInterface<EffectiveTag,RealVector>& ls, const RegularSetInterface<EffectiveTag,RealVector>& rs, const FloatDP& eps);

template ValidatedKleenean overlap(const LocatedSetInterface<EffectiveTag,RealVector>& ls, const RegularSetInterface<EffectiveTag,RealVector>& rs, const ExactBoxType& bx, const FloatDP& eps);
template ValidatedKleenean inside(const LocatedSetInterface<EffectiveTag,RealVector>& ls, const RegularSetInterface<EffectiveTag,RealVector>& rs, const ExactBoxType& bx, const FloatDP& eps);
template ValidatedKleenean separated(const LocatedSetInterface<EffectiveTag,RealVector>& ls, const RegularSetInterface<EffectiveTag,RealVector>& rs, const ExactBoxType& bx, const FloatDP& eps);


} // namespace Ariadne

