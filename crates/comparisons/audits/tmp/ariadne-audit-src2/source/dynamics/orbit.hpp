/***************************************************************************
 *            dynamics/orbit.hpp
 *
 *  Copyright  2007-20  Pieter Collins
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

/*! \file dynamics/orbit.hpp
 *  \brief Orbits of dynamic systems
 */

#ifndef ARIADNE_ORBIT_HPP
#define ARIADNE_ORBIT_HPP

#include <utility>
#include <iostream>
#include <vector>
#include <map>
#include <memory>
#include <mutex>

#include "numeric/numeric.hpp"
#include "io/graphics_interface.hpp"
#include "geometry/function_set.hpp"
#include "geometry/list_set.hpp"
#include "geometry/curve.hpp"
#include "dynamics/enclosure.hpp"
#include "dynamics/storage.hpp"
#include "betterthreads/using.hpp"

namespace Ariadne {

#ifdef DOXYGEN
//! \brief Class for storing evolution data.
template<class E> class Orbit {
  public:
    //! The type used to store a single enclosure set.
    typedef E EnclosureType;
    //! The type used to store a list of enclosure sets.
    typedef EL EnclosureListType;
    //! The initial set of the orbit.
    EnclosureType const& initial() const;
    //! The set of points reached by evolution from the given initial set over the evolution time.
    EnclosureListType const& reach() const;
    //! The set of points reached by evolution from the given initial set at the final evolution time.
    EnclosureListType const& final() const;
};
#endif

template<class ES> class Orbit;
template<class ES> OutputStream& operator<<(OutputStream&, const Orbit<ES>&);

template<class BS> class ListSet;

template<class X> class Point;
class InterpolatedCurve;


template<class F>
class Orbit<Point<F>>
{
  public:
    Orbit(const Point<F>& pt);
    Void insert(F t, const Point<F>& pt);
    const InterpolatedCurve& curve() const { return *this->_curve; }
  private:
    std::shared_ptr<InterpolatedCurve> _curve;
};

template<class F>
class Orbit<Point<Approximation<F>>>
{
public:
    Orbit(const Point<Approximation<F>>& pt) : _curve(new InterpolatedCurve(0,pt)) { }
    Void insert(F t, const Point<Approximation<F>>& pt) { this->_curve->insert(t,pt); }
    const InterpolatedCurve& curve() const { return *this->_curve; }
private:
    std::shared_ptr<InterpolatedCurve> _curve;
};


template<class F>
class Orbit<List<Point<Approximation<F>>>>
{
  public:
    Orbit(const List<Point<Approximation<F>>>& ptLst);
    Void insert(F t, const Point<Approximation<F>>& pt, SizeType curveNumber);
    const List<InterpolatedCurve>& curves() { return *_curves; }
    const InterpolatedCurve& curve(SizeType i) { return _curves->at(i); }
  private:
    std::shared_ptr<List<InterpolatedCurve>> _curves;
};

template<class F>
class Orbit<LabelledPoint<Approximation<F>>>
    : public LabelledDrawable2dInterface
{
  public:
    Orbit(const LabelledPoint<Approximation<F>>& pt) : _curve(new LabelledInterpolatedCurve(pt)) { }
    Void insert(F t, const LabelledPoint<Approximation<F>>& pt) { this->_curve->insert(t,pt); }
    const LabelledInterpolatedCurve& curves() const { return *this->_curve; }
    virtual Void draw(CanvasInterface& canvas, const Variables2d& axes) const override { _curve->draw(canvas,axes); }
    virtual LabelledDrawable2dInterface* clone() const override { return new Orbit<LabelledPoint<Approximation<F>>>(*this); }
  private:
    std::shared_ptr<LabelledInterpolatedCurve> _curve;
};


template<class F>
class Orbit<List<LabelledPoint<Approximation<F>>>>
    : public LabelledDrawable2dInterface
{
  public:
    Orbit(const List<LabelledPoint<Approximation<F>>>& ptLst);
    Void insert(F t, const LabelledPoint<Approximation<F>>& pt, SizeType curveNumber);
    const List<LabelledInterpolatedCurve>& curves() { return *_curves; }
    const LabelledInterpolatedCurve& curve(SizeType i) { return _curves->at(i); }
    virtual Void draw(CanvasInterface& canvas, const Variables2d& axes) const override {
        for (SizeType i=0; i<_curves->size(); i++){ _curves->at(i).draw(canvas, axes); } }
    virtual LabelledDrawable2dInterface* clone() const override { return new Orbit<List<LabelledPoint<Approximation<F>>>>(*this); }
  private:
    std::shared_ptr<List<LabelledInterpolatedCurve>> _curves;
};

template<>
class Orbit<Storage>
{
    struct Data;
  public:
    typedef Storage EnclosureListType;

    Orbit(const Storage& initial);
    Orbit(const Storage& initial, const Storage& reach,
          const Storage& intermediate, const Storage& final);

    Storage const& initial() const;
    Storage const& reach() const;
    Storage const& intermediate() const;
    Storage const& final() const;
  private:
    std::shared_ptr<Data> _data;
};

struct Orbit<Storage>::Data {
    Data(const Grid& grid, const EffectiveVectorMultivariateFunction& auxiliary_mapping)
            : initial(grid,auxiliary_mapping), reach(initial), intermediate(initial), final(initial) { }
    Storage initial;
    Storage reach;
    Storage intermediate;
    Storage final;
};

template<>
class Orbit<Enclosure>
{
    typedef Enclosure ES;
    typedef ListSet<ES> ESL;
  public:
    typedef ES EnclosureType;
    typedef ListSet<ES> EnclosureListType;

    Orbit(const ES& set) : _initial(set) { }
    Void adjoin_reach(const EnclosureType& set) { this->_reach.adjoin(set); }
    Void adjoin_intermediate(const EnclosureType& set) { this->_intermediate.adjoin(set); }
    Void adjoin_final(const EnclosureType& set) { this->_final.adjoin(set); }

    Void adjoin_reach(const EnclosureListType& set) { this->_reach.adjoin(set); }
    Void adjoin_intermediate(const EnclosureListType& set) { this->_intermediate.adjoin(set); }
    Void adjoin_final(const EnclosureListType& set) { this->_final.adjoin(set); }

    EnclosureType const& initial() const { return this->_initial; }
    EnclosureListType const& reach() const { return this->_reach; }
    EnclosureListType const& intermediate() const { return this->_intermediate; }
    EnclosureListType const& final() const { return this->_final; }
  private:
    ES _initial;
    ESL _reach;
    ESL _intermediate;
    ESL _final;
};

template<>
class Orbit<LabelledEnclosure>
    : public LabelledDrawable2dInterface
{
    typedef LabelledEnclosure ES;
    typedef ListSet<LabelledEnclosure> ESL;
  public:
    typedef ES EnclosureType;
    typedef ESL EnclosureListType;

    Orbit(const ES& set) : _initial(set) { }
    virtual Void adjoin_reach(const EnclosureType& set) { this->_reach.adjoin(set); }
    virtual Void adjoin_intermediate(const EnclosureType& set) { this->_intermediate.adjoin(set); }
    virtual Void adjoin_final(const EnclosureType& set) { this->_final.adjoin(set); }

    virtual Void adjoin_reach(const EnclosureListType& set) { this->_reach.adjoin(set); }
    virtual Void adjoin_intermediate(const EnclosureListType& set) { this->_intermediate.adjoin(set); }
    virtual Void adjoin_final(const EnclosureListType& set) { this->_final.adjoin(set); }

    EnclosureType const& initial() const { return this->_initial; }
    EnclosureListType const& reach() const { return this->_reach; }
    EnclosureListType const& intermediate() const { return this->_intermediate; }
    EnclosureListType const& final() const { return this->_final; }

    virtual Void draw(CanvasInterface& canvas, const Variables2d& axes) const override { _reach.draw(canvas,axes); }
    virtual LabelledDrawable2dInterface* clone() const override { return new Orbit<LabelledEnclosure>(*this); }
  private:
    ES _initial;
    ESL _reach;
    ESL _intermediate;
    ESL _final;
};

template<class ES> OutputStream& operator<<(OutputStream& os, const Orbit< ES >& orb);

template<class ES>
OutputStream&
operator<<(OutputStream& os, const Orbit< ES >& orb)
{
    os << "Orbit(\n  initial=" << orb.initial()
       << "\n  intermediate=" << orb.intermediate()
       << "\n  reach=" << orb.reach()
       << "\n  final=" << orb.final()
       << ")\n";
    return os;
}

template<class ES> Void draw(FigureInterface& figure, const Orbit<ES>& orbit) {
    draw(figure,orbit.reach());
    draw(figure,orbit.initial());
    draw(figure,orbit.final());
}

template<class ES> FigureInterface& operator<<(FigureInterface& figure, const Orbit<ES>& orbit) {
    draw(figure,orbit); return figure;
}

} // namespace Ariadne

#endif // ARIADNE_ORBIT_HPP
