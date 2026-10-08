/***************************************************************************
 *            geometry/polyhedron.hpp
 *
 *  Copyright  2005-20  Alberto Casagrande, Pieter Collins
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

/*! \file geometry/polyhedron.hpp
 *  \brief Polyhedra.
 */

#ifndef ARIADNE_POLYHEDRON_HPP
#define ARIADNE_POLYHEDRON_HPP

#include <iosfwd>
#include <vector>

#include "utility/tribool.hpp"

#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"

#include "geometry/set_interface.hpp"
#include "geometry/function_set.hpp"

namespace Ariadne {

class Polytope;
class Polyhedron;




/*! \brief A polyhedron (not necessarily is_bounded polyhedral set) described by a system of linear inequalities.
 *
 *  The set is described as
 *  \f$ \{ x\in\mathbb{R}^d \mid Ax \leq b \} \f$
 *  where \f$A\f$ is a \f$n\times d\f$ matrix and \f$b\f$ is a vector of size \f$n\f$.
 */
class Polyhedron
    : public RegularSetInterface
{
  public:
    //!@{
    //! \name Constructors and destructor

    //! \brief Default constructor constructs a polytope in zero dimensions with no constraints.
    explicit Polyhedron();

    //! \brief Construct full Euclidean space of dimension \a n.
    explicit Polyhedron(DimensionType n=0u);

    //! \brief Construct a polyhedron of dimension \a d with \a nc constraints from the data in the
    //! Array beginning at \a data. The jth element of the ith constraint is stored in position i*(d+1)+j,
    //! and the ith inhomogeneous term is stored in position i*(d+1)+d.
    template<class XX> Polyhedron(DimensionType d, SizeType nc, const XX* data);

    //! \brief Construct the polyhedron defined by the matrix equations \f$Ax\leq b\f$.
    explicit Polyhedron(const Matrix<FloatDP>& A, const Vector<FloatDP>& b);

    //! \brief Convert from a box.
    explicit Polyhedron(const ExactBoxType& bx);

    //! \brief Convert from a polytope.
    explicit Polyhedron(const Polytope& p);

    //! \brief Create a dynamically-allocated copy.
    virtual Polyhedron* clone() const;

    //!@}


    //!@{
    //! \name Data access

    //! \brief The number of constraints.
    SizeType number_of_constraints() const { return this->b().size(); }
    //! \brief The matrix \f$A\f$ in the inequalities \f$Ax\leq b\f$.
    Matrix<FloatDP> A() const;
    //! \brief The vector \f$b\f$ in the inequalities \f$Ax\leq b\f$.
    Vector<FloatDP> b() const;
    //!@}


    //!@{
    //! \name Geometric operations

    //! \brief The dimension of the polyhedron.
    virtual DimensionType dimension() const;

    //! \brief Tests if the polyhedron is empty. (Not currently implemented.)
    virtual ValidatedKleenean empty() const;

    //! \brief Tests if the polyhedron is bounded. (Not currently implemented.)
    virtual ValidatedKleenean is_bounded() const;

    //! \brief Tests if the polyhedron intersects a box. (Not currently implemented.)
    virtual ValidatedKleenean overlaps(const ExactBoxType& bx) const;

    //! \brief Tests if the polyhedron is a superset of a box. (Not currently implemented.)
    virtual ValidatedKleenean covers(const ExactBoxType& bx) const;

    //! \brief Tests if the polyhedron is disjoint from a box. (Not currently implemented.)
    virtual ValidatedKleenean separated(const ExactBoxType& bx) const;

    //! \brief The \a i<sup>th</sup> defining halfspace.
    Polyhedron halfspace(SizeType i) const;

    //! \brief Convert to a polytope.
    operator Polytope () const ;

    //! \brief The intersection of two polyhedra.
    friend Polyhedron intersection(const Polyhedron& p1, const Polyhedron& p2);

    //! \brief Convert a box to a polyhedron.
    friend Polyhedron polyhedron(const ExactBoxType& p);

    //! \brief Convert a polytope to a polyhedron. (Not currently implemented)
    friend Polyhedron polyhedron(const Polytope& p);

    //! \brief Convert a polyhedron to a polytope. (Not currently implemented)
    friend Polytope polytope(const Polyhedron& p);

    //!@}

    //!@{
    //! \name Input/output.

    //! \brief Write to an output stream.
    OutputStream& _write(OutputStream& os) const;
    //!@}
  private:
    Matrix<FloatDP> _A;
    Vector<FloatDP> _b;
};


inline OutputStream& operator<<(OutputStream& os, const Polyhedron& p) {
    return p._write(os);
}


} // namespace Ariadne

#endif // ARIADNE_POLYHEDRON_HPP
