/***************************************************************************
 *            dynamics/enclosure.hpp
 *
 *  Copyright  2011-20  Pieter Collins
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

/*! \file dynamics/enclosure.hpp
 *  \brief Enclosure sets for continuous systems
 */

#ifndef ARIADNE_ENCLOSURE_HPP
#define ARIADNE_ENCLOSURE_HPP

#include <iosfwd>
#include "utility/container.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "geometry/set_interface.hpp"
#include "symbolic/identifier.hpp"
#include "io/graphics_interface.hpp"
#include "io/drawer_interface.hpp"

#include "function/function_patch.hpp"
#include "function/function_model.hpp"

#include "geometry/box.hpp"
#include "geometry/paver.hpp"

namespace Ariadne {

class ValidatedConstrainedImageSet;

template<class X, class R> class Constraint;
typedef Constraint<EffectiveScalarMultivariateFunction,EffectiveNumber> EffectiveConstraint;
typedef Constraint<ValidatedScalarMultivariateFunction,ValidatedNumber> ValidatedConstraint;

class ValidatedAffineConstrainedImageSet;
class BoundedConstraintSet;

template<class BS> class ListSet;

class Grid;
class PavingInterface;
class Storage;

template<class T> class Space;
using RealSpace = Space<Real>;

template<class IVL> class VariablesBox;
using RealVariablesBox = VariablesBox<RealInterval>;

struct Variables2d;

typedef Constraint<ValidatedScalarMultivariateFunctionPatch,FloatDPBounds> ValidatedConstraintModel;

typedef Dyadic StepSizeType;

class EnclosureConfiguration {
  private:
    ValidatedFunctionPatchFactory _function_factory;
    Paver _paver;
    SizeType _reconditioning_num_blocks;
  public:
    explicit EnclosureConfiguration(ValidatedFunctionPatchFactory function_factory, SizeType reconditioning_num_blocks = 3u);
    explicit EnclosureConfiguration(ValidatedFunctionPatchFactory::Interface const& function_factory, SizeType reconditioning_num_blocks = 3u)
        : EnclosureConfiguration(ValidatedFunctionPatchFactory(function_factory.clone()),reconditioning_num_blocks) { }
    EnclosureConfiguration(ValidatedFunctionPatchFactory function_factory, Paver paver, SizeType reconditioning_num_blocks = 3u)
        : _function_factory(function_factory), _paver(paver), _reconditioning_num_blocks(reconditioning_num_blocks) { }
    ValidatedFunctionPatchFactory const& function_factory() const { return _function_factory; }
    Paver const& paver() const { return _paver; }
    SizeType reconditioning_num_blocks() const { return _reconditioning_num_blocks; }
    EnclosureConfiguration& set_paver(Paver paver) { _paver=paver; return *this; }
    EnclosureConfiguration& set_reconditioning_num_blocks(SizeType num_blocks) { _reconditioning_num_blocks = num_blocks; return *this; }
    friend OutputStream& operator<<(OutputStream& os, EnclosureConfiguration const& ec);
};

enum class EnclosureVariableKind : std::uint8_t { INITIAL, TEMPORAL, PARAMETER, INPUT, NOISE, ERROR, UNKNOWN };
OutputStream& operator<<(OutputStream& os, const EnclosureVariableKind& vk);
List<Identifier> canonical_variable_names(const List<EnclosureVariableKind>& vks);

//! \ingroup DynamicsModule
//! \brief An enclosure for part of the reachable or evolved set of a dynamical system.
//! Defined as \f$x=f(s)\f$ for \f$s\in D\f$ satisfying \f$g(s)\leq0\f$ and \f$h(s)=0\f$.
class Enclosure
    : public Drawable2dInterface
{
    ExactBoxType _domain;
    EffectiveVectorMultivariateFunction _auxiliary_mapping;
    ValidatedVectorMultivariateFunctionPatch _state_function;
    ValidatedScalarMultivariateFunctionPatch _time_function;
    ValidatedScalarMultivariateFunctionPatch _dwell_time_function;
    List<ValidatedConstraintModel> _constraints;
    mutable ExactBoxType _reduced_domain;
    mutable Bool _is_fully_reduced;

    List<EnclosureVariableKind> _variable_kinds;

    EnclosureConfiguration _configuration;
  public:
    typedef EnclosureConfiguration ConfigurationType;
  public:
    //! \brief Construct a set with \f$D=\emptyset\f$ in \f$\mathbb{R}^0\f$.
    explicit Enclosure();
    //! \brief Construct a set with \f$D=\emptyset\f$ in \f$\mathbb{R}^0\f$.
    explicit Enclosure(EnclosureConfiguration const& config);
    //! \brief Construct a representation of the box \a bx.
    explicit Enclosure(const RealBox& bx, const EnclosureConfiguration& config);
    //! \brief Construct a representation of the box \a bx.
    explicit Enclosure(const ExactBoxType& bx, const EnclosureConfiguration& config);
    //! \brief Construct the set with parameter domain \a d and image function \a f.
    explicit Enclosure(const ExactBoxType& d, const ValidatedVectorMultivariateFunction& f, const EnclosureConfiguration& config);
    //! \brief Construct the set with parameter domain \a d, image function \a f and constraints \a c.
    explicit Enclosure(const ExactBoxType& d, const ValidatedVectorMultivariateFunction& f, const List<ValidatedConstraint>& c, const EnclosureConfiguration& config);
    //! \brief Construct the set with parameter domain \a d, image function \a sf, time function \a tf and constraints \a c.
    explicit Enclosure(const ExactBoxType& d, const ValidatedVectorMultivariateFunction& sf, const ValidatedScalarMultivariateFunction& tf, const List<ValidatedConstraint>& c, const EnclosureConfiguration& config);
    //! \brief Construct the set with domain \a d, space function \a sf, time function \a tf, negative constraints \a g and equality constraints \a h.
    //!   (Not currently implemented.)
    explicit Enclosure(const ExactBoxType& d, const ValidatedVectorMultivariateFunction& sf, const ValidatedScalarMultivariateFunction tf, const ValidatedVectorMultivariateFunction& g, const ValidatedVectorMultivariateFunction& h, const EnclosureConfiguration& fac);
    //! \brief Construct from an exact bounded constraint \a set.
    explicit Enclosure(const BoundedConstraintSet& set, const EnclosureConfiguration& fac);

    //! \brief Create a dynamically-allocated copy.
    virtual Enclosure* clone() const override;

    //! \brief The classes used to work with the set.
    const EnclosureConfiguration& configuration() const;

    //! \brief The parameter domain \f$D\f$.
    ExactBoxType domain() const;
    ExactBoxType parameter_domain() const;
    //! \brief A subset of the parameter domain containing all feasible points.
    ExactBoxType reduced_domain() const;
    //! \brief An over-approximation to the image of \f$D\f$ under \f$f\f$.
    ExactBoxType codomain() const;
    //! \brief The function giving the state \c x in terms of parameters \c s, \f$x=\xi(s)\f$.
    ValidatedVectorMultivariateFunctionPatch const& state_function() const;
    //! \brief The function giving the time \c t in terms of parameters \c s, \f$t=\tau(s)\f$.
    ValidatedScalarMultivariateFunctionPatch const& time_function() const;
    //! \brief The function giving the auxiliary variables in terms of parameters \c s.
    ValidatedVectorMultivariateFunctionPatch const  auxiliary_function() const;
    //! \brief The function giving the state and auxiliary variables in terms of parameters \c s.
    ValidatedVectorMultivariateFunctionPatch const  state_auxiliary_function() const;
    //! \brief The function giving the state, time and auxiliary variables in terms of parameters \c s.
    ValidatedVectorMultivariateFunctionPatch const  state_time_auxiliary_function() const;
    //! \brief The function giving the time since the last discrete jump in terms of the parameters \c s.
    ValidatedScalarMultivariateFunctionPatch const& dwell_time_function() const;
    //! \brief The function \c g of the constrants \f$g(s)\in C\f$.
    ValidatedVectorMultivariateFunctionPatch const  constraint_function() const;
    //! \brief The bounds \c C of the constrants \f$g(s)\in C\f$.
    ExactBoxType const constraint_bounds() const;
    //! \brief The function of parameters \a s giving the \a i<sup>th</sup> state variable.
    ValidatedScalarMultivariateFunctionPatch const  get_function(SizeType i) const;

    //! \brief Set the auxiliary function.
    Void set_auxiliary_mapping(EffectiveVectorMultivariateFunction const& aux);
    //! \brief Get the auxiliary function.
    EffectiveVectorMultivariateFunction const& auxiliary_mapping() const;


    //! \brief A list detailing the kind of variable each of the domain parameters represents.
    List<EnclosureVariableKind> const& variable_kinds() const;
    //! \brief Substitutes the expression \f$x_j=v(x_1,\ldots,x_{j-1},x_{j+1}\ldots,x_n)\f$ into the function and constraints.
    //! Requires that \f$v(D_1,\ldots,D_{j-1},D_{j+1}\ldots,D_n) \subset D_j\f$ where \f$D\f$ is the domain.
    Void substitute(SizeType j, ValidatedScalarMultivariateFunctionPatch v);
    //! \brief Substitutes the expression \f$x_j=c\f$ into the function and constraints.
    Void substitute(SizeType j, FloatDP c);

    //! \brief Set the time function to zero.
    Void clear_time();

    //! \brief Apply the map \f$r\f$ to the enclosure, obtaining \f$\phi'(s)=r(\phi(s))(x,h)\f$ and \f$\tau'(s)=\tau(s)\f$. \f$f\f$. The state variables are unchanged.
    Void apply_map(ValidatedVectorMultivariateFunction r);
    //! \brief Apply the map \f$r\f$ to the enclosure, obtaining \f$\phi'(s)=r(\phi(s))(x,h)\f$ and \f$\tau'(s)=\tau(s)\f$. \f$f\f$. The auxiliary function is updated to \a aux.
    Void apply_map(ValidatedVectorMultivariateFunction r, EffectiveVectorMultivariateFunction aux);
    //! \brief Apply the map \f$r\f$ to the enclosure, obtaining \f$\phi'(s)=r(\phi(s))(x,h)\f$ and \f$\tau'(s)=\tau(s)\f$. \f$f\f$. The state variables are unchanged. Time is increased by one.
    Void apply_discrete_time_map_step(ValidatedVectorMultivariateFunction r);
    //! \brief Apply the flow \f$\xi'(s)=\phi(\xi(s),h)\f$ and \f$\tau'(s)=\tau(s)+h\f$.
    Void apply_fixed_evolve_step(ValidatedVectorMultivariateFunction phi, StepSizeType h);
    //! \brief Apply the flow \f$xi'(s)=\phi(\xi(s),\epsilon(\xi(s)))\f$, \f$\tau'(s)=\tau(s)+\epsilon(\xi(s))\f$.
    Void apply_space_evolve_step(ValidatedVectorMultivariateFunction phi, ValidatedScalarMultivariateFunction elps);
    //! \brief Apply the flow \f$xi'(s)=\phi(\xi(s),\epsilon(\xi(s),\tau(s)))\f$, \f$\tau'(s)=\tau(s)+\epsilon(\xi(s),\tau(s))\f$.
    Void apply_spacetime_evolve_step(ValidatedVectorMultivariateFunction phi, ValidatedScalarMultivariateFunction elps);
    //! \brief Set \f$\xi'(s)=\phi(\xi(s),\epsilon(s))\f$ and \f$\tau'(s)=\tau(s)+\epsilon(s)\f$.
    Void apply_parameter_evolve_step(ValidatedVectorMultivariateFunction phi, ValidatedScalarMultivariateFunction elps);
    //! \brief Set \f$\xi'(s)=\phi(\xi(s),\omega(s)-\tau(s))\f$ and \f$\tau'(s)=\omega(s)\f$.
    Void apply_finishing_parameter_evolve_step(ValidatedVectorMultivariateFunction phi, ValidatedScalarMultivariateFunction omega);

    //! \brief Set \f$\xi'(s,r)=\phi(\xi(s),r)\f$ and \f$\tau'(s,r)=\tau(s)+r\f$ for \f$0\leq r\leq h\f$.
    Void apply_full_reach_step(ValidatedVectorMultivariateFunctionPatch phi);
    //! \brief Apply the flow \f$xi'(s,r)=\phi(\xi(s),r)\f$, \f$\tau'(s,r)=\tau(s)+r\f$, \f$0\leq r\leq\epsilon(\xi(s),\tau(s))\f$
    Void apply_spacetime_reach_step(ValidatedVectorMultivariateFunctionPatch phi, ValidatedScalarMultivariateFunction elps);
    //! \brief Set \f$\xi'(s,r)=\phi(\xi(s),r)\f$ and \f$\tau'(s,r)=\tau(s)+r\f$ for \f$0\leq r\leq\epsilon(s)\f$.
    Void apply_parameter_reach_step(ValidatedVectorMultivariateFunctionPatch phi, ValidatedScalarMultivariateFunction elps);
/*
    //! \brief Apply the flow \f$\phi(x,t)\f$ for \f$t\in[0,h]\f$
    Void apply_reach_step(ValidatedVectorMultivariateFunction phi, FloatDP h);
    //! \brief Apply the flow \f$\phi(x,t)\f$ for \f$t\in[0,\max(h,\epsilon(x))]\f$
    Void apply_reach_step(ValidatedVectorMultivariateFunction phi, ValidatedScalarMultivariateFunction elps);
*/

    //! \brief Introduces the constraint \f$c\f$ applied to the state \f$x=f(s)\f$.
    Void new_state_constraint(ValidatedConstraint c);
    //! \brief Introduces the constraint \f$c\f$ applied to the state and time \f$(x,t)\f$.
    Void new_state_time_constraint(ValidatedConstraint c);
    //! \brief Introduces the constraint \f$c\f$ applied to the parameter \f$s\f$.
    Void new_parameter_constraint(ValidatedConstraint c);

    //! \brief Introduces the constraint \f$\tau(s)\leq\gamma(\xi(s))\f$.
    Void new_state_time_bound(ValidatedScalarMultivariateFunction gamma);
    //! \brief Introduces the constraint \f$-g(\xi(s)) \leq 0\f$.
    Void new_positive_state_constraint(ValidatedScalarMultivariateFunction g);
    //! \brief Introduces the constraint \f$g(\xi(s)) \leq 0\f$.
    Void new_negative_state_constraint(ValidatedScalarMultivariateFunction g);
    //! \brief Introduces the constraint \f$h(\xi(s)) = 0\f$.
    Void new_zero_state_constraint(ValidatedScalarMultivariateFunction h);
    //! \brief Introduces the constraint \f$g(s) \leq 0\f$.
    Void new_negative_parameter_constraint(ValidatedScalarMultivariateFunction g);
    //! \brief Introduces the constraint \f$h(s) = 0\f$.
    Void new_zero_parameter_constraint(ValidatedScalarMultivariateFunction h);

    //! \brief The number of negative constraints.
    SizeType number_of_constraints() const;
    //! \brief All equality and inequality constraints.
    List<ValidatedConstraintModel> const& constraint_models() const;
    //! \brief All equality and inequality constraints.
    List<ValidatedConstraint> const constraints() const;
    //! \brief The \a i<sup>th</sup> constraint.
    ValidatedConstraintModel const& constraint(SizeType i) const;

    //! \brief  Returns true if \f$g(x)>0\f$ over the whole set,
    //! false \f$g(x)<0\f$ over the whole set,
    //! and indeterminate otherwise.
    ValidatedKleenean satisfies(ValidatedScalarMultivariateFunction g) const;
    //! \brief Tests if the set satisfies the constraint \a c. Returns \c true if all points in the set satisfy
    //! the constraint, and \c false if no points in the set satisfy the constraint.
    ValidatedKleenean satisfies(ValidatedConstraint c) const;

    //! \brief The dimension of the set.
    virtual DimensionType dimension() const override;
    //! \brief The state dimension of the set.
    DimensionType state_dimension() const;
    //! \brief The number of parameters i.e. the dimension of the parameter domain.
    SizeType number_of_parameters() const;
    //! \brief A bounding box for the set.
    UpperBoxType bounding_box() const;
    //! \brief A point in the image of the <em>unconstrained</em> parameter domain.
    Point<FloatDP> centre() const;
    //! \brief An over-approximation to the radius of the set.
    FloatDPError radius() const;
    //! \brief Returns \c true if the set is definitely bounded.
    ValidatedLowerKleenean is_bounded() const;
    //! \brief Returns \c true if the set is provably empty.
    //! May return \c false if the set can (easily) be proved to be nonempty.
    ValidatedLowerKleenean is_empty() const;
    //! \brief Returns \c true if the set can be shown to be disjoint from \a bx.
    ValidatedLowerKleenean separated(const ExactBoxType& bx) const;
    //! \brief Returns \c true if the set can be shown to be a subset of \a bx..
    ValidatedLowerKleenean inside(const ExactBoxType& bx) const;
    //! \brief Returns \c true if the set can be shown to be a subset of \a bx..
    ValidatedLowerKleenean subset(const ExactBoxType& bx) const;

    //! \brief Reduces the size of the effective parameter domain
    //! by pruning away infeasible points. Does not affect the set as a mathematical entity.
    Void reduce() const;
    //! \brief Reconditions the set to give an over-approximation with a simpler representation.
    Void recondition();
    //! \brief Simplifies the representation by changing all uniform errors into independent variables.
    Void uniform_error_recondition();
    //! \brief Simplifies the representation by choosing most significant independent variables to keep, and merging the rest into a single error for each component.
    Void kuhn_recondition();
    //! \brief Restrict the parameter domain to \a subdomain.
    //! \details May also restrict the domain of the defining function models,
    //! resulting in more accurate computations.
    Void restrict(const ExactBoxType& subdomain);
    //! \brief The set obtained by restricting to the \a subdomain.
    friend Enclosure restriction(Enclosure const&, const ExactBoxType& subdomain);

    //! \brief Compute an outer approximation on the \a grid to the given \a fineness.
    Storage outer_approximation(const Grid& grid, Nat fineness) const;
    //! \brief Adjoin an outer approximation to the given \a fineness to the \a paving.
    Void adjoin_outer_approximation_to(Storage& paving, Nat fineness) const;

    //! \brief A collection of parameter subdomains chosen to make the bounding boxes as small as possible.
    List<ExactBoxType> splitting_subdomains_zeroth_order() const;
    //! \brief Split into subsets based on the given subdomains.
    List<Enclosure> split(const List<ExactBoxType>& subdomains);

    //! \brief The direction along which the set should be split to reduce the bounding box.
    SizeType splitting_index_zeroth_order() const;
    //! \brief Split into two by splitting the parameter domain along
    //! the direction which reduces the size of the bounding box.
    Pair<Enclosure,Enclosure> split_zeroth_order() const;
    //! \brief Split into two by splitting the parameter domain along
    //! the direction which reduces the nonlinearity of the set.
    Pair<Enclosure,Enclosure> split_first_order() const;
    //! \brief Split into two by splitting the parameter domain along a suitably-chosed direction.
    //! Currently defaults to split_zeroth_order().
    Pair<Enclosure,Enclosure> split() const;
    //! \brief Split into two by splitting the parameter domain along
    //! the \a k<sup>th</sup> direction.
    Pair<Enclosure,Enclosure> split(SizeType k) const;


    ValidatedConstrainedImageSet state_set() const;
    ValidatedConstrainedImageSet state_auxiliary_set() const;
    ValidatedConstrainedImageSet state_time_auxiliary_set() const;

    //! \brief Draw to a canvas.
    virtual Void draw(CanvasInterface& c, const Projection2d& p) const override;

    //! \brief Write to an output stream.
    OutputStream& _write(OutputStream&) const;
  private:
    //! \brief Introduces a new independent variable with values in the interval \a ivl.
    //! Equivalent to constructing the set \f$S\times I\f$.
    Void _unchecked_new_variable(ExactIntervalType ivl, EnclosureVariableKind vk);
    Void _check() const;
  private:
    friend Enclosure product(const Enclosure&, const ExactIntervalType&);
    friend Enclosure product(const Enclosure&, const ExactBoxType&);
    friend Enclosure product(const Enclosure&, const Enclosure&);
};

//! \related Enclosure \brief Stream output operator.
inline OutputStream& operator<<(OutputStream& os, const Enclosure& s) { return s._write(os); }

//! \related Enclosure \brief The Cartesian product of a constrained image set with an interval in one dimension.
Enclosure product(const Enclosure& set, const ExactIntervalType& ivl);
//! \related Enclosure \brief The Cartesian product of a constrained image set with a box.
Enclosure product(const Enclosure& set, const ExactBoxType& bx);
//! \related Enclosure \brief The Cartesian product of two constrained image sets.
//! \pre The time function of each set is constant with the same value.
Enclosure product(const Enclosure& set1, const Enclosure& set2);

//! \related Enclosure \brief The image of the \a set under the \a function.
Enclosure apply(const ValidatedVectorMultivariateFunction& function, const Enclosure& set);
//! \related Enclosure \brief The image of the \a set under the \a function. Does not perform domain-checking.
Enclosure unchecked_apply(const ValidatedVectorMultivariateFunctionPatch& function, const Enclosure& set);
} // namespace Ariadne

#include "symbolic/space.hpp"
#include "symbolic/expression_set.hpp"

namespace Ariadne {

template<class S> class LabelledSet;

template<class UB> using LabelledInterval = VariableInterval<UB>;
using LabelledExactIntervalType = LabelledInterval<typename ExactIntervalType::UpperBoundType>;

template<class IVL> class LabelledBox;
using LabelledExactBoxType = LabelledBox<ExactIntervalType>;
using LabelledUpperBoxType = LabelledBox<UpperIntervalType>;
using LabelledRealBox = LabelledSet<RealBox>;

class LabelledEnclosure
    : public LabelledDrawable2dInterface, public Enclosure
{
  public:
    typedef LabelledDrawable2dInterface DrawableInterfaceType;
    typedef Box<UpperIntervalType> EuclideanBoundingBoxType;
    typedef LabelledUpperBoxType BoundingBoxType;
  public:
    LabelledEnclosure() : Enclosure() { }
    LabelledEnclosure(EnclosureConfiguration const& config) : Enclosure(config) { }
    LabelledEnclosure(LabelledRealBox const& bx, EnclosureConfiguration const& config);
    LabelledEnclosure(LabelledExactBoxType const& bx, EnclosureConfiguration const& config);
    LabelledEnclosure(ExactBoxType const& bx, RealSpace const& state_space, EnclosureConfiguration const& config);
    LabelledEnclosure(RealVariablesBox const& bx, RealSpace const& state_space, EnclosureConfiguration const& config);
    LabelledEnclosure(RealBox const& bx, RealSpace const& state_space, EnclosureConfiguration const& config);
    LabelledEnclosure(BoundedConstraintSet const& set, RealSpace const& state_space, EnclosureConfiguration const& config);
    LabelledEnclosure(Enclosure const& set, RealSpace const& state_space);
    LabelledEnclosure(Enclosure const& set, RealSpace const& state_space, RealSpace const& auxiliary_space);

    BoundingBoxType bounding_box() const;

    const RealSpace space() const;
    Enclosure const& euclidean_set() const { return *this; }
    Pair<LabelledEnclosure,LabelledEnclosure> split() const;
    Void apply_full_reach_step(ValidatedVectorMultivariateFunctionPatch phi);

    Void apply_map(ValidatedVectorMultivariateFunction const& f);
    Void apply_map(ValidatedVectorMultivariateFunction const& f, RealSpace const& new_state_space);
    Void apply_map(ValidatedVectorMultivariateFunction const& f, RealSpace const& new_state_space,
                   EffectiveVectorMultivariateFunction const& a, RealSpace const& new_auxiliary_space);

    Void set_state_space(RealSpace const&);
    const RealSpace state_space() const;
    const RealVariable time_variable() const;
    const RealSpace auxiliary_space() const;
    const RealSpace state_auxiliary_space() const;
    const RealSpace state_time_auxiliary_space() const;

    Void set_auxiliary(const RealSpace& spc, const EffectiveVectorMultivariateFunction& aux);

    friend LabelledEnclosure product(const LabelledEnclosure&, const ExactIntervalType&);

    friend LabelledEnclosure product(const LabelledEnclosure&, const LabelledExactIntervalType&);
    friend LabelledEnclosure product(const LabelledEnclosure&, const LabelledExactBoxType&);
    friend LabelledEnclosure product(const LabelledEnclosure&, const LabelledEnclosure&);

    virtual LabelledEnclosure* clone() const override;

    using Enclosure::draw;
    virtual Void draw(CanvasInterface&, const Variables2d&) const override;

    OutputStream& _write(OutputStream&) const;
  private:
    List<Identifier> _state_variables;
    List<Identifier> _auxiliary_variables;
};

//! \related Enclosure \brief Stream output operator.
inline OutputStream& operator<<(OutputStream& os, const LabelledEnclosure& s) { return s._write(os); }

using LabelledEnclosureListSet = LabelledSet<ListSet<Enclosure>>;

template<> class ListSet<LabelledEnclosure> : public LabelledDrawable2dInterface {
    using BS=LabelledEnclosure;
  private:
    List<BS> _data;
  public:
    typedef List<BS>::ConstIterator ConstIterator;
    Bool empty() const { return this->_data.empty(); }
    SizeType size() const { return this->_data.size(); }
    BS const& operator[] (SizeType i) const { return this->_data[i]; }
    ConstIterator begin() const { return this->_data.begin(); }
    ConstIterator end() const { return this->_data.end(); }

    Void adjoin(LabelledEnclosure const& other) { this->_data.append(other); }
    Void adjoin(ListSet<LabelledEnclosure> const& other) { this->_data.concatenate(other._data); }

    LabelledUpperBoxType bounding_box() const;
    const ListSet<LabelledSet<UpperBoxType>> bounding_boxes() const;

    virtual ListSet<LabelledEnclosure>* clone() const override;
    virtual Void draw(CanvasInterface&, const Variables2d&) const override;
};

} //namespace Ariadne

#endif /* ARIADNE_ENCLOSURE_HPP */
