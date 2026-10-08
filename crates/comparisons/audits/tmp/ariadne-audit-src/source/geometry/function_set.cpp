/***************************************************************************
 *            geometry/function_set.cpp
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

#include "function/functional.hpp"
#include "config.hpp"

#include "utility/macros.hpp"
#include "conclog/logging.hpp"
#include "function/polynomial.hpp"
#include "function/function.hpp"
#include "function/taylor_function.hpp"
#include "function/formula.hpp"
#include "function/procedure.hpp"
#include "solvers/nonlinear_programming.hpp"
#include "solvers/constraint_solver.hpp"
#include "geometry/function_set.hpp"
#include "geometry/affine_set.hpp"
#include "geometry/grid_paving.hpp"
#include "geometry/paver.hpp"
#include "algebra/algebra.hpp"
#include "algebra/expansion.inl.hpp"

#include "io/graphics_interface.hpp"
#include "io/graphics_manager.hpp"
#include "io/drawer.hpp"

using namespace ConcLog;

namespace Ariadne {

//! \related TaylorConstrainedImageSet \brief The possible types of method used to discretise a nonlinear set.
enum class DiscretisationMethod : std::uint8_t { SUBDIVISION, AFFINE, CONSTRAINT };
//! \related TaylorConstrainedImageSet \brief The type of method currently used to discretise a nonlinear set.
//! HACK: May be replaced by more advanced functionality in the future.
extern DiscretisationMethod DISCRETISATION_METHOD;

DiscretisationMethod DISCRETISATION_METHOD=DiscretisationMethod::SUBDIVISION;
uint DRAWING_ACCURACY=1u;

template<class T> StringType str(const T& t) { StringStream ss; ss<<t; return ss.str(); }

Matrix<FloatDPError> nonlinearities_zeroth_order(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& dom);
Matrix<FloatDPError> nonlinearities_zeroth_order(const ValidatedVectorMultivariateTaylorFunctionModelDP& f, const ExactBoxType& dom);
Matrix<FloatDPError> nonlinearities_first_order(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& dom);
Matrix<FloatDPError> nonlinearities_second_order(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& dom);
Pair<SizeType,FloatDPError> nonlinearity_index_and_error(const ValidatedVectorMultivariateFunction& function, const ExactBoxType& domain);
Pair<SizeType,FloatDPError> nonlinearity_index_and_error(const ValidatedVectorMultivariateTaylorFunctionModelDP& function, const ExactBoxType domain);
Pair<SizeType,FloatDPError> lipschitz_index_and_error(const ValidatedVectorMultivariateFunction& function, const ExactBoxType& domain);

Interval<ValidatedUpperNumber> make_interval(ValidatedLowerNumber const& lb, ValidatedUpperNumber const& ub);
ExactIntervalType over_approximation(Interval<ValidatedUpperNumber> const& ivl);
ExactBoxType under_approximation(const RealBox& rbx);
ExactBoxType over_approximation(const RealBox& rbx);
ExactBoxType approximation(const RealBox& rbx);



Matrix<FloatDPError> nonlinearities_zeroth_order(const ValidatedVectorMultivariateTaylorFunctionModelDP& f, const ExactBoxType& dom)
{
    const SizeType m=f.result_size();
    const SizeType n=f.argument_size();
    ValidatedVectorMultivariateTaylorFunctionModelDP g=restriction(f,dom);

    Matrix<FloatDPError> nonlinearities=Matrix<FloatDPError>::zero(m,n,dp);
    MultiIndex a;
    for(SizeType i=0; i!=m; ++i) {
        const ValidatedTaylorModelDP& tm=g.model(i);
        for(ValidatedTaylorModelDP::ConstIterator iter=tm.begin(); iter!=tm.end(); ++iter) {
            a=iter->index();
            if(a.degree()>1) {
                for(SizeType j=0; j!=n; ++j) {
                    if(a[j]>0) { nonlinearities[i][j]+=mag(iter->coefficient()); }
                }
            }
        }
    }

    return nonlinearities;
}

Matrix<FloatDPError> nonlinearities_first_order(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& dom)
{
    //std::cerr<<"\n\nf="<<f<<"\n";
    //std::cerr<<"dom="<<dom<<"\n";
    const SizeType m=f.result_size();
    const SizeType n=f.argument_size();
    Vector<Differential<FloatDPUpperInterval>> ivl_dx=Differential<FloatDPUpperInterval>::constants(m,n, 1, cast_vector(dom));
    MultiIndex a(n);
    for(SizeType i=0; i!=n; ++i) {
        FloatDPBounds sf=dom[i].radius();
        ++a[i];
        ivl_dx[i].expansion().append(a,UpperIntervalType(sf));
        --a[i];
    }
    //std::cerr<<"dx="<<ivl_dx<<"\n";
    Vector<Differential<FloatDPUpperInterval>> df=derivative_range(f,ivl_dx);
    //std::cerr<<"df="<<df<<"\n";

    Matrix<FloatDPError> nonlinearities=Matrix<FloatDPError>::zero(m,n,dp);
    for(SizeType i=0; i!=m; ++i) {
        const Differential<FloatDPUpperInterval>& d=df[i];
        for(Differential<FloatDPUpperInterval>::ConstIterator iter=d.begin(); iter!=d.end(); ++iter) {
            a=iter->index();
            if(a.degree()==1) {
                for(SizeType j=0; j!=n; ++j) {
                    if(a[j]>0) { nonlinearities[i][j]+=iter->coefficient().radius(); }
                }
            }
        }
    }
    //std::cerr<<"nonlinearities="<<nonlinearities<<"\n";

    return nonlinearities;
}

Matrix<FloatDPError> nonlinearities_second_order(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& dom)
{
    //std::cerr<<"\n\nf="<<f<<"\n";
    //std::cerr<<"dom="<<dom<<"\n";
    const SizeType m=f.result_size();
    const SizeType n=f.argument_size();
    Vector<Differential<FloatDPUpperInterval>> ivl_dx=Differential<FloatDPUpperInterval>::constants(m,n, 2, cast_vector(dom));
    MultiIndex a(n);
    for(SizeType i=0; i!=n; ++i) {
        FloatDPBounds sf=dom[i].radius();
        ++a[i];
        ivl_dx[i].expansion().append(a,FloatDPUpperInterval(sf));
        --a[i];
    }
    //std::cerr<<"dx="<<ivl_dx<<"\n";
    Vector<Differential<FloatDPUpperInterval>> df=derivative_range(f,ivl_dx);
    //std::cerr<<"df="<<df<<"\n";

    Matrix<FloatDPError> nonlinearities=Matrix<FloatDPError>::zero(m,n,dp);
    for(SizeType i=0; i!=m; ++i) {
        const Differential<FloatDPUpperInterval>& d=df[i];
        for(Differential<FloatDPUpperInterval>::ConstIterator iter=d.begin(); iter!=d.end(); ++iter) {
            a=iter->index();
            if(a.degree()==2) {
                for(SizeType j=0; j!=n; ++j) {
                    if(a[j]>0) { nonlinearities[i][j]+=mag(iter->coefficient()); }
                }
            }
        }
    }
    //std::cerr<<"nonlinearities="<<nonlinearities<<"\n";

    return nonlinearities;
}

Pair<SizeType,FloatDPError> nonlinearity_index_and_error(const ValidatedVectorMultivariateTaylorFunctionModelDP& function, const ExactBoxType domain) {
    Matrix<FloatDPError> nonlinearities=Ariadne::nonlinearities_zeroth_order(function,domain);

    // Compute the row of the nonlinearities Array which has the highest norm
    // i.e. the highest sum of $mag(a_ij)$ where mag([l,u])=max(|l|,|u|)
    SizeType jmax_in_row_imax=nonlinearities.column_size();
    FloatDPError max_row_sum(0u,dp);
    for(SizeType i=0; i!=nonlinearities.row_size(); ++i) {
        SizeType jmax=nonlinearities.column_size();
        FloatDPError row_sum(0u,dp);
        FloatDPError max_mag_j_in_i(0u,dp);
        for(SizeType j=0; j!=nonlinearities.column_size(); ++j) {
            row_sum+=abs(nonlinearities[i][j]);
            if(abs(nonlinearities[i][j]).raw()>max_mag_j_in_i.raw()) {
                jmax=j;
                max_mag_j_in_i=abs(nonlinearities[i][j]);
            }
        }
        if(row_sum.raw()>max_row_sum.raw()) {
            max_row_sum=row_sum;
            jmax_in_row_imax=jmax;
        }
    }

    return make_pair(jmax_in_row_imax,max_row_sum);
}

ApproximateNumber max(ValidatedLowerNumber const&, ValidatedUpperNumber const&);

Interval<ValidatedUpperNumber> make_interval(ValidatedLowerNumber const& lb, ValidatedUpperNumber const& ub) {
    return Interval<ValidatedUpperNumber>(lb,ub);
}

ExactIntervalType over_approximation(Interval<ValidatedUpperNumber> const& ivl) {
    return cast_exact_interval(UpperIntervalType(ivl,double_precision));
}

ExactBoxType under_approximation(const RealBox& rbx) {
    DoublePrecision prec;
    return cast_exact_box(LowerBoxType(rbx,prec));
}

ExactBoxType over_approximation(const RealBox& rbx) {
    DoublePrecision prec;
    return cast_exact_box(UpperBoxType(rbx,prec));
}

ExactBoxType approximation(const RealBox& rbx) {
    DoublePrecision prec;
    return cast_exact_box(ApproximateBoxType(rbx,prec));
}


namespace {

SizeType get_argument_size(const List<EffectiveConstraint>& c) {
    SizeType as = ( c.size()==0 ? 0 : c[0].function().argument_size() );
    for(SizeType i=0; i!=c.size(); ++i) {
        ARIADNE_ASSERT_MSG(c[i].function().argument_size()==as,"c="<<c);
    }
    return as;
}

template<class P, class D, class C> VectorMultivariateFunction<P> make_constraint_function(D dom, const List<C>& c) {
    VectorMultivariateFunction<P> f(c.size(),dom);
    for(SizeType i=0; i!=c.size(); ++i) {
        //f[i]=c[i].function();
        f.set(i,c[i].function());
    }
    return f;
}

EffectiveVectorMultivariateFunction constraint_function(EuclideanDomain dom, const List<EffectiveConstraint>& c) {
    return make_constraint_function<EffectiveTag>(dom,c);
}

ValidatedVectorMultivariateFunction constraint_function(BoxDomainType dom, const List<ValidatedConstraint>& c) {
    return make_constraint_function<ValidatedTag>(EuclideanDomain(dom.dimension()),c);
}


RealBox constraint_bounds(const List<EffectiveConstraint>& c) {
    RealBox b(c.size());
    for(SizeType i=0; i!=c.size(); ++i) {
        b[i]=RealInterval(c[i].lower_bound(),c[i].upper_bound());
    }
    return b;
}

List<EffectiveConstraint> constraints(const EffectiveVectorMultivariateFunction& f, const RealBox& b) {
    ARIADNE_ASSERT(f.result_size()==b.size());
    List<EffectiveConstraint> c; c.reserve(b.size());
    for(SizeType i=0; i!=b.size(); ++i) {
        c.append(EffectiveConstraint(b[i].lower_bound(),f[i],b[i].upper_bound()));
    }
    return c;
}

} //namespace

namespace Detail {

template<class OP, class... ARGS> struct LogicalExpression;

template<class OP, class ARG1, class ARG2> struct LogicalExpression<OP,ARG1,ARG2>
    : virtual LogicalInterface, Symbolic<OP,ARG1,ARG2>
{
    using Symbolic<OP,ARG1,ARG2>::Symbolic;
    virtual LogicalInterface* _copy() const { return new LogicalExpression<OP,ARG1,ARG2>(*this); }
    virtual LogicalValue _check(Effort eff) const { return this->_op(this->_arg1,this->_arg2,eff).repr(); }
    virtual OutputStream& _write(OutputStream& os) const { return os << static_cast<Symbolic<OP,ARG1,ARG2>const&>(*this); }
};

} // namespace Detail

template<class OP, class ARG1, class ARG2> decltype(auto) make_logical_expression_handle(OP const& op, ARG1 const& arg1, ARG2 const& arg2) {
    return LogicalHandle(std::make_shared<Detail::LogicalExpression<OP,ARG1,ARG2>>(op,arg1,arg2)); }

ConstraintSet::ConstraintSet(const EffectiveVectorMultivariateFunction& f, const RealBox& b)
    : _dimension(f.argument_size()), _constraints()
{
    this->_constraints=Ariadne::constraints(f,b);
}

ConstraintSet::ConstraintSet(const List<EffectiveConstraint>& c)
    : _dimension(get_argument_size(c)), _constraints(c)
{
}

EffectiveVectorMultivariateFunction const ConstraintSet::constraint_function() const
{
    return Ariadne::constraint_function(EuclideanDomain(this->dimension()),this->constraints());
}

RealBox const ConstraintSet::constraint_bounds() const
{
    return Ariadne::constraint_bounds(this->constraints());
}

ConstraintSet*
ConstraintSet::clone() const
{
    return new ConstraintSet(*this);
}


DimensionType
ConstraintSet::dimension() const
{
    return this->_dimension;
}


LowerKleenean
ConstraintSet::separated(const ExactBoxType& bx) const
{
    return LowerKleenean(make_logical_expression_handle(Separated(),*this,bx));
}

LowerKleenean
ConstraintSet::overlaps(const ExactBoxType& bx) const
{
    return LowerKleenean(make_logical_expression_handle(Overlap(),*this,bx));
}

LowerKleenean
ConstraintSet::covers(const ExactBoxType& bx) const
{
    return LowerKleenean(make_logical_expression_handle(Covers(),*this,bx));
}

ValidatedLowerKleenean
ConstraintSet::separated(const ExactBoxType& bx, Effort eff) const
{
    ExactBoxType codomain=over_approximation(this->codomain());
    return ValidatedConstrainedImageSet(bx,this->constraint_function()).separated(codomain);
}

ValidatedLowerKleenean
ConstraintSet::overlaps(const ExactBoxType& bx, Effort eff) const
{
    ExactBoxType codomain=under_approximation(this->codomain());
    return ValidatedConstrainedImageSet(bx,this->constraint_function()).overlaps(codomain);
}

ValidatedLowerKleenean
ConstraintSet::covers(const ExactBoxType& bx, Effort eff) const
{
    ExactBoxType codomain=under_approximation(this->codomain());
    return UpperBoxType(apply(this->constraint_function(),bx)).inside(codomain);
}


OutputStream&
ConstraintSet::_write(OutputStream& os) const
{
    return os << "ConstraintSet( constraints=" << this->constraints() << " )";
}

ConstraintSet
intersection(const ConstraintSet& cs1,const ConstraintSet& cs2)
{
    return ConstraintSet(catenate(cs1.constraints(),cs2.constraints()));
}

OutputStream& operator<<(OutputStream& os, ConstraintSet const& cs) {
    return cs._write(os);
}


BoundedConstraintSet::BoundedConstraintSet(const RealBox& bx)
    : _domain(bx), _constraints()
{
}

BoundedConstraintSet::BoundedConstraintSet(const RealBox& d, const EffectiveVectorMultivariateFunction& f, const RealBox& b)
    : _domain(d), _constraints(Ariadne::constraints(f,b))
{
    ARIADNE_ASSERT(b.size()==f.result_size());
    ARIADNE_ASSERT(d.size()==f.argument_size());
}

BoundedConstraintSet::BoundedConstraintSet(const RealBox& d, const List<EffectiveConstraint>& c)
    : _domain(d), _constraints(c)
{
}

EffectiveVectorMultivariateFunction const BoundedConstraintSet::constraint_function() const
{
    return Ariadne::constraint_function(EuclideanDomain(this->dimension()),this->constraints());
}

RealBox const BoundedConstraintSet::constraint_bounds() const
{
    return Ariadne::constraint_bounds(this->constraints());
}

BoundedConstraintSet*
BoundedConstraintSet::clone() const
{
    return new BoundedConstraintSet(*this);
}


DimensionType
BoundedConstraintSet::dimension() const
{
    return this->_domain.size();
}


LowerKleenean
BoundedConstraintSet::separated(const ExactBoxType& bx) const
{
    return LowerKleenean(make_logical_expression_handle(Separated(),*this,bx));
}

LowerKleenean
BoundedConstraintSet::overlaps(const ExactBoxType& bx) const
{
    return LowerKleenean(make_logical_expression_handle(Overlap(),*this,bx));
}

LowerKleenean
BoundedConstraintSet::covers(const ExactBoxType& bx) const
{
    return LowerKleenean(make_logical_expression_handle(Covers(),*this,bx));
}

LowerKleenean
BoundedConstraintSet::inside(const ExactBoxType& bx) const
{
    return LowerKleenean(make_logical_expression_handle(Inside(),*this,bx));
}

ValidatedLowerKleenean
BoundedConstraintSet::separated(const ExactBoxType& bx, Effort eff) const
{
    ExactBoxType domain=over_approximation(this->domain());
    if(Ariadne::disjoint(domain,bx)) { return true; }
    ExactBoxType codomain=over_approximation(this->codomain());
    return ValidatedConstrainedImageSet(Ariadne::intersection(bx,domain),this->constraint_function()).separated(codomain);
}


ValidatedLowerKleenean
BoundedConstraintSet::overlaps(const ExactBoxType& bx, Effort eff) const
{
    if(Ariadne::disjoint(over_approximation(this->domain()),bx)) { return false; }
    if(this->codomain().dimension() == 0 && Ariadne::intersect(under_approximation(this->domain()),bx)) { return true; }
    ExactBoxType domain=under_approximation(this->domain());
    ExactBoxType codomain=under_approximation(this->codomain());
    return ValidatedConstrainedImageSet(Ariadne::intersection(bx,domain),this->constraint_function()).overlaps(codomain);
}


ValidatedLowerKleenean
BoundedConstraintSet::covers(const ExactBoxType& bx, Effort eff) const
{
    ExactBoxType domain=under_approximation(this->domain());
    ExactBoxType codomain=under_approximation(this->codomain());
    if(!Ariadne::covers(domain,bx)) { return false; }
    return UpperBoxType(apply(this->constraint_function(),bx)).inside(codomain);
}

ValidatedLowerKleenean
BoundedConstraintSet::inside(const ExactBoxType& bx, Effort eff) const
{
    return Ariadne::inside(UpperBoxType(over_approximation(this->domain())),bx);
}

UpperBoxType
BoundedConstraintSet::bounding_box() const
{
    ExactBoxType result=over_approximation(this->_domain);
    return widen(result);
}


OutputStream&
BoundedConstraintSet::_write(OutputStream& os) const
{
    return os << "BoundedConstraintSet( domain=" << this->domain() << ", constraints=" << this->constraints() << ")";
}

Void
BoundedConstraintSet::draw(CanvasInterface& c, const Projection2d& p) const
{
    return ConstrainedImageSet(*this).draw(c,p);
}


BoundedConstraintSet
intersection(const ConstraintSet& cs,const RealBox& bx)
{
    return BoundedConstraintSet(bx,cs.constraints());
}

BoundedConstraintSet
intersection(const RealBox& bx,const ConstraintSet& cs)
{
    return intersection(cs,bx);
}

BoundedConstraintSet
intersection(const BoundedConstraintSet& bcs1,const BoundedConstraintSet& bcs2)
{
    return BoundedConstraintSet(intersection(bcs1.constraint_bounds(),bcs2.constraint_bounds()),catenate(bcs1.constraints(),bcs2.constraints()));
}

BoundedConstraintSet
intersection(const BoundedConstraintSet& bcs,const RealBox& bx)
{
    return BoundedConstraintSet(intersection(bcs.constraint_bounds(),bx),bcs.constraints());
}

BoundedConstraintSet
intersection(const RealBox& bx,const BoundedConstraintSet& bcs)
{
    return intersection(bcs,bx);
}

BoundedConstraintSet
intersection(const BoundedConstraintSet& bcs,const ConstraintSet& cs)
{
    return BoundedConstraintSet(bcs.constraint_bounds(),catenate(bcs.constraints(),cs.constraints()));
}

BoundedConstraintSet
intersection(const ConstraintSet& cs,const BoundedConstraintSet& bcs)
{
    return intersection(bcs,cs);
}

OutputStream& operator<<(OutputStream& os, BoundedConstraintSet const& bcs) {
    return bcs._write(os);
}







ConstrainedImageSet::ConstrainedImageSet(const BoundedConstraintSet& set)
    : _domain(over_approximation(set.domain())), _function(EffectiveVectorMultivariateFunction::identity(set.dimension()))
{
    for(SizeType i=0; i!=set.number_of_constraints(); ++i) {
        this->new_parameter_constraint(set.constraint(i));
    }
}


const EffectiveVectorMultivariateFunction ConstrainedImageSet::constraint_function() const
{
    return Ariadne::constraint_function(this->function().domain(),this->constraints());
}

const RealBox ConstrainedImageSet::constraint_bounds() const
{
    RealBox result(this->number_of_constraints());
    for(SizeType i=0; i!=this->number_of_constraints(); ++i) {
        result[i]=RealInterval(this->constraint(i).lower_bound(),this->constraint(i).upper_bound());
    }
    return result;
}


UpperBoxType ConstrainedImageSet::bounding_box() const
{
    return Ariadne::apply(this->_function,over_approximation(this->_domain));
}

inline Matrix<FloatDP> cast_exact(Matrix<FloatDPBounds> vA) {
    Matrix<FloatDPApproximation> aA=vA;
    return reinterpret_cast<Matrix<FloatDP>&>(aA);
}

ValidatedAffineConstrainedImageSet
ConstrainedImageSet::affine_approximation() const
{
    DoublePrecision prec;
    const Box<ExactIntervalType> D=approximation(this->domain());
    Vector<FloatDP> m=midpoint(D);
    Matrix<FloatDP> G=cast_exact(jacobian(this->_function,m));
    Vector<FloatDP> h=cast_exact(this->_function.evaluate(m)-G*m);
    ValidatedAffineConstrainedImageSet result(D,G,h);

    for(List<EffectiveConstraint>::ConstIterator iter=this->_constraints.begin();
        iter!=this->_constraints.end(); ++iter)
    {
        ValidatedAffineModelDP a=affine_model(D,iter->function(),prec);
        ExactIntervalType b=iter->bounds();
        result.new_constraint(ValidatedAffineModelConstraintDP(b.lower_bound(),a,b.upper_bound()));
    }

    return result;
}


ValidatedKleenean ConstrainedImageSet::satisfies(const EffectiveConstraint& nc, Effort eff) const
{
    if( definitely(subset(Ariadne::apply(nc.function(),this->bounding_box()),nc.bounds())) ) {
        return true;
    }

    ConstraintSolver solver;
    const RealBox& domain=this->_domain;
    List<EffectiveConstraint> all_constraints=this->_constraints;
    EffectiveScalarMultivariateFunction composed_function = compose(nc.function(),this->_function);
    const EffectiveNumber& lower_bound = nc.lower_bound();
    const EffectiveNumber& upper_bound = nc.upper_bound();

    ValidatedKleenean result;
    if(definitely(upper_bound<+infinity)) {
        all_constraints.append( composed_function >= upper_bound );
        result=solver.feasible(over_approximation(domain),all_constraints).first;
        all_constraints.pop_back();
        if(definitely(result)) { return false; }
    }
    if(definitely(lower_bound>-infinity)) {
        all_constraints.append(composed_function <= lower_bound);
        result = result || solver.feasible(over_approximation(domain),all_constraints).first;
    }
    return !result;
}


LowerKleenean
ConstrainedImageSet::inside(const ExactBoxType& bx) const
{
    return LowerKleenean(make_logical_expression_handle(Inside(),*this,bx));
}

LowerKleenean
ConstrainedImageSet::separated(const ExactBoxType& bx) const
{
    return LowerKleenean(make_logical_expression_handle(Separated(),*this,bx));
}

LowerKleenean
ConstrainedImageSet::overlaps(const ExactBoxType& bx) const
{
    return LowerKleenean(make_logical_expression_handle(Overlap(),*this,bx));
}

//! \brief Test if the set is contained in (the interior of) a box.
ValidatedLowerKleenean ConstrainedImageSet::inside(const ExactBoxType& bx, Effort eff) const {
    return this->bounding_box().inside(bx);
}

ValidatedLowerKleenean ConstrainedImageSet::separated(const ExactBoxType& bx, Effort eff) const
{
    UpperBoxType subdomain = over_approximation(this->_domain);
    EffectiveVectorMultivariateFunction function = join(this->function(),this->constraint_function());
    ExactBoxType codomain = product(bx,ExactBoxType(over_approximation(this->constraint_bounds())));
    ConstraintSolver solver;
    solver.reduce(subdomain,function,codomain);
    return subdomain.is_empty();
}

ValidatedLowerKleenean ConstrainedImageSet::overlaps(const ExactBoxType& bx, Effort eff) const
{
    return ValidatedConstrainedImageSet(under_approximation(this->_domain),this->_function,this->_constraints).overlaps(bx);
}


Void
ConstrainedImageSet::adjoin_outer_approximation_to(PavingInterface& paving, Nat fineness) const
{
    ValidatedConstrainedImageSet set(over_approximation(this->domain()),this->function(),this->constraints());
    return set.adjoin_outer_approximation_to(paving,fineness);
}


Pair<ConstrainedImageSet,ConstrainedImageSet>
ConstrainedImageSet::split() const
{
    ARIADNE_ASSERT(this->number_of_parameters()>0);
    auto rmax(this->domain()[0].radius());
    SizeType k=0;
    for(SizeType j=1; j!=this->number_of_parameters(); ++j) {
       auto rj(this->domain()[j].radius());
       if(decide(rj>rmax)) {
            k=j;
            rmax=rj;
        }
    }
    return this->split(k);
}

Pair<ConstrainedImageSet,ConstrainedImageSet>
ConstrainedImageSet::split(SizeType j) const
{
    RealInterval interval = this->domain()[j];
    Real midpoint = interval.midpoint();
    Pair<RealBox,RealBox> subdomains(this->domain(),this->domain());
    subdomains.first[j]=RealInterval(interval.lower_bound(),midpoint);
    subdomains.second[j]=RealInterval(midpoint,interval.upper_bound());
    return make_pair(ConstrainedImageSet(subdomains.first,this->_function,this->_constraints),
                     ConstrainedImageSet(subdomains.second,this->_function,this->_constraints));
}


ConstrainedImageSet image(const BoundedConstraintSet& set, const EffectiveVectorMultivariateFunction& function) {
    ARIADNE_ASSERT(set.dimension()==function.argument_size());
    ConstrainedImageSet result(set.domain(),function);
    for(SizeType i=0; i!=set.number_of_constraints(); ++i) {
        result.new_parameter_constraint(set.constraint(i));
    }
    return result;
}

ConstrainedImageSet image(ConstrainedImageSet set, const EffectiveVectorMultivariateFunction& function) {
    set.apply(function); return set;
}

OutputStream& operator<<(OutputStream& os, ConstrainedImageSet const& set) {
    return set._write(os);
}


Matrix<FloatDPError> nonlinearities_zeroth_order(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& dom)
{
    return nonlinearities_zeroth_order(dynamic_cast<const ValidatedVectorMultivariateTaylorFunctionModelDP&>(*f.raw_pointer()),dom);
}

/*
Matrix<FloatDP> nonlinearities_first_order(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& dom)
{
    //std::cerr<<"\n\nf="<<f<<"\n";
    //std::cerr<<"dom="<<dom<<"\n";
    const SizeType m=f.result_size();
    const SizeType n=f.argument_size();
    Vector<UpperIntervalDifferentialType> ivl_dx=UpperIntervalDifferentialType::constants(m,n, 1, dom);
    MultiIndex a(n);
    for(SizeType i=0; i!=n; ++i) {
        FloatDP sf=dom[i].radius();
        ++a[i];
        ivl_dx[i].expansion().append(a,ExactIntervalType(sf));
        --a[i];
    }
    //std::cerr<<"dx="<<ivl_dx<<"\n";
    Vector<UpperIntervalDifferentialType> df=f.evaluate(ivl_dx);
    //std::cerr<<"df="<<df<<"\n";

    Matrix<FloatDP> nonlinearities=Matrix<FloatDP>::zero(m,n);
    for(SizeType i=0; i!=m; ++i) {
        const UpperIntervalDifferentialType& d=df[i];
        for(UpperIntervalDifferentialType::ConstIterator iter=d.begin(); iter!=d.end(); ++iter) {
            a=iter->index();
            if(a.degree()==1) {
                for(SizeType j=0; j!=n; ++j) {
                    if(a[j]>0) { nonlinearities[i][j]+=radius(iter->coefficient()); }
                }
            }
        }
    }
    //std::cerr<<"nonlinearities="<<nonlinearities<<"\n";

    return nonlinearities;
}

Matrix<FloatDP> nonlinearities_second_order(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& dom)
{
    //std::cerr<<"\n\nf="<<f<<"\n";
    //std::cerr<<"dom="<<dom<<"\n";
    const SizeType m=f.result_size();
    const SizeType n=f.argument_size();
    Vector<UpperIntervalDifferentialType> ivl_dx=UpperIntervalDifferentialType::constants(m,n, 2, dom);
    MultiIndex a(n);
    for(SizeType i=0; i!=n; ++i) {
        FloatDP sf=dom[i].radius();
        ++a[i];
        ivl_dx[i].expansion().append(a,ExactIntervalType(sf));
        --a[i];
    }
    //std::cerr<<"dx="<<ivl_dx<<"\n";
    Vector<UpperIntervalDifferentialType> df=f.evaluate(ivl_dx);
    //std::cerr<<"df="<<df<<"\n";

    Matrix<FloatDP> nonlinearities=Matrix<FloatDP>::zero(m,n);
    for(SizeType i=0; i!=m; ++i) {
        const UpperIntervalDifferentialType& d=df[i];
        for(UpperIntervalDifferentialType::ConstIterator iter=d.begin(); iter!=d.end(); ++iter) {
            a=iter->index();
            if(a.degree()==2) {
                for(SizeType j=0; j!=n; ++j) {
                    if(a[j]>0) { nonlinearities[i][j]+=mag(iter->coefficient()); }
                }
            }
        }
    }
    //std::cerr<<"nonlinearities="<<nonlinearities<<"\n";

    return nonlinearities;
}
*/

Pair<SizeType,FloatDPError> lipschitz_index_and_error(const ValidatedVectorMultivariateFunction& function, const ExactBoxType& domain)
{
    Matrix<UpperIntervalType> jacobian=Ariadne::jacobian_range(function,cast_vector(domain));

    // Compute the column of the matrix which has the norm
    // i.e. the highest sum of $mag(a_ij)$ where mag([l,u])=max(|l|,|u|)
    SizeType jmax=domain.size();
    FloatDPError max_column_norm(0u,dp);
    for(SizeType j=0; j!=domain.size(); ++j) {
        FloatDPError column_norm(0u,dp);
        for(SizeType i=0; i!=function.result_size(); ++i) {
            column_norm+=mag(jacobian[i][j]);
        }
        column_norm *= domain[j].radius().upper();
        if(column_norm.raw()>max_column_norm.raw()) {
            max_column_norm=column_norm;
            jmax=j;
        }
    }
    return make_pair(jmax,max_column_norm);
}

Pair<SizeType,FloatDPError> nonlinearity_index_and_error(const ValidatedVectorMultivariateFunction& function, const ExactBoxType& domain)
{
    Matrix<FloatDPError> nonlinearities=Ariadne::nonlinearities_zeroth_order(function,domain);

    // Compute the row of the nonlinearities Array which has the highest norm
    // i.e. the highest sum of $mag(a_ij)$ where mag([l,u])=max(|l|,|u|)
    SizeType jmax_in_row_imax=nonlinearities.column_size();
    FloatDPError max_row_sum(0u,dp);
    for(SizeType i=0; i!=nonlinearities.row_size(); ++i) {
        SizeType jmax=nonlinearities.column_size();
        FloatDPError row_sum(0u,dp);
        FloatDPError max_mag_j_in_i(0u,dp);
        for(SizeType j=0; j!=nonlinearities.column_size(); ++j) {
            row_sum+=nonlinearities[i][j];
            if(nonlinearities[i][j].raw()>max_mag_j_in_i.raw()) {
                jmax=j;
                max_mag_j_in_i=mag(nonlinearities[i][j]);
            }
        }
        if(row_sum.raw()>max_row_sum.raw()) {
            max_row_sum=row_sum;
            jmax_in_row_imax=jmax;
        }
    }

    return make_pair(jmax_in_row_imax,max_row_sum);
}








Void
ConstrainedImageSet::draw(CanvasInterface& cnvs, const Projection2d& proj) const
{
    ValidatedConstrainedImageSet(approximation(this->_domain),this->_function,this->_constraints).draw(cnvs,proj);
}



OutputStream&
ConstrainedImageSet::_write(OutputStream& os) const
{
    return os << "ConstrainedImageSet( domain=" << this->_domain
              << ", function=" << this->_function << ", constraints=" << this->_constraints << " )";
}




template<class SF> struct FunctionTraits;
template<class X> struct FunctionTraits< ScalarMultivariateFunction<X> > { typedef VectorMultivariateFunction<X> VectorMultivariateFunctionType; };
template<> struct FunctionTraits< ValidatedScalarMultivariateTaylorFunctionModelDP > { typedef ValidatedVectorMultivariateTaylorFunctionModelDP VectorMultivariateFunctionType; };

template<class SF> class TemplatedConstraintSet;
template<class SF> class TemplatedConstrainedImageSet;



ValidatedVectorMultivariateFunction ValidatedConstrainedImageSet::constraint_function() const
{
    return Ariadne::constraint_function(this->function().domain(),this->constraints());
}

ExactBoxType ValidatedConstrainedImageSet::constraint_bounds() const
{
    ExactBoxType result(this->number_of_constraints());
    for(SizeType i=0; i!=this->number_of_constraints(); ++i) {
        result[i]=over_approximation(make_interval(this->constraint(i).lower_bound(),this->constraint(i).upper_bound()));
    }
    return result;
}


UpperBoxType
ValidatedConstrainedImageSet::bounding_box() const
{
    return Ariadne::apply(this->_function,this->_reduced_domain);
}


ValidatedAffineConstrainedImageSet
ValidatedConstrainedImageSet::affine_over_approximation() const
{
    DoublePrecision prec;

    Box<ExactIntervalType> domain = this->domain();
    Vector<ValidatedAffineModelDP> space_models=affine_models(domain,this->function(),prec);
    List<ValidatedAffineModelConstraintDP> constraint_models;
    constraint_models.reserve(this->number_of_constraints());
    for(SizeType i=0; i!=this->number_of_constraints(); ++i) {
        const ValidatedConstraint& constraint=this->constraint(i);
        auto am=affine_model(domain,constraint.function(),prec);
        constraint_models.append(ValidatedAffineModelConstraintDP(constraint.lower_bound(),am,constraint.upper_bound()));
    }

    return ValidatedAffineConstrainedImageSet(domain,space_models,constraint_models);
}

ValidatedAffineConstrainedImageSet ValidatedConstrainedImageSet::affine_approximation() const
{
    DoublePrecision prec;

    Box<ExactIntervalType> domain = this->domain();

    Vector<ValidatedAffineModelDP> space_models=affine_models(domain,this->function(),prec);
    List<ValidatedAffineModelConstraintDP> constraint_models;
    constraint_models.reserve(this->number_of_constraints());
    for(SizeType i=0; i!=this->number_of_constraints(); ++i) {
        const ValidatedConstraint& constraint=this->constraint(i);
        auto am=affine_model(domain,constraint.function(),prec);
        constraint_models.append(ValidatedAffineModelConstraintDP(constraint.lower_bound(),am,constraint.upper_bound()));
    }

    for(SizeType i=0; i!=space_models.size(); ++i) { space_models[i].set_error(0u); }
    for(SizeType i=0; i!=constraint_models.size(); ++i) { constraint_models[i].function().set_error(0u); }

    return ValidatedAffineConstrainedImageSet(domain,space_models,constraint_models);
}


Pair<ValidatedConstrainedImageSet,ValidatedConstrainedImageSet> ValidatedConstrainedImageSet::split(SizeType j) const
{
    Pair<ExactBoxType,ExactBoxType> subdomains = Ariadne::split(this->_domain,j);
    subdomains.first=intersection(subdomains.first,this->_reduced_domain);
    subdomains.second=intersection(subdomains.second,this->_reduced_domain);

    Pair<ValidatedConstrainedImageSet,ValidatedConstrainedImageSet> result(
        ValidatedConstrainedImageSet(subdomains.first,this->_function),
        ValidatedConstrainedImageSet(subdomains.second,this->_function));

    for(SizeType i=0; i!=this->_constraints.size(); ++i) {
        result.first.new_parameter_constraint(this->_constraints[i]);
        result.second.new_parameter_constraint(this->_constraints[i]);
        //result.first.new_parameter_constraint(Ariadne::restrict(this->_negative_constraints[i],subdomains.first));
        //result.second.new_parameter_constraint(Ariadne::restrict(this->_negative_constraints[i],subdomains.second));
    }
    return result;
}

Pair<ValidatedConstrainedImageSet,ValidatedConstrainedImageSet>
ValidatedConstrainedImageSet::split() const
{
    SizeType k=this->number_of_parameters();
    FloatDP rmax(-inf,dp);
    for(SizeType j=0; j!=this->number_of_parameters(); ++j) {
        FloatDPUpperBound rj=this->domain()[j].radius();
        if(rj.raw()>rmax) {
            k=j;
            rmax=rj.raw();
        }
    }
    return this->split(k);
}


inline ValidatedScalarMultivariateFunction const& _restriction(ValidatedScalarMultivariateFunction const& f, ExactBoxType dom) { return f; }
inline ValidatedVectorMultivariateFunction const& _restriction(ValidatedVectorMultivariateFunction const& f, ExactBoxType dom) { return f; }


ValidatedConstrainedImageSet
ValidatedConstrainedImageSet::restriction(ExactBoxType const& new_domain) const
{
    ARIADNE_ASSERT(subset(new_domain,this->domain()));
    ExactBoxType new_reduced_domain = intersection(new_domain,this->reduced_domain());
    ValidatedConstrainedImageSet result(new_domain,_restriction(this->function(),new_domain));
    for(SizeType i=0; i!=this->_constraints.size(); ++i) {
        ValidatedConstraint const& constraint=this->_constraints[i];
        ValidatedConstraint new_constraint(constraint.lower_bound(),_restriction(constraint.function(),new_reduced_domain),constraint.upper_bound());
        result.new_parameter_constraint(new_constraint);
    }
    return result;
}

Void
ValidatedConstrainedImageSet::reduce()
{
    ConstraintSolver solver;
    UpperBoxType& reduced_domain=reinterpret_cast<UpperBoxType&>(this->_reduced_domain);
    solver.reduce(reduced_domain, this->constraint_function(), this->constraint_bounds());
}

ValidatedKleenean ValidatedConstrainedImageSet::is_empty() const
{
    const_cast<ValidatedConstrainedImageSet*>(this)->reduce();
    return this->_reduced_domain.is_empty();
}

ValidatedLowerKleenean ValidatedConstrainedImageSet::inside(const ExactBoxType& bx) const
{
    return Ariadne::inside(this->bounding_box(),bx);
}

ValidatedLowerKleenean ValidatedConstrainedImageSet::separated(const ExactBoxType& bx) const
{
    UpperBoxType subdomain = this->_reduced_domain;
    ValidatedVectorMultivariateFunction function(this->dimension()+this->number_of_constraints(),EuclideanDomain(this->number_of_parameters()));
    for(SizeType i=0; i!=this->dimension(); ++i) { function[i]=this->_function[i]; }
    for(SizeType i=0; i!=this->number_of_constraints(); ++i) { function[i+this->dimension()]=this->_constraints[i].function(); }
    //ValidatedVectorMultivariateFunction function = join(this->_function,this->constraint_function());
    ExactBoxType codomain = product(bx,this->constraint_bounds());
    ConstraintSolver solver;
    solver.reduce(subdomain,function,codomain);
    return subdomain.is_empty();
}

ValidatedLowerKleenean ValidatedConstrainedImageSet::overlaps(const ExactBoxType& bx) const
{
    //std::cerr<<"domain="<<this->_domain<<"\n";
    //std::cerr<<"subdomain="<<this->_reduced_domain<<"\n";

    ExactBoxType subdomain = this->_reduced_domain;
    ValidatedVectorMultivariateFunction space_function = this->_function;
    ValidatedVectorMultivariateFunction constraint_function = this->constraint_function();
    ValidatedVectorMultivariateFunction function = join(space_function,constraint_function);
    //std::cerr<<"function="<<function<<"\n";
    ExactBoxType constraint_bounds = intersection(this->constraint_bounds(),cast_exact_box(Ariadne::apply(this->constraint_function(),subdomain)));
    ExactBoxType codomain = product(bx,constraint_bounds);
    //std::cerr<<"codomain="<<codomain<<"\n";
    NonlinearInfeasibleInteriorPointOptimiser optimiser;

    List<Pair<Nat,ExactBoxType> > subdomains;
    Nat splittings(0);
    Nat MAX_SPLITTINGS=2;
    ValidatedKleenean feasible = false;
    subdomains.append(make_pair(splittings,subdomain));

    while(!subdomains.empty()) {
        make_lpair(splittings,subdomain)=subdomains.back();
        subdomains.pop_back();
        ValidatedKleenean found_feasible = optimiser.feasible(subdomain,function,codomain);
        if(definitely(found_feasible)) { return true; }
        if(possibly(found_feasible)) {
            if(splittings==MAX_SPLITTINGS) {
                feasible = ValidatedKleenean(indeterminate);
            } else {
                Pair<ExactBoxType,ExactBoxType> split_subdomains=subdomain.split();
                subdomains.append(make_pair(splittings+1,split_subdomains.first));
                subdomains.append(make_pair(splittings+1,split_subdomains.second));
            }
        }
    }
    return false;

}


GridTreePaving ValidatedConstrainedImageSet::outer_approximation(const Grid& grid, Nat fineness) const
{
    GridTreePaving paving(grid);
    this->adjoin_outer_approximation_to(paving,fineness);
    return paving;
}

Void ValidatedConstrainedImageSet::adjoin_outer_approximation_to(PavingInterface& paving, Nat fineness) const
{
    ValidatedConstrainedImageSet const& set=*this;

    switch(DISCRETISATION_METHOD) {
        case DiscretisationMethod::SUBDIVISION:
            SubdivisionPaver().adjoin_outer_approximation(paving,set,fineness);
            break;
        case DiscretisationMethod::AFFINE:
            AffinePaver().adjoin_outer_approximation(paving,set,fineness);
            break;
        case DiscretisationMethod::CONSTRAINT:
            ConstraintPaver().adjoin_outer_approximation(paving,set,fineness);
            break;
        default:
            ARIADNE_FAIL_MSG("Unknown discretisation method");
    }

    paving.recombine();
}



ValidatedKleenean ValidatedConstrainedImageSet::satisfies(const ValidatedConstraint& nc) const
{
    if( definitely(subset(Ariadne::apply(nc.function(),this->bounding_box()),nc.bounds())) ) {
        return true;
    }

    ConstraintSolver solver;
    const ExactBoxType& domain=this->_domain;
    List<ValidatedConstraint> all_constraints=this->constraints();
    ValidatedScalarMultivariateFunction composed_function = compose(nc.function(),this->_function);
    const ExactIntervalType& bounds = nc.bounds();

    ValidatedKleenean result;
    if(definitely(bounds.upper_bound()<+infty)) {
        all_constraints.append( composed_function >= bounds.upper_bound() );
        result=solver.feasible(domain,all_constraints).first;
        all_constraints.pop_back();
        if(definitely(result)) { return false; }
    }
    if(definitely(bounds.lower_bound()>-infty)) {
        all_constraints.append(composed_function <= bounds.lower_bound());
        result = result || solver.feasible(domain,all_constraints).first;
    }
    return !result;
}


Void
ValidatedConstrainedImageSet::draw(CanvasInterface& cnvs, const Projection2d& proj) const {
    GraphicsManager::instance().drawer().draw(cnvs,proj,*this);
}

ValidatedConstrainedImageSet
join(const ValidatedConstrainedImageSet& set1, const ValidatedConstrainedImageSet& set2)
{
    ARIADNE_ASSERT(set1.dimension()==set2.dimension());
    ARIADNE_ASSERT(set1.number_of_parameters()==set2.number_of_parameters());
    ARIADNE_ASSERT(set1.number_of_constraints()==set2.number_of_constraints());

    const SizeType np = set1.number_of_parameters();

    const ExactBoxType& domain1 = set1.domain();
    const ExactBoxType& domain2 = set2.domain();

    SizeType join_parameter=np;
    for(SizeType i=0; i!=np; ++i) {
        if(domain1[i]!=domain2[i]) {
            if(domain1[i].upper_bound() < domain2[i].lower_bound() || domain1[i].lower_bound() > domain2[i].upper_bound()) {
                ARIADNE_FAIL_MSG("Cannot joint sets "<<set1<<" and "<<set2<<" since domains are separated.");
            }
            if(domain1[i].upper_bound() >= domain2[i].lower_bound() || domain1[i].lower_bound() <= domain2[i].upper_bound()) {
                if(join_parameter==np) {
                    join_parameter=i;
                } else {
                    ARIADNE_FAIL_MSG("Cannot joint sets "<<set1<<" and "<<set2<<" since domains do not join to form a box.");
                }
            }
        }
    }

    ExactBoxType new_domain = hull(domain1,domain2);

    ValidatedVectorMultivariateFunctionModelDP function1
        = ValidatedVectorMultivariateFunctionModelDP( dynamic_cast<ValidatedVectorMultivariateFunctionModelDP::Interface const&>(set1.function().reference()));
    Vector<FloatDPError> function_error1=function1.errors();
    function1.clobber();
    function1.restrict(new_domain);

    ValidatedVectorMultivariateFunctionModelDP function2
        = ValidatedVectorMultivariateFunctionModelDP( dynamic_cast<ValidatedVectorMultivariateFunctionModelDP::Interface const&>(set2.function().reference()));
    Vector<FloatDPError> function_error2=function2.errors();
    function2.clobber();
    function2.restrict(new_domain);

    ValidatedVectorMultivariateFunctionModelDP new_function=(function1+function2)*FloatDP(0.5_x,dp);
    new_function.clobber();
    for(SizeType i=0; i!=new_function.result_size(); ++i) {
        function_error1[i]=norm(new_function[i]-function1[i])+function_error1[i];
        function_error2[i]=norm(new_function[i]-function2[i])+function_error2[i];
        FloatDPError new_function_error = max(function_error1[i],function_error2[i]);
        new_function[i] = new_function[i] + FloatDPBounds(-new_function_error,+new_function_error);
    }

    ARIADNE_ASSERT(set1.number_of_constraints()==0);
    return ValidatedConstrainedImageSet(new_domain,new_function);
}


OutputStream& ValidatedConstrainedImageSet::_write(OutputStream& os) const
{
    return os << "ValidatedConstrainedImageSet( domain=" << this->domain() << ", function="<< this->function() << ", constraints=" << this->constraints() << " )";
}

OutputStream& operator<<(OutputStream& os, const ValidatedConstrainedImageSet& set) {
    return set._write(os);
}


template<> String class_name<EffectiveConstraintSet>() { return "EffectiveConstraintSet"; }
template<> String class_name<EffectiveBoundedConstraintSet>() { return "EffectiveBoundedConstraintSet"; }
template<> String class_name<EffectiveConstrainedImageSet>() { return "EffectiveConstrainedImageSet"; }
template<> String class_name<ValidatedConstrainedImageSet>() { return "ValidatedConstrainedImageSet"; }
template<> String class_name<ValidatedAffineConstrainedImageSet>() { return "ValidatedAffineConstrainedImageSet"; }

} // namespace Ariadne;
