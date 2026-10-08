/***************************************************************************
 *            geometry/affine_set.cpp
 *
 *  Copyright  2009-20  Pieter Collins
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

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "solvers/linear_programming.hpp"
#include "function/function.hpp"
#include "function/affine.hpp"
#include "function/affine_model.hpp"

#include "geometry/box.hpp"
#include "geometry/grid_paving.hpp"
#include "geometry/affine_set.hpp"

#include "io/graphics_interface.hpp"
#include "io/geometry2d.hpp"


namespace Ariadne {

typedef Vector<FloatDP> RawFloatVector;
typedef Vector<ExactIntervalType> ExactIntervalVectorType;

Pair<Interval<FloatDP>,FloatDPError> make_domain(Interval<Real> const& ivl);
Pair<Interval<FloatDP>,FloatDPError> make_domain(Interval<FloatDPBall> const& ivl);

template<class X, class XX>
struct LinearProgram {
    LinearProgram() : LinearProgram(0u,0u,X(dp)) { }
    LinearProgram(SizeType m, SizeType n, X const& z);
    Matrix<X> A;
    Vector<X> b;
    Vector<X> c;
    Vector<X> l;
    Vector<X> u;
    Array<Slackness> vt;
    Array<SizeType> p;
    Matrix<XX> B;
    Vector<XX> x;
    Vector<XX> y;
    Vector<XX> z;
};

template<class X, class XX> LinearProgram<X,XX>::LinearProgram(SizeType m, SizeType n, X const& zero)
    : A(m,n,zero), b(m,zero), c(n,zero), l(n,zero), u(n,zero)
    , vt(n), p(n), B(m,m,zero)
    , x(n,zero), y(m,zero),z(n,zero) { }


ValidatedAffineConstrainedImageSet::ValidatedAffineConstrainedImageSet(const RealBox& bx)
    : ValidatedAffineConstrainedImageSet(Box<Interval<FloatDPBall>>(bx,dp))
{
    FloatDPBall(0_z,dp);
}


Pair<Interval<FloatDP>,FloatDPError> make_domain(Interval<Real> const& ivl) {
    FloatDPBounds dlb(ivl.lower_bound(),dp);
    FloatDPBounds dub(ivl.upper_bound(),dp);
    FloatDPApproximation dla(ivl.lower_bound(),dp);
    FloatDPApproximation dua(ivl.upper_bound(),dp);
    FloatDP dl(FloatDP(Float32(dla.raw(),near)));
    FloatDP du(FloatDP(Float32(dua.raw(),near)));
    FloatDPError e=cast_positive(max(max(dub.upper()-du,du-dub.lower()),max(dlb.upper()-dl,dl-dlb.lower())));
    return make_pair(make_interval(dl,du),e);
}

Pair<Interval<FloatDP>,FloatDPError> make_domain(Interval<FloatDPBall> const& ivl) {
    FloatDPBall const& dlb=ivl.lower_bound();
    FloatDPBall const& dub=ivl.upper_bound();
    FloatDP dl(FloatDP(Float32(dlb.value(),near)));
    FloatDP du(FloatDP(Float32(dub.value(),near)));
    FloatDPError e=cast_positive(max(mag(dlb.value()-dl)+dlb.error(),mag(dlb.value()-dl)+dlb.error()));
    return make_pair(make_interval(dl,du),e);
}

ValidatedAffineConstrainedImageSet::ValidatedAffineConstrainedImageSet(const Box<Interval<FloatDPBall>>& bx)
    : _domain(bx.dimension(),Interval<FloatDP>(0_z,dp)), _space_models(bx.dimension(),ValidatedAffineModelDP(bx.dimension(),dp))
{
    Vector<FloatDPError> errs(bx.dimension(),dp);

    for(SizeType i=0; i!=bx.dimension(); ++i) {
        make_lpair(_domain[i],errs[i])=make_domain(bx[i]);
    }

    for(SizeType i=0; i!=bx.dimension(); ++i) {
        _space_models[i]=ValidatedAffineModelDP::scaling(bx.dimension(),i,_domain[i],dp);
        _space_models[i].error()+=errs[i];
    }
}

ValidatedAffineConstrainedImageSet::ValidatedAffineConstrainedImageSet(const ExactBoxType& d)
    : _domain(d), _space_models(ValidatedAffineModelDP::scalings(d,dp))
{
}


ValidatedAffineConstrainedImageSet::ValidatedAffineConstrainedImageSet(const ExactBoxType& d,
                     const Vector<Affine<FloatDPBounds>>& f)
    : _domain(d), _space_models(f.size(),ValidatedAffineModelDP(d.size(),dp))
{
    if(d==ExactBoxType::unit_box(d.size())) {
        for(SizeType i=0; i!=f.size(); ++i) {
            ARIADNE_ASSERT_MSG(_domain.dimension() == f[i].argument_size(),"The domain dimension ("<<_domain.dimension()<<") does not match the function argument size ("<<f[i].argument_size()<<").");
            _space_models[i] = affine_model(f[i]);
        }
    }
}

ValidatedAffineConstrainedImageSet::ValidatedAffineConstrainedImageSet(const ExactBoxType& d,
                     const Vector<Affine<FloatDPBounds>>& f,
                     const List<ValidatedAffineConstraintDP>& c)
    : _domain(d), _space_models(f.size(),ValidatedAffineModelDP(d.size(),dp)), _constraint_models()
{
    ARIADNE_ASSERT_MSG(_domain.dimension() == f[0].argument_size(),"The domain dimension ("<<_domain.dimension()<<") does not match the function argument size ("<<_space_models[0].argument_size()<<").");

    if(d==ExactBoxType::unit_box(d.size())) {
        for(SizeType i=0; i!=f.size(); ++i) {
            _space_models[i] = affine_model(f[i]);
        }
        for(SizeType i=0; i!=c.size(); ++i) {
            ARIADNE_ASSERT_MSG(_domain.dimension() == c[i].argument_size(),"The domain dimension ("<<_domain.dimension()<<") does not match the constraint argument size ("<<c[i].argument_size()<<").");
            _constraint_models.append(ValidatedAffineModelConstraintDP(c[i].lower_bound(),affine_model(c[i].function()),c[i].upper_bound()));
        }
    } else {
        ARIADNE_NOT_IMPLEMENTED;
    }
}

ValidatedAffineConstrainedImageSet::ValidatedAffineConstrainedImageSet(const ExactBoxType& d,
                     const Vector<ValidatedAffineModelDP>& f,
                     const List<ValidatedAffineModelConstraintDP>& c)
    : _domain(d), _space_models(f), _constraint_models(c)
{
    ARIADNE_ASSERT_MSG(_domain.dimension() == f[0].argument_size(),"The domain dimension ("<<_domain.dimension()<<") does not match the function argument size ("<<_space_models[0].argument_size()<<").");
    for (auto cons : _constraint_models)
        ARIADNE_ASSERT_MSG(_domain.dimension() == cons.argument_size(),"The domain dimension ("<<_domain.dimension()<<") does not match the constraint argument size ("<<cons.argument_size()<<").");
}

ValidatedAffineConstrainedImageSet::ValidatedAffineConstrainedImageSet(const Vector<ValidatedAffineModelDP>& f,
                     const List<ValidatedAffineModelConstraintDP>& c)
    : ValidatedAffineConstrainedImageSet(ExactBoxType::unit_box(f[0].argument_size()),f,c)
{
}

ValidatedAffineConstrainedImageSet::ValidatedAffineConstrainedImageSet(const Vector<ValidatedAffineModelDP>& f)
    : _domain(ExactBoxType::unit_box(f[0].argument_size())), _space_models(f)
{
    ARIADNE_ASSERT_MSG(_domain.dimension() == f[0].argument_size(),"The domain dimension ("<<_domain.dimension()<<") does not match the function argument size ("<<_space_models[0].argument_size()<<").");
}

ValidatedAffineConstrainedImageSet::ValidatedAffineConstrainedImageSet(const ExactBoxType& D, const Matrix<FloatDP>& G, const Vector<FloatDP>& h)
{
    this->construct(D,G,h);
}

Void ValidatedAffineConstrainedImageSet::construct(const ExactBoxType& D, const Matrix<FloatDP>& G, const Vector<FloatDP>& h)
{
    ARIADNE_ASSERT_MSG(G.row_size()==h.size() && G.row_size()>0,"G="<<G<<", h="<<h);
    this->_domain=D;
    this->_space_models=Vector<ValidatedAffineModelDP>(G.row_size(),ValidatedAffineModelDP(G.column_size(),dp));
    for(SizeType i=0; i!=G.row_size(); ++i) {
        ValidatedAffineModelDP x(G.column_size(),dp);
        x=h[i];
        for(SizeType j=0; j!=G.column_size(); ++j) {
            x[j]=G[i][j];
        }
        this->_space_models[i]=x;
    }
}

Void
ValidatedAffineConstrainedImageSet::new_constraint(const ValidatedAffineModelConstraintDP& c)
{
    ARIADNE_ASSERT(this->_space_models.size()>0);
    ARIADNE_ASSERT_MSG(this->number_of_parameters()==c.argument_size(),"c["<<c.argument_size()<<"]="<<c<<" f["<<this->number_of_parameters()<<"]="<<this->_space_models);
    _constraint_models.append(c);
}

Void
ValidatedAffineConstrainedImageSet::new_parameter_constraint(const ValidatedAffineConstraintDP& c)
{
    ARIADNE_ASSERT_MSG(this->_space_models.size()>0,"f="<<this->_space_models);
    ARIADNE_ASSERT_MSG(this->number_of_parameters()==c.argument_size(),"c="<<c<<" f="<<this->_space_models);
    this->new_constraint(ValidatedAffineModelConstraintDP(c.lower_bound(),affine_model(c.function()),c.upper_bound()));
}





ValidatedAffineConstrainedImageSet*
ValidatedAffineConstrainedImageSet::clone() const
{
    return new ValidatedAffineConstrainedImageSet(*this);
}


DimensionType
ValidatedAffineConstrainedImageSet::dimension() const
{
    return this->_space_models.size();
}

SizeType
ValidatedAffineConstrainedImageSet::number_of_parameters() const
{
    ARIADNE_ASSERT(this->_space_models.size()>0);
    return this->_space_models[0].argument_size();
}

SizeType
ValidatedAffineConstrainedImageSet::number_of_constraints() const
{
    return this->_constraint_models.size();
}

ExactBoxType
ValidatedAffineConstrainedImageSet::domain() const
{
    return this->_domain;
}

ValidatedKleenean ValidatedAffineConstrainedImageSet::is_bounded() const {
    return ValidatedKleenean(ExactBoxType(this->domain()).is_bounded()) || ValidatedKleenean(indeterminate);
}

UpperBoxType ValidatedAffineConstrainedImageSet::bounding_box() const {
    UpperBoxType result(this->dimension());
    ExactBoxType domain=this->domain();
    for(SizeType i=0; i!=this->dimension(); ++i) {
        result[i]=this->_space_models[i].range();
    }
    return result;
}





ValidatedLowerKleenean ValidatedAffineConstrainedImageSet::separated(const ExactBoxType& bx) const {
    ARIADNE_PRECONDITION_MSG(this->dimension()==bx.dimension(),"set="<<*this<<", box="<<bx);
    ExactBoxType wbx=cast_exact_box(widen(bx));
    LinearProgram<FloatDP> lp;
    this->construct_linear_program(lp);
    for(SizeType i=0; i!=bx.size(); ++i) {
        lp.l[i]=FloatDP(sub(down,wbx[i].lower_bound(),this->_space_models[i].error().raw()));
        lp.u[i]=FloatDP(add(up,wbx[i].upper_bound(),this->_space_models[i].error().raw()));
    }
    ValidatedKleenean feasible=indeterminate;
    try {
        InteriorPointSolver optimiser;
        feasible=optimiser.feasible(lp.l,lp.u,lp.A,lp.b);
        //feasible=SimplexSolver<FloatDP>().hotstarted_feasible(lp.A,lp.b,lp.l,lp.u,lp.vt,lp.p,lp.B,lp.x,lp.y);
    }
    catch(const DegenerateFeasibilityProblemException& e) {
        feasible=indeterminate;
    }
    return !feasible;
}

ValidatedLowerKleenean ValidatedAffineConstrainedImageSet::is_empty() const {
    return ValidatedLowerKleenean(this->separated(cast_exact_box(this->bounding_box()))) || ValidatedKleenean(indeterminate);
}

ValidatedLowerKleenean ValidatedAffineConstrainedImageSet::inside(const ExactBoxType& bx) const {
    ARIADNE_PRECONDITION_MSG(this->dimension()==bx.dimension(),"set="<<*this<<", box="<<bx);
    return widen(this->bounding_box()).inside(bx);
}


ValidatedAffineConstrainedImageSet image(ValidatedAffineConstrainedImageSet set, ValidatedVectorMultivariateFunction const& h) {
    set._space_models=compose(h,set._space_models);
    return set;
}


GridTreePaving
ValidatedAffineConstrainedImageSet::outer_approximation(const Grid& g, Nat depth) const {
    GridTreePaving r(g);
    this->adjoin_outer_approximation_to(r,depth);
    return r;
}


Void ValidatedAffineConstrainedImageSet::_adjoin_outer_approximation_to(PavingInterface& paving, LinearProgram<FloatDP>& lp, const Vector<FloatDPError>& errors, GridCell& cell, Nat fineness)
{

    // No need to check if cell is already part of the set
    if(paving.superset(cell)) {
        return;
    }

    // Find concrete cell box
    const ExactBoxType bx=cell.box();

    // Make part of linear program dependent on cell
    for(SizeType i=0; i!=cell.dimension(); ++i) {
        //lp.l[i]=bx[i].lower_bound();
        //lp.u[i]=bx[i].upper_bound();
        lp.l[i]=FloatDP(sub(down,bx[i].lower_bound(),errors[i].raw()));
        lp.u[i]=FloatDP(add(up,bx[i].upper_bound(),errors[i].raw()));
    }

    Int cell_depth=cell.depth();
    Int maximum_depth=fineness*cell.dimension();

    // Check for disjointness using linear program
    //ValidatedKleenean feasible=SimplexSolver<FloatDP>().hotstarted_feasible(lp.A,lp.b,lp.l,lp.u,lp.vt,lp.p,lp.B,lp.x,lp.y);
    InteriorPointSolver optimiser;
    ValidatedKleenean feasible=optimiser.feasible(lp.l,lp.u,lp.A,lp.b);
    //feasible=verify_feasibility(lp.A,lp.b,lp.l,lp.u,lp.vt);
    if(definitely(!feasible)) { return; }

    // If not disjoint, and we are at maximum depth, adjoin the set.
    // Otherwise, split and continue.

    if(cell_depth>=maximum_depth) {
        paving.adjoin(cell);
    } else {
        GridCell subcell1 = cell.split(0);
        GridCell subcell2 = cell.split(1);
        _adjoin_outer_approximation_to(paving,lp,errors,subcell1,fineness);
        _adjoin_outer_approximation_to(paving,lp,errors,subcell2,fineness);
    }

}



Void
ValidatedAffineConstrainedImageSet::construct_linear_program(LinearProgram<FloatDP>& lp) const
{
    // Set up linear programming problem.
    // We have parameter e and point x, which need to satisfy
    //  x=Gp+h;  Cp+d<=0; Ep+f = 0; lb<=x<=ub, -1<=p<=+1
    // Add slack variables for the inequality constraints Cp+s=d; s>=0
    // The standard form is then
    //  -x+Gp=-h,  Cp+s=-d; Ep = -f; lb<=x<=ub, -1<=p<=+1, 0<=s
    // The only dependence on the cell is in the inequality constraints for x

    // Set up linear program of form Ax=b; l<=x<=u.
    //  Order variables as x,e,s
    //  A=( -I G 0 )  b=( -h )  l=(xl -1 0) u=(xu +1 inf)
    //    (  0 C I )    ( -d )
    //    (  0 E 0 )    ( -f )
    // Spacial dimension nx; parameter dimension ne; number of constraints nc

	// Warning: Uniform part of space function is not included!

    CONCLOG_SCOPE_CREATE;

    const SizeType nx=this->dimension();
    const SizeType np=this->number_of_parameters();
    const SizeType nc=this->_constraint_models.size();

    lp.A.resize(nx+nc,nx+np+nc);
    lp.b.resize(nx+nc);
    lp.c.resize(nx+np+nc);
    lp.l.resize(nx+np+nc);
    lp.u.resize(nx+np+nc);

    // Make part of linear program only dependent on set
    // Need to set all values since matrix is uninitialised
    for(SizeType i=0; i!=nx; ++i) {
        for(SizeType j=0; j!=nx; ++j) {
            lp.A[i][j]=0;
        }
        lp.A[i][i] = -1;
        for(SizeType j=0; j!=np; ++j) {
            lp.A[i][nx+j] = +this->_space_models[i].gradient(j);
        }
        for(SizeType j=0; j!=nc; ++j) {
            lp.A[i][nx+np+j]=0;
        }
        lp.b[i] = -this->_space_models[i].value();
    }
    for(SizeType i=0; i!=nc; ++i) {
        for(SizeType j=0; j!=nx; ++j) {
            lp.A[nx+i][j]=0;
        }
        for(SizeType j=0; j!=np; ++j) {
            lp.A[nx+i][nx+j] = +this->_constraint_models[i].function().gradient(j);
        }
        for(SizeType j=0; j!=nc; ++j) {
            lp.A[nx+i][nx+np+j]=0;
        }
        lp.A[nx+i][nx+np+i] = +1;
        lp.b[nx+i] = -this->_constraint_models[i].function().value();
    }

    for(SizeType i=0; i!=np; ++i) {
        //lp.l[nx+i]=this->_domain[i].lower_bound();
        //lp.u[nx+i]=this->_domain[i].upper_bound();
        lp.l[nx+i]=-1;
        lp.u[nx+i]=+1;
    }
    for(SizeType i=0; i!=nc; ++i) {
        lp.l[nx+np+i]=-this->_constraint_models[i].upper_bound().value();
        lp.u[nx+np+i]=-this->_constraint_models[i].lower_bound().value();
    }

    // Make part of linear program dependent on cell be +/-infinity
    for(SizeType i=0; i!=nx; ++i) {
        lp.l[i]=-infty;
        lp.u[i]=+infty;
    }

    CONCLOG_PRINTLN("set="<<*this<<", A="<<lp.A<<", b="<<lp.b<<", l="<<lp.l<<", u="<<lp.u);
}


Void
ValidatedAffineConstrainedImageSet::adjoin_outer_approximation_to(PavingInterface& paving, Nat fineness) const
{
    ARIADNE_ASSERT(this->dimension()==paving.dimension());

    GridCell bounding_cell=GridCell::smallest_enclosing_primary_cell(this->bounding_box(),paving.grid());

    // Create linear program
    LinearProgram<FloatDP> lp;
    this->construct_linear_program(lp);
    Vector<FloatDPError> errors(this->dimension(),dp);
    for(SizeType i=0; i!=this->dimension(); ++i) {
        errors[i]=this->_space_models[i].error();
    }
    _adjoin_outer_approximation_to(paving,lp,errors,bounding_cell,fineness);
}




Void ValidatedAffineConstrainedImageSet::_robust_adjoin_outer_approximation_to(PavingInterface& paving, LinearProgram<FloatDP>& lp, const Vector<FloatDPError>& errors, GridCell& cell, Nat fineness)
{
    SimplexSolver<FloatDP> lpsolver;

    const SizeType nx=cell.dimension();
    const SizeType nc=lp.A.row_size()-nx;
    const SizeType ne=lp.A.column_size()-lp.A.row_size()-1u;


    // No need to check if cell is already part of the set
    if(paving.superset(cell)) {
        return;
    }

    // Find concrete cell box
    const ExactBoxType& bx=cell.box();

    // Make part of linear program dependent on cell
    for(SizeType i=0; i!=nx; ++i) {
        lp.l[i]=bx[i].lower_bound();
        lp.u[i]=bx[i].upper_bound();
    }

    Int cell_depth=cell.depth();
    Int maximum_depth=fineness*cell.dimension();

    // Check for disjointness using linear program
    ValidatedKleenean feasible=lpsolver.hotstarted_feasible(lp.l,lp.u,lp.A,lp.b,lp.vt,lp.p,lp.B,lp.x,lp.y);

    if (definitely(not feasible)) {
        return;
    }

    Bool done=false;
    while(!done && possibly(lp.x[ne+nx+nc]<0)) {
        done=lpsolver.lpstep(lp.c,lp.l,lp.u,lp.A,lp.b,lp.vt,lp.p,lp.B,lp.x);
    }
    // FIXME: Should be rigorous!
    Vector<FloatDPBounds> x=lpsolver.compute_x(lp.l,lp.u,lp.A,lp.b,lp.vt);
    if(possibly(x[ne+nx+nc]<0)) { return; } // No feasible solution

    //feasible=verify_feasibility(lp.A,lp.b,lp.l,lp.u,lp.vt);
    //if(!feasible) { return; }

    // If not disjoint, and we are at maximum depth, adjoin the set.
    // Otherwise, split and continue.

    if(cell_depth>=maximum_depth) {
        paving.adjoin(cell);
    } else {
        GridCell subcell1 = cell.split(0);
		GridCell subcell2 = cell.split(1);
        ValidatedAffineConstrainedImageSet::_adjoin_outer_approximation_to(paving,lp,errors,subcell1,fineness);
        ValidatedAffineConstrainedImageSet::_adjoin_outer_approximation_to(paving,lp,errors,subcell2,fineness);
    }

}



Void
ValidatedAffineConstrainedImageSet::robust_adjoin_outer_approximation_to(PavingInterface& paving, Nat fineness) const {
    CONCLOG_SCOPE_CREATE;

    ARIADNE_ASSERT(this->dimension()==paving.dimension());

    SimplexSolver<FloatDP> lpsolver;

    GridCell bounding_cell=GridCell::smallest_enclosing_primary_cell(this->bounding_box(),paving.grid());

    // Set up linear programming problem.
    // We have parameter e and point x, which need to satisfy
    //  x=Ge+h;  Ce+d>=0; lb<=x<=ub, -1<=e<=+1
    // Add slack variables for the inequality constraints Ce+s=d; s>=0
    // The standard form is then
    //  -Ge+x=h,  -Ce+s=d; -1<=e<=+1, lb<=x<=ub, 0<=s
    // The only dependence on the cell is in the inequality constraints for x

    // Set up linear program Ax=b; l<=x<=u.
    // Order variables as e,x,s

    // Spacial dimension nx; parameter dimension np; number of constraints nc
    const SizeType nx=this->dimension();
    const SizeType ne=this->number_of_parameters();
    const SizeType nc=this->number_of_constraints();

    // Create linear program
    LinearProgram<FloatDP> lp;
    lp.A.resize(nx+nc,nx+ne+nc+1u);
    lp.b.resize(nx+nc);
    lp.c.resize(nx+ne+nc+1u);
    lp.l.resize(nx+ne+nc+1u);
    lp.u.resize(nx+ne+nc+1u);
    lp.vt.resize(ne+nx+nc+1u);
    lp.p.resize(ne+nx+nc+1u);

    // Make part of linear program only dependent on set
    // Need to set all values since matrix is uninitialised
    for(SizeType i=0; i!=nx; ++i) {
        for(SizeType j=0; j!=ne; ++j) {
            lp.A[i][j]=-this->_space_models[i].gradient(j);
        }
        for(SizeType j=0; j!=nx; ++j) {
            lp.A[i][ne+j]=0;
        }
        lp.A[i][ne+i]=+1;
        for(SizeType j=0; j!=nc; ++j) {
            lp.A[i][ne+nx+j]=0;
        }
        lp.b[i]=this->_space_models[i].value();
    }
    for(SizeType i=0; i!=nc; ++i) {
        for(SizeType j=0; j!=ne; ++j) {
            lp.A[nx+i][j]=this->_constraint_models[i].function().gradient(j);
        }
        for(SizeType j=0; j!=nx; ++j) {
            lp.A[nx+i][ne+j]=0;
        }
        for(SizeType j=0; j!=nc; ++j) {
            lp.A[nx+i][ne+nx+j]=0;
        }
        lp.A[nx+i][nx+ne+i]=+1;
        lp.b[nx+i]=-this->_constraint_models[i].function().value();
    }
    for(SizeType i=0; i!=ne; ++i) {
        lp.l[i]=-1;
        lp.u[i]=+1;
    }
    for(SizeType i=0; i!=nc; ++i) {
        lp.l[ne+nx+i]=0;
        lp.u[ne+nx+i]=infty;
    }


    // Make part of linear program used for robustness
    for(SizeType i=0; i!=ne+nx+nc; ++i) {
        lp.c[i]=0;
    }
    for(SizeType i=0; i!=nx+nc; ++i) {
        lp.A[i][ne+nx+nc]=1;
    }
    lp.c[ne+nx+nc]=-1;
    //lp.l[ne+nx+nc]=0;
    lp.l[ne+nx+nc]=-1;
    lp.u[ne+nx+nc]=infty;
    lp.vt[ne+nx+nc]=Slackness::LOWER;
    lp.p[ne+nx+nc]=ne+nx+nc;


    // Make part of linear program dependent on cell
    const ExactBoxType bx=bounding_cell.box();
    for(SizeType i=0; i!=nx; ++i) {
        lp.l[ne+i]=bx[i].lower_bound();
        lp.u[ne+i]=bx[i].upper_bound();
    }

    // Take x and s variables to be basic, so the initial basis matrix is the
    // identity
    for(SizeType i=0; i!=ne; ++i) {
        lp.vt[i]=Slackness::LOWER;
        lp.p[nx+nc+i]=i;
    }
    for(SizeType i=0; i!=nx+nc; ++i) {
        lp.vt[ne+i]=Slackness::BASIS;
        lp.p[i]=ne+i;
    }
    lp.B=Matrix<FloatDPBounds>::identity(nx+nc,dp);

    Vector<FloatDPError> errors(this->dimension(),dp);
    for(SizeType i=0; i!=this->dimension(); ++i) {
        errors[i]=this->_space_models[i].error();
    }

    CONCLOG_PRINTLN("A="<<lp.A<<"\nb="<<lp.b<<"\nl="<<lp.l<<"\nu="<<lp.u);
    ValidatedKleenean feasible=lpsolver.hotstarted_feasible(lp.l,lp.u,lp.A,lp.b,lp.vt,lp.p,lp.B,lp.x,lp.y);
    CONCLOG_PRINTLN("  vt="<<lp.vt<<"\nx="<<lp.x);
    if(definitely(not feasible)) { return; } // no intersection

    _adjoin_outer_approximation_to(paving,lp,errors,bounding_cell,fineness);
}


class PerturbationGenerator {
  public:
    PerturbationGenerator() : _n(0) { }
    FloatDPApproximation operator()() { _n=_n+3; if(_n>19) { _n=_n-37; } return _n*FloatDPApproximation(1e-13_pr,dp); }
  private:
    Int _n;
};

List<Point2d>
ValidatedAffineConstrainedImageSet::boundary(SizeType xind, SizeType yind) const
{
    CONCLOG_SCOPE_CREATE;

    CONCLOG_PRINTLN("xind="<<xind<<", yind="<<yind);

    SimplexSolver<FloatDPApproximation> lpsolver;
    PerturbationGenerator eps;

    static const Int MAX_STEPS=1000;
    static const ExactDouble ERROR_TOLERANCE = cast_exact(static_cast<double>(std::numeric_limits<float>::epsilon()));
    static const FloatDPApproximation _inf ( Ariadne::inf, dp );

    const SizeType nx=_domain.size();
    const SizeType ne=2u;
    const SizeType nc=_constraint_models.size();
    const SizeType np=nx+ne+nc;

    // Set up matrix of function values
    ValidatedAffineModelDP const& xa=this->_space_models[xind];
    ValidatedAffineModelDP const& ya=this->_space_models[yind];

    // The set is given by pt=Gx+h, where Ax=b and l<=x<=u
    Matrix<FloatDPApproximation> G=Matrix<FloatDPApproximation>::zero(ne,np,dp);
    for(SizeType j=0; j!=nx; ++j) {
        G[0][j]=FloatDPApproximation(xa.gradient(j))+eps();
        G[1][j]=FloatDPApproximation(ya.gradient(j))+eps();
    }
    G[0][nx+nc+0]=FloatDPApproximation(1.0_x,dp);
    G[1][nx+nc+1]=FloatDPApproximation(1.0_x,dp);
    Vector<FloatDPApproximation> h(ne,dp);
    h[0]=FloatDPApproximation(xa.value());
    h[1]=FloatDPApproximation(ya.value());
    CONCLOG_PRINTLN("G="<<G)
    CONCLOG_PRINTLN("h="<<h);

    // Set up linear programming problem Ax=b; l<=x<=u
    // Since the parameter domain is given by cl<=Ay+b+/-e<=cu, -1<=y<=+1, introduce slack variables z such that z-Ay=b, with cl-e<=z<=cu+e
    // Need to keep point error variables, introduce extra variables

    Matrix<FloatDPApproximation> A(nc,np,dp);
    Vector<FloatDPApproximation> b(nc,dp);
    Vector<FloatDPApproximation> l(np,dp);
    Vector<FloatDPApproximation> u(np,dp);

    for(SizeType j=0; j!=nx; ++j) {
        l[j]=FloatDPApproximation(-1.0_x,dp);
        u[j]=FloatDPApproximation(+1.0_x,dp);
    }

    for(SizeType i=0; i!=nc; ++i) {
        for(SizeType j=0; j!=nx; ++j) {
            A[i][j] = neg( numeric_cast<FloatDPApproximation>(this->_constraint_models[i].function().gradient(j)) );
        }
        for(SizeType j=nx; j!=nx+nc+ne; ++j) {
            A[i][j] = 0.0_x;
        }
        A[i][nx+i]=1.0_x;
        FloatDPApproximation fb=numeric_cast<FloatDPApproximation>(this->_constraint_models[i].function().value());
        FloatDPApproximation fe=numeric_cast<FloatDPApproximation>(this->_constraint_models[i].function().error().raw());
        FloatDPApproximation cl=numeric_cast<FloatDPApproximation>(this->_constraint_models[i].lower_bound().value());
        FloatDPApproximation cu=numeric_cast<FloatDPApproximation>(this->_constraint_models[i].upper_bound().value());
        b[i]=fb+eps();
        l[nx+i]=cl-fe;
        u[nx+i]=cu+fe;
    }

    l[nx+nc]=static_cast<FloatDPApproximation>(-xa.error().raw());
    u[nx+nc]=static_cast<FloatDPApproximation>(+xa.error().raw());
    l[nx+nc+1]=static_cast<FloatDPApproximation>(-ya.error().raw());
    u[nx+nc+1]=static_cast<FloatDPApproximation>(+ya.error().raw());
    CONCLOG_PRINTLN("A="<<A<<" b="<<b<<" l="<<l<<" u="<<u);

    List<Point2d> vertices;

    // Set up simplex algorithm working variables
    Array<Slackness> vt(0); Array<SizeType> p(nc); Matrix<FloatDPApproximation> B(nc,nc,dp);
    Vector<FloatDPApproximation> x(np,dp); Vector<FloatDPApproximation> y(nc,dp);

    // Find an initial feasible point
    ValidatedKleenean feasible = lpsolver.hotstarted_feasible(l,u,A,b, vt, p,B, x,y);
    CONCLOG_PRINTLN("A="<<A<<" b="<<b<<" l="<<l<<" u="<<u<<" vt="<<vt<<" p="<<p)
    CONCLOG_PRINTLN("x="<<x<<" Ax="<< A*x );
    lpsolver.consistency_check(l,u,A,b, vt,p,B,x);

    // If problem not feasible, then set is empty; return empty list
    if(!possibly(feasible)) { return vertices; }

    Vector<FloatDPApproximation> c(np,FloatDPApproximation(0.0_x,dp));
    for(SizeType j=0; j!=np; ++j) { c[j]=G[0][j]; }

    // Find a point on the boundary; choose the point minimising the spacial x-coordinate
    x=lpsolver.hotstarted_minimise(c,l,u,A,b, vt,p,B);
    CONCLOG_PRINTLN("A="<<A<<" b="<<b<<" l="<<l<<" u="<<u<<" vt="<<vt<<" p="<<p)
    CONCLOG_PRINTLN("x="<<x<<" Ax="<<(A*x)<<" c="<<c<<" cx="<<dot(c,x)<<" pt="<<G*x+h);

    lpsolver.consistency_check(l,u,A,b, vt,p,B,x);

    // We always want to turn left, so we need to choose a direction d such that
    // (v0*d0+v1*d1) is positive and the ratio (v0*d1-v1*d0)/(v0*d0+v1*d1) is maximised.
    // Note that in in all cases, v0*d1-v1*d0 must be positive.

    Int STEPS=0;
    Array<Slackness> initial_variable_type=vt;

    Vector<FloatDPApproximation> pt=G*x+h; // The current point in space
    Vector<FloatDPApproximation> last_vec({0.0_x,-1.0_x},dp); // The direction in space of the last step along the boundary
                                        // Should be set orthogonal to the direction (+1,0) which is minimised in finding first boundary point
    Vector<FloatDPApproximation> best_next_vec(2,dp);
    Vector<FloatDPApproximation> trial_vec(2,dp);
    Vector<FloatDPApproximation> Aj(nc,dp); // The jth column of A
    Vector<FloatDPApproximation> BAj(nc,dp); // The jth column of A

    SizeType last_exiting_variable=np; // Last variable to exit the basis

    // FIXME: Do we need check below?
    // if(nx==ne) { vertices.push_back(Point2d(pt[0],pt[1])); return vertices; }

    do {
        ++STEPS;
        FloatDPApproximation cot_theta_max(-inf,dp);
        SizeType s=np; // The index giving the variable x[p[s]] to enter the basis

        // Compute direction the point Gx moves in when variable x[j]=x[p[k]] enters the basis
        // This direction is given by Gd where d_B=-A_B^{-1} A_N e_j and
        //   di=+1/-1 depending on whether variable x[j] is at lower or upper bound
        for(SizeType k=nc; k!=np; ++k) {
            // Test variable to enter basis; there are m to test, one for each dimension of the domain
            SizeType j=p[k];
            if(j!=last_exiting_variable || true) {
                Aj=column(A,j);
                BAj=B*Aj;

                // Compute the direction the point in space moves for x[j] entering the basis
                Vector<FloatDPApproximation> d(np,FloatDPApproximation(0.0_x,dp)); // The direction x moves in in changing the basis
                d[j] = (vt[j]==Slackness::LOWER ? +1 : -1);
                for(SizeType i=0; i!=nc; ++i) {
                    d[p[i]]=-BAj[i]*d[j];
                }
                trial_vec=G*d;

                // Compare the direction moved in this step with the last step.
                // dot compares the direction of the two steps;
                //   positive means an obtuse angle i.e. the same general direction.
                // cross compares whether we turn to the left or the right.
                // Since we traverse the boundary anticlockwise, cross should be positive.
                FloatDPApproximation dot=last_vec[0]*trial_vec[0]+last_vec[1]*trial_vec[1];
                FloatDPApproximation cross=last_vec[0]*trial_vec[1]-last_vec[1]*trial_vec[0];
                FloatDPApproximation cot_theta=dot/cross;
                CONCLOG_PRINTLN_AT(1,"k="<<k<<" p[k]="<<p[k]<<" d="<<d<<" trial_vec="<<trial_vec<<" last_vec="<<last_vec<<
                              " dot="<<dot<<" cross="<<cross<<" cot="<<cot_theta);


                // Due to roundoff error, the computed cross-product may be negative.
                // If this lies within a reasonable tolerance, set to zero,
                // otherwise abort.
                ARIADNE_ASSERT_MSG(cross.raw() >= -ERROR_TOLERANCE,
                                   "ValidatedAffineConstrainedImageSet::boundary(...): cross product is="<<cross<<"; should be positive.");

                if(cross.raw()<=0.0_x ) {
                    cross=+0;
                    cot_theta=(dot.raw()>0.0_x) ? +inf : -inf;
                }

                // Allow for equality; in particular, if cot_theta=-infty,
                // then may turn round and go backwards.
                if(cot_theta.raw()>=cot_theta_max.raw()) {
                    cot_theta_max=cot_theta;
                    best_next_vec=trial_vec;
                    s=k;
                }
            } // k!=r
        }

        ARIADNE_ASSERT_MSG(s<np,"Could not find direction to move along boundary of ValidatedAffineConstrainedImageSet.");
        ARIADNE_DEBUG_ASSERT(vt[p[s]]!=Slackness::BASIS);
        CONCLOG_PRINTLN_AT(1,"Choosing variable x["<<p[s]<<"]=x[p["<<s<<"]] to enter basis");
        lpsolver.lpstep(l,u,A,b, vt,p,B,x,s);
        last_exiting_variable=p[s];
        pt=G*x+h;
        last_vec=best_next_vec;
        CONCLOG_PRINTLN_AT(2,"G="<<G<<" h="<<h<<" x="<<x<<" pt="<<pt);
        CONCLOG_PRINTLN_AT(1,"A="<<A<<" b="<<b<<" l="<<l<<" u="<<u<<" vt="<<vt<<" p="<<p<<" x="<<x<<" Ax="<<A*x<<" pt="<<pt<<" vec="<<best_next_vec);

        vertices.push_back(Point2d(pt[0].raw(),pt[1].raw()));

    } while(STEPS<MAX_STEPS && vt!=initial_variable_type);

    CONCLOG_PRINTLN("vertices="<<vertices);
    return vertices;

}

Void ValidatedAffineConstrainedImageSet::draw(CanvasInterface& canvas, const Projection2d& projection) const {
    CONCLOG_SCOPE_CREATE;
    CONCLOG_PRINTLN("set="<<*this);
    CONCLOG_PRINTLN("projection="<<projection);

    List<Point2d> boundary;

    try {
        boundary=this->boundary(projection.x_coordinate(),projection.y_coordinate());
        CONCLOG_PRINTLN("boundary="<<boundary);
    } catch(const std::runtime_error& e) {
        throw e;
    }

    if(boundary.empty()) { return; }

    canvas.fill_boundary(boundary);
}


OutputStream& ValidatedAffineConstrainedImageSet::_write(OutputStream& os) const {
    return os << "ValidatedAffineConstrainedImageSet( domain=" << this->_domain << ", function=" << this->_space_models << ", constraints=" << this->_constraint_models <<" )";
}


} // namespace Ariadne
