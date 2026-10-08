/****************************************************************************
 *            geometry/polytope.cpp
 *
 *  Copyright  2007-20  Alberto Casagrande, Pieter Collins
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

#include "helper/stlio.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "geometry/point.hpp"
#include "geometry/box.hpp"
#include "geometry/zonotope.hpp"
#include "geometry/curve.hpp"
#include "function/function.hpp"

#include "geometry/polytope.hpp"

namespace Ariadne {

ValidatedKleenean
Polytope::separated(const ExactBoxType& bx) const {
    return this->bounding_box().separated(bx) || indeterminate;
}

ValidatedKleenean
Polytope::overlaps(const ExactBoxType& bx) const {
    return bx.covers(baricentre(*this)) || indeterminate;
}

ValidatedKleenean
Polytope::inside(const ExactBoxType& bx) const {
    return this->bounding_box().inside(bx) || indeterminate;
}

ExactBoxType
Polytope::bounding_box() const
{
    const Polytope& p=*this;
    ExactBoxType res(p._vertices[0]);
    for(SizeType i=1; i!=p._vertices.size(); ++i) {
        res=hull(res,p._vertices[i]);
    }
    return res;
}


OutputStream&
Polytope::_write(OutputStream& os) const
{
    return os << "Polytope( vertices=" << this->vertices() << " )";
}






FloatDP
slope2d(const ExactPoint& pt1, const ExactPoint& pt2)
{
    ARIADNE_ASSERT(pt1.dimension()==2 && pt2.dimension()==2);
    return (pt2[1]-pt1[1])/(pt2[0]-pt1[0]);
}



Polytope polytope(const ExactBoxType& bx)
{
    return Polytope(bx.vertices());
}


Polytope polytope(const Zonotope& z)
{
    //std::cerr<<ARIADNE_PRETTY_FUNCTION<<std::endl;
    Polytope res;
    SizeType ng=z.generators().column_size();
    SizeType nv=1<<ng;
    for(SizeType i=0; i!=nv; ++i) {
        ExactPoint pt=z.centre();
        for(SizeType j=0; j!=ng; ++j) {
            if(i & 1<<j) {
                pt+=column(z.generators(),j);
            } else {
                pt-=column(z.generators(),j);
            }
        }
        res.new_vertex(pt);
    }
    if(res.dimension()==2) {
        return reduce2d(res);
    } else {
        return res;
    }
}


Polytope polytope(const Polytope& p)
{
    Polytope r=p;
    if(r.dimension()==2) {
        return reduce2d(r);
    } else {
        return r;
    }
}


Polytope& reduce2d(Polytope& p)
{
    ARIADNE_ASSERT(p.dimension()==2);
    //std::cerr << this->_vertices << std::endl;
    std::sort(p.vertices().begin(),p.vertices().end());
    //std::cerr << this->_vertices << std::endl;

    const std::vector<ExactPoint>& old_vertices=p.vertices();
    std::vector<ExactPoint> new_vertices;

    // Sweep lower boundary from bottom-left to top right
    SizeType min_size=1;
    for(std::vector<ExactPoint>::ConstIterator vertex_iter=old_vertices.begin();
        vertex_iter!=old_vertices.end(); ++vertex_iter)
        {
            const ExactPoint& vertex=*vertex_iter;
            while(new_vertices.size()>min_size) {
                const ExactPoint& penultimate=new_vertices[new_vertices.size()-2];
                const ExactPoint& last=new_vertices[new_vertices.size()-1];
                if(penultimate[0]==last[0]) {
                    new_vertices.pop_back();
                } else if(last[0]==vertex[0]) {
                    break;
                } else if(slope2d(penultimate,vertex) <= slope2d(penultimate,last)) {
                    new_vertices.pop_back();
                } else {
                    break;
                }
            }
            new_vertices.push_back(vertex);
        }
    new_vertices.pop_back();

    // UpperTag sweep
    //std::cerr << "Forward pass\n";
    min_size=new_vertices.size()+1;
    for(std::vector<ExactPoint>::const_reverse_iterator vertex_iter=old_vertices.rbegin();
        vertex_iter!=old_vertices.rend(); ++vertex_iter)
        {
            const ExactPoint& vertex=*vertex_iter;
            while(new_vertices.size()>min_size) {
                const ExactPoint& penultimate=new_vertices[new_vertices.size()-2];
                const ExactPoint& last=new_vertices[new_vertices.size()-1];
                if(penultimate[0]==last[0]) {
                    new_vertices.pop_back();
                } else if(last[0]==vertex[0]) {
                    break;
                } else if(slope2d(penultimate,vertex) <= slope2d(penultimate,last)) {
                    new_vertices.pop_back();
                } else {
                    break;
                }
            }
            new_vertices.push_back(vertex);
        }
    new_vertices.pop_back();

    new_vertices.swap(p.vertices());
    return p;
}


ExactPoint
baricentre(const Polytope& p)
{
    const std::vector<ExactPoint>& vertices=p.vertices();
    ExactPoint baricentre(p.dimension());

    for (SizeType j=0; j!=vertices.size(); ++j) {
        for (SizeType i=0; i<2; i++) {
            baricentre[i]=baricentre[i]+vertices[j][i];
        }
    }
    for (SizeType i=0; i!=2; ++i) {
        baricentre[i]/=vertices.size();
    }
    return baricentre;
}









Void Polytope::draw(CanvasInterface& c, const Projection2d& p) const {
    SizeType xi=p.x_coordinate(); SizeType yi=p.y_coordinate();
    ARIADNE_ASSERT(max(xi,yi)<this->dimension());

    Polytope pr(2);
    ExactPoint prv(2);

    // Project polytope onto canvas coordinates
    for(SizeType i=0; i!=this->number_of_vertices(); ++i) {
        const ExactPoint& v=this->vertex(i);
        prv[0]=v[xi]; prv[1]=v[yi];
        pr.new_vertex(prv);
    }

    // Reduce boundary of projected polytope
    reduce2d(pr);

    // Trace boundary
    prv=pr.vertex(pr.number_of_vertices()-1);
    c.move_to(prv[0],prv[1]);
    for(SizeType i=0; i!=pr.number_of_vertices(); ++i) {
        prv=pr.vertex(i);
        c.line_to(prv[0],prv[1]);
    }

    c.fill();
}


} // namespace Ariadne
