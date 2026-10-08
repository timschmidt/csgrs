/***************************************************************************
 *            algebra/tensor.tpl.hpp
 *
 *  Copyright  2008-21  Mirko Albanese
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

/*! \file tensor.tpl.hpp
 *  \brief Specialisation of drawing routines for Tensor.
 */

#include "algebra/tensor.hpp"
#include "numeric/float.decl.hpp"
#include "algebra/algebra.hpp"

#include "numeric/floatdp.hpp"
#include "numeric/floatmp.hpp"
#include "numeric/float_bounds.hpp"

namespace Ariadne {

template<SizeType N,class X> Void Tensor<N, X>::draw(CanvasInterface& canvas, const Projection2d& p) const { }
template <SizeType N, class X> Void Tensor<N, X>::draw(CanvasInterface& canvas, const Projection3d& p) const { }

template<> Void Tensor<2ul, Ariadne::Vector<Ariadne::Bounds<Ariadne::FloatDP>>>::draw(Ariadne::CanvasInterface&, Ariadne::Variables3d const&) const { }

template <> Void Tensor<2, Float<DoublePrecision>>::draw(CanvasInterface& canvas, const Projection2d& p) const {
        //2D Drawing
        for(SizeType frame=0; frame!=_ns[1]; ++frame){
            SizeType index = _index({0, frame});
            canvas.move_to(0.0, _a[index].get_d());
            for(SizeType i=1; i!=_ns[0]; ++i){
                index = _index({i, frame});
                canvas.line_to(numeric_cast<double>(i), _a[index].get_d());
            }
            canvas.fill();       
        }
    }

template<> Void Tensor<2, Float<DoublePrecision>>::draw(CanvasInterface& canvas, const Projection3d& p) const { }

template<> Void Tensor<2, Float<MultiplePrecision>>::draw(CanvasInterface& canvas, const Projection2d& p) const {
    //2D Drawing
    for(SizeType frame=0; frame!=_ns[1]; ++frame){
        SizeType index = _index({0, frame});
        canvas.move_to(0.0, _a[index].get_d());
        for(SizeType i=1; i!=_ns[0]; ++i){
            index = _index({i, frame});
            canvas.line_to(numeric_cast<double>(i), _a[index].get_d());
        }
        canvas.fill();
    }
}

template <> Void Tensor<2, Float<MultiplePrecision>>::draw(CanvasInterface& canvas, const Projection3d& p) const { }
template <> Void Tensor<3, Float<DoublePrecision>>::draw(CanvasInterface& canvas, const Projection2d& p) const {
    ARIADNE_ASSERT(p.argument_size() == this->dimension());
    if(p.x_coordinate() == 0 && p.y_coordinate() == 1){
        canvas.set_heat_map(true);
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            SizeType index = _index({0, 0, frame});
            canvas.move_to(0.0, _a[index].get_d());
            for(SizeType x2=0; x2!=_ns[1]; ++x2){
                for(SizeType x1=0; x1!=_ns[0]; ++x1){
                    if (x2 == 0 && x1 == 0){
                        x1++;
                    }
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.line_to(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
            }
            canvas.fill_3d();
        }
    }
    else if(p.x_coordinate() == 1 && p.y_coordinate() == 0)
    {
        canvas.set_heat_map(true);
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            SizeType index = _index({0, 0, frame});
            canvas.move_to(0.0, _a[index].get_d());
            for(SizeType x1=0; x1!=_ns[1]; ++x1){
                for(SizeType x2=0; x2!=_ns[0]; ++x2){
                    if (x2 == 0 && x1 == 0){
                        x2++;
                    }
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.line_to(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
            }
            canvas.fill_3d();
        }
    }
    else if(p.x_coordinate() == 0 && p.y_coordinate() == 2)
    {
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x2=0; x2!=_ns[1]; ++x2){
                SizeType index = _index({0, x2, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x1=1; x1!=_ns[0]; ++x1){
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.fill();
            }
        }
    }
    else if(p.x_coordinate() == 2 && p.y_coordinate() == 0)
    {
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x2=0; x2!=_ns[1]; ++x2){
                SizeType index = _index({0, x2, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x1=1; x1!=_ns[0]; ++x1){
                    index = _index({x1, x2, frame});
                    canvas.line_to(_a[index].get_d(), numeric_cast<double>(x1));
                }
                canvas.fill();
            }
        } 
    }
    else if(p.x_coordinate() == 1 && p.y_coordinate() == 2){
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x1=0; x1!=_ns[1]; ++x1){
                SizeType index = _index({x1, 0, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x2=1; x2!=_ns[0]; ++x2){
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.fill();
            }
        }
    }
    else if(p.x_coordinate() == 2 && p.y_coordinate() == 1){
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x1=0; x1!=_ns[1]; ++x1){
                SizeType index = _index({x1, 0, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x2=1; x2!=_ns[0]; ++x2){
                    index = _index({x1, x2, frame});
                    canvas.line_to(_a[index].get_d(), numeric_cast<double>(x1));
                }
                canvas.fill();
            }
        }
    }
    
 }

template <> Void Tensor<3, Float<DoublePrecision>>::draw(CanvasInterface& canvas, const Projection3d& p) const {
    for(SizeType frame=0; frame!=_ns[2]; ++frame){
    SizeType index = _index({0, 0, frame});
    canvas.move_to(0.0, _a[index].get_d());
    for(SizeType x2=0; x2!=_ns[1]; ++x2){
        for(SizeType x1=0; x1!=_ns[0]; ++x1){
            if (x2 == 0 && x1 == 0){
                x1++;
            }
            index = _index({x1, x2, frame});
            canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
        }
        canvas.line_to(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
    }
        canvas.fill_3d();
    }
}

template <> Void Tensor<3, Float<MultiplePrecision>>::draw(CanvasInterface& canvas, const Projection2d& p) const {
    ARIADNE_ASSERT(p.argument_size() == this->dimension());
    if(p.x_coordinate() == 0 && p.y_coordinate() == 1){
        canvas.set_heat_map(true);
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            SizeType index = _index({0, 0, frame});
            canvas.move_to(0.0, _a[index].get_d());
            for(SizeType x2=0; x2!=_ns[1]; ++x2){
                for(SizeType x1=0; x1!=_ns[0]; ++x1){
                    if (x2 == 0 && x1 == 0){
                        x1++;
                    }
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.line_to(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
            }
            canvas.fill_3d();
        }
    }
    else if(p.x_coordinate() == 1 && p.y_coordinate() == 0)
    {   canvas.set_heat_map(true);
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            SizeType index = _index({0, 0, frame});
            canvas.move_to(0.0, _a[index].get_d());
            for(SizeType x1=0; x1!=_ns[1]; ++x1){
                for(SizeType x2=0; x2!=_ns[0]; ++x2){
                    if (x2 == 0 && x1 == 0){
                        x2++;
                    }
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.line_to(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
            }
            canvas.fill_3d();
        }
    }
    else if(p.x_coordinate() == 0 && p.y_coordinate() == 2)
    {
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x2=0; x2!=_ns[1]; ++x2){
                SizeType index = _index({0, x2, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x1=1; x1!=_ns[0]; ++x1){
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.fill();
            }
        }
    }
    else if(p.x_coordinate() == 2 && p.y_coordinate() == 0)
    {
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x2=0; x2!=_ns[1]; ++x2){
                SizeType index = _index({0, x2, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x1=1; x1!=_ns[0]; ++x1){
                    index = _index({x1, x2, frame});
                    canvas.line_to(_a[index].get_d(), numeric_cast<double>(x1));
                }
                canvas.fill();
            }
        } 
    }
    else if(p.x_coordinate() == 1 && p.y_coordinate() == 2){
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x1=0; x1!=_ns[1]; ++x1){
                SizeType index = _index({x1, 0, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x2=1; x2!=_ns[0]; ++x2){
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.fill();
            }
        }
    }
    else if(p.x_coordinate() == 2 && p.y_coordinate() == 1){
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x1=0; x1!=_ns[1]; ++x1){
                SizeType index = _index({x1, 0, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x2=1; x2!=_ns[0]; ++x2){
                    index = _index({x1, x2, frame});
                    canvas.line_to(_a[index].get_d(), numeric_cast<double>(x1));
                }
                canvas.fill();
            }
        }
    }
 }

template <> Void Tensor<3, Float<MultiplePrecision>>::draw(CanvasInterface& canvas, const Projection3d& p) const {
    for(SizeType frame=0; frame!=_ns[2]; ++frame){
    SizeType index = _index({0, 0, frame});
    canvas.move_to(0.0, _a[index].get_d());
    for(SizeType x2=0; x2!=_ns[1]; ++x2){
        for(SizeType x1=0; x1!=_ns[0]; ++x1){
            if (x2 == 0 && x1 == 0){
                x1++;
            }
            index = _index({x1, x2, frame});
            canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
        }
        canvas.line_to(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
    }
        canvas.fill_3d();
    }
}

template <SizeType N, class X> Void Tensor<N, X>::draw(CanvasInterface& canvas, const Variables2d& p) const { }
template <SizeType N, class X> Void Tensor<N, X>::draw(CanvasInterface& canvas, const Variables3d& p) const { }

template <> Void Tensor<2, Float<DoublePrecision>>::draw(CanvasInterface& canvas, const Variables2d& p) const {
    for(SizeType frame=0; frame!=_ns[1]; ++frame){
        SizeType index = _index({0, frame});
        canvas.move_to(0.0, _a[index].get_d());
        for(SizeType i=1; i!=_ns[0]; ++i){
            index = _index({i, frame});
            canvas.line_to(numeric_cast<double>(i), _a[index].get_d());
        }
        canvas.fill();
    }
}

template <> Void Tensor<2, Float<DoublePrecision>>::draw(CanvasInterface& canvas, const Variables3d& p) const { }
template <> Void Tensor<2, Float<MultiplePrecision>>::draw(CanvasInterface& canvas, const Variables2d& p) const {
    for(SizeType frame=0; frame!=_ns[1]; ++frame){
        SizeType index = _index({0, frame});
        canvas.move_to(0.0, _a[index].get_d());
        for(SizeType i=1; i!=_ns[0]; ++i){
            index = _index({i, frame});
            canvas.line_to(numeric_cast<double>(i), _a[index].get_d());
        }
        canvas.fill();
    }
}

template <> Void Tensor<2, Float<MultiplePrecision>>::draw(CanvasInterface& canvas, const Variables3d& p) const { }
template <> Void Tensor<3, Float<DoublePrecision>>::draw(CanvasInterface& canvas, const Variables2d& p) const {
    if(p.x() == RealVariable("x") && p.y() == RealVariable("y")){
        canvas.set_heat_map(true);
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            SizeType index = _index({0, 0, frame});
            canvas.move_to(0.0, _a[index].get_d());
            for(SizeType x2=0; x2!=_ns[1]; ++x2){
                for(SizeType x1=0; x1!=_ns[0]; ++x1){
                    if (x2 == 0 && x1 == 0){
                        x1++;
                    }
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.line_to(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
            }
            canvas.fill_3d();
        }
    }
    else if(p.x() == RealVariable("y") && p.y() == RealVariable("x"))
    {   canvas.set_heat_map(true);
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            SizeType index = _index({0, 0, frame});
            canvas.move_to(0.0, _a[index].get_d());
            for(SizeType x1=0; x1!=_ns[1]; ++x1){
                for(SizeType x2=0; x2!=_ns[0]; ++x2){
                    if (x2 == 0 && x1 == 0){
                        x2++;
                    }
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.line_to(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
            }
            canvas.fill_3d();
        }
    }
    else if(p.x() == RealVariable("x") && p.y() == RealVariable("z"))
    {
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x2=0; x2!=_ns[1]; ++x2){
                SizeType index = _index({0, x2, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x1=1; x1!=_ns[0]; ++x1){
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.fill();
            }
        }
    }
    else if(p.x() == RealVariable("z") && p.y() == RealVariable("x"))
    {
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x2=0; x2!=_ns[1]; ++x2){
                SizeType index = _index({0, x2, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x1=1; x1!=_ns[0]; ++x1){
                    index = _index({x1, x2, frame});
                    canvas.line_to(_a[index].get_d(), numeric_cast<double>(x1));
                }
                canvas.fill();
            }
        } 
    }
    else if(p.x() == RealVariable("y") && p.y() == RealVariable("z")){
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x1=0; x1!=_ns[1]; ++x1){
                SizeType index = _index({x1, 0, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x2=1; x2!=_ns[0]; ++x2){
                    index = _index({x1, x2, frame});
                    canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
                }
                canvas.fill();
            }
        }
    }
    else if(p.x() == RealVariable("z") && p.y() == RealVariable("y")){
        for(SizeType frame=0; frame!=_ns[2]; ++frame){
            for(SizeType x1=0; x1!=_ns[1]; ++x1){
                SizeType index = _index({x1, 0, frame});
                canvas.move_to(0.0, _a[index].get_d());
                for(SizeType x2=1; x2!=_ns[0]; ++x2){
                    index = _index({x1, x2, frame});
                    canvas.line_to(_a[index].get_d(), numeric_cast<double>(x1));
                }
                canvas.fill();
            }
        }
    }
 }

template <> Void Tensor<3, Float<DoublePrecision>>::draw(CanvasInterface& canvas, const Variables3d& p) const {
    for(SizeType frame=0; frame!=_ns[2]; ++frame){
    SizeType index = _index({0, 0, frame});
    canvas.move_to(0.0, _a[index].get_d());
    for(SizeType x2=0; x2!=_ns[1]; ++x2){
        for(SizeType x1=0; x1!=_ns[0]; ++x1){
            if (x2 == 0 && x1 == 0){
                x1++;
            }
            index = _index({x1, x2, frame});
            canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
        }
        canvas.line_to(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
    }
        canvas.fill_3d();
    }
}

template <> Void Tensor<3, Float<MultiplePrecision>>::draw(CanvasInterface& canvas, const Variables2d& p) const {

 }
template <> Void Tensor<3, Float<MultiplePrecision>>::draw(CanvasInterface& canvas, const Variables3d& p) const {
    for(SizeType frame=0; frame!=_ns[2]; ++frame){
    SizeType index = _index({0, 0, frame});
    canvas.move_to(0.0, _a[index].get_d());
    for(SizeType x2=0; x2!=_ns[1]; ++x2){
        for(SizeType x1=0; x1!=_ns[0]; ++x1){
            if (x2 == 0 && x1 == 0){
                x1++;
            }
            index = _index({x1, x2, frame});
            canvas.line_to(numeric_cast<double>(x1), _a[index].get_d());
        }
        canvas.line_to(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
    }
        canvas.fill_3d();
    }
}

}
