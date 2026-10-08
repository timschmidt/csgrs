/***************************************************************************
 *            numeric/dispatching.hpp
 *
 *  Copyright  2013-22  Pieter Collins
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

/*! \file numeric/dispatching.hpp
 *  \brief Utility classes supporting double dispatching
 */



#ifndef ARIADNE_DISPATCHING_HPP
#define ARIADNE_DISPATCHING_HPP


namespace Ariadne {


template<class... YS> struct Aware;

template<class DI> struct Managed;
template<class DI> using ManagedTypes = typename Managed<DI>::Types;

template<class T> struct DispatcherTraits;
template<class T> using DispatcherInterface = typename DispatcherTraits<T>::Interface;
template<class T> using Dispatcher = typename DispatcherTraits<T>::Interface;

template<class I> struct InterfaceTraits;
template<class X, class I> using MixinType = typename InterfaceTraits<I>::template MixinType<X>;
template<class X, class I> using WrapperType = typename InterfaceTraits<I>::template WrapperType<X>;



template<class R, class OP, class YS, class BYS=YS> struct RightOperableInterface;
template<class R, class OP, class BYS, class Y> struct RightOperableInterface<R,OP,Aware<Y>,BYS> {
    virtual R _concrete_apply_right(OP op, Y const& y) const = 0;
};
template<class R, class OP, class BYS, class Y, class... YS> struct RightOperableInterface<R,OP,Aware<Y,YS...>,BYS>
    : public virtual RightOperableInterface<R,OP,Aware<YS...>,BYS>
{
    using RightOperableInterface<R,OP,Aware<YS...>,BYS>::_concrete_apply_right;
    virtual R _concrete_apply_right(OP op, Y const& y) const = 0;
};

template<class R, class OP, class YS, class RYS=YS> struct LeftOperableInterface;
template<class R, class OP, class RYS, class Y> struct LeftOperableInterface<R,OP,Aware<Y>,RYS>
    : public virtual RightOperableInterface<R,OP,RYS>
{
    virtual R _concrete_apply_left(OP op, Y const& y) const = 0;
};
template<class R, class OP, class RYS, class Y, class... YS> struct LeftOperableInterface<R,OP,Aware<Y,YS...>,RYS>
    : public virtual LeftOperableInterface<R,OP,Aware<YS...>,RYS>
{
    using LeftOperableInterface<R,OP,Aware<YS...>,RYS>::_concrete_apply_left;
    virtual R _concrete_apply_left(OP op, Y const& y) const = 0;
};

template<class R, class OP, class YS> struct SelfOperableInterface
    : public RightOperableInterface<R,OP,YS,YS> { };

template<class R, class OP, class YS> struct OperableInterface
    : public LeftOperableInterface<R,OP,YS,YS> { };


/*
template<class R, class OP, class X> inline R _concrete_apply(OP op, X const& x);
template<class R, class OP, class X1, class X2> inline R _concrete_apply(OP op, X1 const& x1, X2 const& x2);
*/

template<class X, class I, class R, class OP, class YS, class BYS=YS> struct RightOperableMixin;
template<class X, class I, class R, class OP, class BYS> struct RightOperableMixin<X,I,R,OP,Aware<>,BYS>
    : virtual RightOperableInterface<R,OP,BYS>
{
    X const& self() const { return static_cast<WrapperType<X,I>const&>(*this); }
};
template<class X, class I, class R, class OP, class BYS, class Y, class... YS> struct RightOperableMixin<X,I,R,OP,Aware<Y,YS...>,BYS>
    : RightOperableMixin<X,I,R,OP,Aware<YS...>,BYS>
{
    using RightOperableMixin<X,I,R,OP,Aware<YS...>,BYS>::_concrete_apply_right;
    virtual R _concrete_apply_right(OP op, Y const& y) const override { return _concrete_apply<R>(op,y,this->self()); }
};

template<class X, class I, class R, class OP, class YS, class BYS=YS> struct LeftOperableMixin;
template<class X, class I, class R, class OP, class BYS> struct LeftOperableMixin<X,I,R,OP,Aware<>,BYS>
    : virtual LeftOperableInterface<R,OP,BYS>
{
    X const& self() const { return static_cast<WrapperType<X,I>const&>(*this); }
};
template<class X, class I, class R, class OP, class BYS, class Y, class... YS> struct LeftOperableMixin<X,I,R,OP,Aware<Y,YS...>,BYS>
    : LeftOperableMixin<X,I,R,OP,Aware<YS...>,BYS>
{
    using LeftOperableMixin<X,I,R,OP,Aware<YS...>,BYS>::_concrete_apply_left;
    using LeftOperableMixin<X,I,R,OP,Aware<YS...>,BYS>::_concrete_apply_right;
    virtual R _concrete_apply_left(OP op, Y const& y) const override { return _concrete_apply<R>(op,this->self(),y); }
    virtual R _concrete_apply_right(OP op, Y const& y) const override { return _concrete_apply<R>(op,y,this->self()); }
};

template<class X, class I, class R, class OP, class YS> struct SelfOperableMixin
    : public RightOperableMixin<X,I,R,OP,YS,YS> { };
template<class X, class I, class R, class OP, class YS> struct OperableMixin
    : public LeftOperableMixin<X,I,R,OP,YS,YS> { };



template<class R, class X, class OP, class I> inline R _apply(X const& self, OP op, I const* self_ptr, I const* other_ptr) {
    using AW=ManagedTypes<Dispatcher<X>>;
    auto aware_other_ptr=dynamic_cast<RightOperableInterface<R,OP,AW> const*>(other_ptr);
    if(aware_other_ptr) { return aware_other_ptr->_concrete_apply_right(op,self); }
    else { return other_ptr->_rapply(op,self_ptr); }
}
template<class R, class X, class OP, class I> inline R _rapply(X const& self, OP op, I const* self_ptr, I const* other_ptr) {
    using AW=ManagedTypes<Dispatcher<X>>;
    auto aware_other_ptr=dynamic_cast<LeftOperableInterface<R,OP,AW> const*>(other_ptr);
    if(aware_other_ptr) { return aware_other_ptr->_concrete_apply_left(op,self); }
    else { return make_symbolic(op,other_ptr,self_ptr); }
}



template<class X, class RI, class OP, class I> struct UnaryOperationMixin : public virtual I {
    static X const& _cast(UnaryOperationMixin<X,RI,OP,I> const& self) { return static_cast<MixinType<X,I> const&>(self); }
    template<class R> static RI* _make_wrapper(R&& r) { return new WrapperType<R,I>(r); }
    virtual RI* _apply(OP op) const final { return _concrete_apply<RI*>(op,_cast(*this)); }
};

/*
template<class X, class RI, class OP, class I> struct UnaryOperationMixin : public virtual I {
    static X const& _cast(UnaryOperationMixin<X,RI,OP,I> const& self) { return static_cast<MixinType<X,I> const&>(self); }
    template<class R> static RI* _make_wrapper(R&& r) { return new WrapperType<R,I>(r); }
    virtual RI* _apply(OP op) const final { return _make_wrapper(op(_cast(*this))); }
    //virtual RI* _apply(OP op) const final { return _concrete_unary_apply<RI*,OP,X>(op,_cast(*this)); }
    //virtual RI* _apply(OP op) const final { X const& x=_cast(*this); return op.accept([&x](auto _op){return _make_wrapper(_op(x));}); }
    //virtual RI* _apply(OP op) const final { X const& x=_cast(*this); return op.accept([&x](auto _op){return _make_wrapper(_op(x));}); }
};
*/

template<class X, class RI, class OP, class I> struct BinaryOperationMixin : public virtual I {
    static X const& _cast(BinaryOperationMixin<X,RI,OP,I> const& self) { return static_cast<MixinType<X,I> const&>(self); }
    virtual RI* _apply(OP op, I const* other) const final { return Ariadne::_apply<RI*,X>(_cast(*this),op,static_cast<I const*>(this),other); }
    virtual RI* _rapply(OP op, I const* other) const final { return Ariadne::_rapply<RI*,X>(_cast(*this),op,static_cast<I const*>(this),other); }
};

template<class X, class RI, class OP, class I, class N> struct GradedOperationMixin : public virtual I {
    static X const& _cast(GradedOperationMixin<X,RI,OP,I,N> const& self) { return static_cast<MixinType<X,I> const&>(self); }
    template<class R> static RI* _make_wrapper(R&& r) { return new WrapperType<R,I>(r); }
    virtual RI* _apply(OP op, N n) const final { return _make_wrapper(op(_cast(*this),n)); }
};


} // namespace Ariadne

#endif /* ARIADNE_DISPATCHING_HPP */
