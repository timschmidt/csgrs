/***************************************************************************
 *            multifunction.hpp
 *
 *  Copyright 2008-21  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*! \file multifunction.hpp
 *  \brief Multivalued functions
 */

#ifndef ARIADNE_MULTIFUNCTION_HPP
#define ARIADNE_MULTIFUNCTION_HPP

#include <cstdarg>
#include <iosfwd>
#include <iostream>

#include "utility/declarations.hpp"
#include "utility/macros.hpp"
#include "utility/pointer.hpp"
#include "utility/container.hpp"
#include "utility/metaprogramming.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/differential.hpp"

#include "function/function.hpp"
//#include "function/function_model.hpp"

#include "geometry/set.hpp"
#include "geometry/interval.hpp"
#include "geometry/box.hpp"
#include "geometry/function_set.hpp"



namespace Ariadne {


template<class P> class GenericPoint {
    class Interface {
        virtual ~Interface() = 0;
        virtual Vector<Interval<Number<P>>> _bounds() const;
        GenericPoint<P> _apply(VectorFunction<P> const& f) const;
    };
    SharedPointer<const Interface> _ptr;
  public:
    GenericPoint(SharedPointer<const Interface> ptr) : _ptr(ptr) { }
    Vector<Interval<Number<P>>> bounds() { return this->_ptr->_bounds(); }
    Interval<Number<P>> operator[](SizeType i) { return this->bounds()[i]; }
    friend GenericPoint<P> apply(VectorFunction<P> const& f, GenericPoint<P> const& pt) { return pt._ptr->_apply(f); }
};

template<class P> class ImageGenericPoint : public GenericPoint<P>::Interface {
    VectorFunction<P> _h;
    ImageGenericPoint(VectorFunction<P> const& h) : _h(h) { }
    friend ImageGenericPoint<P> apply(VectorFunction<P> const& f, GenericPoint<P> const& pt) {
        return ImageGenericPoint<P>(f(pt._h)); }
};


template<class P, class SIG, template<class,class>class SET=LocatedSet> class Multifunction;

template<class P> using VectorMultivariateMultifunction = Multifunction<P,RealVector(RealVector)>;
using ValidatedVectorMultivariateMultifunction = VectorMultivariateMultifunction<ValidatedTag>;


template<class P, class SIG, template<class,class>class SET=LocatedSet> class MultifunctionInterface;

template<class P, class SIG, template<class,class>class SET> class MultifunctionInterface {
    using ARG=typename SignatureTraits<SIG>::ArgumentKind;
    using RES=typename SignatureTraits<SIG>::ResultKind;
  public:
    template<class Y> using Argument = typename SignatureTraits<SIG>::template Argument<Y>;
    virtual ~MultifunctionInterface() = default;
    virtual SET<P,RES> _call(Argument<Number<P>> const& x) const = 0;
    virtual OutputStream& _write(OutputStream& os) const = 0;
};

template<class F, class P, class SIG, template<class,class>class SET> struct IsMultifunction {
    using ARG=typename SignatureTraits<SIG>::ArgumentKind;
    using RES=typename SignatureTraits<SIG>::ResultKind;
    template<class Y> using Argument = typename SignatureTraits<SIG>::template Argument<Y>;

    template<class FF, class=decltype(declval<SET<P,RES>>()=declval<FF>()(declval<Argument<Number<P>>>()))>
        static std::true_type test(int);
    template<class FF>
        static std::false_type test(...);
    static const bool value = decltype(test<F>(1))::value;
};

template<class F, class P, class SIG, template<class,class>class SET> concept AMultifunction = IsMultifunction<F,P,SIG,SET>::value;


//! \ingroup Function
//! \brief Functions \f$\X\mvto\Y\f$ returning a \ref LocatedSet at each point.
template<class P, class SIG, template<class,class>class SET> class Multifunction
    : public Handle<const MultifunctionInterface<P,SIG,SET>>
{
    using ARG=typename SignatureTraits<SIG>::ArgumentKind;
    using RES=typename SignatureTraits<SIG>::ResultKind;
    using ResultSetType = SET<P,RES>;
  public:
    typedef MultifunctionInterface<P,SIG,SET> Interface;
    template<class Y> using Argument = typename SignatureTraits<SIG>::template Argument<Y>;

    using Handle<const Interface>::Handle;
    //! \brief <p/>
    template<AMultifunction<P,SIG,SET> MF> explicit Multifunction(MF const& mf);
    //! \brief <p/>
    SET<P,RES> operator() (Argument<Number<P>> const& x) const { return this->reference()._call(x); }
    //! \brief <p/>
    friend SET<P,RES> apply (Multifunction<P,SIG,SET> const& f, SET<P,ARG> const& x);
    //! \brief <p/>
    friend OutputStream& operator<<(OutputStream& os,Multifunction<P,SIG,SET> const& mf) { return mf.reference()._write(os); }
};

template<class MF, class P, class SIG, template<class,class>class SET> class MultifunctionWrapper
    : public virtual MultifunctionInterface<P,SIG,SET>
{
    using ARG=typename SignatureTraits<SIG>::ArgumentKind;
    using RES=typename SignatureTraits<SIG>::ResultKind;
  public:
    template<class Y> using Argument = typename SignatureTraits<SIG>::template Argument<Y>;

    MultifunctionWrapper(MF mf) : _mf(mf) { }
    MF const& base() const { return this->_mf; }
    virtual SET<P,RES> _call(Argument<Number<P>> const& x) const final override {
        return static_cast<SET<P,RES>>(this->_mf(x)); }
    virtual OutputStream& _write(OutputStream& os) const  final override {
        return os << this->base(); }
  private:
    MF _mf;
};

template<class P, class SIG,template<class,class>class SET> template<AMultifunction<P,SIG,SET> MF>
Multifunction<P,SIG,SET>::Multifunction(MF const& mf)
    : Multifunction<P,SIG,SET>(std::make_shared<const MultifunctionWrapper<MF,P,SIG,SET>>(mf)) {
}


using ValidatedImageSet = ValidatedConstrainedImageSet;

template<class P, class SIG, template<class,class>class SET=LocatedSet> class MultifunctionPatch;
template<class P> using VectorMultivariateMultifunctionPatch = MultifunctionPatch<P,RealVector(RealVector)>;
using ValidatedVectorMultivariateMultifunctionPatch = VectorMultivariateMultifunctionPatch<ValidatedTag>;

template<class P> class MultifunctionPatch<P,RealVector(RealVector)> {
    using ARG=RealVector; using RES=RealVector; using SIG=RES(ARG);
    Function<P,SIG> _f;
    BoxDomainType _pdom;
  public:
    MultifunctionPatch(VectorMultivariateFunction<P> f, BoxDomainType pdom) : _f(f), _pdom(pdom) { }

    SizeType argument_size() const { return this->_f.argument_size()-this->_pdom.dimension(); }
    BoxDomainType error_domain() const { return this->_pdom; }

    ValidatedImageSet operator() (Vector<ValidatedNumber> const& x) const;

    friend OutputStream& operator<<(OutputStream& os, MultifunctionPatch<P,SIG> const& fm) {
        return os << "MultifunctionPatch(function="<<fm._f<<", parameter_domain="<<fm._pdom<<")"; }
};

template<class P> auto
MultifunctionPatch<P,RealVector(RealVector)>::operator() (Vector<ValidatedNumber> const& x) const -> ValidatedImageSet
{
    auto fx=ValidatedFunction<SIG>::constant(error_domain().dimension(),x);
    auto fid=ValidatedFunction<SIG>::identity(error_domain().dimension());
    Function<P,SIG> fim=compose(_f,join(fx,fid));
    return ValidatedImageSet(this->error_domain(),fim);
}

static_assert(AMultifunction<MultifunctionPatch<ValidatedTag,RealVector(RealVector)>,ValidatedTag,RealVector(RealVector),LocatedSet>);



template<class PR> class Sweeper;

template<class P, class SIG, class PR> class MultifunctionModel;
template<class P, class PR> using VectorMultivariateMultifunctionModel = MultifunctionModel<P,RealVector(RealVector),PR>;
template<class PR> using ValidatedVectorMultivariateMultifunctionModel = VectorMultivariateMultifunctionModel<ValidatedTag,PR>;

template<class P, class PR> class MultifunctionModel<P,RealVector(RealVector),PR> {
    using ARG=RealVector; using RES=RealVector; using SIG=RES(ARG);
    using FLT=RawFloatType<PR>;
    FunctionModel<P,SIG,PR> _f;
    SizeType _as;
  public:
    MultifunctionModel(BoxDomainType dom, VectorMultivariateFunction<P> f, BoxDomainType params, Sweeper<FLT> swp);

    SizeType argument_size() const { return this->_as; }
    BoxDomainType domain() const { return project(this->_f.domain(),Range(0u,this->argument_size())); }
    BoxDomainType error_domain() const { return project(this->_f.domain(),Range(this->argument_size(),this->_f.argument_size())); }

    ValidatedImageSet operator() (Vector<ValidatedNumber> const& x) const;

    friend OutputStream& operator<<(OutputStream& os, MultifunctionModel<P,SIG,PR> const& fm) {
        return os << "MultifunctionModel(model="<<fm._f<<", argument_size="<<fm._as<<")"; }
};


template<class P, class PR> auto
MultifunctionModel<P,RealVector(RealVector),PR>::operator() (Vector<ValidatedNumber> const& x) const -> ValidatedImageSet
{
    auto fx=factory(this->_f).create_constants(error_domain(),x);
    auto fid=factory(this->_f).create_identity(error_domain());
    FunctionModel<P,SIG,PR> fim=compose(_f,join(fx,fid));
    return ValidatedImageSet(this->error_domain(),fim);
}

static_assert(AMultifunction<MultifunctionModel<ValidatedTag,RealVector(RealVector),DoublePrecision>,ValidatedTag,RealVector(RealVector),LocatedSet>);


template<class P, class RES, class ARG> class Function<P,CompactSet<P,RES>(ARG)>
    : public Multifunction<P,RES(ARG),CompactSet>
{
    using Multifunction<P,RES(ARG),CompactSet>::Multifunction;
};
template<class P, class RES, class ARG> class Function<P,LocatedSet<P,RES>(ARG)>
    : public Multifunction<P,RES(ARG),LocatedSet>
{
    using Multifunction<P,RES(ARG),LocatedSet>::Multifunction;
};



} // namespace Ariadne



#include "../geometry/set_wrapper.hpp"

namespace Ariadne {

template<class P, class SIG, template<class,class>class SET=LocatedSet> class FunctionSet;

template<class F, class P, class SIG, template<class,class>class SET> struct IsFunctionSet {
    using ARG=typename SignatureTraits<SIG>::ArgumentKind;
    using RES=typename SignatureTraits<SIG>::ResultKind;
    template<class Y> using Argument = typename SignatureTraits<SIG>::template Argument<Y>;

    template<class FF, class=decltype(declval<SET<P,RES>>()=declval<FF>()(declval<Argument<Number<P>>>()))>
        static std::true_type test(int);
    template<class FF>
        static std::false_type test(...);
    static const bool value = decltype(test<F>(1))::value;
};

template<class F, class P, class SIG, template<class,class>class SET> concept AFunctionSet = IsFunctionSet<F,P,SIG,SET>::value;


template<class P, class SIG, template<class,class>class SET> class FunctionSetInterface {
    using ARG=typename SignatureTraits<SIG>::ArgumentKind;
    using RES=typename SignatureTraits<SIG>::ResultKind;
  public:
    template<class Y> using Argument = typename SignatureTraits<SIG>::template Argument<Y>;
    virtual ~FunctionSetInterface() = default;
    virtual SET<P,RES> _call(Argument<Number<P>> const& x) const = 0;
    virtual OutputStream& _write(OutputStream& os) const = 0;
};

template<class P, class SIG, template<class,class>class SET> class FunctionSet
    : public Handle<const FunctionSetInterface<P,SIG,SET>>
{
    using ARG=typename SignatureTraits<SIG>::ArgumentKind;
    using RES=typename SignatureTraits<SIG>::ResultKind;
  public:
    typedef FunctionSetInterface<P,SIG,SET> Interface;
    template<class Y> using Argument = typename SignatureTraits<SIG>::template Argument<Y>;

    using Handle<const Interface>::Handle;

    //! \brief <p/>
    template<AFunctionSet<P,SIG,SET> FS> explicit FunctionSet(FS const& fs);
    //! \brief <p/>
    SET<P,RES> operator() (Argument<Number<P>> const& x) const { return this->reference()._call(x); }
    //! \brief <p/>
    friend SET<P,RES> apply (FunctionSet<P,SIG,SET> const& fs, SET<P,ARG> const& x);
    //! \brief <p/>
    friend OutputStream& operator<<(OutputStream& os,FunctionSet<P,SIG,SET> const& fs) { return fs.reference()._write(os); }
};

template<class FS, class P, class SIG, template<class,class>class SET> class FunctionSetWrapper
    : public virtual FunctionSetInterface<P,SIG,SET>
{
    using ARG=typename SignatureTraits<SIG>::ArgumentKind;
    using RES=typename SignatureTraits<SIG>::ResultKind;
  public:
    template<class X> using Argument = typename SignatureTraits<SIG>::template Argument<X>;

    FunctionSetWrapper(FS mf) : _fs(mf) { }
    FS const& base() const { return this->_fs; }
    virtual SET<P,RES> _call(Argument<Number<P>> const& x) const final override {
        return static_cast<SET<P,RES>>(this->_fs(x)); }
    virtual OutputStream& _write(OutputStream& os) const  final override {
        return os << this->base(); }
  private:
    FS _fs;
};

template<class P, class SIG,template<class,class>class SET> template<AFunctionSet<P,SIG,SET> FS>
FunctionSet<P,SIG,SET>::FunctionSet(FS const& fs)
    : Handle<const Interface>(std::make_shared<const FunctionSetWrapper<FS,P,SIG,SET>>(fs)) { }

template<class RES, class ARG> class CompactSet<ValidatedTag,RES(ARG)>
    : public FunctionSet<ValidatedTag,RES(ARG),CompactSet>
{
    using FunctionSet<ValidatedTag,RES(ARG),CompactSet>::FunctionSet;
};
template<class RES, class ARG> class LocatedSet<ValidatedTag,RES(ARG)>
    : public FunctionSet<ValidatedTag,RES(ARG),LocatedSet>
{
    using FunctionSet<ValidatedTag,RES(ARG),LocatedSet>::FunctionSet;
};




template<class A1, class A2> using JoinType = decltype(join(declval<A1>(),declval<A2>()));


template<class P, class RES, class PAR, class ARG, template<class,class>class SET> class FunctionSetFunction {
    typedef JoinType<ARG,PAR> ARGS;
  public:
    template<class Y> using Argument = typename DomainTraits<ARG>::template Type<Y>;
    typedef SET<P,RES(PAR)> FunctionSetType;
    class Interface {
      private: public:
        virtual SET<P,RES(PAR)> _call(Argument<Number<P>> const&) const = 0;
        virtual Function<P,SET<P,RES>(ARGS)> _curry() const = 0;
        virtual OutputStream& _write(OutputStream&) const = 0;
    };
    SET<P,RES(PAR)> operator() (Argument<Number<P>> const& x) const { return this->_handle._ptr->_call(x); }
    Function<P,SET<P,RES>(ARGS)> curry() const { return this->_handle._ptr->_curry(); }
  private:
    Handle<Interface> _handle;
};


template<class P, class RES, class PAR, class ARG> class Function<P,CompactSet<P,RES(PAR)>(ARG)>
    : public FunctionSetFunction<P,RES,PAR,ARG,CompactSet>
{ };

template<class P, template<class,class>class SET=LocatedSet> using Multiflow = Function<P,SET<P,RealVector(Real)>(RealVector)>;
template<class P> using CompactMultiflow = Function<P,CompactSet<P,RealVector(Real)>(RealVector)>;
template<class P> using LocatedMultiflow = Multiflow<P,LocatedSet>;

using ValidatedCompactMultiflow = Multiflow<ValidatedTag,CompactSet>;



} // namespace Ariadne

#endif

