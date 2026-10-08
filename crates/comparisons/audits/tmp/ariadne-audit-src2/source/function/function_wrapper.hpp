/***************************************************************************
 *            function_wrapper.hpp
 *
 *  Copyright  2022  Pieter Collins
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

#ifndef ARIADNE_FUNCTION_WRAPPER_HPP
#define ARIADNE_FUNCTION_WRAPPER_HPP

#include "function_concepts.hpp"
#include "function_mixin.hpp"

namespace Ariadne {


template<class T, class I> class Wrapper;

template<class F, class P, class SIG> requires AFunction<F,P,SIG> class FunctionWrapper;

template<class F, class P, class SIG> struct Wrapper<F,FunctionInterface<P,SIG>> : public FunctionWrapper<F,P,SIG> { virtual ~Wrapper() = default; using FunctionWrapper<F,P,SIG>::FunctionWrapper; };

template<class P, class SIG, class F> requires AFunction<F,P,SIG> FunctionWrapper<F,P,SIG>* make_function_wrapper(F f);


template<class F, class P, class SIG> class FunctionWrapperGetterMixin;

template<class F, class P, class... ARGS> class FunctionWrapperGetterMixin<F,P,RealScalar(ARGS...)> {
};

template<class F, class P, class... ARGS> class FunctionWrapperGetterMixin<F,P,RealVector(ARGS...)>
    : public virtual VectorOfFunctionInterface<P,ARGS...>
{
    virtual ScalarFunctionInterface<P,ARGS...>* _get(SizeType i) const override;
};



template<class F, class P, class SIG> requires AFunction<F,P,SIG>
class FunctionWrapper
    : public FunctionMixin<FunctionWrapper<F,P,SIG>,P,SIG>
    , public FunctionWrapperGetterMixin<FunctionWrapper<F,P,SIG>,P,SIG>
    , public F
{
    using typename FunctionInterface<P,SIG>::ArgumentSizeType;
    using typename FunctionInterface<P,SIG>::ResultSizeType;
    using typename FunctionInterface<P,SIG>::ArgumentIndexType;
    template<class X> using Argument = typename SignatureTraits<SIG>::template Argument<X>;
    template<class X> using Result = typename SignatureTraits<SIG>::template Result<X>;
  public:
    virtual ~FunctionWrapper() = default;
    FunctionWrapper(F f) : F(f) { }
    operator F const& () const { return *this; }
    F const& wrapped() const { return *this; }
    virtual ArgumentSizeType argument_size() const override { return this->wrapped().argument_size(); }
    virtual ResultSizeType result_size() const override { return this->wrapped().result_size(); }
    virtual FunctionInterface<P,SIG>* _derivative(ArgumentIndexType k) const override {
        return make_function_wrapper<P,SIG>(derivative(this->wrapped(),k)); }
    template<class T> Result<T> operator() (Argument<T> const& x) const { return this->wrapped()(x); }
    friend inline OutputStream& operator<<(OutputStream& os, FunctionWrapper<F,P,SIG> const& f) { return os << f.wrapped(); }
    friend inline OutputStream& operator<<(OutputStream& os, Representation<FunctionWrapper<F,P,SIG>> const& f) { return os << Representation<F>(f.reference().wrapped()); }
        template<class I> decltype(auto) operator[](I i) const { return this->wrapped()[i]; }
};

template<class P, class SIG> template<class F> requires (not IsFunctionClass<F,SIG>) and AFunction<F,P,SIG>
Function<P,SIG>::Function(F const& f)
    : Handle<const Interface>(make_function_wrapper<P,SIG>(f)) { }

template<class P, class SIG, class F> requires AFunction<F,P,SIG>
FunctionWrapper<F,P,SIG>* make_function_wrapper(F f) { return new Wrapper<F,FunctionInterface<P,SIG>>(f); }

template<class F, class P, class SIG> F const& extract(Function<P,SIG> const& f) {
    if constexpr(DerivedFrom<F,FunctionInterface<P,SIG>>) {
        return dynamic_cast<F const&>(f.reference());
    } else {
        return dynamic_cast<FunctionWrapper<F,P,SIG> const&>(f.reference());
    }
}

} // namespace Ariadne

#endif // ARIADNE_FUNCTION_WRAPPER_HPP
