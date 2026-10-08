/***************************************************************************
 *            test/utility.hpp
 *
 *  Copyright 2007-17  Pieter Collins
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

#ifndef ARIADNE_TEST_UTILITY_HPP
#define ARIADNE_TEST_UTILITY_HPP

namespace Ariadne {
//namespace Test {

struct OperatorPlus {
    template<class A> decltype(auto) operator()(A a) const { return +a; }
    template<class A1, class A2> decltype(auto) operator()(A1 a1, A2 a2) const { return a1+a2; }
};
struct OperatorMinus {
    template<class A> decltype(auto) operator()(A a) const { return -a; }
    template<class A1, class A2> decltype(auto) operator()(A1 a1, A2 a2) const { return a1-a2; }
};
struct OperatorTimes {
    template<class A1, class A2> decltype(auto) operator()(A1 a1, A2 a2) const { return a1*a2; }
};
struct OperatorDivides {
    template<class A1, class A2> decltype(auto) operator()(A1 a1, A2 a2) const { return a1/a2; }
};

struct OperatorAnd { template<class A1, class A2> decltype(auto) operator()(A1 a1, A2 a2) const { return a1 && a2; } };
struct OperatorOr { template<class A1, class A2> decltype(auto) operator()(A1 a1, A2 a2) const { return a1 || a2; } };
struct OperatorNot { template<class A> decltype(auto) operator()(A a) const { return !a; } };

struct OperatorEqual { template<class A1, class A2> decltype(auto) operator()(A1 a1, A2 a2) const { return a1==a2; } };
struct OperatorEquals { template<class A1, class A2> decltype(auto) operator()(A1 a1, A2 a2) const { return a1==a2; } };
struct OperatorLess { template<class A1, class A2> decltype(auto) operator()(A1 a1, A2 a2) const { return a1< a2; } };

namespace Detail {

template<class OP, class... AS> struct SafeTypedef {
    template<class O, class = ResultOf<O(AS...)>> static ResultOf<O(AS...)> safe(int);
    template<class O> static Fallback safe(...);
    typedef decltype(safe<OP>(1)) Type;
};

template<class A1, class A2> struct SafeTypedef<OperatorEquals,A1,A2> {
    template<class AA1, class AA2, class = decltype(declval<AA1>()==declval<AA2>())> static decltype(declval<AA1>()==declval<AA2>()) safe(int);
    template<class AA1, class AA2> static Fallback safe(...);
    typedef decltype(safe<A1,A2>(1)) Type;
};
template<class A1, class A2> struct SafeTypedef<OperatorLess,A1,A2> {
    template<class AA1, class AA2, class = decltype(declval<AA1>()<declval<AA2>())> static decltype(declval<AA1>()<declval<AA2>()) safe(int);
    template<class AA1, class AA2> static Fallback safe(...);
    typedef decltype(safe<A1,A2>(1)) Type;
};

template<class R, template<class>class T, class A1, class = decltype(declval<R>()=declval<T<A1>>())> True is(int);
template<class R, template<class>class T, class A1> False is(...);
template<class R, template<class,class>class T, class A1, class A2, class = decltype(declval<R>()=declval<T<A1,A2>>())> True is(int,int);
template<class R, template<class,class>class T, class A1, class A2> False is(...);
template<class R, template<class,class,class>class T, class A1, class A2, class A3, class = decltype(declval<R>()=declval<T<A1,A2,A3>>())> True is(int,int,int);
template<class R, template<class,class,class>class T, class A1, class A2, class A3> False is(...);


} //namespace Detail

template<class R, template<class...>class T, class... AS> struct Is;
template<class R, template<class>class T, class A> struct Is<Return<R>,T,A> : decltype(Detail::is<R,T,A>(1)) { };
template<class R, template<class,class>class T, class A1, class A2> struct Is<Return<R>,T,A1,A2> : decltype(Detail::is<R,T,A1,A2>(1,2)) { };
template<class R, template<class,class,class>class T, class A1, class A2, class A3> struct Is<Return<R>,T,A1,A2,A3> : decltype(Detail::is<R,T,A1,A2,A3>(1,2,3)) { };


template<class OP, class... AS> using ReturnType = decltype(declval<OP>().operator()(declval<AS...>()));
template<class OP, class A1> using UnaryReturnType = decltype(declval<OP>().operator()(declval<A1>()));
template<class OP, class A1, class A2> using BinaryReturnType = decltype(declval<OP>().operator()(declval<A1>(),declval<A2>()));

template<class OP, class... AS> concept HasOperator = Invocable<OP,AS...>;
template<class R, class OP, class... AS> concept HasOperatorReturning = InvocableReturning<R,OP,AS...>;

template<class OP, class... AS> using SafeType = typename Detail::SafeTypedef<OP,AS...>::Type;

template<class T> struct Expect { template<class U> requires Same<T,U> Expect<T>& operator=(U const&) { return *this; } };

//} // namespace Test
//using namespace Test;
} // namespace Ariadne

#endif // ARIADNE_TEST_UTILITY_HPP

