/***************************************************************************
 *            check_numeric.cpp
 *
 *  Copyright  2006-20  Alberto Casagrande, Pieter Collins
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

#include <tuple>

#include "utility/module.hpp"

namespace Ariadne {
using Nat=unsigned int; using Int=int; using Dbl=double; class Rational;
// Add neg and rec for builtin types; note builtin negation of unsigned int gives unsigned!!
decltype(-declval<Nat>()) neg(Nat); Int neg(Int); Dbl neg(Dbl);
Rational rec(Nat); Rational rec(Int); Dbl rec(Dbl);
} // namespace Ariadne

#include "config.hpp"
#include "foundations/logical.hpp"
#include "numeric/number.hpp"
#include "numeric/floats.hpp"
#include "numeric/integer.hpp"
#include "numeric/rational.hpp"
#include "numeric/real.hpp"
#include "numeric/floatdp.hpp"
#include "numeric/floatmp.hpp"

#include "numeric/concepts.hpp"
#include "numeric/archetypes.hpp"

#include "../test.hpp"
#include "utility.hpp"
#include "check_numeric.hpp"

using namespace std;
using namespace Ariadne;

namespace Ariadne {

template class Bounds<RoundedArchetype>;
template class LowerBound<RoundedArchetype>;
template class UpperBound<RoundedArchetype>;
template class Approximation<RoundedArchetype>;

using std::declval;
template<class T> String class_name();

// Add signum operation; returns true if x>=0
template<BuiltinIntegral N> bool signum(N n);
Decidable signum(Integer z);
Decidable signum(Rational q);
Quasidecidable signum(Real r);
Decidable signum(Number<ExactTag> n);
Quasidecidable signum(Number<EffectiveTag> n);
LogicalType<ValidatedTag> signum(Number<ValidatedTag> n);
LogicalType<LowerTag> signum(Number<ValidatedUpperTag> n);
LogicalType<UpperTag> signum(Number<ValidatedLowerTag> n);
LogicalType<ApproximateTag> signum(Number<ApproximateTag> n);
template<class P, class PR> decltype(signum(declval<Number<P>>())) signum(Float<P,PR>);
struct Sig { template<class A> auto operator() (A&& a) const -> decltype(signum(a)) { return signum(std::move(a)); } };

// An empty class storing template parameters
template<class T> struct Tag { };
template<class... TS> struct Tags;
template<class T, class... TS> struct Tags<T,TS...> { typedef T Head; typedef Tags<TS...> Tail; };
template<> struct Tags<> { };
template<class T> using Head = typename T::Head;
template<class T> using Tail = typename T::Tail;
String class_names_impl(Tags<>) { return ""; }
template<class T> String class_names_impl(Tags<T>) { return class_name<T>(); }
template<class T1, class T2, class... TS> String class_names_impl(Tags<T1,T2,TS...>) { return class_name<T1>() + "," + class_names_impl(Tags<T2,TS...>()); }
template<class... TS> String class_names() { return class_names_impl(Tags<TS...>()); }

template<class... A1S, class... A2S> auto
cat(Tags<A1S...>,Tags<A2S...>) -> Tags<A1S...,A2S...>;
template<class... A1S, class... A2S, class... A3S> auto
cat(Tags<A1S...>,Tags<A2S...>,Tags<A3S...>) -> Tags<A1S...,A2S...,A3S...>;
template<class... A1S, class... A2S, class... A3S, class... A4S> auto
cat(Tags<A1S...>,Tags<A2S...>,Tags<A3S...>,Tags<A4S...>) -> Tags<A1S...,A2S...,A3S...,A4S...>;

template<class X> using SafeNegationType = SafeType<OperatorMinus,X>;
template<class X1, class X2=X1> using SafeSumType = SafeType<OperatorPlus,X1,X2>;
template<class X1, class X2=X1> using SafeDifferenceType = SafeType<OperatorMinus,X1,X2>;
template<class X1, class X2=X1> using SafeProductType = SafeType<OperatorTimes,X1,X2>;
template<class X1, class X2=X1> using SafeQuotientType = SafeType<OperatorDivides,X1,X2>;

template<class X1, class X2=X1> using SafeArithmeticType = SafeSumType<SafeProductType<X1,X2>>;
template<class X1, class X2=X1> using SafeEqualsType = SafeType<OperatorEquals,X1,X2>;
template<class X1, class X2=X1> using SafeLessType = SafeType<OperatorLess,X1,X2>;

template<class X1, class X2> struct IsEquivalent : And<IsConvertible<X1,X2>,IsConvertible<X2,X1>> { };
template<class T, class U> struct IsNotConvertible : Not<IsConvertible<T,U>> { };
template<class T, class U> struct IsExplicitlyConvertible : IsConstructible<U,T> { };
template<class T, class U> struct IsNotExplicitlyConvertible : Not<IsConstructible<U,T>> { };
template<class T, class U> struct IsStrictlyConvertible : And<IsConstructible<U,T>,IsNotConvertible<T,U>> { };

template<class OP> String op_name();
template<> String op_name<Add>() { return "add"; }
template<> String op_name<Sub>() { return "sub"; }
template<> String op_name<Mul>() { return "mul"; }
template<> String op_name<Div>() { return "div"; }
template<> String op_name<Neg>() { return "neg"; }
template<> String op_name<Rec>() { return "rec"; }
template<> String op_name<Sig>() { return "sig"; }

template<> String op_name<Plus>() { return "operator+"; }
template<> String op_name<Minus>() { return "operator-"; }
template<> String op_name<Times>() { return "operator*"; }
template<> String op_name<Divides>() { return "operator/"; }
template<> String op_name<Equal>() { return "operator=="; }
template<> String op_name<Less>() { return "operator<"; }



void output_check_result(Bool p, String e, String r, String op, String as) {
    std::cout << "Checking " << op << "(" << as << ") -> " << e << ":";
    if(p) {
        std::cout << " OK\n";
    } else {
        ++ARIADNE_TEST_FAILURES;
        std::cout << " Failed! Actual type is " << r << "\n";
        std::cerr << "ERROR : " << op << "(" << as << ") -> " << r << "; expected " << e << "\n";
    }
}

template<class E, class OP, class... AS> struct Check { Check(); };

// Check if the result of OP(AS...) has expected type E
#ifndef ARIADNE_COMPILE_TIME_CHECK
template<class E, class OP, class... AS> Check<E,OP,AS...>::Check() {
    typedef SafeType<OP,AS...>  R ;
    Bool p=IsSame< R ,E>();
    output_check_result(p,class_name<E>(),class_name< R >(),op_name<OP>(),class_names<AS...>());
}
#else
template<class E, class  R , class OP, class... AS> void sm() {
    static_assert(Same<E,  R >);
}
template<class E, class OP, class... AS> Check<E,OP,AS...>::Check() {
    typedef SafeType<OP,AS...>  R ;
    sm<E, R ,OP,AS...>();
}
#endif

template<class E, class OP, class... AS> void check() { Check<E,OP,AS...>(); }


template<class TE, class OP, class... TAS> struct TableCheckAll;

template<class TE, class OP, class TA> struct TableCheckAll<TE,OP,TA> {
    TableCheckAll() { Check<Head<TE>,OP,Head<TA>>(); TableCheckAll<Tail<TE>,OP,Tail<TA>>(); }
};

template<class OP> struct TableCheckAll<Tags<>,OP,Tags<>> {
    TableCheckAll() { }
};

template<class TE, class OP, class A1> struct TableCheckAll<TE,OP,Tag<A1>,Tags<>> {
};

template<class TE, class OP, class A1, class A2, class... A2S> struct TableCheckAll<TE,OP,Tag<A1>,Tags<A2,A2S...>> {
    TableCheckAll() { Check<Head<TE>,OP,A1,A2>(); TableCheckAll<Tail<TE>,OP,Tag<A1>,Tags<A2S...>>(); }
};

template<class TE, class OP, class TA2S> struct TableCheckAll<TE,OP,Tags<>,TA2S> {
};

template<class TE, class OP, class TA2S, class A1, class... A1S> struct TableCheckAll<TE,OP,Tags<A1,A1S...>,TA2S> {
    TableCheckAll() { TableCheckAll<Head<TE>,OP,Tag<A1>,TA2S>(); TableCheckAll<Tail<TE>,OP,Tags<A1S...>,TA2S>(); }
};

template<class TE, class OP, class... TAS> Void table_check_all() { TableCheckAll<TE,OP,TAS...>(); }



template<class E, class OP, class... TAS> struct CheckAll;


template<class E, class OP, class A1> struct CheckAll<E,OP,Tag<A1>,Tags<>> {
};

template<class E, class OP, class A1, class A2, class... A2S> struct CheckAll<E,OP,Tag<A1>,Tags<A2,A2S...>> {
    CheckAll() { typedef typename E::template Type<A1,A2> ER; Check<ER,OP,A1,A2>(); CheckAll<E,OP,Tag<A1>,Tags<A2S...>>(); }
};

template<class E, class OP, class TA2S> struct CheckAll<E,OP,Tags<>,TA2S> {
};

template<class E, class OP, class TA2S, class A1, class... A1S> struct CheckAll<E,OP,Tags<A1,A1S...>,TA2S> {
    CheckAll() { CheckAll<E,OP,Tag<A1>,TA2S>(); CheckAll<E,OP,Tags<A1S...>,TA2S>(); }
};

template<class E, class OP, class... TAS> Void check_all() { CheckAll<E,OP,TAS...>(); }




void output_check_explicitly_constructible_result(Bool p, String t, String f, Bool c) {
    std::cout << "Checking " << t << " is constructible from " << f << ":";
    if(p) {
        std::cout << " OK\n";
    } else {
        std::cout << " Failed!\n";
        if(c) {
            std::cerr << "ERROR : " << t << " is convertible from  " << f << ", but only explict construction should be allowed.\n";
        } else {
            std::cerr << "ERROR : " << t << " is not constructible from " << f << ".\n";
        }
    }
}

template<class T, class F> void check_explicitly_constructible() {
    Bool c=Convertible<F,T>;
    Bool p=Constructible<T,F> and not c;
    output_check_explicitly_constructible_result(p,class_name<T>(),class_name<F>(),c);
}


void output_check_convertible_result(Bool p, String t, String f) {
    std::cout << "Checking " << t << " is convertible from " << f << ":";
    if(p) {
        std::cout << " OK\n";
    } else {
        std::cout << " Failed!\n";
        std::cerr << "ERROR : " << t << " is not convertible from " << f << ".\n";
    }
}

template<class T, class F> void check_convertible() {
    Bool p=Convertible<F,T>;
    output_check_convertible_result(p,class_name<T>(),class_name<F>());
}

void output_check_not_constructible_result(Bool p, String t, String f) {
    std::cout << "Checking " << t << " is not constructible from " << f << ":";
    if(p) {
        std::cout << " OK\n";
    } else {
        std::cout << " Failed!\n";
        std::cerr << "ERROR : " << t << " is constructible from " << f << "\n";
    }
}

template<class T, class F> void check_not_constructible() {
    Bool p=Constructible<T,F>;
    output_check_not_constructible_result(p,class_name<T>(),class_name<F>());
}

struct Cid { } ; struct Eid { }; struct Nid { };
template<class E, class F, class A1, class A2> void chk() { Check<E,F,A1,A2>(); }
template<class  R , class CNV, class X> void chk() { chk< R ,X>(CNV()); }
template<class  R , class X> void chk(Cid) { check_convertible< R ,X>(); }
template<class  R , class X> void chk(Eid) { check_explicitly_constructible< R ,X>(); }
template<class  R , class X> void chk(Nid) { check_not_constructible< R ,X>(); }

template<class T> String class_name() { return "Unknown"; }

#define ARIADNE_CLASS_NAME(Class) \
    template<> String class_name<Class>() { return #Class; }

ARIADNE_CLASS_NAME(Fallback);

ARIADNE_CLASS_NAME(ApproximateTag);
ARIADNE_CLASS_NAME(LowerTag);
ARIADNE_CLASS_NAME(UpperTag);
ARIADNE_CLASS_NAME(BoundedTag);
ARIADNE_CLASS_NAME(ValidatedTag);
ARIADNE_CLASS_NAME(ExactTag);

ARIADNE_CLASS_NAME(LogicalType<ApproximateTag>);
ARIADNE_CLASS_NAME(LogicalType<ValidatedLowerTag>);
ARIADNE_CLASS_NAME(LogicalType<ValidatedUpperTag>);
ARIADNE_CLASS_NAME(LogicalType<ValidatedTag>);
ARIADNE_CLASS_NAME(LogicalType<EffectiveLowerTag>);
ARIADNE_CLASS_NAME(LogicalType<EffectiveUpperTag>);
ARIADNE_CLASS_NAME(LogicalType<EffectiveTag>);
ARIADNE_CLASS_NAME(LogicalType<ExactTag>);

ARIADNE_CLASS_NAME(bool);
ARIADNE_CLASS_NAME(uint);

ARIADNE_CLASS_NAME(Integer);
ARIADNE_CLASS_NAME(Rational);
ARIADNE_CLASS_NAME(Real);

ARIADNE_CLASS_NAME(Number<ApproximateTag>)
ARIADNE_CLASS_NAME(Number<ValidatedLowerTag>)
ARIADNE_CLASS_NAME(Number<ValidatedUpperTag>)
ARIADNE_CLASS_NAME(Number<ValidatedTag>)
ARIADNE_CLASS_NAME(Number<EffectiveLowerTag>)
ARIADNE_CLASS_NAME(Number<EffectiveUpperTag>)
ARIADNE_CLASS_NAME(Number<EffectiveTag>)
ARIADNE_CLASS_NAME(Number<ExactTag>)

ARIADNE_CLASS_NAME(FloatDPApproximation);
ARIADNE_CLASS_NAME(FloatDPLowerBound);
ARIADNE_CLASS_NAME(FloatDPUpperBound);
ARIADNE_CLASS_NAME(FloatDPBounds);
ARIADNE_CLASS_NAME(FloatDPBall);
ARIADNE_CLASS_NAME(FloatDPError);
ARIADNE_CLASS_NAME(FloatDP);

ARIADNE_CLASS_NAME(FloatMPApproximation);
ARIADNE_CLASS_NAME(FloatMPLowerBound);
ARIADNE_CLASS_NAME(FloatMPUpperBound);
ARIADNE_CLASS_NAME(FloatMPBounds);
ARIADNE_CLASS_NAME(FloatMPBall);
ARIADNE_CLASS_NAME(FloatMPError);
ARIADNE_CLASS_NAME(FloatMP);

#undef ARIADNE_CLASS_NAME

typedef bool B; typedef uint Nat; typedef int Int; typedef double Dbl;
typedef Integer  Z ; typedef Rational  Q ; typedef Real  R ;
typedef Number<ExactTag> ExN; typedef Number<EffectiveTag> EfN; typedef Number<ValidatedTag> VaN;
typedef Number<ValidatedUpperTag> UpN; typedef Number<ValidatedLowerTag> LoN; typedef Number<ApproximateTag> ApN;
typedef FloatDP ExF; typedef FloatDPBall MeF; typedef FloatDPBounds BoF;
typedef FloatDPUpperBound UpF; typedef FloatDPLowerBound LoF; typedef FloatDPApproximation ApF;
typedef LogicalType<ExactTag> ExL; typedef LogicalType<EffectiveTag> EfL; typedef LogicalType<ValidatedTag> VaL;
typedef LogicalType<UpperTag> UpL; typedef LogicalType<LowerTag> LoL; typedef LogicalType<ApproximateTag> ApL;

//typedef decltype(declval<ExF>() + declval<ExF>()) EfF;
typedef decltype(declval<ExN>() + declval<ExF>()) EfF;
typedef decltype(declval<MeF>() + declval<BoF>()) VaF;
typedef decltype(declval<UpF>() * declval<UpF>()) PrF;
typedef decltype(declval<double>() + declval<Number<ApproximateTag>>()) ApD;
//typedef FloatDPBoundsType VaF;

} // namespace Ariadne

class CheckNumeric
{
  public:
    void check();
  private:
    typedef SafeSumType<FloatDP,FloatDP> ExactFloatArithmeticType;

    void notifications();
    void check_conversions();
    void check_negation();
    void check_reciprocal();
    void check_signum();
    void check_addition();
    void check_subtraction();
    void check_multiplication();
    void check_division();
    void check_comparison();
    void check_equality();

    typedef Tags<Nat,Int,Dbl> BuiltinTypes;
    typedef Tags<Integer,Rational,Real> UserTypes;
    typedef Tags<Number<ExactTag>,Number<EffectiveTag>,Number<ValidatedTag>,Number<ValidatedUpperTag>,Number<ValidatedLowerTag>,Number<ApproximateTag>> GenericTypes;
    typedef Tags<FloatDP,FloatDPBall,FloatDPBounds,FloatDPUpperBound,FloatDPLowerBound,FloatDPApproximation> FloatDPTypes;

    typedef decltype(cat(declval<BuiltinTypes>(),declval<UserTypes>(),declval<GenericTypes>(),declval<FloatDPTypes>())) NumericTypes;

    using ExpectedNegatives =
    Tags<Nat,Int,Dbl,  Z , Q , R , ExN,EfN,VaN,LoN,UpN,ApN, ExF,MeF,BoF,LoF,UpF,ApF>;

    using ExpectedReciprocals =
    Tags< Q , Q ,Dbl,  Q , Q , R , ExN,EfN,VaN,LoN,UpN,ApN, EfF,MeF,BoF,LoF,UpF,ApF>;

    using ExpectedWeakerTable =
    //        Nat,Int,Dbl,  Z , Q , R , ExN,EfN,VaN,UpN,LoN,ApN, ExF,MeF,BoF,UpF,LoF,ApF
    Tags<Tags<Nat,Nat,Dbl,  Z , Q , R , ExN,EfN,VaN,UpN,LoN,ApN, EfF,MeF,BoF,UpF,LoF,ApF>, //  J
         Tags<Nat,Int,Dbl,  Z , Q , R , ExN,EfN,VaN,UpN,LoN,ApN, EfF,MeF,BoF,UpF,LoF,ApF>, //  Int
         Tags<Dbl,Dbl,Dbl, ApD,ApD,ApD, ApD,ApD,ApD,ApD,ApD,ApD, ApF,ApF,ApF,ApF,ApF,ApF>, //  Dbl
         Tags< Z , Z ,ApD,  Z , Q , R , ExN,EfN,VaN,UpN,LoN,ApN, EfF,MeF,BoF,UpF,LoF,ApF>, //   Z
         Tags< Q , Q ,ApD,  Q , Q , R , ExN,EfN,VaN,UpN,LoN,ApN, EfF,MeF,BoF,UpF,LoF,ApF>, //   Q
         Tags< R , R ,ApD,  R , R , R , EfN,EfN,VaN,UpN,LoN,ApN, EfF,MeF,BoF,UpF,LoF,ApF>, //   R
         Tags<ExN,ExN,ApN, ExN,ExN,EfN, ExN,EfN,VaN,UpN,LoN,ApN, EfF,MeF,BoF,UpF,LoF,ApF>, // ExN
         Tags<EfN,EfN,ApN, EfN,EfN,EfN, EfN,EfN,VaN,UpN,LoN,ApN, EfF,MeF,BoF,UpF,LoF,ApF>, // EfN
         Tags<VaN,VaN,ApN, VaN,VaN,VaN, VaN,VaN,VaN,UpN,LoN,ApN, EfF,MeF,BoF,UpF,LoF,ApF>, // VaN
         Tags<UpN,UpN,ApN, UpN,UpN,UpN, UpN,UpN,UpN,UpN,ApN,ApN, UpF,UpF,UpF,UpF,ApF,ApF>, // UpN
         Tags<LoN,LoN,ApN, LoN,LoN,LoN, LoN,LoN,LoN,ApN,LoN,ApN, LoF,LoF,LoF,ApF,LoF,ApF>, // LoN
         Tags<ApN,ApN,ApN, ApN,ApN,ApN, ApN,ApN,ApN,ApN,ApN,ApN, ApF,ApF,ApF,ApF,ApF,ApF>, // ApN
         Tags<EfF,EfF,ApF, EfF,EfF,EfF, EfF,EfF,EfF,UpF,LoF,ApF, EfF,MeF,BoF,UpF,LoF,ApF>, // ExF
         Tags<MeF,MeF,ApF, MeF,MeF,MeF, MeF,MeF,MeF,UpF,LoF,ApF, MeF,MeF,VaF,UpF,LoF,ApF>, // MeF
         Tags<BoF,BoF,ApF, BoF,BoF,BoF, BoF,BoF,BoF,UpF,LoF,ApF, BoF,VaF,BoF,UpF,LoF,ApF>, // BoF
         Tags<UpF,UpF,ApF, UpF,UpF,UpF, UpF,UpF,UpF,UpF,ApF,ApF, UpF,UpF,UpF,UpF,ApF,ApF>, // UpF
         Tags<LoF,LoF,ApF, LoF,LoF,LoF, LoF,LoF,LoF,ApF,LoF,ApF, LoF,LoF,LoF,ApF,LoF,ApF>, // LoF
         Tags<ApF,ApF,ApF, ApF,ApF,ApF, ApF,ApF,ApF,ApF,ApF,ApF, ApF,ApF,ApF,ApF,ApF,ApF>>;// ApF

    using ExpectedSignums =
    //   Nat,Int,Dbl,  Z , Q , R , ExN,EfN,VaN,UpN,LoN,ApN, ExF,MeF,BoF,UpF,LoF,ApF
    Tags< B , B , B , ExL,ExL,EfL, ExL,EfL,VaL,LoL,UpL,ApL, ExL,VaL,VaL,LoL,UpL,ApL>;

    using ExpectedNonzeros =
    //   Nat,Int,Dbl,  Z , Q , R , ExN,EfN,VaN,UpN,LoN,ApN, ExF,MeF,BoF,UpF,LoF,ApF
    Tags< B , B , B , ExL,ExL,LoL, ExL,LoL,LoL,ApL,ApL,ApL, ExL,LoL,LoL,ApL,ApL,ApL>;


    using ExpectedSums = ExpectedWeakerTable;

    using ExpectedProducts = ExpectedWeakerTable;

    struct ExpectedDifference { template<class A1, class A2> using Type = SafeType<Plus,A1,SafeType<Neg,A2>>; };
    struct ExpectedQuotient { template<class A1, class A2> using Type = SafeType<Times,A1,SafeType<Rec,A2>>; };
    struct ExpectedLess { template<class A1, class A2> using Type = SafeType<Sgn,SafeType<Minus,A1,A2>>; };
    struct ExpectedEqual { template<class A1, class A2> using Type = SafeType<Equal,A1,A2>; };

};

void CheckNumeric::check()
{
    ARIADNE_TEST_CALL(notifications());
    ARIADNE_TEST_CALL(check_conversions());
    ARIADNE_TEST_CALL(check_negation());
    ARIADNE_TEST_CALL(check_reciprocal());
    ARIADNE_TEST_CALL(check_signum());
    ARIADNE_TEST_CALL(check_addition());
    ARIADNE_TEST_CALL(check_subtraction());
    ARIADNE_TEST_CALL(check_multiplication());
    ARIADNE_TEST_CALL(check_division());
//    ARIADNE_TEST_CALL(check_comparison());
//    ARIADNE_TEST_CALL(check_equality());
}

String to_str(bool b) { return b?"true":"false"; }

void CheckNumeric::notifications()
{
    // Operations on FloatDP: display what is being used.
    ARIADNE_TEST_NOTIFY(String("Conversion double -> ApproximateNumericType: ")+to_str(Convertible<double,ApproximateNumericType>));
    ARIADNE_TEST_NOTIFY(String("Conversion double -> FloatDPApproximation: ")+to_str(Convertible<double,FloatDPApproximation>));
    ARIADNE_TEST_NOTIFY(String("Conversion double -> FloatDP: ")+to_str(Convertible<double,FloatDP>));
    ARIADNE_TEST_NOTIFY(String("Construction double -> ExactNumericType: ")+to_str(Constructible<ExactNumericType,double>));
    ARIADNE_TEST_NOTIFY(String("Construction double -> FloatDP: ")+to_str(Constructible<FloatDP,double>));
    ARIADNE_TEST_NOTIFY(String("Conversion int -> FloatDP: ")+to_str(Convertible<int,FloatDP>));
    ARIADNE_TEST_NOTIFY(String("Construction Integer -> FloatDP: ")+to_str(Constructible<FloatDP,Integer>)+"");

    ARIADNE_TEST_NOTIFY((String("UpperNumericType * UpperNumericType -> ")+class_name<SafeProductType<UpperNumericType,UpperNumericType>>()+"\n"));

    ARIADNE_TEST_NOTIFY((String("FloatDP + FloatDP -> ")+class_name<SafeSumType<FloatDP,FloatDP>>()));
    ARIADNE_TEST_NOTIFY((String("FloatDPBall + FloatDPBounds -> ")+class_name<SafeSumType<FloatDPBall,FloatDPBounds>>()));
    ARIADNE_TEST_NOTIFY((String("FloatDP + ValidatedNumericType -> ")+class_name<SafeSumType<FloatDP,ValidatedNumericType>>()));
    ARIADNE_TEST_NOTIFY((String("FloatDPUpperBound * FloatDPUpperBound -> ")+class_name<SafeProductType<FloatDPUpperBound,FloatDPUpperBound>>()+"\n"));

    ARIADNE_TEST_NOTIFY((String("Integer + double -> ")+class_name<SafeSumType<Integer,double>>()));
    ARIADNE_TEST_NOTIFY((String("ExactNumericType + double -> ")+class_name<SafeSumType<ExactNumericType,double>>()));
    ARIADNE_TEST_NOTIFY((String("ApproximateNumericType + double -> ")+class_name<SafeSumType<ApproximateNumericType,double>>()));
    ARIADNE_TEST_NOTIFY((String("FloatDP + double -> ")+class_name<SafeSumType<FloatDP,double>>()+"\n"));

    ARIADNE_TEST_NOTIFY((String("Rational == double -> ")+class_name<SafeEqualsType<Rational,double>>()));
    ARIADNE_TEST_NOTIFY((String("Rational < double -> ")+class_name<SafeLessType<Rational,double>>()+"\n"));

    ARIADNE_TEST_STATIC_ASSERT(IsStronger<Paradigm<ExactFloatArithmeticType>,ValidatedTag>)
}

void CheckNumeric::check_conversions() {
    // Cid is conversion, Eid is explicit construction, Nid is no construction

    // Conversions involving int
    chk<FloatDPError,Cid,Nat>(); chk<FloatDPError,Nid,Int>(); chk<FloatDPError,Eid,Dbl>();

    // Concrete numbers
    chk<Nat,Cid,Nat>(); chk<Nat,Cid,Int>(); chk<Nat,Cid,Dbl>(); chk<Nat,Nid, Z >(); chk<Int,Nid, Q >(); chk<Int,Nid, R >();
    chk<Int,Cid,Nat>(); chk<Int,Cid,Int>(); chk<Int,Cid,Dbl>(); chk<Int,Nid, Z >(); chk<Int,Nid, Q >(); chk<Int,Nid, R >();
    chk<Dbl,Cid,Nat>(); chk<Dbl,Cid,Int>(); chk<Dbl,Cid,Dbl>(); chk<Dbl,Nid, Z >(); chk<Dbl,Nid, Q >(); chk<Dbl,Nid, R >();
    chk< Z ,Cid,Nat>(); chk< Z ,Cid,Int>(); chk< Z ,Nid,Dbl>(); chk< Z ,Cid, Z >(); chk< Z ,Nid, Q >(); chk< Z ,Nid, R >();
    chk< Q ,Cid,Nat>(); chk< Q ,Cid,Int>(); chk< Q ,Eid,Dbl>(); chk< Q ,Cid, Z >(); chk< Q ,Cid, Q >(); chk< Q ,Nid, R >();
    chk< R ,Cid,Nat>(); chk< R ,Cid,Int>(); chk< R ,Eid,Dbl>(); chk< R ,Cid, Z >(); chk< R ,Cid, Q >(); chk< R ,Cid, R >();

    // Generic numbers
    chk<ExN,Cid,ExN>(); chk<ExN,Nid,EfN>(); chk<ExN,Nid,VaN>(); chk<ExN,Nid,UpN>(); chk<ExN,Nid,LoN>(); chk<ExN,Nid,ApN>();
    chk<EfN,Cid,ExN>(); chk<EfN,Cid,EfN>(); chk<EfN,Nid,VaN>(); chk<EfN,Nid,UpN>(); chk<EfN,Nid,LoN>(); chk<EfN,Nid,ApN>();
    chk<VaN,Cid,ExN>(); chk<VaN,Cid,EfN>(); chk<VaN,Cid,VaN>(); chk<VaN,Nid,UpN>(); chk<VaN,Nid,LoN>(); chk<VaN,Nid,ApN>();
    chk<UpN,Cid,ExN>(); chk<UpN,Cid,EfN>(); chk<UpN,Cid,VaN>(); chk<UpN,Cid,UpN>(); chk<UpN,Nid,LoN>(); chk<UpN,Nid,ApN>();
    chk<LoN,Cid,ExN>(); chk<LoN,Cid,EfN>(); chk<LoN,Cid,VaN>(); chk<LoN,Nid,UpN>(); chk<LoN,Cid,LoN>(); chk<LoN,Nid,ApN>();
    chk<ApN,Cid,ExN>(); chk<ApN,Cid,EfN>(); chk<ApN,Cid,VaN>(); chk<ApN,Cid,UpN>(); chk<ApN,Cid,LoN>(); chk<ApN,Cid,ApN>();

    // Mixed Generic # Concrete numbers
    chk<ExN,Cid,Nat>(); chk<ExN,Cid,Int>(); chk<ExN,Nid,Dbl>(); chk<ExN,Cid, Z >(); chk<ExN,Cid, Q >(); chk<ExN,Nid, R >();
    chk<EfN,Cid,Nat>(); chk<EfN,Cid,Int>(); chk<EfN,Nid,Dbl>(); chk<EfN,Cid, Z >(); chk<EfN,Cid, Q >(); chk<EfN,Cid, R >();
    chk<VaN,Cid,Nat>(); chk<VaN,Cid,Int>(); chk<VaN,Nid,Dbl>(); chk<VaN,Cid, Z >(); chk<VaN,Cid, Q >(); chk<VaN,Cid, R >();
    chk<UpN,Cid,Nat>(); chk<UpN,Cid,Int>(); chk<UpN,Nid,Dbl>(); chk<UpN,Cid, Z >(); chk<UpN,Cid, Q >(); chk<UpN,Cid, R >();
    chk<LoN,Cid,Nat>(); chk<LoN,Cid,Int>(); chk<LoN,Nid,Dbl>(); chk<LoN,Cid, Z >(); chk<LoN,Cid, Q >(); chk<LoN,Cid, R >();
    chk<ApN,Cid,Nat>(); chk<ApN,Cid,Int>(); chk<ApN,Cid,Dbl>(); chk<ApN,Cid, Z >(); chk<ApN,Cid, Q >(); chk<ApN,Cid, R >();

    // Mixed Concrete # Generic numbers
    chk<Nat,Nid,ExN>(); chk<Nat,Nid,EfN>(); chk<Nat,Nid,VaN>(); chk<Nat,Nid,UpN>(); chk<Nat,Nid,LoN>(); chk<Nat,Nid,ApN>();
    chk<Int,Nid,ExN>(); chk<Int,Nid,EfN>(); chk<Int,Nid,VaN>(); chk<Int,Nid,UpN>(); chk<Int,Nid,LoN>(); chk<Int,Nid,ApN>();
    chk<Dbl,Nid,ExN>(); chk<Dbl,Nid,EfN>(); chk<Dbl,Nid,VaN>(); chk<Dbl,Nid,UpN>(); chk<Dbl,Nid,LoN>(); chk<Dbl,Nid,ApN>();
    chk< Z ,Nid,ExN>(); chk< Z ,Nid,EfN>(); chk< Z ,Nid,VaN>(); chk< Z ,Nid,UpN>(); chk< Z ,Nid,LoN>(); chk< Z ,Nid,ApN>();
    chk< Q ,Nid,ExN>(); chk< Q ,Nid,EfN>(); chk< Q ,Nid,VaN>(); chk< Q ,Nid,UpN>(); chk< Q ,Nid,LoN>(); chk< Q ,Nid,ApN>();
    chk< R ,Nid,ExN>(); chk< R ,Nid,EfN>(); chk< R ,Nid,VaN>(); chk< R ,Nid,UpN>(); chk< R ,Nid,LoN>(); chk< R ,Nid,ApN>();

    // FloatDP numbers
    chk<ExF,Cid,ExF>(); chk<ExF,Nid,MeF>(); chk<ExF,Nid,BoF>(); chk<ExF,Nid,UpF>(); chk<ExF,Nid,LoF>(); chk<ExF,Nid,ApF>();
    chk<MeF,Cid,ExF>(); chk<MeF,Cid,MeF>(); chk<MeF,Cid,BoF>(); chk<MeF,Nid,UpF>(); chk<MeF,Nid,LoF>(); chk<MeF,Nid,ApF>();
    chk<BoF,Cid,ExF>(); chk<BoF,Cid,MeF>(); chk<BoF,Cid,BoF>(); chk<BoF,Nid,UpF>(); chk<BoF,Nid,LoF>(); chk<BoF,Nid,ApF>();
    chk<UpF,Cid,ExF>(); chk<UpF,Cid,MeF>(); chk<UpF,Cid,BoF>(); chk<UpF,Cid,UpF>(); chk<UpF,Nid,LoF>(); chk<UpF,Nid,ApF>();
    chk<LoF,Cid,ExF>(); chk<LoF,Cid,MeF>(); chk<LoF,Cid,BoF>(); chk<LoF,Nid,UpF>(); chk<LoF,Cid,LoF>(); chk<LoF,Nid,ApF>();
    chk<ApF,Cid,ExF>(); chk<ApF,Cid,MeF>(); chk<ApF,Cid,BoF>(); chk<ApF,Cid,UpF>(); chk<ApF,Cid,LoF>(); chk<ApF,Cid,ApF>();

    // Mixed FloatDP # Generic and Generic # FloatDP
    chk<ExF,Nid,ExN>(); chk<ExF,Nid,EfN>(); chk<ExF,Nid,VaN>(); chk<ExF,Nid,UpN>(); chk<ExF,Nid,LoN>(); chk<ExF,Nid,ApN>();
    chk<MeF,Eid,ExN>(); chk<MeF,Eid,EfN>(); chk<MeF,Eid,VaN>(); chk<MeF,Nid,UpN>(); chk<MeF,Nid,LoN>(); chk<MeF,Nid,ApN>();
    chk<BoF,Eid,ExN>(); chk<BoF,Eid,EfN>(); chk<BoF,Eid,VaN>(); chk<BoF,Nid,UpN>(); chk<BoF,Nid,LoN>(); chk<BoF,Nid,ApN>();
    chk<UpF,Eid,ExN>(); chk<UpF,Eid,EfN>(); chk<UpF,Eid,VaN>(); chk<UpF,Eid,UpN>(); chk<UpF,Nid,LoN>(); chk<UpF,Nid,ApN>();
    chk<LoF,Eid,ExN>(); chk<LoF,Eid,EfN>(); chk<LoF,Eid,VaN>(); chk<LoF,Nid,UpN>(); chk<LoF,Eid,LoN>(); chk<LoF,Nid,ApN>();
    chk<ApF,Eid,ExN>(); chk<ApF,Eid,EfN>(); chk<ApF,Eid,VaN>(); chk<ApF,Eid,UpN>(); chk<ApF,Eid,LoN>(); chk<ApF,Eid,ApN>();

    chk<ExN,Cid,ExF>(); chk<ExN,Nid,MeF>(); chk<ExN,Nid,BoF>(); chk<ExN,Nid,UpF>(); chk<ExN,Nid,LoF>(); chk<ExN,Nid,ApF>();
    chk<EfN,Cid,ExF>(); chk<EfN,Nid,MeF>(); chk<EfN,Nid,BoF>(); chk<EfN,Nid,UpF>(); chk<EfN,Nid,LoF>(); chk<EfN,Nid,ApF>();
    chk<VaN,Cid,ExF>(); chk<VaN,Cid,MeF>(); chk<VaN,Cid,BoF>(); chk<VaN,Nid,UpF>(); chk<VaN,Nid,LoF>(); chk<VaN,Nid,ApF>();
    chk<UpN,Cid,ExF>(); chk<UpN,Cid,MeF>(); chk<UpN,Cid,BoF>(); chk<UpN,Cid,UpF>(); chk<UpN,Nid,LoF>(); chk<UpN,Nid,ApF>();
    chk<LoN,Cid,ExF>(); chk<LoN,Cid,MeF>(); chk<LoN,Cid,BoF>(); chk<LoN,Nid,UpF>(); chk<LoN,Cid,LoF>(); chk<LoN,Nid,ApF>();
    chk<ApN,Cid,ExF>(); chk<ApN,Cid,MeF>(); chk<ApN,Cid,BoF>(); chk<ApN,Cid,UpF>(); chk<ApN,Cid,LoF>(); chk<ApN,Cid,ApF>();

    // Mixed FloatDP # Concrete and Concrete # FloatDP
    chk<ExF,Eid,Dbl>(); chk<ExF,Cid,Int>(); chk<ExF,Eid, Z >(); chk<ExF,Nid, Q >(); chk<ExF,Nid, R >();
    chk<MeF,Eid,Dbl>(); chk<MeF,Cid,Int>(); chk<MeF,Eid, Z >(); chk<MeF,Eid, Q >(); chk<MeF,Eid, R >();
    chk<BoF,Eid,Dbl>(); chk<BoF,Cid,Int>(); chk<BoF,Eid, Z >(); chk<BoF,Eid, Q >(); chk<BoF,Eid, R >();
    chk<UpF,Eid,Dbl>(); chk<UpF,Cid,Int>(); chk<UpF,Eid, Z >(); chk<UpF,Eid, Q >(); chk<UpF,Eid, R >();
    chk<LoF,Eid,Dbl>(); chk<LoF,Cid,Int>(); chk<LoF,Eid, Z >(); chk<LoF,Eid, Q >(); chk<LoF,Eid, R >();
    chk<ApF,Cid,Dbl>(); chk<ApF,Cid,Int>(); chk<ApF,Eid, Z >(); chk<ApF,Eid, Q >(); chk<ApF,Eid, R >();

    chk<Dbl,Nid,ExF>(); chk<Dbl,Nid,MeF>(); chk<Dbl,Nid,BoF>(); chk<Dbl,Nid,UpF>(); chk<Dbl,Nid,LoF>(); chk<Dbl,Nid,ApF>();
    chk<Int,Nid,ExF>(); chk<Int,Nid,MeF>(); chk<Int,Nid,BoF>(); chk<Int,Nid,UpF>(); chk<Int,Nid,LoF>(); chk<Int,Nid,ApF>();
    chk< Z ,Nid,ExF>(); chk< Z ,Nid,MeF>(); chk< Z ,Nid,BoF>(); chk< Z ,Nid,UpF>(); chk< Z ,Nid,LoF>(); chk< Z ,Nid,ApF>();
    chk< Q ,Eid,ExF>(); chk< Q ,Nid,MeF>(); chk< Q ,Nid,BoF>(); chk< Q ,Nid,UpF>(); chk< Q ,Nid,LoF>(); chk< Q ,Nid,ApF>();
    chk< R ,Eid,ExF>(); chk< R ,Nid,MeF>(); chk< R ,Nid,BoF>(); chk< R ,Nid,UpF>(); chk< R ,Nid,LoF>(); chk< R ,Nid,ApF>();
}

void CheckNumeric::check_negation() {
    //   Nat,Int,Dbl,  Z  ,  Q  ,  R  , ExN,EfN,VaN,UpN,LoN,ApN, ExF,MeF,BoF,UpF,LoF,ApF
    table_check_all<ExpectedNegatives,Minus,NumericTypes>();
    table_check_all<ExpectedNegatives,Neg,NumericTypes>();
}

void CheckNumeric::check_reciprocal() {
    table_check_all<ExpectedReciprocals,Rec,NumericTypes>();
}

void CheckNumeric::check_signum() {
//    table_check_all<ExpectedSignums,Sig,NumericTypes>();
}

void CheckNumeric::check_addition() {
    table_check_all<ExpectedSums,Plus,NumericTypes,NumericTypes>();
}

void CheckNumeric::check_subtraction() {
    check_all<ExpectedDifference, Minus, NumericTypes, NumericTypes>();
}

void CheckNumeric::check_multiplication() {
    using ExpectedProducts = ExpectedSums;
    table_check_all<ExpectedProducts,Times,NumericTypes,NumericTypes>();
}

void CheckNumeric::check_division() {
    check_all<ExpectedQuotient, Divides, NumericTypes, NumericTypes>();
}


void CheckNumeric::check_comparison() {
//    check_all<ExpectedLess,Less,NumericTypes,NumericTypes>();
}

void CheckNumeric::check_equality() {
//    check_all<ExpectedEqual, Equal, NumericTypes, NumericTypes>();
}


int main() {
    // CheckNumeric().check();
    std::cerr<<"SKIPPED ";
    return ARIADNE_TEST_FAILURES;
}
