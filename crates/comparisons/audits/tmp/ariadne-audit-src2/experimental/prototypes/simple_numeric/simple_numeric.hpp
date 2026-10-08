#include <tuple>
#include <iostream>
#include <utility>

namespace Ariadne {

using SizeType = std::size_t;
using OutputStream = std::ostream;
template<class... TS> using Tuple = std::tuple<TS...>;
template<SizeType... IS> using IndexSequence = std::index_sequence<IS...>;

struct Sub { }; struct Div { };

struct Integer; struct Dyadic; struct Rational;

struct Integer {
    Integer();
    friend Integer sub(Integer,Integer);
};
struct Dyadic {
    Dyadic(); Dyadic(Integer);
    friend Dyadic sub(Dyadic,Dyadic);
};
struct Rational {
    Rational(); Rational(Integer); Rational(Dyadic);
    friend Rational sub(Rational,Rational);
};

struct Effort;
struct Real; struct UpperReal; struct LowerReal; struct NaiveReal;
struct ValidatedReal; struct ValidatedUpperReal; struct ValidatedLowerReal; struct ApproximateReal;
struct PositiveReal; struct PositiveUpperReal; struct PositiveLowerReal; struct PositiveNaiveReal;
struct PositiveValidatedReal; struct PositiveValidatedUpperReal; struct PositiveValidatedLowerReal; struct PositiveApproximateReal;

template<class T> class Positive : public T {
    explicit Positive(T const&);
    template<class U, EnableIf<IsConvertible<U,T>> =dummy>
        Positive(Positive<U> const& pu) : Positive(static_cast<T>(static_cast<U const&>(pu)) { }
};


struct Real {
    Real(); Real(Rational); Real(Dyadic); Real(Integer);
    ValidatedReal check(Effort);
    friend Real sub(Real,Real);
            friend UpperReal sub(UpperReal,LowerReal);
            friend LowerReal sub(LowerReal,UpperReal);
            friend NaiveReal sub(NaiveReal,NaiveReal);
    friend Real div(Real,Real);
            friend NaiveReal div(NaiveReal,NaiveReal);
};
struct UpperReal {
    UpperReal(); UpperReal(Real);
    ValidatedUpperReal check(Effort);
            friend Real sub(Real,Real);
    friend UpperReal sub(UpperReal,LowerReal);
    friend LowerReal sub(LowerReal,UpperReal);
        friend NaiveReal sub(NaiveReal,NaiveReal);
        friend NaiveReal div(NaiveReal,NaiveReal);
};
struct LowerReal {
    LowerReal(); LowerReal(Real);
    ValidatedLowerReal check(Effort);
            friend Real sub(Real,Real);
    friend LowerReal sub(UpperReal,LowerReal);
    friend UpperReal sub(LowerReal,UpperReal);
        friend NaiveReal sub(NaiveReal,NaiveReal);
        friend NaiveReal div(NaiveReal,NaiveReal);
};
struct NaiveReal {
    NaiveReal(); NaiveReal(Real); NaiveReal(UpperReal); NaiveReal(LowerReal);
    ApproximateReal check(Effort);
            friend Real sub(Real,Real);
            friend UpperReal sub(LowerReal,UpperReal);
            friend UpperReal sub(UpperReal,LowerReal);
    friend NaiveReal sub(NaiveReal,NaiveReal);
        friend ApproximateReal sub(ApproximateReal,ApproximateReal);
    friend NaiveReal div(NaiveReal,NaiveReal);
        friend ApproximateReal div(ApproximateReal,ApproximateReal);
};


struct PositiveReal : Real {
    PositiveReal();
    friend PositiveReal div(PositiveReal,PositiveReal);
            friend PositiveUpperReal div(PositiveUpperReal,PositiveLowerReal);
            friend PositiveLowerReal div(PositiveLowerReal,PositiveUpperReal);
            friend PositiveNaiveReal div(PositiveNaiveReal,PositiveNaiveReal);
};
struct PositiveUpperReal : UpperReal {
    PositiveUpperReal(); PositiveUpperReal(PositiveReal);
    friend PositiveUpperReal div(PositiveUpperReal,PositiveLowerReal);
    friend PositiveLowerReal div(PositiveLowerReal,PositiveUpperReal);
        friend PositiveNaiveReal div(PositiveNaiveReal,PositiveNaiveReal);
};
struct PositiveLowerReal : LowerReal {
    PositiveLowerReal(); PositiveLowerReal(PositiveReal);
    friend PositiveLowerReal div(PositiveUpperReal,PositiveLowerReal);
    friend PositiveUpperReal div(PositiveLowerReal,PositiveUpperReal);
        friend PositiveNaiveReal div(PositiveNaiveReal,PositiveNaiveReal);
};
struct PositiveNaiveReal : NaiveReal {
    PositiveNaiveReal(); PositiveNaiveReal(PositiveReal); PositiveNaiveReal(PositiveUpperReal); PositiveNaiveReal(PositiveLowerReal);
    friend PositiveNaiveReal div(PositiveNaiveReal,PositiveNaiveReal);
//        friend PositiveApproximateReal div(PositiveApproximateReal,PositiveApproximateReal);
};


struct ValidatedReal {
    ValidatedReal(); ValidatedReal(Real);
    friend ValidatedReal sub(ValidatedReal,ValidatedReal);
        friend ValidatedUpperReal sub(ValidatedUpperReal,ValidatedLowerReal);
        friend ValidatedLowerReal sub(ValidatedLowerReal,ValidatedUpperReal);
            //friend ApproximateReal sub(ApproximateReal,ApproximateReal);
    friend ValidatedReal div(ValidatedReal,ValidatedReal);
        friend ApproximateReal div(ApproximateReal,ApproximateReal);
};
struct ValidatedUpperReal {
    ValidatedUpperReal(Real); ValidatedUpperReal(UpperReal);
    ValidatedUpperReal(); ValidatedUpperReal(ValidatedReal);
    friend ValidatedUpperReal sub(ValidatedUpperReal,ValidatedLowerReal);
    friend ValidatedLowerReal sub(ValidatedLowerReal,ValidatedUpperReal);
        friend ApproximateReal sub(ApproximateReal,ApproximateReal);
        friend ApproximateReal div(ApproximateReal,ApproximateReal);
};
struct ValidatedLowerReal {
    ValidatedLowerReal(Real); ValidatedLowerReal(LowerReal);
    ValidatedLowerReal(); ValidatedLowerReal(ValidatedReal);
    friend ValidatedLowerReal sub(ValidatedUpperReal,ValidatedLowerReal);
    friend ValidatedUpperReal sub(ValidatedLowerReal,ValidatedUpperReal);
        friend ApproximateReal sub(ApproximateReal,ApproximateReal);
        friend ApproximateReal div(ApproximateReal,ApproximateReal);
};
struct ApproximateReal {
    ApproximateReal(Real); ApproximateReal(UpperReal); ApproximateReal(LowerReal); ApproximateReal(NaiveReal);
    ApproximateReal(); ApproximateReal(ValidatedReal); ApproximateReal(ValidatedUpperReal); ApproximateReal(ValidatedLowerReal);
    friend ApproximateReal sub(ApproximateReal,ApproximateReal);
    friend ApproximateReal div(ApproximateReal,ApproximateReal);
};

struct SP { }; struct DP { }; struct MP { };
template<class PR> struct FloatValue; template<class PR, class PRE=PR> struct FloatBall; template<class PR> struct FloatBounds;
template<class PR> struct FloatUpperBound; template<class PR> struct FloatLowerBound; template<class PR> struct FloatApproximation;
template<class PR> struct PositiveFloatValue; template<class PR, class PRE=PR> struct PositiveFloatBall; template<class PR> struct PositiveFloatBounds;
template<class PR> struct PositiveFloatUpperBound; template<class PR> struct PositiveFloatLowerBound; template<class PR> struct PositiveFloatApproximation;

template<class PR> struct DeclareFloatOperations {
    using PRE = PR;
    friend FloatBall<PR> sub(FloatValue<PR>,FloatValue<PR>);
    friend FloatBall<PR> sub(FloatBall<PR,PRE>,FloatBall<PR,PRE>);
    friend FloatBounds<PR> sub(FloatBounds<PR>,FloatBounds<PR>);
    friend FloatUpperBound<PR> sub(FloatUpperBound<PR>,FloatLowerBound<PR>);
    friend FloatLowerBound<PR> sub(FloatLowerBound<PR>,FloatUpperBound<PR>);
    friend FloatApproximation<PR> sub(FloatApproximation<PR>,FloatApproximation<PR>);

//        friend FloatBall<PR,PRE> sub(FloatValue<PR>,FloatBall<PR,PRE>);
//        friend FloatBall<PR,PRE> sub(FloatBall<PR,PRE>,FloatValue<PR>);
//        friend FloatBounds<PR> sub(FloatBounds<PR>,FloatBall<PR,PRE>);
//        friend FloatBounds<PR> sub(FloatBall<PR,PRE>,FloatBounds<PR>);
//            friend FloatBounds<PR> sub(FloatBounds<PR>,FloatValue<PR>);
//            friend FloatBounds<PR> sub(FloatValue<PR>,FloatBounds<PR>);

    friend FloatBall<PR,PRE> sub(FloatBall<PR,PRE>,ValidatedReal);
    friend FloatBounds<PR> sub(FloatBounds<PR>,ValidatedReal);
    friend FloatUpperBound<PR> sub(FloatUpperBound<PR>,ValidatedLowerReal);
    friend FloatLowerBound<PR> sub(FloatLowerBound<PR>,ValidatedUpperReal);
    friend FloatApproximation<PR> sub(FloatApproximation<PR>,ApproximateReal);

    friend FloatBall<PR,PRE> sub(ValidatedReal,FloatBall<PR,PRE>);
    friend FloatBounds<PR> sub(ValidatedReal,FloatBounds<PR>);
    friend FloatUpperBound<PR> sub(ValidatedUpperReal,FloatLowerBound<PR>);
    friend FloatLowerBound<PR> sub(ValidatedLowerReal,FloatUpperBound<PR>);
    friend FloatApproximation<PR> sub(ApproximateReal,FloatApproximation<PR>);
};

template<class PR> struct FloatValue : DeclareFloatOperations<PR> {
    FloatValue(PR); FloatValue(Dyadic,PR); FloatValue<PR>& operator=(Dyadic); operator Dyadic();
//    friend FloatBall<PR> sub(FloatValue<PR>,FloatValue<PR>);
};
template<class PR, class PRE> struct FloatBall : DeclareFloatOperations<PR> {
    FloatBall(PR); FloatBall(ValidatedReal,PR); FloatBall<PR,PRE>& operator=(ValidatedReal); operator ValidatedReal();
    FloatBall(FloatValue<PR>); explicit FloatBall(FloatBounds<PR>);
    operator FloatBounds<PR>(); operator FloatUpperBound<PR>(); operator FloatLowerBound<PR>(); operator FloatApproximation<PR>();
//    friend FloatBall<PR,PRE> sub(FloatBall<PR,PRE>,FloatBall<PR,PRE>);
        friend FloatBall<PR,PRE> sub(FloatValue<PR>,FloatBall<PR,PRE>);
        friend FloatBall<PR,PRE> sub(FloatBall<PR,PRE>,FloatValue<PR>);
        friend FloatBounds<PR> sub(FloatBounds<PR>,FloatBall<PR,PRE>);
        friend FloatBounds<PR> sub(FloatBall<PR,PRE>,FloatBounds<PR>);
//    friend FloatBall<PR,PRE> sub(FloatBall<PR,PRE>,ValidatedReal);
//    friend FloatBall<PR,PRE> sub(ValidatedReal,FloatBall<PR,PRE>);
};
template<class PR> struct FloatBounds : DeclareFloatOperations<PR> {
    FloatBounds(PR); FloatBounds(ValidatedReal,PR); FloatBounds<PR>& operator=(ValidatedReal); operator ValidatedReal();
    FloatBounds(FloatValue<PR>);
//    friend FloatBounds<PR> sub(FloatBounds<PR>,FloatBounds<PR>);
//    friend FloatBounds<PR> sub(ValidatedReal,FloatBounds<PR>);
};
template<class PR> struct FloatUpperBound : DeclareFloatOperations<PR> {
    FloatUpperBound(PR); FloatUpperBound(ValidatedUpperReal,PR); FloatUpperBound<PR>& operator=(ValidatedUpperReal); operator ValidatedUpperReal();
    FloatUpperBound(FloatValue<PR>); FloatUpperBound(FloatBounds<PR>);
//    friend FloatUpperBound<PR> sub(FloatUpperBound<PR>,FloatLowerBound<PR>);
//    friend FloatLowerBound<PR> sub(FloatLowerBound<PR>,FloatUpperBound<PR>);
//    friend FloatUpperBound<PR> sub(FloatUpperBound<PR>,ValidatedLowerReal);
//    friend FloatLowerBound<PR> sub(ValidatedLowerReal,FloatUpperBound<PR>);
//        friend FloatApproximation<PR> sub(FloatApproximation<PR>,FloatApproximation<PR>);
};
template<class PR> struct FloatLowerBound : DeclareFloatOperations<PR> {
    FloatLowerBound(PR); FloatLowerBound(ValidatedLowerReal,PR); FloatLowerBound<PR>& operator=(ValidatedLowerReal); operator ValidatedLowerReal();
    FloatLowerBound(FloatValue<PR>); FloatLowerBound(FloatBounds<PR>);
//    friend FloatLowerBound<PR> sub(FloatLowerBound<PR>,FloatUpperBound<PR>);
//    friend FloatUpperBound<PR> sub(FloatUpperBound<PR>,FloatLowerBound<PR>);
//    friend FloatLowerBound<PR> sub(FloatLowerBound<PR>,ValidatedUpperReal);
//    friend FloatUpperBound<PR> sub(ValidatedUpperReal,FloatLowerBound<PR>);
//        friend FloatApproximation<PR> sub(FloatApproximation<PR>,FloatApproximation<PR>);
};
template<class PR> struct FloatApproximation : DeclareFloatOperations<PR> {
    FloatApproximation(PR); FloatApproximation(ApproximateReal,PR); FloatApproximation<PR>& operator=(ApproximateReal); operator ApproximateReal();
    FloatApproximation(FloatValue<PR>); FloatApproximation(FloatBounds<PR>);
    FloatApproximation(FloatUpperBound<PR>); FloatApproximation(FloatLowerBound<PR>);
//    friend FloatApproximation<PR> sub(FloatApproximation<PR>,FloatApproximation<PR>);
//    friend FloatApproximation<PR> sub(FloatApproximation<PR>,ApproximateReal);
//    friend FloatApproximation<PR> sub(ApproximateReal,FloatApproximation<PR>);
};

} // namespace Ariadne
