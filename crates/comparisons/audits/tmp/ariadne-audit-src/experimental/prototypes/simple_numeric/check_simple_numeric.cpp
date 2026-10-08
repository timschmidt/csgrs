#include "simple_numeric.hpp"

using namespace Ariadne;

template<class T> const char* class_name();
#define CLASS_NAME(T) template<> const char* class_name<T>() { return #T; }
CLASS_NAME(Real)
CLASS_NAME(UpperReal)
CLASS_NAME(LowerReal)
CLASS_NAME(NaiveReal)
CLASS_NAME(PositiveReal)
CLASS_NAME(PositiveUpperReal)
CLASS_NAME(PositiveLowerReal)
CLASS_NAME(PositiveNaiveReal)
CLASS_NAME(ValidatedReal)
CLASS_NAME(ValidatedUpperReal)
CLASS_NAME(ValidatedLowerReal)
CLASS_NAME(ApproximateReal)
CLASS_NAME(PositiveValidatedReal)
CLASS_NAME(PositiveValidatedUpperReal)
CLASS_NAME(PositiveValidatedLowerReal)
CLASS_NAME(PositiveApproximateReal)
CLASS_NAME(FloatValue<DP>)
CLASS_NAME(FloatBall<DP>)
CLASS_NAME(FloatBounds<DP>)
CLASS_NAME(FloatUpperBound<DP>)
CLASS_NAME(FloatLowerBound<DP>)
CLASS_NAME(FloatApproximation<DP>)
CLASS_NAME(PositiveFloatValue<DP>)
CLASS_NAME(PositiveFloatBall<DP>)
CLASS_NAME(PositiveFloatBounds<DP>)
CLASS_NAME(PositiveFloatUpperBound<DP>)
CLASS_NAME(PositiveFloatLowerBound<DP>)
CLASS_NAME(PositiveFloatApproximation<DP>)
#undef CLASS_NAME



template<class A1, class A2> void sub_verbose(A1 const& a1, A2 const& a2) {
    std::cout << "sub(" << class_name<A1>() << "," << class_name<A2>() << ") -> ";
    typedef decltype(sub(a1,a2)) R; std::cout << class_name<R>() << "\n"; }

void sub_all() { }
template<class A1, class... AS> void sub_all(A1 const& a1, AS... as) {
    sub_verbose(a1,a1); sub_all(as...);
}

template<class... AS> class Tags { };
template<class A> class Tag { };

template<class A1, class A2> void check_element(Sub,Tag<A1>,Tag<A2>) {
    std::cout << "sub(" << class_name<A1>() << "," << class_name<A2>() << ") -> ";
    typedef decltype(sub(declval<A1>(),declval<A2>())) R; std::cout << class_name<R>() << "\n"; }
template<class A1, class A2> void check_element(Div,Tag<A1>,Tag<A2>) {
    std::cout << "div(" << class_name<A1>() << "," << class_name<A2>() << ") -> ";
    typedef decltype(div(declval<A1>(),declval<A2>())) R; std::cout << class_name<R>() << "\n"; }

template<class OP, class A1> void check_row(OP,Tag<A1>,Tags<>) { }
template<class OP, class A1, class A2, class... A2S> void check_row(OP,Tag<A1>,Tags<A2,A2S...>) {
    check_element(OP(),Tag<A1>(),Tag<A2>()); check_row(OP(),Tag<A1>(),Tags<A2S...>()); }

template<class OP, class TAGS2> void check_table(OP,Tags<>,TAGS2) { }
template<class OP, class TAGS2, class A1, class... A1S> void check_table(OP,Tags<A1,A1S...>,TAGS2) {
    check_row(OP(),Tag<A1>(),TAGS2()); check_table(OP(),Tags<A1S...>(),TAGS2()); }

template<class OP, class... AS> void check_all() { OP op; Tags<AS...> tags; check_table(op,tags,tags); }


/*
template<class A1, class A2> void sub_verbose(A1 const& a1, A2 const& a2) {
    std::cout << "sub(" << class_name<A1>() << "," << class_name<A2>() << ") -> ";
    typedef decltype(sub(a1,a2)) R; std::cout << class_name<R>() << "\n"; }

template<class TUP, SizeType... Inds> void sub_all_impl(TUP const& tup, IndexSequence<Inds...>) {
    using swallow = int[]; // guaranties left to right order
    (void)swallow{0, (void(sub_verbose(std::get<Inds>(tup),std::get<Inds>(tup))),0)...};
}
template<class... AS> void sub_all(Tuple<AS...> const& tup) {
    sub_all_impl(tup, std::index_sequence_for<AS...>());
}
*/

using namespace std;

int main() {

    using GenericTypes = Tuple<Real,UpperReal,LowerReal,NaiveReal, ValidatedReal,ValidatedUpperReal,ValidatedLowerReal,ApproximateReal>;

    using FloatTypes = Tuple<FloatValue<DP>,FloatBall<DP>,FloatBounds<DP>,FloatUpperBound<DP>,FloatLowerBound<DP>,FloatApproximation<DP>>;

    //sub_all(generics);
    //sub_all(Real(),UpperReal(),LowerReal(),NaiveReal(), ValidatedReal(),ValidatedUpperReal(),ValidatedLowerReal(),ApproximateReal());
    check_all<Sub,Real,UpperReal,LowerReal,NaiveReal, ValidatedReal,ValidatedUpperReal,ValidatedLowerReal,ApproximateReal>();
    check_all<Div,Real,UpperReal,LowerReal,NaiveReal, ValidatedReal,ValidatedUpperReal,ValidatedLowerReal,ApproximateReal>();

   // check_sub_all<FloatValue<DP>,FloatBall<DP>,FloatBounds<DP>,FloatUpperBound<DP>,FloatLowerBound<DP>,FloatApproximation<DP>>();
}
