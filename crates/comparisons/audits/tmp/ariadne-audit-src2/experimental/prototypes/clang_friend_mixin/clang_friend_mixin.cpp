#include "clang_friend_mixin.hpp"

template<class PR> struct Operations<FloatApproximation<PR>> {
    static FloatApproximation<PR> _neg(FloatApproximation<PR> x) {
        return FloatApproximation<PR>(neg(near,x.raw())); }
    static FloatApproximation<PR> _add(FloatApproximation<PR> x1, FloatApproximation<PR> x2) {
        return FloatApproximation<PR>(add(near,x1.raw(),x2.raw())); }

};

template<class PR> struct Operations<FloatUpperBound<PR>,FloatLowerBound<PR>> {
    static FloatLowerBound<PR> _neg(FloatUpperBound<PR> x) {
        return FloatLowerBound<PR>(neg(down,x.raw())); }
    static FloatUpperBound<PR> _add(FloatUpperBound<PR> x1, FloatUpperBound<PR> x2) {
        return FloatUpperBound<PR>(add(up,x1.raw(),x2.raw())); }
};

template<class PR> struct Operations<PositiveFloatUpperBound<PR>> : Operations<FloatUpperBound<PR>,FloatLowerBound<PR>> {
    static PositiveFloatUpperBound<PR> _add(PositiveFloatUpperBound<PR> x1, PositiveFloatUpperBound<PR> x2) {
        return PositiveFloatUpperBound<PR>(add(up,x1.raw(),x2.raw())); }
    static PositiveFloatUpperBound<PR> _mul(PositiveFloatUpperBound<PR> x1, PositiveFloatUpperBound<PR> x2) {
        return PositiveFloatUpperBound<PR>(mul(up,x1.raw(),x2.raw())); }
};

template<class PR> struct Operations<FloatBounds<PR>> {
    static FloatBounds<PR> _neg(FloatBounds<PR> x) {
        return FloatBounds<PR>(neg(down,x.upper_raw()),neg(up,x.lower_raw())); }
    static FloatBounds<PR> _add(FloatBounds<PR> x1, FloatBounds<PR> x2) {
        return FloatBounds<PR>(add(down,x1.lower_raw(),x2.lower_raw()),add(up,x1.upper_raw(),x2.upper_raw())); }
};

template struct Operations<FloatBounds<DP>>;
template struct Operations<FloatUpperBound<DP>,FloatLowerBound<DP>>;
template struct Operations<FloatApproximation<DP>>;

template struct Operations<PositiveFloatUpperBound<DP>>;
