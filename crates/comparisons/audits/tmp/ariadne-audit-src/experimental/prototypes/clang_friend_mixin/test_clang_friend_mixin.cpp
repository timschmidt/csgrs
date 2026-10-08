#include "clang_friend_mixin.hpp"

#define PRINT(expr) { std::cout << #expr << ": " << (expr) << std::endl; }

template<class PR> void test() {
    PR pr;
    ApproximateNumber y(3);
    FloatApproximation<PR> x(4.25,pr);

    add(x,x);
    neg(x);
    add(x,y);

    Add op;
    op(x,y);

    FloatLowerBound<PR> l(4.25,pr);
    FloatUpperBound<PR> u(4.25,pr);

    PositiveFloatUpperBound<PR> pu(u);
    pu=add(pu,pu);
    pu=mul(pu,pu);
    l=neg(pu);

    FloatError<PR> e(4.25,pr);
    PRINT(e);
    e=add(e,e);
    e=mul(e,e);
    l=neg(e);

    e=add(3u,e);
    u=add(2,e);
};

int main() {
    test<DP>();
}
