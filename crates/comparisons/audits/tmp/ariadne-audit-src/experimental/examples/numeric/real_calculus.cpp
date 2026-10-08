/***************************************************************************
 *            real_calculus.cpp
 *
 *  Copyright  2017-20  Pieter Collins
 *      (Based on joint work with M. Konecny, N. Mueller and M. Ziegler)
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
 *  You should have received a copy of the GNU G3c767e04cec413f9afb4c30b521ca71ceb5b0409eneral Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <vector>
#include <iostream>
#include <functional>
#include <initializer_list>

//void print(std::initializer_list<double> const& lst) { for(auto x:lst) { std::cout << x << ","; } }
//int main() { double a,b,c; print({a=2,b=a+1,c=b+2}); }


namespace Ariadne {

template<class T> using InitializerList = std::initializer_list<T>;

class Natural {
    uint _m;
  public:
    Natural(uint m=0) { _m=m; }
    Natural& operator++() { ++_m; return *this; }
    friend Natural operator-(Natural, Natural);

};

class Dyadic {
    double _d;
  public:
    Dyadic(double d) : _d(d) { }
};
class StrictlyPositiveDyadic : public Dyadic {
};

template<class T> class List : public std::vector<T> {
public:
    using std::vector<T>::vector;
    T const& operator[] (Natural i) const;
    T& operator[] (Natural i);
};

template<class S> class Function : public std::function<S> { using std::function<S>::function; };
template<class R> class Sequence : public Function<R(Natural)> { };
template<class R> class Array { public: Array(Natural n); R operator[](Natural) const; R& operator[](Natural); };
template<class R> class ApproximationScheme : public Function<R(StrictlyPositiveDyadic)> { using Function<R(StrictlyPositiveDyadic)>::Function; };
template<class R, class... AS> class ApproximationSchemeFrom : public Function<R(AS...,StrictlyPositiveDyadic)> { };

struct Range { Range(Natural, Natural); };

class Real;
using ArrayOfReal = Array<Real>;
using ApproximationSchemeOfReal = ApproximationScheme<Real>;
using SequenceOfReal = Sequence<Real>;
using ListOfReal = List<Real>;

class Logical {
public:
    explicit operator bool() const;
    friend bool definitely(Logical);
};

struct Statement { };
struct RealAssignment : Statement { RealAssignment(Real& r, Real const& a); operator Real&(); };
struct Block { Block(InitializerList<Statement>); Block(List<Statement>); Block(Statement); };
Block operator,(RealAssignment,RealAssignment);
Block operator,(Statement,Statement);
Block operator,(Block,Statement);

class Real {
  public:
    Real();
    Real(int);
    Real(Natural);
    Real(Dyadic);
    explicit Real(ApproximationSchemeOfReal);
    RealAssignment operator=(Real const& a);

};
Real neg(Real);
Real rec(Real);
Real abs(Real);
Real add(Real,Real);
Real sub(Real,Real);
Real mul(Real,Real);
Real div(Real,Real);
Logical leq(Real,Real);
Logical lt(Real,Real);

inline Real operator+(Real r1, Real r2) { return add(r1,r2); }
inline Real operator-(Real r1, Real r2) { return sub(r1,r2); }
inline Real operator*(Real r1, Real r2) { return mul(r1,r2); }
inline Real operator/(Real r1, Real r2) { return div(r1,r2); }
inline Logical operator<=(Real r1, Real r2) { return leq(r1,r2); }
inline Logical operator<(Real r1, Real r2) { return lt(r1,r2); }


SequenceOfReal iterate(Function<Real(Real)>,Real);
Real limit(SequenceOfReal);

Real newton_iter(Function<Real(Real)> f, Function<Real(Real)> df, Real x) { return sub(x,div(f(x),df(x))); }

//struct RealAssignment { };
struct RealVariable { RealAssignment operator=(Real const&); };
RealVariable let(Real&);
//Real& let(Real&);


template<class L, class A> struct IfThenElse { IfThenElse(L,A,A); };
template<class L, class A> struct IfThen { IfThen(L,A); Block else_(A); };//IfThenElse<L,A> else_(A); };
template<class L> struct If { If(L); template<class... AS> IfThen<L,Block> then_(AS...); IfThen<L,Block> then_(Block); };
template<class L> If<L> if_(L);

template<class L, class A> struct WhileDo { WhileDo(L,A); };
template<class L> struct While { While(L); template<class... AS> WhileDo<L,Block> do_(AS...); WhileDo<L,Block> do_(Block); };
template<class L> While<L> while_(L);

//template<class T> void while_do(Function<Logical(Real)>,T);
Statement while_do(Logical,List<RealAssignment>);
//void while_do(Logical,List<Real&>);

Statement if_then_else(Logical, Block, Block);

Statement for_do(Natural& i, Range n, Block);



Real sqrt_approx(Real x, StrictlyPositiveDyadic e) {
    Real y=1;
    while( (bool) (abs(x-y)<e) ) { y=(y+x/y)/2; }
    while_do ( abs(x-y)<e ,  { y=(y+x/y)/2 } );
    return y;
}

Real sqrt(Real x) {
    return Real([x](StrictlyPositiveDyadic e){return sqrt_approx(x,e);});
}

struct capture { capture(InitializerList<RealAssignment>); };

struct Infinity { Infinity(); friend bool operator!=(Natural, Infinity); };
static const Infinity inf;

} // namespace Ariadne

using namespace Ariadne;

int main() {
    Real a=3;
    Function<Real(Real)> fa=[&a](Real r){return sub(mul(r,r),a);};
    Function<Real(Real)> dfa=[](Real r){return 2*r;};

    Real r=1;
    r = newton_iter(fa,dfa,r);

    SequenceOfReal seq = iterate([&](Real r){return newton_iter(fa,dfa,r);},r);

    Array<Real> ary(3);
    Natural i; Real t;
    for_do( i , Range(0,3), { t=Real(i)*i, ary[i]=t+1 } );
    a=0; Real b=1; Real c=3;

    while_( lt(a,3) ).
    do_({ a = a+b/2+1, b=(a+3)/2});
    while_( lt(a,3) ).
    do_( a = a+b/2+1, b=(a+3)/2);
//    do_{ a = a+b/2+1, b=(a+3)/2};
//    while_do( lt(a,3),{a=a/2+1, a=(a+3)/2});
    while( definitely(lt(a,3)) ) { a=a/2+1; a=(a+3)/2;}

    ListOfReal expseq;
    expseq[0]=1;


    (a,b=Real(2),c);
    for_do( i, Range(0,-1u), { expseq[i]=expseq[i-1]/i} );
    for(Natural i=1; i!=inf; ++i) {
        expseq[i]=expseq[i]/i;
    }
    if_(a<3).
    then_(a=a+1, b=a/2+1).
    else_({a=a-1});

    capture{ a=1, b=a+1, a=b+2 };
    return 0;

}
// It's entirely feasible to have higher-order constructs in the language.
// It seems impossible to do so using usual C++ syntax.
// We *need* functions, and since the arguments to a function must be *expressions*, not *statements*,
// we *need* to generate a list of "statements" in the code of a loop or conditional.
// We can't generate a list of C++ statements (without global variables) so need to generate a list of (assignment) expressions.
// This list must be separated by commas, not semicolons, and cannot declare any new variables.
// It can either go through an initializer_list, or use the sequencing operator.
// Alternatively, we can use a functional approach to compound statements.
