/***************************************************************************
 *            dynamic_game.cpp
 *
 *  Copyright  2015-17  Pieter Collins, Chun Wen
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

#include <cassert>
#include <vector>
#include <initializer_list>
#include <vector>
#include <set>
#include <utility>
#include <memory>
#include <iostream>
#include <sstream>

#include "numeric/rational.hpp"
#include "numeric/real.hpp"
#include "numeric/floatdp.hpp"
#include "symbolic/variable.hpp"
#include "symbolic/expression.hpp"
#include "symbolic/assignment.hpp"

typedef std::ostream OutputStream;
typedef std::string String;
typedef unsigned int Nat;
template<class T> using Array = std::vector<T>;
template<class T> using List = std::vector<T>;
template<class T> using Set = std::set<T>;
template<class T> using InitializerList = std::initializer_list<T>;
template<class T1, class T2> using Pair = std::pair<T1,T2>;

template<class T> using SharedPointer=std::shared_ptr<T>;
using std::make_pair;

using Float = Ariadne::FloatDP;
using Real = Ariadne::Real;
using Expression = Ariadne::RealExpression;

//template<class T> OutputStream& operator<<(OutputStream& os, std::vector<T> const& ary) {
//    for(Nat i=0; i!=ary.size(); ++i) { os << (i==0?"[ ":", ") << ary[i]; } return os << " ]"; }
template<class T1, class T2> OutputStream& operator<<(OutputStream& os, std::pair<T1,T2> const& pr) {
    return os << "(" << pr.first << "," << pr.second << ")"; }

String to_string(Nat index) {
    std::stringstream ss;
    ss<<index;
    return ss.str();
}


class Rational : public Ariadne::Rational {
  public:
    using Ariadne::Rational::Rational;
    Rational(double x=0.0) : Ariadne::Rational(Float(x)) { }
};
bool operator==(Rational const& q, int n) {return q==Rational(n); }
bool operator<=(Rational const& q, int n) {return q<=Rational(n); }
bool operator>=(Rational const& q, int n) {return q>=Rational(n); }


class Variable : public Ariadne::RealVariable {
  public:
    Variable(String name) : Ariadne::RealVariable(name) { }
    Variable(String name, Nat index) : Variable(name+to_string(index)) { }
    Variable(String name, Nat index1, Nat index2) : Variable(name+to_string(index1)+to_string(index2)) { }
    friend OutputStream& operator<<(OutputStream& os, Variable const& v) {
        return os << static_cast<Ariadne::RealVariable const&>(v); }
};
class Variables : public List<List<Variable>> {
  public:
    Variables(String name, Nat size, Nat sizes) : List<List<Variable>>(size,List<Variable>(sizes,Variable(name))) {
        for(Nat i=0; i!=size; ++i) { for(Nat j=0; j!=sizes; ++j) { (*this)[i][j]=Variable(name,i,j); } } }


};

struct Plus { friend OutputStream& operator<<(OutputStream& os, Plus op) { return os << " + "; } };
struct Minus { friend OutputStream& operator<<(OutputStream& os, Minus op) { return os << " - "; } };
struct Times { friend OutputStream& operator<<(OutputStream& os, Times op) { return os << "*"; } };
struct Divides { friend OutputStream& operator<<(OutputStream& os, Divides op) { return os << " / "; } };
struct Equals { friend OutputStream& operator<<(OutputStream& os, Equals op) { return os << "=="; } };
struct LessEqual { friend OutputStream& operator<<(OutputStream& os, LessEqual op) { return os << "<="; } };

class Probability {
    Rational _p;
  public:
    Probability() : _p(0) { }
    Probability(double  d) : _p(d) { assert(_p>=0&&_p<=1); }
    Probability(Rational q) : _p(q) { assert(_p>=0&&_p<=1); }
    operator Rational() const { return _p; }
    friend bool operator>=(Probability p, double d) { return p._p>=Rational(d); }
    friend OutputStream& operator<<(OutputStream& os, Probability p) { return os << p._p; }
};

struct Player { Nat _p; Player(Nat p):_p(p){} operator Nat() const { return _p; } Player& operator++() { ++_p; } };
struct State { Nat _s; State(Nat s):_s(s){} operator Nat() const { return _s; } State& operator++() { ++_s; } };
struct Action { Nat _a; Action(Nat a):_a(a){} operator Nat() const { return _a; } Action& operator++() { ++_a; } };
Pair<Action,Action> operator,(Action i, Action j) { return std::make_pair(i,j); }
Rational beta=Rational(3,4);

struct States { Nat _n; };

static const Nat number_of_players=2u;
static const Nat number_of_states=2u;
static const Nat maximum_number_of_actions=3u;

Variables x("x",number_of_states,maximum_number_of_actions);
Variables y("y",number_of_states,maximum_number_of_actions);
Variables al("a",number_of_players,number_of_states);

typedef Pair<Action,Action> Actions;
typedef Pair<Rational,Rational> Rewards;

class Probabilities {
    Array<Probability> _ary;
  public:
    Probabilities(Nat n) : _ary(n) {
        for(auto i=1u; i!=_ary.size(); ++i) { _ary[i]=Rational(1,n); } }
    Probabilities(InitializerList<Probability> pr) : _ary(pr) {
        Rational sum=_ary[0]; for(auto i=1u; i!=_ary.size(); ++i) { assert(Rational(_ary[i])>=0); sum+=_ary[i]; } assert(sum==1); }
    Nat size() const { return _ary.size(); }
    auto operator[] (State s) const -> Probability { return _ary[s]; }
    auto operator[] (State s) -> Probability& { return _ary[s]; }
    friend OutputStream& operator<<(OutputStream& os, Probabilities const& p) { return os << p._ary; }
};

template<class T> class Matrix {
    Array<Array<T>> _ary;
  public:
    Matrix(Nat m1, Nat m2, T t=T()) : _ary(m1,Array<T>(m2,t)) { }
    auto operator[] (Pair<Nat,Nat> a) const -> T { return _ary[a.first][a.second]; }
    auto operator[] (Actions a) const -> T { return _ary[a.first][a.second]; }
    auto operator[] (Actions a) -> T& { return _ary[a.first][a.second]; }
    friend OutputStream& operator<<(OutputStream& os, Matrix<T> const& A) { return os << A._ary; }
};

Expression operator*(Rational q, Expression e) {
    return Real(q)*e; }
Expression operator*(Probability p, Expression e) {
    return Rational(p)*e; }

Expression operator*(Probabilities p, List<Variable> v) {
    assert(p.size()==v.size()); assert(!v.empty()); Expression r=p[0]*v[0]; for(Nat i=1; i!=v.size(); ++i) { r=r+p[1]*v[1]; } return r; }
Expression operator*(Probabilities p, List<Expression> e) {
    assert(p.size()==e.size()); assert(!e.empty()); Expression r=p[0]*e[0]; for(Nat i=1; i!=e.size(); ++i) { r=r+p[1]*e[1]; } return r; }

struct Transitions : List<Matrix<Probabilities>> {
    using List<Matrix<Probabilities>>::List;
    Expression operator() (State t, State s, List<Variable> x, List<Variable> y) {
        auto p=*this;
        Expression r;
        for(Action i=0; i!=x.size(); ++i) { for(Action j=0; j!=y.size(); ++j) { r=r+p[s][(i,j)][t]*x[i]*y[j]; } }
        return r;
    }
    Expression operator() (State t, State s, Variables x, Variables y) {
        auto p=*this;
        Expression r;
        for(Action i=0; i!=x[s].size(); ++i) { for(Action j=0; j!=y[s].size(); ++j) { r=r+p[s][(i,j)][t]*x[s][i]*y[s][j]; } }
        return r;
    }
    Expression operator() (State t, State s, Action i, Variables y) {
        auto p=*this; Expression r=p[s][(i,0)][t]*y[s][0];
        for(Action j=1; j!=y[s].size(); ++j) { r=r+p[s][(i,j)][t]*y[s][j]; }
        return r;
    }
    Expression operator() (State t, State s, Variables x, Action j) {
        auto p=*this; Expression r=p[s][(0,j)][t]*x[s][0];
        for(Action i=0; i!=x[s].size(); ++i) { r=r+p[s][(i,j)][t]*x[s][i]; }
        return r;
    }
};

struct AllRewards : List< Matrix<Rewards> > {
    using List< Matrix<Rewards> >::List;
    Expression operator() (Player k, State s, List<Variable> x, List<Variable> y) {
        auto rw=*this;
        Expression r;
        for(Action i=0; i!=x.size(); ++i) { for(Action j=0; j!=y.size(); ++j) { r=r+rw[s][(i,j)].first*x[i]*y[j]; } }
        return r;
    }
};

class DynamicGame {
  public:
    DynamicGame(Nat number_of_players, Nat number_of_states, Nat maximum_number_of_actions) {
        assert(number_of_players==2);
        Probabilities uniform_probability(number_of_states);
        Matrix<Rewards> zero_rewards(maximum_number_of_actions,maximum_number_of_actions);
        Matrix<Probabilities> uniform_probabilities(maximum_number_of_actions,maximum_number_of_actions,uniform_probability);
        rewards=AllRewards(number_of_states,zero_rewards);
        transitions=Transitions(number_of_states,uniform_probabilities); }
    void set_data(State s, Actions a, Probabilities p, Rewards r) {
        rewards[s][a]=r; transitions[s][a]=p; }
    auto reward(State s, Probabilities x, Probabilities y) -> Real;
    friend OutputStream& operator<<(OutputStream& os, DynamicGame gm) {
        return os << "\n\n" << gm.rewards << "\n" << gm.transitions << "\n\n"; }
  private:
  public:
    AllRewards rewards;
    Transitions transitions;
};

//RealExpression f= sum[k] ( sum[s] ( al[k,s] * (1-beta)* r[k](s,x[s],y[s])-beta*sum(p(t,s,x[s],y[s])*al[k][t]) ) );
//RealConstraint all[i], all[s], a[p1][s] >= (1-beta) * r[p1](s,i,y[s]) + beta* sum[t](p(t,s,i,y[s]))*al[p1][t]
//RealConstraint all[j], all[s], a[p1][s] >= (1-beta) * r[p2](s,x[s],j) + beta* sum[t](p(t,s,x[s],j))*al[p2][t]

struct SumOver { };

struct Sum { };

Sum sum;


int main() {
    const Nat number_of_players = 2;
    const Nat number_of_states = 2;
    const Nat maximum_number_of_actions = 2;

    Player p1=0; Player p2=1;
    State s1=0; State s2=1;
    Action i1=0; Action i2=1;
    Action j1=0; Action j2=1;

    DynamicGame game(number_of_players,number_of_states,maximum_number_of_actions);
    game.set_data(s1,{i1,j1},{0.5,0.5},{3,4});
    game.set_data(s1,{i1,j2},{0.0,1.0},{1,1});
    game.set_data(s1,{i2,j1},{0.0,1.0},{2,2});
    game.set_data(s1,{i2,j2},{0.5,0.5},{4,3});
    game.set_data(s2,{i1,j1},{0.5,0.5},{4,5});
    game.set_data(s2,{i1,j2},{1.0,0.0},{3,3});
    game.set_data(s2,{i2,j1},{1.0,0.0},{2,2});
    game.set_data(s2,{i2,j2},{0.5,0.5},{5,4});
    std::cout << game << "\n";

    auto x=Variables("x",number_of_players,maximum_number_of_actions);
    auto y=Variables("y",number_of_states,maximum_number_of_actions);
    auto al=Variables("al",number_of_players,number_of_states);
    std::cout << x << "\n";
    std::cout << y << "\n";
    std::cout << al << "\n";

    auto p=game.transitions;
    auto r=game.rewards;
    std::cout<<"p[1][0,0]*x[1]="<<p[1][make_pair(0,0)]*x[1]<<"\n";
    std::cout<<"p(0,0,x,y)="<<p(0,0,x,y)<<"\n";
    std::cout<<"p(0,0,1,y)="<<p(0,0,1,y)<<"\n";
    std::cout<<"p(0,0,x,1)="<<p(0,0,x,1)<<"\n";

    Player k=0; List<Player> players={p1,p2};
    Expression fs=Expression::constant(0);
    auto states=List<State>{s1,s2};
    for(Player k : players) {
        for(State s : states) {
            fs=fs+al[k][s]*(1-beta)*r(k,s,x[s],y[s]);
            for(State t : states) {
                fs=fs-beta*p(t,s,x[s],y[s])*al[k][t];
            }
        }
    }
   // RealExpression f= sum[k] ( sum[s] ( al[k,s] * (1-beta)* r[k](s,x[s],y[s])-beta*sum[t](p(t,s,x[s],y[s])*al[k][t]) ) );
}
