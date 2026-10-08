#include <cassert>
#include <memory>
#include <tuple>
#include <iostream>

#include "../../source/config.hpp"

#if defined HAVE_GMPXX_H

#include <gmpxx.h>

namespace Ariadne {

template<class T> using SharedPointer = std::shared_ptr<T>;
template<class... TS> using Tuple = std::tuple<TS...>;
using OutputStream = std::ostream;



template<class I, class X> I* heap_wrap_move(X&& x);

template<class I, class OP, class A> struct DoubleDispatchAwareInterface {
    virtual I* _rapply(OP op, A const* other) const = 0;
    virtual I* _lapply(OP op, A const* other) const = 0;
};

template<class I, class OP, class... AS> struct DoubleDispatchInterface;

template<class I, class OP> struct DoubleDispatchInterface<I,OP> : public I {
    virtual I* _apply(OP op, I const* other) const = 0;
    virtual I* _rapply(OP op, I const* other) const = 0;
};
template<class I, class OP, class A, class... AS> struct DoubleDispatchInterface<I,OP,A,AS...>
    : public DoubleDispatchInterface<I,OP,AS...>, public DoubleDispatchAwareInterface<I,OP,A> {
};

template<class X, class I, class OP, class TAS, class... AS> struct DoubleDispatchImplementation;

template<class X, class I, class OP, class... AS> struct DoubleDispatchImplementation<X,I,OP,Tuple<AS...>>
    : public DoubleDispatchInterface<I,OP,AS...>
{
    virtual I* _apply(OP op, I const* other) const {
        auto aother=dynamic_cast<DoubleDispatchAwareInterface<I,OP,X> const*>(other);
        if(aother) { return aother->_rapply(op,static_cast<X const*>(this)); }
        else { static_cast<DoubleDispatchInterface<I,OP>const*>(other)->_rapply(this); }
    }
    virtual I* _rapply(OP op, I const* other) const {
        auto aother=dynamic_cast<DoubleDispatchAwareInterface<I,OP,X> const*>(other);
        if(aother) { return aother->_apply(op,static_cast<X const*>(this)); }
        else { std::cerr << "Cannot apply '" << op << "' to " << *other << " and " << *this << "\n"; assert(false); }
    }
};

template<class X, class I, class OP, class TAS, class A> struct DoubleDispatchImplementation<X,I,OP,TAS,A>
    : public DoubleDispatchImplementation<X,I,OP,TAS>
{
    I* _lapply(OP op, A const* other) const override final { X const* self=static_cast<X const*>(this); return heap_wrap_move<I>(op(*self,*other)); }
    virtual I* _rapply(OP op, A const* other) const override final { X const* self=static_cast<X const*>(this); return heap_wrap_move<I>(op(*other,*self)); }
};
template<class X, class I, class OP, class TAS, class A, class... AS> struct DoubleDispatchImplementation<X,I,OP,TAS,A,AS...>
    : public DoubleDispatchImplementation<X,I,OP,TAS,AS...>
{
    using DoubleDispatchImplementation<X,I,OP,TAS,AS...>::_lapply;
    using DoubleDispatchImplementation<X,I,OP,TAS,AS...>::_rapply;
    I* _lapply(OP op, A const* other) const override final { X const* self=static_cast<X const*>(this); return heap_wrap_move<I>(op(*self,*other)); }
    I* _rapply(OP op, A const* other) const override final { X const* self=static_cast<X const*>(this); return heap_wrap_move<I>(op(*other,*self)); }
};

template<class X, class I, class OP, class TAS> class DoubleDispatchMixin;
template<class X, class I, class OP, class... AS> class DoubleDispatchMixin<X,I,OP,Tuple<AS...>> : public DoubleDispatchImplementation<X,I,OP,Tuple<AS...>,AS...> {
};



struct Add {
    template<class A1, class A2> decltype(auto) operator() (A1&& a1, A2&& a2) const { return add(std::forward<A1>(a1),std::forward<A2>(a2)); }
    friend OutputStream& operator<<(OutputStream& os, Add op) { return os << "add"; }
};
struct Sub {
    template<class A1, class A2> decltype(auto) operator() (A1&& a1, A2&& a2) const { return sub(std::forward<A1>(a1),std::forward<A2>(a2)); }
    friend OutputStream& operator<<(OutputStream& os, Sub op) { return os << "sub"; }
};
struct Mul {
    template<class A1, class A2> decltype(auto) operator() (A1&& a1, A2&& a2) const { return mul(std::forward<A1>(a1),std::forward<A2>(a2)); }
    friend OutputStream& operator<<(OutputStream& os, Mul op) { return os << "mul"; }
};

enum class OperatorCode : char { ADD, SUB, MUL, DIV };

struct RingOperator {
    OperatorCode _cd;
    RingOperator(Add op) : _cd(OperatorCode::ADD) { }
    RingOperator(Sub op) : _cd(OperatorCode::SUB) { }
    RingOperator(Mul op) : _cd(OperatorCode::MUL) { }

    template<class V> friend decltype(auto) visit(V const& v, RingOperator op) {
        switch (op._cd) {
            case OperatorCode::ADD: return v(Add());
            case OperatorCode::SUB: return v(Sub());
            case OperatorCode::MUL: return v(Mul());
            default: assert(false);
        }
    }
    template<class A1, class A2> decltype(auto) operator() (A1&& a1, A2&& a2) const {
        return visit([&](auto op){return op(a1,a2);},*this); }

    friend OutputStream& operator<<(OutputStream& os, RingOperator op) {
        return visit([&](auto op)->OutputStream&{os << op; return os;},op); }
};

class Integer : public mpz_class {
  public:
    explicit Integer(mpz_class mpz) : mpz_class(mpz) { }
    using mpz_class::mpz_class;
    friend Integer add(Integer const& z1, Integer const& z2) { return z1+z2; }
    friend Integer sub(Integer const& z1, Integer const& z2) { return z1-z2; }
    friend Integer mul(Integer const& z1, Integer const& z2) { return z1*z2; }
};
class Dyadic : public mpf_class {
  public:
    explicit Dyadic(mpf_class mpf) : mpf_class(mpf) { }
    using mpf_class::mpf_class;
    friend Dyadic add(Dyadic const& w1, Dyadic const& w2) { return w1+w2; }
    friend Dyadic sub(Dyadic const& w1, Dyadic const& w2) { return w1-w2; }
    friend Dyadic mul(Dyadic const& w1, Dyadic const& w2) { return w1*w2; }
};
class Rational : public mpq_class {
  public:
    explicit Rational(mpq_class mpq) : mpq_class(mpq) { }
    using mpq_class::mpq_class;
    friend Rational add(Rational const& q1, Rational const& q2) { return q1+q2; }
    friend Rational sub(Rational const& q1, Rational const& q2) { return q1-q2; }
    friend Rational mul(Rational const& q1, Rational const& q2) { return q1*q2; }
};

class Number;

class NumberInterface {
    friend class Number;
  public:
    virtual ~NumberInterface() = default;
    virtual NumberInterface* _apply(RingOperator op, NumberInterface const* other) const = 0;
    virtual OutputStream& _write(OutputStream& os) const = 0;
    friend OutputStream& operator<<(OutputStream& os, NumberInterface const& y) { return y._write(os); }
};

class Number {
    SharedPointer<NumberInterface> _ptr;
    static Number make(NumberInterface* ptr) { return Number(SharedPointer<NumberInterface>(ptr)); }
    template<class OP> static Number _apply(OP op, Number const& y1, Number const& y2) {
        return Number(SharedPointer<NumberInterface>(y1._ptr->_apply(op,y2._ptr.operator->()))); }
  public:
    explicit Number(SharedPointer<NumberInterface> ptr) : _ptr(ptr) { }
    explicit Number(Integer const& z);
    explicit Number(Dyadic const& w);
    friend Number add(Number const& y1, Number const& y2) { return _apply(Add(),y1,y2); }
    friend Number sub(Number const& y1, Number const& y2) { return _apply(Sub(),y1,y2); }
    friend Number mul(Number const& y1, Number const& y2) { return _apply(Mul(),y1,y2); }
    friend OutputStream& operator<<(OutputStream& os, Number const& y) { return y._ptr->_write(os); }
};

template<class T> struct Aware { typedef Tuple<T> Types; };
//template<> struct Aware<Integer> { typedef Tuple<Integer> Types; };
template<> struct Aware<Dyadic> { typedef Tuple<Dyadic,Integer> Types; };
template<> struct Aware<Rational> { typedef Tuple<Rational,Dyadic,Integer> Types; };
template<class T> using AwareTypes = typename Aware<T>::Types;


template<class X> class NumberWrapper;

template<class X, class I, class OP, class... AS> struct DoubleDispatchImplementation<NumberWrapper<X>,I,OP,Tuple<AS...>>
    : public DoubleDispatchInterface<I,OP,AS...>
{
    virtual I* _apply(OP op, I const* other) const {
        auto aother=dynamic_cast<DoubleDispatchAwareInterface<I,OP,X> const*>(other);
        if(aother) { return aother->_rapply(op,static_cast<NumberWrapper<X> const*>(this)); }
        else { static_cast<DoubleDispatchInterface<I,OP>const*>(other)->_rapply(op,this); }
    }
    virtual I* _rapply(OP op, I const* other) const {
        auto aother=dynamic_cast<DoubleDispatchAwareInterface<I,OP,X> const*>(other);
        if(aother) { return aother->_lapply(op,static_cast<NumberWrapper<X> const*>(this)); }
        else { std::cerr << "Cannot apply '" << op << "' to " << *other << " and " << *this << "\n"; assert(false); }
    }
};

template<class Y> class NumberWrapper
    : public DoubleDispatchMixin<NumberWrapper<Y>,NumberInterface,RingOperator,AwareTypes<Y>>, public Y
{
  public:
    NumberWrapper(Y const& y) : Y(y) { }
    NumberWrapper(Y&& y) : Y(std::move(y)) { }
    OutputStream& _write(OutputStream& os) const { return os << static_cast<Y const&>(*this); }
};

Number::Number(Integer const& z) : _ptr(new NumberWrapper<Integer>(z)) { }
Number::Number(Dyadic const& w) : _ptr(new NumberWrapper<Dyadic>(w)) { }

template<class I, class X> I* heap_wrap_move(X&& x) { return new NumberWrapper<X>(x); }

} // namespace Ariadne

using namespace Ariadne;


#define PRINT(expr) { std::cout << #expr << ": " << (expr) << "\n"; }

int main() {
    PRINT(sizeof(NumberWrapper<Integer>)-sizeof(Integer))
    PRINT(sizeof(NumberWrapper<Dyadic>)-sizeof(Dyadic))
    PRINT(sizeof(NumberWrapper<Rational>)-sizeof(Rational))

    Number yz(Integer(3));
    Number yw(Dyadic(5)/4);
    PRINT(yz);
    PRINT(yw);

    PRINT(add(yz,yz));
    PRINT(add(yz,yw));
    PRINT(add(yw,yz));
    PRINT(add(yw,yw));

    PRINT(sub(yz,yz));
    PRINT(sub(yz,yw));
    PRINT(sub(yw,yz));
    PRINT(sub(yw,yw));

    PRINT(mul(yz,yz));
    PRINT(mul(yz,yw));
    PRINT(mul(yw,yz));
    PRINT(mul(yw,yw));

}

#endif
