#include <type_traits>
#include <iostream>

using Dummy=int; static const Dummy dummy=0;
template<class P, class T=Dummy> using EnableIf = typename std::enable_if<P::value,T>::type;
using True = std::true_type;
using False = std::false_type;

using OutputStream = std::ostream;

struct Neg {
    template<class A> decltype(auto) operator()(A&& a) const { return neg(a); }
};
struct Add {
    template<class A1, class A2> decltype(auto) operator()(A1&& a1, A2&& a2) const { return add(a1,a2); }
};
struct Mul {
    template<class A1, class A2> decltype(auto) operator()(A1&& a1, A2&& a2) const { return mul(a1,a2); }
};

template<class X, class NX=X> struct Operations;

template<class X, class NX=X, class R=X> struct ProvideGroupOperations {
    typedef Operations<X,NX> OperationsType;
    friend NX operator-(X const& x) { return neg(x); }
    friend R operator+(X const& x1, X const& x2) { return add(x1,x2); }
    friend NX neg(X const& x) { return OperationsType::_neg(x); }
    friend R add(X const& x1, X const& x2) { return OperationsType::_add(x1,x2); }
};
template<class X, class R=X, class QR=R> struct ProvideSemiringOperations {
    typedef Operations<X> OperationsType;
    friend R operator+(X const& x1, X const& x2) { return add(x1,x2); }
    friend R operator*(X const& x1, X const& x2) { return mul(x1,x2); }
    friend R add(X const& x1, X const& x2) { return OperationsType::_add(x1,x2); }
    friend R mul(X const& x1, X const& x2) { return OperationsType::_mul(x1,x2); }
};
template<class X, class R=X> struct ProvideRingOperations {
    typedef Operations<X> OperationsType;
    friend X operator-(X const& x) { return neg(x); }
    friend R operator+(X const& x1, X const& x2) { return add(x1,x2); }
    friend R operator*(X const& x1, X const& x2) { return mul(x1,x2); }
    friend X neg(X const& x) { return OperationsType::_neg(x); }
    friend R add(X const& x1, X const& x2) { return OperationsType::_add(x1,x2); }
    friend R mul(X const& x1, X const& x2) { return OperationsType::_mul(x1,x2); }
};

template<class Y> struct IsGenericNumber : False { };
template<> struct IsGenericNumber<uint> : True { };
template<> struct IsGenericNumber<int> : True { };

template<class P> class Number;
template<class P> struct IsGenericNumber<Number<P>> : True { };


template<class X, class Y=void> struct ProvideConcreteGenericGroupOperations {
    typedef Operations<X> OperationsType;
    friend decltype(auto) add(X const& x1, Y const& y2) { return add(x1,factory(x1).create(y2)); }
    friend decltype(auto) add(Y const& y1, X const& x2) { return add(factory(x2).create(y1),x2); }
};

template<class X> struct ProvideConcreteGenericGroupOperations<X,void> {
    template<class Y, EnableIf<IsGenericNumber<Y>> =dummy> friend decltype(auto)
        add(Y const& y, X const& x) { return add(factory(x).create(y),x); }
    template<class Y, EnableIf<IsGenericNumber<Y>> =dummy> friend decltype(auto)
        add(X const& x, Y const& y) { return add(x,factory(x).create(y)); }
};


template<class X, class NX> struct Operations {
    static NX _neg(X x);
    static X _add(X x1, X x2);
    static X _mul(X x1, X x2);
};

struct ValidatedTag;
struct ApproximateTag;
template<class P> class Number {
    int _n;
  public:
    Number(int n) : _n(n) { }
    double get_d() { return _n; }
    template<class OP> Number(OP op, Number<P> y1, Number<P> y2) { return op(y1._n,y2._n); }
    friend Number<P> add(Number<P> y1, Number<P> y2) { return Number<P>(Add(),y1,y2); }
    friend Number<P> mul(Number<P> y1, Number<P> y2) { return Number<P>(Mul(),y1,y2); }
};
using ApproximateNumber = Number<ApproximateTag>;
using ValidatedNumber = Number<ValidatedTag>;

class RoundingMode {
  public:
    explicit RoundingMode(char c) { }
};
static const RoundingMode up = RoundingMode(+1);
static const RoundingMode near = RoundingMode(0);
static const RoundingMode down = RoundingMode(-1);

template<class PR> class Float {
    double _d;
  public:
    Float(double x, PR pr) : _d(x) { }
    double get_d() const { return _d; }
    PR precision() const { return PR(); }
    friend Float<PR> neg(RoundingMode rnd, Float<PR> x) {
        return Float<PR>(-x.get_d(),x.precision()); }
    friend Float<PR> add(RoundingMode rnd, Float<PR> x1, Float<PR> x2) {
        return Float<PR>(x1.get_d()+x2.get_d(),x1.precision()); }
    friend Float<PR> mul(RoundingMode rnd, Float<PR> x1, Float<PR> x2) {
        return Float<PR>(x1.get_d()*x2.get_d(),x1.precision()); }

    friend OutputStream& operator<<(OutputStream& os, Float<PR> const& x) { return os << x._d; }
};


template<class PR> class FloatBounds;
template<class PR> class FloatUpperBound;
template<class PR> class FloatLowerBound;

template<class PR> class Float
    : public ProvideRingOperations<FloatBounds<PR>>
{
    Float<PR> _v;
    friend class FloatBounds<PR>;
  public:
    Float(double x, PR pr) : _v(x,pr) { }
    Float<PR> raw() const { return _v; }
    PR precision() const { return _v.precision(); }
    friend OutputStream& operator<<(OutputStream& os, Float<PR> const& x) { return os << 'V' << x.raw(); }
};

template<class PR> class FloatBounds
    : ProvideRingOperations<FloatBounds<PR>>
    , ProvideConcreteGenericGroupOperations<FloatBounds<PR>,ValidatedNumber>
    , ProvideConcreteGenericGroupOperations<FloatBounds<PR>>
{
    Float<PR> _l, _u;
    friend class FloatUpperBound<PR>; friend class FloatLowerBound<PR>;
  public:
    explicit FloatBounds(double x, PR pr) : FloatBounds(x,x,pr) { }
    explicit FloatBounds(double xl, double xu, PR pr) : _l(xl,pr), _u(xu,pr) { }
    explicit FloatBounds(Float<PR> l, Float<PR> u) : _l(l), _u(u) { }
    FloatBounds(Float<PR> const& x) : _l(x._v), _u(x._v) { }
    PR precision() const { return max(_l.precision(),_u.precision()); }
    Float<PR> upper_raw() const { return _u; }
    Float<PR> lower_raw() const { return _l; }
};

template<class PR> class FloatUpperBound
    : ProvideGroupOperations<FloatUpperBound<PR>,FloatLowerBound<PR>>
    , ProvideConcreteGenericGroupOperations<FloatUpperBound<PR>>
{
    Float<PR> _u;
  public:
    explicit FloatUpperBound(double d, PR pr) : _u(d,pr) { }
    explicit FloatUpperBound(Float<PR> const& u) : _u(u) { }
    FloatUpperBound(Float<PR> const& x) : FloatUpperBound(x.raw()) { }
    FloatUpperBound(FloatBounds<PR> const& x) : FloatUpperBound(x._u) { }
    Float<PR> raw() const { return _u; }
    PR precision() const { return _u.precision(); }
    friend OutputStream& operator<<(OutputStream& os, FloatUpperBound<PR> const& x) { return os << 'U' << x.raw(); }
};

template<class PR> class FloatLowerBound
    : ProvideGroupOperations<FloatLowerBound<PR>,FloatUpperBound<PR>>
{
    Float<PR> _l;
  public:
    explicit FloatLowerBound(double d, PR pr) : _l(d,pr) { }
    explicit FloatLowerBound(Float<PR> const& l) : _l(l) { }
    FloatLowerBound(FloatBounds<PR> const& x) : _l(x._l) { }
    Float<PR> raw() const { return _l; }
    PR precision() const { return PR(); }
};

template<class PR> class FloatApproximation
    : ProvideRingOperations<FloatApproximation<PR>>
    , ProvideConcreteGenericGroupOperations<FloatApproximation<PR>,ApproximateNumber>
    , ProvideConcreteGenericGroupOperations<FloatApproximation<PR>>
{
    Float<PR> _a;
  public:
    explicit FloatApproximation(double d, PR pr) : _a(d,pr) { }
    explicit FloatApproximation(Float<PR> const& a) : _a(a) { }
    FloatApproximation(FloatBounds<PR> const& x);
    Float<PR> raw() const { return _a; }
    PR precision() const { return PR(); }
};


template<class Y> class Positive;

template<class PR> class Positive<FloatUpperBound<PR>>
    : public FloatUpperBound<PR>
    , ProvideSemiringOperations<Positive<FloatUpperBound<PR>>>
    , ProvideConcreteGenericGroupOperations<Positive<FloatUpperBound<PR>>>
{
  public:
    explicit Positive<FloatUpperBound<PR>>(Float<PR> const& x) : FloatUpperBound<PR>(x) { }
    explicit Positive<FloatUpperBound<PR>>(FloatUpperBound<PR> const& x) : FloatUpperBound<PR>(x) { }
};
template<class PR> using PositiveFloatUpperBound = Positive<FloatUpperBound<PR>>;


template<class PR> class FloatError
    : ProvideGroupOperations<FloatUpperBound<PR>,FloatLowerBound<PR>>
    , ProvideSemiringOperations<Positive<FloatUpperBound<PR>>>
    , ProvideConcreteGenericGroupOperations<FloatUpperBound<PR>>
    , ProvideConcreteGenericGroupOperations<Positive<FloatUpperBound<PR>>>
{
    Float<PR> _e;
  public:
    explicit FloatError(double d, PR pr) : _e(d,pr) { }
    explicit FloatError(Float<PR> const& x) : _e(x) { }
    Float<PR> raw() const { return _e; }
    PR precision() const { return _e.precision(); }
    FloatError(PositiveFloatUpperBound<PR> const& x) : _e(x.raw()) { }
    operator PositiveFloatUpperBound<PR> () const { return PositiveFloatUpperBound<PR>(_e); }
    friend OutputStream& operator<<(OutputStream& os, FloatError<PR> const& x) { return os << 'E' << x.raw(); }
};

template<class PR> class FloatFactory {
    PR _pr;
  public:
    FloatFactory(PR pr) : _pr(pr) { }
    FloatApproximation<PR> create(Number<ApproximateTag> y) { return FloatApproximation<PR>(y.get_d(),this->_pr); }
    FloatBounds<PR> create(Number<ValidatedTag> y) { return FloatBounds<PR>(y.get_d(),this->_pr); }
    FloatError<PR> create(unsigned int m) { return FloatError<PR>(m,this->_pr); }
    Float<PR> create(int n) { return Float<PR>(n,this->_pr); }
};
template<class PR> FloatFactory<PR> factory(Float<PR> const& x) { return FloatFactory<PR>(x.precision()); }
template<class PR> FloatFactory<PR> factory(FloatBounds<PR> const& x) { return FloatFactory<PR>(x.precision()); }
template<class PR> FloatFactory<PR> factory(FloatUpperBound<PR> const& x) { return FloatFactory<PR>(x.precision()); }
template<class PR> FloatFactory<PR> factory(FloatApproximation<PR> const& x) { return FloatFactory<PR>(x.precision()); }
template<class PR> FloatFactory<PR> factory(FloatError<PR> const& x) { return FloatFactory<PR>(x.precision()); }

template<class X> struct IsConcreteNumber : False { };
template<class PR> struct IsConcreteNumber<FloatApproximation<PR>> : True { };
template<class PR> struct IsConcreteNumber<FloatError<PR>> : True { };
template<class X, class Y> struct AreConcreteGenericNumbers : False { };
template<class X, class P> struct AreConcreteGenericNumbers<X,Number<P>> : IsConcreteNumber<X> { };

class DoublePrecision { friend DoublePrecision max(DoublePrecision dp1,DoublePrecision dp2) { return dp1; } };
using DP=DoublePrecision;

/*
template<class X, class Y, EnableIf<AreConcreteGenericNumbers<X,Y>> =dummy>
decltype(auto) add(Y const& y, X const& x) { return add(factory(x).create(y),x); }
template<class X, class Y, EnableIf<AreConcreteGenericNumbers<X,Y>> =dummy>
decltype(auto) add(X const& x, Y const& y) { return add(x,factory(x).create(y)); }
*/
