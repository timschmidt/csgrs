/***************************************************************************
 *            numeric/operators.hpp
 *
 *  Copyright  2008-20  Pieter Collins
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

/*! \file numeric/operators.hpp
 *  \brief Numerical operator classes
 */

#ifndef ARIADNE_OPERATORS_HPP
#define ARIADNE_OPERATORS_HPP

#include <iostream>
#include <cassert>

#include "foundations/logical.decl.hpp"
#include "number.decl.hpp"

#include "utility/variant.hpp"
#include "utility/variant.inl.hpp"

namespace Ariadne {

typedef void Void;
typedef bool Bool;
typedef unsigned int Nat;
typedef int Int;

typedef std::ostream OutputStream;

template<class X> struct Logic;
template<> struct Logic<String> { typedef Boolean Type; };
template<> struct Logic<Integer> { typedef Boolean Type; };
template<> struct Logic<Real> { typedef Kleenean Type; };
template<class X> using LogicType = typename Logic<X>::Type;

template<class X1, class X2> using RingType = DifferenceType<ProductType<X1,X2>,ProductType<X2,X1>>;
template<class X1, class X2> using FieldType = DifferenceType<QuotientType<X1,X2>,QuotientType<X2,X1>>;
template<class X> using TranscendentalType = decltype(sin(declval<X>()));


class Operator {
  public:
    enum class Code : ComparableEnumerationType;
    enum class Kind : ComparableEnumerationType;
    friend Kind get_kind(Code);
  private:
    Code _code;
    Kind _kind;
  public:
    Operator(Code c, Kind k) : _code(c), _kind(k) { }
    Operator(Code c) : _code(c), _kind(get_kind(_code)) { }
    template<class OP> Operator(const OP& op) : _code(op.code()), _kind(op.kind()) { }

    operator Code () const { return _code; }
    Code code() const { return _code; }
    Kind kind() const { return _kind; }
};

using OperatorCode=Operator::Code;
using OperatorKind=Operator::Kind;



enum class Operator::Kind : ComparableEnumerationType {
    VARIABLE,
    COORDINATE,
    NULLARY,
    UNARY,
    BINARY,
    TERNARY,
    GRADED,
    SCALAR,
    PREDICATE,
    COMPARISON
};

enum class Operator::Code : ComparableEnumerationType {
    CNST,  // A constant value
    VAR,   // A named variable
    IND,   // A numbered index
    ADD,   // Addition
    SUB,   // Subtraction
    MUL,   // Multiplication
    DIV,   // Division
    FMA,   // Fused multiply-and-add
    POW,   // Integer power
    ROOT,   // Integer root
    NUL,   // Zero
    POS,   // Unary plus
    NEG,   // Unary negation
    HLF,   // Halve
    REC,   // Reciprocal
    SQR,   // Square
    SQRT,  // Square root
    EXP,   // Natural exponential
    LOG,   // Natural logarithm
    SIN,   // Sine
    COS,   // Cosine
    TAN,   // Tangent
    ASIN,   // ArcSine
    ACOS,   // ArcCosine
    ATAN,   // ArcTangent
    SADD,  // Addition of a scalar constant
    SSUB,   // Scalar Subtraction
    SMUL,  // Multiplication by a scalar constant
    SDIV,   // Scalar Division
    SFMA,   // Fused multiply-and-add with scalar constants
    ABS,   // Absolute value
    MAX,   // Maximum
    MIN,   // Minimum
    NOT,   // Logical not
    AND,   // Logical and
    OR,    // Logical or
    XOR,   // Logical exclusive or
    IMPL,  // Logical implication
    ITOR,   // Conversion of Int to Real
    GET,   // Subscript operator
    VEC,   // Vectorisation operator
    PUSH,
    PULL,
    EQ=-6,    // Equal
    NEQ=-5,   // Not equal
    GEQ=-4,   // Greater or equal
    LEQ=-3,   // Less or equal
    GT=-2,    // Greater than
    LT=-1,    // Less than
    SGN=-7,   // Compare with zero
    SUBS=-8,   // Compare as a subset
    DISJ=-9    // Compare disjointness
};

OutputStream& operator<<(OutputStream& os, const OperatorKind& knd);
OutputStream& operator<<(OutputStream&, const OperatorCode& op);
const char* name(const OperatorCode& op);
const char* symbol(const OperatorCode& op);


template<class OBJ> struct Object { OBJ const& upcast() const { return static_cast<OBJ const&>(*this); } };
template<class OP> struct OperatorObject : Object<OP> { };
template<class CMP> struct ComparisonObject : Object<CMP> { };

template<class OP> inline OutputStream& operator<<(OutputStream& os, OperatorObject<OP> const& op) {
    return os << op.upcast().code(); }

template<class CMP> inline OutputStream& operator<<(OutputStream& os, ComparisonObject<CMP> const& cmp) {
    return os << cmp.upcast().code(); }

struct GtrZero {}; struct LessZero {};

struct Gtr : ComparisonObject<Gtr> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1> a2) { return a1 >  a2; }
    static constexpr OperatorCode code() { return OperatorCode::GT; } OperatorKind kind() { return OperatorKind::COMPARISON; }
};

struct Less : ComparisonObject<Less> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1< a2) { return a1 <  a2; }
    static constexpr OperatorCode code() { return OperatorCode::LT; } static constexpr OperatorKind kind() { return OperatorKind::COMPARISON; }
};

struct Geq : ComparisonObject<Geq> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1>=a2) { return a1 >= a2; }
    static constexpr OperatorCode code() { return OperatorCode::GEQ; } static constexpr OperatorKind kind() { return OperatorKind::COMPARISON; }
};

struct Leq : ComparisonObject<Leq> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1<=a2) { return a1 <= a2; }
    static constexpr OperatorCode code() { return OperatorCode::LEQ; } static constexpr OperatorKind kind() { return OperatorKind::COMPARISON; }
};

struct Equal : ComparisonObject<Equal> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1==a2) { return a1 == a2; }
    static constexpr OperatorCode code() { return OperatorCode::EQ; } static constexpr OperatorKind kind() { return OperatorKind::COMPARISON; }
};

struct Unequal : ComparisonObject<Unequal> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1!=a2) { return a1 != a2; }
    static constexpr OperatorCode code() { return OperatorCode::NEQ; } static constexpr OperatorKind kind() { return OperatorKind::COMPARISON; }
};

using Eq=Equal; using Ne=Unequal; using Lt=Less; using Gt=Gtr; using Le=Leq; using Ge=Geq;

struct XOrOp : OperatorObject<XOrOp> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1 xor a2) { return a1 xor a2; }
    static constexpr OperatorCode code() { return OperatorCode::XOR; } static constexpr OperatorKind kind() { return OperatorKind::BINARY; }
};
struct AndOp : OperatorObject<AndOp> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1 and a2) { return a1 and a2; }
    static constexpr OperatorCode code() { return OperatorCode::AND; } static constexpr OperatorKind kind() { return OperatorKind::BINARY; }
};
struct OrOp : OperatorObject<OrOp> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1 or a2) { return a1 or a2; }
    static constexpr OperatorCode code() { return OperatorCode::OR; } static constexpr OperatorKind kind() { return OperatorKind::BINARY; }
};
struct NotOp : OperatorObject<NotOp> {
    template<class A> auto operator()(A&& a) const -> decltype(not a) { return not a; }
    static constexpr OperatorCode code() { return OperatorCode::NOT; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
};

struct Cnst : OperatorObject<Cnst> {
    static constexpr OperatorCode code() { return OperatorCode::CNST; } static constexpr OperatorKind kind() { return OperatorKind::NULLARY; }
};

struct Ind : OperatorObject<Ind> {
    static constexpr OperatorCode code() { return OperatorCode::IND; } static constexpr OperatorKind kind() { return OperatorKind::COORDINATE; }
};

struct Var : OperatorObject<Var> {
    static constexpr OperatorCode code() { return OperatorCode::VAR; } static constexpr OperatorKind kind() { return OperatorKind::VARIABLE; }
};

struct Plus {
    template<class A> auto operator()(A&& a) const -> decltype(+a) { return +a; }
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1+a2) { return a1+a2; }
};
struct Minus {
    template<class A> auto operator()(A&& a) const -> decltype(-a) { return -a; }
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1-a2) { return a1-a2; }
};
struct Times {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1*a2) { return a1*a2; }
};
struct Divides {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(a1/a2) { return a1/a2; }
};

struct RAdd : OperatorObject<RAdd> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(add(a2,a1)) { return add(a2,a1); }
};
struct RSub : OperatorObject<RSub> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(sub(a2,a1)) { return sub(a2,a1); }
};
struct RMul : OperatorObject<RMul> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(mul(a2,a1)) { return mul(a2,a1); }
};
struct RDiv : OperatorObject<RDiv> {
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(div(a2,a1)) { return div(a2,a1); }
};

struct FirstArgumentTag { };
struct SecondArgumentTag { };
struct ThirdArgumentTag { };

struct Add : OperatorObject<Add> {
    static constexpr OperatorCode code() { return OperatorCode::ADD; } static constexpr OperatorKind kind() { return OperatorKind::BINARY; }
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(add(a1,a2)) { return add(a1,a2); }
    template<class X> X derivative(const X& a1, const X& a2, FirstArgumentTag) const { return a1.create(1); }
    template<class X> X derivative(const X& a1, const X& a2, SecondArgumentTag) const { return a2.create(1); }
    template<class X,class D> D derivative(const X& a1, const D& d1, const X& a2, const D& d2) const { return d1+d2; }
};
struct Sub : OperatorObject<Sub> {
    static constexpr OperatorCode code() { return OperatorCode::SUB; } static constexpr OperatorKind kind() { return OperatorKind::BINARY; }
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(sub(a1,a2)) { return sub(a1,a2); }
    template<class X> X derivative(const X& a1, const X& a2, FirstArgumentTag) const { return a1.create(+1); }
    template<class X> X derivative(const X& a1, const X& a2, SecondArgumentTag) const { return a2.create(-1); }
    template<class X,class D> D derivative(const X& a1, const D& d1, const X& a2, const D& d2) const { return d1-d2; }
};
struct Mul : OperatorObject<Mul> {
    static constexpr OperatorCode code() { return OperatorCode::MUL; } static constexpr OperatorKind kind() { return OperatorKind::BINARY; }
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(mul(a1,a2)) { return mul(a1,a2); }
    template<class X> X derivative(const X& a1, const X& a2, FirstArgumentTag) const { return a2; }
    template<class X> X derivative(const X& a1, const X& a2, SecondArgumentTag) const { return a1; }
    template<class X,class D> D derivative(const X& a1, const D& d1, const X& a2, const D& d2) const { return a2*d1+a1*d2; }
};
struct Div : OperatorObject<Div> {
    static constexpr OperatorCode code() { return OperatorCode::DIV; } static constexpr OperatorKind kind() { return OperatorKind::BINARY; }
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(div(a1,a2)) { return div(a1,a2); }
    template<class X> X derivative(const X& a1, const X& a2, FirstArgumentTag) const { return rec(a2); }
    template<class X> X derivative(const X& a1, const X& a2, SecondArgumentTag) const { return div(neg(a1),sqr(a2)); }
    template<class X,class D> D derivative(const X& a1, const D& d1, const X& a2, const D& d2) const { return (d1-d2*(a1/a2))/a2; }
};

struct Pow : OperatorObject<Pow> {
    static constexpr OperatorCode code() { return OperatorCode::POW; } static constexpr OperatorKind kind() { return OperatorKind::GRADED; }
    template<class A, class N> auto operator()(A&& a, N&& n) const -> decltype(pow(a,n)) { return pow(a,n); }
    template<class X, class N> X derivative(const X& a, N n) const { return n*pow(a,n-1); }
    template<class X,class D, class N> D derivative(const X& a, const D& d, N n) const { return derivative(a,n)*d; }
};
struct Root : OperatorObject<Root> {
    OperatorCode code() const { return OperatorCode::ROOT; } OperatorKind kind() const { return OperatorKind::GRADED; }
    template<class A, class N> friend auto root(A&& a, N&& n) -> decltype(exp(log(a)/n)) { return exp(log(a)/n); }
    template<class A, class N> auto operator()(A&& a, N&& n) const -> decltype(root(a,n)) { return root(a,n); }
};
struct Fma : OperatorObject<Fma> {
    static constexpr OperatorCode code() { return OperatorCode::FMA; } static constexpr OperatorKind kind() { return OperatorKind::TERNARY; }
    template<class A1, class A2, class A3> auto operator()(A1&& a1, A2&& a2, A3&& a3) const -> decltype(fma(a1,a2,a3)) { return fma(a1,a2,a3); }
    template<class X> X derivative(const X& a1, const X& a2, const X& a3, FirstArgumentTag) const { return a2; }
    template<class X> X derivative(const X& a1, const X& a2, const X& a3, SecondArgumentTag) const { return a1; }
    template<class X> X derivative(const X& a1, const X& a2, const X& a3, ThirdArgumentTag) const { return nul(a3)+1; }
    template<class X,class D> D derivative(const X& a1, const D& d1, const X& a2, const D& d2, const X& a3, const D& d3) const {
        return a2*d1+a1*d2+d3; }
};

struct Nul : OperatorObject<Nul> {
    static constexpr OperatorCode code() { return OperatorCode::NUL; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(nul(a)) { return nul(a); }
    template<class X> X derivative(const X& a) const { return nul(a); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return nul(d); }
};
struct Pos : OperatorObject<Pos> {
    static constexpr OperatorCode code() { return OperatorCode::POS; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(pos(a)) { return pos(a); }
    template<class X> X derivative(const X& a) const { return nul(a)+1; }
    template<class X,class D> D derivative(const X& a, const D& d) const { return d; }
};
struct Neg : OperatorObject<Neg> {
    static constexpr OperatorCode code() { return OperatorCode::NEG; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(neg(a)) { return neg(a); }
    template<class X> X derivative(const X& a) const { return nul(a)-1; }
    template<class X,class D> D derivative(const X& a, const D& d) const { return -d; }
};
struct Hlf : OperatorObject<Hlf> {
    static constexpr OperatorCode code() { return OperatorCode::HLF; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(hlf(a)) { return hlf(a); }
    template<class X> X derivative(const X& a) const { return hlf(a); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return hlf(d); }
};
struct Rec : OperatorObject<Rec> {
    static constexpr OperatorCode code() { return OperatorCode::REC; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(rec(a)) { return rec(a); }
    template<class X> X derivative(const X& a) const { return rec(neg(sqr(a))); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return d/(neg(sqr(a))); }
};
struct Sqr : OperatorObject<Sqr> {
    static constexpr OperatorCode code() { return OperatorCode::SQR; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(sqr(a)) { return sqr(a); }
    template<class X> X derivative(const X& a) const { return 2*a; }
    template<class X,class D> D derivative(const X& a, const D& d) const { return (2*a)*d; }
};
struct Sqrt : OperatorObject<Sqrt> {
    static constexpr OperatorCode code() { return OperatorCode::SQRT; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(sqrt(a)) { return sqrt(a); }
    template<class X> X derivative(const X& a) const { return rec(2*sqrt(a)); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return d/(2*sqrt(a)); }
};
struct Exp : OperatorObject<Exp> {
    static constexpr OperatorCode code() { return OperatorCode::EXP; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(exp(a)) { return exp(a); }
    template<class X> X derivative(const X& a) const { return exp(a); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return exp(a)*d; }
};
struct Log : OperatorObject<Log> {
    static constexpr OperatorCode code() { return OperatorCode::LOG; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(log(a)) { return log(a); }
    template<class X> X derivative(const X& a) const { return rec(a); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return d/a; }
};
struct Sin : OperatorObject<Sin> {
    static constexpr OperatorCode code() { return OperatorCode::SIN; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(sin(a)) { return sin(a); }
    template<class X> X derivative(const X& a) const { return cos(a); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return derivative(a)*d; }
};
struct Cos : OperatorObject<Cos> {
    static constexpr OperatorCode code() { return OperatorCode::COS; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(cos(a)) { return cos(a); }
    template<class X> X derivative(const X& a) const { return neg(sin(a)); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return derivative(a)*d; }
};
struct Tan : OperatorObject<Tan> {
    static constexpr OperatorCode code() { return OperatorCode::TAN; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(tan(a)) { return tan(a); }
    template<class X> X derivative(const X& a) const { return rec(sqr(cos(a))); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return derivative(a)*d; }
};
struct Asin : OperatorObject<Asin> {
    static constexpr OperatorCode code() { return OperatorCode::ASIN; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(asin(a)) { return asin(a); }
    template<class X> X derivative(const X& a) const { return rec(1-sqrt(a)); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return derivative(a)*d; }
};
struct Acos : OperatorObject<Acos> {
    static constexpr OperatorCode code() { return OperatorCode::ACOS; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(acos(a)) { return acos(a); }
    template<class X> X derivative(const X& a) const { return rec(1-sqrt(a)); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return derivative(a)*d; }
};
struct Atan : OperatorObject<Atan> {
    static constexpr OperatorCode code() { return OperatorCode::ATAN; } static constexpr OperatorKind kind() { return OperatorKind::UNARY; }
    template<class A> auto operator()(A&& a) const -> decltype(atan(a)) { return atan(a); }
    template<class X> X derivative(const X& a) const { return rec(1+sqr(a)); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return derivative(a)*d; }
};

struct Max : OperatorObject<Max> {
    static constexpr OperatorCode code() { return OperatorCode::MAX; } static constexpr OperatorKind kind() { return OperatorKind::BINARY; }
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(max(a1,a2)) { return max(a1,a2); }
    template<class X,class D> D derivative(const X& a1, const D& d1, const X& a2, const D& d2) const { return a1>=a2 ? d1 : d2; }
};

struct Min : OperatorObject<Min> {
    static constexpr OperatorCode code() { return OperatorCode::MIN; } static constexpr OperatorKind kind() { return OperatorKind::BINARY; }
    template<class A1, class A2> auto operator()(A1&& a1, A2&& a2) const -> decltype(min(a1,a2)) { return min(a1,a2); }
    template<class X,class D> D derivative(const X& a1, const D& d1, const X& a2, const D& d2) const { return a1<=a2 ? d1 : d2; }
};

struct Abs : OperatorObject<Abs> {
    static constexpr OperatorCode code() { return OperatorCode::ABS; } static constexpr OperatorKind kind() { return OperatorKind::BINARY; }
    template<class A> auto operator()(A&& a) const -> decltype(abs(a)) { return abs(a); }
    template<class X,class D> D derivative(const X& a, const D& d) const { return a>=0 ? a : -a; }
};

struct Sgn : ComparisonObject<Sgn> {
    static constexpr OperatorCode code() { return OperatorCode::SGN; }
    static OperatorKind kind() { return OperatorKind::PREDICATE; }
    template<class A> auto operator()(A&& a) const -> decltype(sgn(a)) { return sgn(a); }
    Kleenean operator()(const Real& a) const;
};

/*
template<class OP> struct CodedOperator {
    typedef OP CodeType;
    operator Operator() const { return Operator(static_cast<Operator::Code>(_code)); }
    CodeType code() const { return _code; }
  protected:
    CodeType _code;
};
struct SpecialOperator : CodedOperator<SpecialOperator::Code> {
    enum class Code : char;
};
*/

template<class... OPS> class OperatorVariant
    : public CodedVariant<OperatorCode, OPS...>
{
  public:
    template<class OP> requires OneOf<OP,OPS...> OperatorVariant(OP op) : CodedVariant<OperatorCode,OPS...>(op) { }
    explicit OperatorVariant(OperatorCode code) : CodedVariant<OperatorCode,OPS...>(code) { }
    OperatorKind kind() const {
        return this->accept([](auto op){return op.kind();}); }
    template<class... AS> decltype(auto) operator()(AS&& ... as) const {
        return this->accept([&as...](auto op){return op(std::forward<AS>(as)...);}); }
    template<class... AS> decltype(auto) call(AS&& ... as) const {
        return this->accept([&as...](auto op){return op(std::forward<AS>(as)...);}); }
    template<class R, class... AS> R call_as(AS&& ... as) const {
        return this->accept([&as...](auto op){return static_cast<R>(op(std::forward<AS>(as)...));}); }
    template<class... AS> friend decltype(auto) evaluate(OperatorVariant<OPS...>const& ops, AS&& ... as) {
        return ops.call(as...); }
    template<class R, class... AS> friend R evaluate_as(OperatorVariant<OPS...>const& ops, AS&& ... as) {
        return ops.call_as(as...); }
    friend OutputStream& operator<<(OutputStream& os, OperatorVariant<OPS...> const& op) {
        op.accept([&os](auto op_){os << op_;}); return os; }
};



struct UnaryLogicalOperator : OperatorVariant<NotOp> { using OperatorVariant::OperatorVariant; };
struct UnaryComparisonOperator : OperatorVariant<Sgn> { using OperatorVariant::OperatorVariant; };
struct UnaryRingOperator : OperatorVariant<Neg> { using OperatorVariant::OperatorVariant; };
struct UnaryArithmeticOperator : OperatorVariant<Nul,Pos,Neg,Sqr,Rec> { using OperatorVariant::OperatorVariant; };
struct UnaryTranscendentalOperator : OperatorVariant<Pos,Neg,Sqr,Hlf,Rec,Sqrt,Exp,Log,Sin,Cos,Tan,Atan> { using OperatorVariant::OperatorVariant; };
struct UnaryElementaryOperator : OperatorVariant<Nul,Pos,Neg,Sqr,Hlf,Rec,Sqrt,Exp,Log,Sin,Cos,Tan,Asin,Acos,Atan,Abs> {
    using OperatorVariant::OperatorVariant;
    template<class X> decltype(sin(declval<X>())) operator()(X&& x) const { typedef decltype(sin(x)) R;
        return this->accept([&x](auto op){return static_cast<R>(op(std::forward<X>(x)));}); } };
struct UnaryLatticeOperator : OperatorVariant<Abs> { using OperatorVariant::OperatorVariant; };
struct BinaryLogicalOperator : OperatorVariant<AndOp,OrOp> { using OperatorVariant::OperatorVariant; };
struct BinaryComparisonOperator : OperatorVariant<Equal,Unequal,Less,Gtr,Leq,Geq> { using OperatorVariant::OperatorVariant; };
struct BinaryLatticeOperator : OperatorVariant<Max,Min> { using OperatorVariant::OperatorVariant; };
struct BinaryRingOperator : OperatorVariant<Add,Sub,Mul> { using OperatorVariant::OperatorVariant;
    template<class X1,class X2> ArithmeticType<X1,X2> operator()(X1&& x1, X2&& x2) const {
        return this->accept([&x1,&x2](auto op){return static_cast<ArithmeticType<X1,X2>>(op(std::forward<X1>(x1),std::forward<X2>(x2)));}); } };
struct BinaryFieldOperator : OperatorVariant<Add,Sub,Mul,Div> { using OperatorVariant::OperatorVariant;
    template<class X1,class X2> QuotientType<X1,X2> operator()(X1&& x1, X2&& x2) const {
        return this->accept([&x1,&x2](auto op){return static_cast<QuotientType<X1,X2>>(op(std::forward<X1>(x1),std::forward<X2>(x2)));}); } };
using BinaryArithmeticOperator = BinaryFieldOperator;
struct BinaryElementaryOperator : OperatorVariant<Add,Sub,Mul,Div,Max,Min> { using OperatorVariant::OperatorVariant;
    template<class X1,class X2> QuotientType<X1,X2> operator()(X1&& x1, X2&& x2) const {
        return this->accept([&x1,&x2](auto op){return static_cast<QuotientType<X1,X2>>(op(std::forward<X1>(x1),std::forward<X2>(x2)));}); } };
struct GradedRingOperator : OperatorVariant<Pow> { using OperatorVariant::OperatorVariant;
    template<class X,class N> decltype(pow(declval<X>(),declval<N>())) operator()(X&& x, N const& n) const {
        return this->accept([&x,&n](auto op){return static_cast<decltype(pow(declval<X>(),declval<N>()))>(op(std::forward<X>(x),n));}); } };
struct GradedFieldOperator : OperatorVariant<Pow> { using OperatorVariant::OperatorVariant;
    template<class X,class N> decltype(pow(declval<X>(),declval<N>())) operator()(X&& x, N const& n) const {
        return this->accept([&x,&n](auto op){return static_cast<decltype(pow(declval<X>(),declval<N>()))>(op(std::forward<X>(x),n));}); } };
struct GradedElementaryOperator : OperatorVariant<Pow> { using OperatorVariant::OperatorVariant;
    template<class X,class N> decltype(pow(declval<X>(),declval<N>())) operator()(X&& x, N const& n) const {
        return this->accept([&x,&n](auto op){return static_cast<decltype(pow(declval<X>(),declval<N>()))>(op(std::forward<X>(x),n));}); } };
struct TernaryArithmeticOperator : OperatorVariant<Fma> { using OperatorVariant::OperatorVariant; };



constexpr Pos inverse(Pos) { return Pos(); }
constexpr Neg inverse(Neg) { return Neg(); }
constexpr Rec inverse(Rec) { return Rec(); }
constexpr Sqrt inverse(Sqr) { return Sqrt(); }
constexpr Sqr inverse(Sqrt) { return Sqr(); }
constexpr Log inverse(Exp) { return Log(); }
constexpr Exp inverse(Log) { return Exp(); }
constexpr Atan inverse(Tan) { return Atan(); }
constexpr Tan inverse(Atan) { return Tan(); }

template<class OP> using InverseType = decltype(inverse(declval<OP>()));

template<class OP> concept HasInverse = requires(OP op) { inverse(op); };

namespace {
template<class OP, class... OPS> Bool _are_inverses(OP const& op1, OperatorVariant<OPS...> const& ops2) {
    if constexpr(HasInverse<decltype(op1)>) { return inverse(op1).code() == ops2.code(); }
    else { return false; }
}
} // namespace

template<class... OPS> Bool are_inverses(OperatorVariant<OPS...> const& ops1, OperatorVariant<OPS...> const& ops2) {
    return ops1.accept([&ops2](auto op1){ return _are_inverses(op1,ops2); });
}

struct SpecialOperator {
    enum class Code : char { CNST=(char)OperatorCode::CNST, VAR, IND };
    operator Operator() const { return Operator(static_cast<Operator::Code>(_code)); }
    Code code() const { return _code; }
  private:
    Code _code;
};
class UnaryOperator {
    OperatorCode _op;
  public:
    template<class OP> UnaryOperator(OP op) : _op(op) { }
    template<class X> X operator() (X const& x) { return compute(_op,x); }
    template<class X> X derivative(X const& x) { return derivative(_op,x); }
};


inline OutputStream& operator<<(OutputStream& os, const Less& v) { return os << "<"; }
inline OutputStream& operator<<(OutputStream& os, const Gtr& v) { return os << ">"; }
inline OutputStream& operator<<(OutputStream& os, const Leq& v) { return os << "<="; }
inline OutputStream& operator<<(OutputStream& os, const Geq& v) { return os << ">="; }
inline OutputStream& operator<<(OutputStream& os, const Equal& v) { return os << "=="; }
inline OutputStream& operator<<(OutputStream& os, const Unequal& v) { return os << "!="; }

inline OutputStream& operator<<(OutputStream& os, const AndOp& v) { return os << "&&"; }
inline OutputStream& operator<<(OutputStream& os, const OrOp& v) { return os << "||"; }
inline OutputStream& operator<<(OutputStream& os, const NotOp& v) { return os << "!"; }

inline OutputStream& operator<<(OutputStream& os, const Plus& v) { return os << "+"; }
inline OutputStream& operator<<(OutputStream& os, const Minus& v) { return os << "-"; }
inline OutputStream& operator<<(OutputStream& os, const Times& v) { return os << "*"; }
inline OutputStream& operator<<(OutputStream& os, const Divides& v) { return os << "/"; }


} // namespace Ariadne

#endif // ARIADNE_OPERATORS_HPP
