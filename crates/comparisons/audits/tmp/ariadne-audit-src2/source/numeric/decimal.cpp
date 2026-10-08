/***************************************************************************
 *            numeric/decimal.cpp
 *
 *  Copyright  2014-20  Pieter Collins
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

#include "utility/standard.hpp"
#include "config.hpp"

#include "utility/macros.hpp"
#include "numeric/integer.hpp"
#include "numeric/dyadic.hpp"
#include "numeric/decimal.hpp"
#include "numeric/rational.hpp"
#include "numeric/floats.hpp"

namespace Ariadne {

const Integer Decimal::_ten = Integer(10);

namespace {
static const Integer& ten=Decimal::_ten;
}

void Decimal::canonicalize()
{
    while(this->_q>0u && rem(this->_p,ten)==0) {
        this->_p = quot(this->_p,ten);
        --this->_q;
    }
}


Decimal::Decimal() : _p(0), _q(0u)
{
}

Decimal::Decimal(Integer p, Nat q) : _p(p), _q(q)
{
}

Decimal::Decimal(Dyadic const& w)
{
    assert(w.exponent()>=0);
    this->_p=w.mantissa();
    this->_q=static_cast<Nat>(w.exponent());
    Integer five(5);
    this->_p *= pow(five,this->_q);
    this->canonicalize();
}

Decimal operator"" _decimal(long double x)
{
    return Decimal(static_cast<double>(x));
}

Decimal operator"" _dec(long double x)
{
    return operator"" _decimal(x);
}

Decimal operator"" _decimal(unsigned long long int n)
{
    return Decimal(Integer(n));
}

Decimal operator"" _dec(unsigned long long int n)
{
    return operator"" _decimal(n);
}

Decimal operator"" _decimal(const char* s, std::size_t)
{
    return Decimal(String(s));
}

Decimal operator"" _dec(const char* s, std::size_t n)
{
    return operator"" _decimal(s,n);
}

Decimal operator+(Decimal const& d)
{
    return Decimal(d._p,d._q);
}

Decimal operator-(Decimal const& d)
{
    return Decimal(-d._p,d._q);
}

Decimal operator+(Decimal const& d1, Decimal const& d2)
{
    Integer q1=pow(ten,d1._q);
    Integer q2=pow(ten,d2._q);
    Decimal r(d1._p*q2+d2._p*q1,d1._q+d2._q);
    r.canonicalize();
    return r;
}

Decimal operator-(Decimal const& d1, Decimal const& d2)
{
    Integer q1=pow(ten,d1._q);
    Integer q2=pow(ten,d2._q);
    Decimal r(d1._p*q2-d2._p*q1,d1._q+d2._q);
    r.canonicalize();
    return r;
}

Decimal operator*(Decimal const& d1, Decimal const& d2)
{
    return Decimal(d1._p*d2._p,d1._q+d2._q);
}

Decimal& operator+=(Decimal& d1, Decimal const& d2)
{
    return d1=d1+d2;
}

Decimal nul(Decimal const& d)
{
    return Decimal(0,0u);
}

Decimal sqr(Decimal const& d)
{
    return Decimal(sqr(d._p),2u*d._q);
}

Decimal hlf(Decimal const& d)
{
    if (rem(d._p,2)==0) {
        return Decimal(quot(d._p,2),d._q);
    } else {
        return Decimal(5*d._p,d._q+1u);
    }
}

Decimal abs(Decimal const& d)
{
    return Decimal(abs(d._p),d._q);
}

Decimal max(Decimal const& d1, Decimal const& d2)
{
    return std::max(d1,d2);
}

Decimal min(Decimal const& d1, Decimal const& d2)
{
    return std::min(d1,d2);
}

Rational operator/(Decimal const& d1, Decimal const& d2)
{
    Rational q(d1._p,d2._p);
    int e=int(d1._q)-int(d2._q);
    if(e>0) { return q/pow(ten,e); }
    else { return q*pow(ten,-e); }
}

Comparison cmp(Integer const& z1, Integer const& z2);
Boolean eq(Integer const& d1, Integer const& d2);
Boolean lt(Integer const& d1, Integer const& d2);

Comparison cmp(Decimal const& d1, Decimal const& d2)
{
    return cmp(d1._p*pow(ten,d2._q),d2._p*pow(ten,d1._q));
}

Boolean eq(Decimal const& d1, Decimal const& d2)
{
    return eq(d1._p*pow(ten,d2._q),d2._p*pow(ten,d1._q));
}

Boolean lt(Decimal const& d1, Decimal const& d2)
{
    return lt(d1._p*pow(ten,d2._q),d2._p*pow(ten,d1._q));
}

Decimal::Decimal(double x)
{
    static const Nat sf=9; // The number of significant figures allowed
    static const double acc=1/std::pow(10.0,sf);  // The accuracy at the number of significant figures
    static const double tol=1e-15; // The tolerance of the result

    if(x==0) { *this=Decimal(0,0u); return; }

    Int sgn = x>0 ? +1 : -1;
    double y = sgn*x;

    Int exp=0;
    while(y<1.0) { y*=10; exp-=1; }
    while(y>=1.0) { y/=10; exp+=1; }
    // Now 0.1<=y<1.0; and |x| = y*10^exp
    long int n=std::round(y/acc); // An approximation of y*10^sf
    exp-=static_cast<Int>(sf);
    double re=std::fabs(y-n*acc); // The error of n/10^sf

    if(std::fabs(re)>=tol) {
        ARIADNE_THROW(std::runtime_error,"Decimal(double)","double-precision floating-point number must have a relative error of "<<tol<<" with respect to its approximation to "<<sf<<" significant figures; number "<<std::setprecision(17)<<x<<" has a relative error of "<<re);
    }

    this->_p=sgn*n;
    if(exp>0) {
        this->_p *= pow(ten,Nat(exp));
        this->_q=0u;
    } else {
        this->_q=static_cast<Nat>(-exp);
    }

    this->canonicalize();
}

Decimal::Decimal(String const& str)
{
    // Parse string to ensure correctness
    Bool found_decimal_point=false;
    Nat trailing_zeros=0u;
    const char* c_ptr=str.c_str();
    const char& ch=*c_ptr;

    Int s = +1;
    if(ch=='-') {
        s = -1; ++c_ptr;
    } else if (ch=='+') {
        ++c_ptr;
    }

    this->_p=0;
    this->_q=0u;
    while(*c_ptr != 0) {
        const char& c=*c_ptr;
        if(c=='.') {
            if(found_decimal_point) {
                ARIADNE_THROW(std::runtime_error,"Decimal(String)","real literal \""<<str<<"\" has more than one decimal point.");
            } else {
                found_decimal_point=true;
            }
        } else if(c<'0' || c>'9') {
            ARIADNE_THROW(std::runtime_error,"Decimal(String)","invalid symbol '"<<c<<"' in string literal \""<<str<<"\"");
        } else {
            Int d=(c-'0');
            assert(0<=d && d<=9);

            if(found_decimal_point) {
                if(d==0) {
                    ++trailing_zeros;
                } else {
                    if(trailing_zeros!=0) {
                        this->_p *= pow(Integer(10),trailing_zeros);
                        this->_q += trailing_zeros;
                        trailing_zeros = 0;
                    }
                    this->_p *= 10;
                    this->_p += d;
                    ++this->_q;
                }
            } else {
                this->_p *= 10;
                this->_p += d;
            }
        }
        ++c_ptr;
    }
    this->_p *= s;

}

String Decimal::literal() const {
    StringStream ss;
    ss << *this;
    return ss.str();
}

template<> String class_name<Decimal>() { return "Decimal"; }
template<> String class_name<DecimalBounds>() { return "DecimalBounds"; }

OutputStream& operator<<(OutputStream& os, Decimal const& d) {
    Integer p=abs(d._p);
    Integer q=pow(ten,d._q);
    Integer n = quot(p,q);
    Integer r = p-n*q;
    if(d._p<0) { os << '-'; }
    os << n << ".";
    if (r!=0) {
        // Pad zeros after point
        while(r*ten<q) { q=quot(q,ten); os << "0"; }
        os << r;
    }
    return os;
}

Decimal::operator Rational() const {
    Integer const& num=this->_p;
    Integer den=pow(ten,this->_q);
    return Rational(num,den);
}

Decimal::operator ExactNumber() const {
    return Rational(*this).operator ExactNumber();
}

} // namespace Ariadne

