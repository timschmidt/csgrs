/***************************************************************************
 *            numeric/float_ball.cpp
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

#include "float_ball.hpp"
#include "float_ball.tpl.hpp"

#include "float_bounds.tpl.hpp" //< FIXME: Needed for log10floor

namespace Ariadne {

template<> OutputStream& Operations<FloatBall<DoublePrecision>>::_write(OutputStream& os, FloatBall<DoublePrecision> const& x);
template<> OutputStream& Operations<FloatBall<MultiplePrecision,DoublePrecision>>::_write(OutputStream& os, FloatBall<MultiplePrecision,DoublePrecision> const& x);
template<> OutputStream& Operations<FloatBall<MultiplePrecision>>::_write(OutputStream& os, FloatBall<MultiplePrecision> const& x);


template class Ball<FloatDP,FloatDP>;
template class Operations<Ball<FloatDP,FloatDP>>;
template class Ball<FloatMP,FloatDP>;
template class Operations<Ball<FloatMP,FloatDP>>;
template class Ball<FloatMP,FloatMP>;
template class Operations<Ball<FloatMP,FloatMP>>;

template<> String class_name<Ball<FloatDP>>() { return "FloatDPBall"; }
template<> String class_name<Ball<FloatMP,FloatDP>>() { return "FloatMDPBall"; }
template<> String class_name<Ball<FloatMP>>() { return "FloatMPBall"; }

Ball<FloatDP> FloatDP::pm(Error<FloatDP> const& e) const { return Ball<FloatDP>(*this,e); }
Ball<FloatMP> FloatMP::pm(Error<FloatMP> const& e) const { return Ball<FloatMP>(*this,e); }
Ball<FloatMP,FloatDP> FloatMP::pm(Error<FloatDP> const& e) const { return Ball<FloatMP,FloatDP>(*this,e); }


template<> OutputStream& Operations<FloatBall<MultiplePrecision>>::_write(OutputStream& os, FloatBall<MultiplePrecision> const& x) {
    // Write based on number of correct digits
    static const double log2ten = 3.3219280948873621817;
    static const char pmstr[] = "\u00b1";
    static const char hlfstr[] = "\u00bd";
    FloatMP const& v=x.value_raw();
    FloatMP const& e=x.error_raw();
    double edbl=e.get_d();
    // Compute the number of decimal places to be displayed
    Nat errplc = static_cast<Nat>(FloatError<MultiplePrecision>::output_places);
    Nat log10err = static_cast<Nat>(abslog10floor(edbl));
    Nat dgtserr = errplc-(log10err+1);
    Nat dgtsval = (x.value()==0) ? dgtserr : std::floor((x.value().precision()+1-x.value().exponent())/log2ten);
    Nat dgts = std::max(std::min(dgtsval,dgtserr),errplc);
    if(edbl==0.0) { dgts = dgtsval; }

    DecimalPlaces plcs{dgts};

    // Get string version of mpfr values
    String vstr=print(v,plcs,MPFR_RNDN);
    String estr=print(e,plcs,MPFR_RNDU);

    // Find position of first significan digit of error
    auto vcstr=vstr.c_str(); auto ecstr=estr.c_str();
    size_t cpl=0;
    if(edbl==0.0) {
        cpl=std::strlen(vcstr);
    } else if(edbl<1.0) {
        const char* vptr = std::strchr(vcstr,'.');
        const char* eptr = std::strchr(ecstr,'.');
        ++vptr; ++eptr;
        while((*eptr)=='0') { ++eptr; ++vptr; }

        cpl = static_cast<size_t>(vptr-vcstr);
    }


    // Chop and catenate strings
    static const size_t buf_sz = 1024;
    char ocstr[buf_sz];
    ocstr[0]='\0';
    std::strncat(ocstr,vcstr,cpl);
    std::strcat(ocstr,"[");
    std::strcat(ocstr,vcstr+cpl);
    std::strcat(ocstr,pmstr);
    std::strcat(ocstr,ecstr+cpl);
    std::strcat(ocstr,hlfstr);
    std::strcat(ocstr,"]");
    return os << ocstr;

    return os << x.value() << "\u00b1" << x.error();
}


template<> OutputStream& Operations<FloatBall<MultiplePrecision,DoublePrecision>>::_write(OutputStream& os, FloatBall<MultiplePrecision,DoublePrecision> const& x) {
    MultiplePrecision prec(64);
    return os << FloatBall<MultiplePrecision,MultiplePrecision>(x.value_raw(),FloatMP(x.error_raw(),prec));
}


template<> OutputStream& Operations<FloatBall<DoublePrecision>>::_write(OutputStream& os, FloatBall<DoublePrecision> const& x) {
    MultiplePrecision prec(64);
    return os << FloatBall<MultiplePrecision>(FloatMP(x.value_raw(),prec),FloatMP(x.error_raw(),prec));
}

} // namespace Ariadne
