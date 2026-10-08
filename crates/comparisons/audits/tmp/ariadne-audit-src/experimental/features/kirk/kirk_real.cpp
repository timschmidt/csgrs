/***************************************************************************
 *            external/kirk_real.cpp
 *
 *  Copyright  2017-20  Pieter Collins, Franz Brausse
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
 *  You should have received _a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include "kirk_real.hpp"

#include <mpfr.h>

#include "numeric/real.hpp"
#include "numeric/real_interface.hpp"

namespace Ariadne {

class KirkReal : public RealInterface {
  public:
    ~KirkReal();
    KirkReal(kirk_real_t* r);
  private:
    ValidatedReal _compute(Effort eff) const;
    FloatMPBounds _compute_get(MultiplePrecision pr) const;
    FloatDPBounds _compute_get(DoublePrecision pr) const;
    OutputStream& _write(OutputStream& os) const;
  private:
    kirk_real_t* _real;
};

} // namespace Ariadne



extern "C" {
#include "kirk/kirk-c-types.h"
}
#include "kirk/kirk-real-obj.h"


#include "numeric/dyadic.hpp"
#include "numeric/real.hpp"
#include "numeric/real_interface.hpp"
#include "numeric/floatdp.hpp"
#include "numeric/floatmp.hpp"
#include "numeric/float_bounds.hpp"
#include "numeric/float_value.hpp"
#include "numeric/float_error.hpp"


namespace Ariadne {

inline FloatDP to_floatdp(kirk_bound_t const* err) {
    assert(err->exponent>-1023 && err->exponent<1023);
    return ldexp(static_cast<double>(err->mantissa),err->exponent);
}

inline kirk_bound_t to_kirk_bound_t(FloatDP const& x) {
    kirk_bound_t kirk_bound;
    kirk_bound_set_d(&kirk_bound,x.get_d());
    return kirk_bound;
}

inline
KirkReal::~KirkReal() {
    // Unreference the Kirk real
    kirk_real_unref(_real);
}

inline
KirkReal::KirkReal(kirk_real_t* r)
    : _real(r)
{
    // Claim a reference to the Kirk real
    kirk_real_ref(_real);
}

inline
ValidatedReal KirkReal::_compute(Effort eff) const {
    assert(false);
    MultiplePrecision pr(eff.work());
    //return ValidatedReal(DyadicBounds(this->_compute_get(pr)));
}

inline
FloatMPBounds KirkReal::_compute_get(MultiplePrecision pr) const {
    // Declare a Kirk struct bounds on a real number
    kirk_apx_t apx;
    // Initialise apx mpfr centre
    mpfr_init(apx.center);
    // Compute the bounds on the real from Kirk using pr bits
    kirk_real_apx_eff(_real,&apx,pr.bits());
    // Construct an Ariadne MPFR number.
    //   The RawPtr() tag is to prevent the number '0' being interpreted
    //   as a pointer elsewhere in the code
    FloatMP c(apx.center,RawPtr());
    // Convert the Kirk radius to an Ariadne double-precision number
    FloatDP r(to_floatdp(&apx.radius));
    // Convert the centre-radius representation to lower and upper bounds
    return FloatMPBounds(sub(down,c,r),add(up,c,r));
}

inline
FloatDPBounds KirkReal::_compute_get(DoublePrecision pr) const {
    return FloatDPBounds(this->_compute_get(MultiplePrecision(pr)),pr);
}

inline
OutputStream& KirkReal::_write(OutputStream& os) const {
    return os << "KirkReal(?\?)";
}



struct kirk_ariadne_real_t;
void kirk_ariadne_real_destroy(kirk_real_obj_t* ptr);
void kirk_ariadne_apx_eff(const kirk_real_t * real, kirk_apx_t * apt, kirk_eff_t eff);
void kirk_ariadne_apx_abs(const kirk_real_t * real, kirk_apx_t * apt, kirk_abs_t acc);


static const kirk_real_obj_class_t kirk_real_obj_class = {
    .parent = {
        .ref     = kirk_real_obj_default_ref,
        .unref   = kirk_real_obj_default_unref,
        .apx_abs = kirk_ariadne_apx_abs,
        .apx_eff = kirk_ariadne_apx_eff,
    },
    .finalize = kirk_real_obj_default_finalize,
};

struct kirk_ariadne_real_t : kirk_real_obj_t
{
    kirk_ariadne_real_t(Real const& r)
        : kirk_real_obj_t KIRK_REAL_OBJ_INIT(&kirk_real_obj_class.parent,&kirk_ariadne_real_destroy)
        , _real(r)
    { }
    Real _real;
};

void kirk_ariadne_real_destroy(kirk_real_obj_t* ptr) {
    delete static_cast<kirk_ariadne_real_t*>(ptr); ptr=nullptr;
}

void kirk_ariadne_apx_eff(const kirk_real_t * real, kirk_apx_t * apt, kirk_eff_t eff) {
    const kirk_ariadne_real_t* ariadne_real = reinterpret_cast<const kirk_ariadne_real_t*>(real);
    Effort ariadne_eff(eff);
    MultiplePrecision prec(eff);
    FloatMPBounds ariadne_bounds=ariadne_real->_real.compute(eff).get(prec);
    FloatMP ariadne_midpoint=ariadne_bounds.value().raw();
    FloatMP ariadne_error=hlf(sub(up,ariadne_bounds.upper_raw(),ariadne_bounds.lower_raw()));
    FloatDP ariadne_radius(Dyadic(ariadne_error),up,double_precision);
    const mpfr_t& centre = ariadne_midpoint.raw().get_mpfr();
    kirk_bound_t radius = to_kirk_bound_t(ariadne_radius);
    kirk_apx_set(apt,centre,&radius);
}

void kirk_ariadne_apx_abs(const kirk_real_t * real, kirk_apx_t * apt, kirk_abs_t acc) {
    const kirk_ariadne_real_t* ariadne_real = reinterpret_cast<const kirk_ariadne_real_t*>(real);
    Accuracy ariadne_acc(acc);
    ValidatedReal ariadne_bounds=ariadne_real->_real.compute(acc);
    assert(false);
}

kirk_real_t* to_kirk_real(Real const& r) {
    kirk_ariadne_real_t* ariadne_real=new kirk_ariadne_real_t(r);
    return &ariadne_real->parent;
}

Real from_kirk_real(kirk_real_t* r) {
    return Real(std::make_shared<KirkReal>(r));
}

Void delete_kirk_real(kirk_real_t* r) {
    // FIXME: How to safely delete a pointer to a kirk_real_t?
    // kirk_real_unref(r);
    // delete r;
}


} //namespace Ariadne
