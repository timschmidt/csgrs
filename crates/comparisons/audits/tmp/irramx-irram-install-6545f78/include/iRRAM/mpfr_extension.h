
#ifndef iRRAM_MPFR_EXTENSION_H
#define iRRAM_MPFR_EXTENSION_H

#ifndef GMP_RNDN 
#include <mpfr.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

void mpfr_ext_exp (mpfr_ptr r, mpfr_srcptr u,int p);
void mpfr_ext_log (mpfr_ptr r, mpfr_srcptr u,int p);
void mpfr_ext_sin (mpfr_ptr r, mpfr_srcptr u,int p);
void mpfr_ext_cos (mpfr_ptr r, mpfr_srcptr u,int p);
void mpfr_ext_tan (mpfr_ptr r, mpfr_srcptr u,int p);
void mpfr_ext_test (mpfr_ptr r, mpfr_srcptr u,int p,mp_rnd_t rnd_mode);

#ifdef __cplusplus
}
#endif

#endif
