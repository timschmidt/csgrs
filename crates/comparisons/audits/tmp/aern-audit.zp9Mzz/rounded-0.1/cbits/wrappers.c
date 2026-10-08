#include <mpfr.h>

static int get_flags(void)
{
  return
    ((!!mpfr_underflow_p ()) << 0) |
    ((!!mpfr_overflow_p  ()) << 1) |
    ((!!mpfr_divby0_p    ()) << 2) |
    ((!!mpfr_nanflag_p   ()) << 3) |
    ((!!mpfr_inexflag_p  ()) << 4) |
    ((!!mpfr_erangeflag_p()) << 5);
}

static void set_flags(int flags)
{
  if (flags & (1 << 0)) mpfr_set_underflow (); else mpfr_clear_underflow ();
  if (flags & (1 << 1)) mpfr_set_overflow  (); else mpfr_clear_overflow  ();
  if (flags & (1 << 2)) mpfr_set_divby0    (); else mpfr_clear_divby0    ();
  if (flags & (1 << 3)) mpfr_set_nanflag   (); else mpfr_clear_nanflag   ();
  if (flags & (1 << 4)) mpfr_set_inexflag  (); else mpfr_clear_inexflag  ();
  if (flags & (1 << 5)) mpfr_set_erangeflag(); else mpfr_clear_erangeflag();
}

int wrapped_mpfr_get_z(mpz_t rop, mpfr_t op, mpfr_rnd_t rnd, int *flags)
{
  int flags0 = get_flags();
  mpfr_clear_flags();
  int retval = mpfr_get_z(rop, op, rnd);
  *flags = get_flags();
  set_flags(flags0);
  return retval;
}

mpfr_exp_t wrapped_mpfr_get_z_2exp(mpz_t rop, mpfr_t op, int *flags)
{
  int flags0 = get_flags();
  mpfr_clear_flags();
  mpfr_exp_t retval = mpfr_get_z_2exp(rop, op);
  *flags = get_flags();
  set_flags(flags0);
  return retval;
}

int wrapped_mpfr_set_ld(mpfr_t rop, long double *op, mpfr_rnd_t rnd, int *flags)
{
  int flags0 = get_flags();
  mpfr_clear_flags();
  int retval = mpfr_set_ld(rop, *op, rnd);
  *flags = get_flags();
  set_flags(flags0);
  return retval;
}

void wrapped_mpfr_get_ld(long double *rop, mpfr_t op, mpfr_rnd_t rnd, int *flags)
{
  int flags0 = get_flags();
  mpfr_clear_flags();
  *rop = mpfr_get_ld(op, rnd);
  *flags = get_flags();
  set_flags(flags0);
}

void wrapped_mpfr_get_ld_2exp(long double *rop, long *e, mpfr_t op, mpfr_rnd_t rnd, int *flags)
{
  int flags0 = get_flags();
  mpfr_clear_flags();
  *rop = mpfr_get_ld_2exp(e, op, rnd);
  *flags = get_flags();
  set_flags(flags0);
}

int wrapped_mpfr_cmp_ld(mpfr_t rop, long double *op)
{
  int flags0 = get_flags();
  mpfr_clear_flags();
  int retval = mpfr_cmp_ld(rop, *op);
  set_flags(flags0);
  return retval;
}
