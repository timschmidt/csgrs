#include <stdio.h>
#include <gmp.h>
/* Checkpoint46: bounded canonical rational functions and public whole-object
 * aliases. Full coefficients are independently checked by BigInt. No parser,
 * invalid context/size, zero divisor or historical crash reproduction. */
#include "fmpq.h"
#include "fmpz_mpoly_q.h"
#include <assert.h>
#include <stdio.h>

static unsigned long values, aliases, rows, failures;
static const int numerator_scale[20] = {0, 1, -1, 1, 1, 1, 1, 1, 1, 1,
                                        1, 1, 1,  2, 3, 1, 1, 1, 1, -2};
static const int denominator_scale[20] = {1, 1, 1, 1, 1, 2, 3, 2, 3, 1,
                                          1, 1, 1, 3, 2, 1, 1, 1, 1, -4};
static const char *nf[20] = {"", "",  "",  "A", "B", "A",  "B", "A", "B", "",
                             "", "C", "E", "A", "B", "AD", "A", "A", "C", "C"};
static const char *df[20] = {"",  "",   "",  "",  "",    "",   "",
                             "B", "A",  "A", "B", "AB",  "AB", "B",
                             "A", "BD", "B", "B", "ABC", "BC"};

static void poly(const fmpz_mpoly_t p, const fmpz_mpoly_ctx_t ctx,
                 slong nvars) {
  fmpz_t c;
  fmpz_init(c);
  mpz_t z;
  mpz_init(z);
  ulong exps[4];
  printf("[");
  for (slong t = 0; t < fmpz_mpoly_length(p, ctx); t++) {
    if (t)
      printf(",");
    assert(fmpz_mpoly_term_exp_fits_ui(p, t, ctx));
    fmpz_mpoly_get_term_exp_ui(exps, p, t, ctx);
    fmpz_mpoly_get_term_coeff_fmpz(c, p, t, ctx);
    fmpz_get_mpz(z, c);
    gmp_printf("[\"%Zx\",[", z);
    for (slong j = 0; j < nvars; j++) {
      if (j)
        printf(",");
      assert(exps[j] <= 12);
      printf("%lu", exps[j]);
    }
    printf("]]");
  }
  printf("]");
  mpz_clear(z);
  fmpz_clear(c);
}
static void mask(const int *m, slong nvars) {
  printf("[");
  for (slong j = 0; j < nvars; j++) {
    if (j)
      printf(",");
    printf("%d", m[j]);
  }
  printf("]");
}
static void emit(const fmpz_mpoly_q_t q, const fmpz_mpoly_ctx_t ctx,
                 slong nvars) {
  values++;
  if (!fmpz_mpoly_q_is_canonical(q, ctx)) {
    failures++;
    fprintf(stderr, "noncanonical value %lu\n", values);
  }
  printf("[");
  poly(fmpz_mpoly_q_numref(q), ctx, nvars);
  printf(",");
  poly(fmpz_mpoly_q_denref(q), ctx, nvars);
  printf(",[%d,%d,%d,%d]", fmpz_mpoly_q_is_zero(q, ctx),
         fmpz_mpoly_q_is_one(q, ctx), fmpz_mpoly_q_is_fmpz(q, ctx),
         fmpz_mpoly_q_is_fmpq(q, ctx));
  int nm[4] = {7, 7, 7, 7}, dm[4] = {7, 7, 7, 7}, um[4] = {7, 7, 7, 7};
  fmpz_mpoly_q_used_vars_num(nm, q, ctx);
  fmpz_mpoly_q_used_vars_den(dm, q, ctx);
  fmpz_mpoly_q_used_vars(um, q, ctx);
  printf(",");
  mask(nm, nvars);
  printf(",");
  mask(dm, nvars);
  printf(",");
  mask(um, nvars);
  fmpq_t c;
  fmpq_init(c);
  mpz_t a, b;
  mpz_inits(a, b, NULL);
  fmpz_mpoly_q_content(c, q, ctx);
  fmpz_get_mpz(a, fmpq_numref(c));
  fmpz_get_mpz(b, fmpq_denref(c));
  gmp_printf(",[\"%Zx\",\"%Zx\"]]", a, b);
  mpz_clears(a, b, NULL);
  fmpq_clear(c);
}
static void large(fmpz_t c, unsigned bits) {
  mpz_t z;
  mpz_init_set_ui(z, 1);
  mpz_mul_2exp(z, z, bits);
  mpz_add_ui(z, z, 7);
  fmpz_set_mpz(c, z);
  mpz_clear(z);
}
static void make(fmpz_mpoly_q_t q, unsigned recipe,
                 fmpz_mpoly_struct factors[5], const fmpz_mpoly_ctx_t ctx) {
  fmpz_mpoly_set_si(fmpz_mpoly_q_numref(q), numerator_scale[recipe], ctx);
  fmpz_mpoly_set_si(fmpz_mpoly_q_denref(q), denominator_scale[recipe], ctx);
  for (const char *s = nf[recipe]; *s; s++)
    fmpz_mpoly_mul(fmpz_mpoly_q_numref(q), fmpz_mpoly_q_numref(q),
                   factors + (*s - 'A'), ctx);
  for (const char *s = df[recipe]; *s; s++)
    fmpz_mpoly_mul(fmpz_mpoly_q_denref(q), fmpz_mpoly_q_denref(q),
                   factors + (*s - 'A'), ctx);
  if (recipe == 16 || recipe == 17) {
    fmpz_t c;
    fmpz_init(c);
    large(c, recipe == 16 ? 65 : 129);
    fmpz_mpoly_scalar_mul_fmpz(fmpz_mpoly_q_numref(q), fmpz_mpoly_q_numref(q),
                               c, ctx);
    fmpz_mpoly_scalar_mul_fmpz(fmpz_mpoly_q_denref(q), fmpz_mpoly_q_denref(q),
                               c, ctx);
    fmpz_clear(c);
  }
}
