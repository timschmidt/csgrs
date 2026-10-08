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
static void context(unsigned id, slong nvars, ordering_t order) {
  fmpz_mpoly_ctx_t ctx;
  fmpz_mpoly_ctx_init(ctx, nvars, order);
  fmpz_mpoly_struct factors[5];
  for (unsigned i = 0; i < 5; i++)
    fmpz_mpoly_init(factors + i, ctx);
  fmpz_mpoly_gen(factors, 0, ctx);
  fmpz_mpoly_add_si(factors, factors, 1, ctx);
  fmpz_mpoly_gen(factors + 1, nvars - 1, ctx);
  fmpz_mpoly_add_si(factors + 1, factors + 1, 2, ctx);
  fmpz_mpoly_add(factors + 2, factors, factors + 1, ctx);
  fmpz_mpoly_gen(factors + 3, 0, ctx);
  fmpz_mpoly_sub(factors + 4, factors, factors + 1, ctx);
  fmpz_mpoly_q_struct input[20];
  fmpz_mpoly_q_t z;
  fmpz_mpoly_q_init(z, ctx);
  for (unsigned i = 0; i < 20; i++) {
    fmpz_mpoly_q_init(input + i, ctx);
    make(input + i, i, factors, ctx);
    printf("{\"kind\":\"input\",\"ctx\":%u,\"vars\":%ld,\"order\":%d,\"i\":%u,"
           "\"raw\":[",
           id, nvars, (int)order, i);
    poly(fmpz_mpoly_q_numref(input + i), ctx, nvars);
    printf(",");
    poly(fmpz_mpoly_q_denref(input + i), ctx, nvars);
    fmpz_mpoly_q_canonicalise(input + i, ctx);
    printf("],\"value\":");
    emit(input + i, ctx, nvars);
    printf("}\n");
    rows++;
  }
  for (unsigned i = 0; i < 20; i++) {
    const unsigned js[] = {0, i, (i + 1) % 20, (i + 7) % 20, 11, 14};
    for (unsigned jpos = 0; jpos < 6; jpos++)
      for (unsigned op = 0; op < 4; op++) {
        unsigned j = js[jpos];
        if (op == 3 && j == 0)
          continue;
        void (*fn)(fmpz_mpoly_q_struct *, const fmpz_mpoly_q_struct *,
                   const fmpz_mpoly_q_struct *, const fmpz_mpoly_ctx_struct *) =
            op == 0   ? fmpz_mpoly_q_add
            : op == 1 ? fmpz_mpoly_q_sub
            : op == 2 ? fmpz_mpoly_q_mul
                      : fmpz_mpoly_q_div;
        printf("{\"kind\":\"pair\",\"ctx\":%u,\"i\":%u,\"j\":%u,\"op\":%u,"
               "\"values\":[",
               id, i, j, op);
        for (unsigned alias = 0; alias < 3; alias++) {
          if (alias)
            printf(",");
          if (alias == 0) {
            fmpz_mpoly_q_set_si(z, 7, ctx);
            fn(z, input + i, input + j, ctx);
          } else if (alias == 1) {
            fmpz_mpoly_q_set(z, input + i, ctx);
            fn(z, z, input + j, ctx);
            aliases++;
          } else {
            fmpz_mpoly_q_set(z, input + j, ctx);
            fn(z, input + i, z, ctx);
            aliases++;
          }
          emit(z, ctx, nvars);
        }
        printf("]}\n");
        rows++;
      }
    for (unsigned op = 0; op < 2; op++) {
      if (op == 1 && i == 0)
        continue;
      void (*fn)(fmpz_mpoly_q_struct *, const fmpz_mpoly_q_struct *,
                 const fmpz_mpoly_ctx_struct *) =
          op ? fmpz_mpoly_q_inv : fmpz_mpoly_q_neg;
      printf("{\"kind\":\"unary\",\"ctx\":%u,\"i\":%u,\"op\":%u,\"values\":[",
             id, i, op);
      fmpz_mpoly_q_set_si(z, 7, ctx);
      fn(z, input + i, ctx);
      emit(z, ctx, nvars);
      printf(",");
      fmpz_mpoly_q_set(z, input + i, ctx);
      fn(z, z, ctx);
      aliases++;
      emit(z, ctx, nvars);
      printf("]}\n");
      rows++;
    }
    for (unsigned s = 0; s < 4; s++) {
      fmpq_t scalar;
      fmpq_init(scalar);
      if (s == 0)
        fmpq_zero(scalar);
      else if (s == 1)
        fmpq_set_si(scalar, -3, 1);
      else if (s == 2)
        fmpq_set_si(scalar, 2, 3);
      else {
        large(fmpq_numref(scalar), 65);
        fmpz_add_ui(fmpq_denref(scalar), fmpq_numref(scalar), 2);
        fmpq_canonicalise(scalar);
      }
      for (unsigned op = 0; op < 4; op++)
        for (unsigned route = 0; route < (s < 2 ? 3 : 1); route++) {
          if (op == 3 && s == 0)
            continue;
          printf("{\"kind\":\"scalar\",\"ctx\":%u,\"i\":%u,\"s\":%u,\"op\":%u,"
                 "\"route\":%u,\"values\":[",
                 id, i, s, op, route);
          for (unsigned alias = 0; alias < 2; alias++) {
            if (alias) {
              printf(",");
              fmpz_mpoly_q_set(z, input + i, ctx);
              aliases++;
            } else
              fmpz_mpoly_q_set_si(z, 7, ctx);
            const fmpz_mpoly_q_struct *x = alias ? z : input + i;
            if (route == 0) {
              if (op == 0)
                fmpz_mpoly_q_add_fmpq(z, x, scalar, ctx);
              else if (op == 1)
                fmpz_mpoly_q_sub_fmpq(z, x, scalar, ctx);
              else if (op == 2)
                fmpz_mpoly_q_mul_fmpq(z, x, scalar, ctx);
              else
                fmpz_mpoly_q_div_fmpq(z, x, scalar, ctx);
            } else if (route == 1) {
              if (op == 0)
                fmpz_mpoly_q_add_fmpz(z, x, fmpq_numref(scalar), ctx);
              else if (op == 1)
                fmpz_mpoly_q_sub_fmpz(z, x, fmpq_numref(scalar), ctx);
              else if (op == 2)
                fmpz_mpoly_q_mul_fmpz(z, x, fmpq_numref(scalar), ctx);
              else
                fmpz_mpoly_q_div_fmpz(z, x, fmpq_numref(scalar), ctx);
            } else {
              slong si = s == 0 ? 0 : -3;
              if (op == 0)
                fmpz_mpoly_q_add_si(z, x, si, ctx);
              else if (op == 1)
                fmpz_mpoly_q_sub_si(z, x, si, ctx);
              else if (op == 2)
                fmpz_mpoly_q_mul_si(z, x, si, ctx);
              else
                fmpz_mpoly_q_div_si(z, x, si, ctx);
            }
            emit(z, ctx, nvars);
          }
          printf("]}\n");
          rows++;
        }
      fmpq_clear(scalar);
    }
  }
  for (unsigned i = 0; i < 20; i++) {
    printf("{\"kind\":\"preserved\",\"ctx\":%u,\"i\":%u,\"value\":", id, i);
    emit(input + i, ctx, nvars);
    printf("}\n");
    rows++;
    fmpz_mpoly_q_clear(input + i, ctx);
  }
  fmpz_mpoly_q_clear(z, ctx);
  for (unsigned i = 0; i < 5; i++)
    fmpz_mpoly_clear(factors + i, ctx);
  fmpz_mpoly_ctx_clear(ctx);
}
int main(void) {
  assert(FLINT_BITS == 64);
  const slong nvars[] = {1, 2, 4};
  const ordering_t orders[] = {ORD_LEX, ORD_DEGLEX, ORD_DEGREVLEX};
  for (unsigned n = 0; n < 3; n++)
    for (unsigned o = 0; o < 3; o++)
      context(n * 3 + o, nvars[n], orders[o]);
  flint_cleanup();
  printf("{\"summary\":true,\"contexts\":9,\"rows\":%lu,\"values\":%lu,"
         "\"aliases\":%lu,\"failures\":%lu}\n",
         rows, values, aliases, failures);
  return failures ? 1 : 0;
}
