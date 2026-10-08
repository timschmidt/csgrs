/* Checkpoint47: finite formal-expression and generic-ring conversions.
 * Valid constructors, matching variable/context lengths and small exponents.
 * No malformed inputs, resource-boundary probes or old failure reproduction. */
#include "mpoly-bridge-helpers.h"
#include "fexpr.h"
#include "gr.h"

static void converted(const fexpr_t expr, const fexpr_vec_t vars,
                      const fmpz_mpoly_ctx_t ctx, slong nvars) {
  fmpz_mpoly_q_t q;
  fexpr_t normal;
  fmpz_mpoly_q_init(q, ctx);
  fexpr_init(normal);
  assert(fexpr_get_fmpz_mpoly_q(q, expr, vars, ctx) == 1);
  emit(q, ctx, nvars);
  printf(",");
  assert(fexpr_expanded_normal_form(normal, expr, 0) == 1);
  assert(fexpr_get_fmpz_mpoly_q(q, normal, vars, ctx) == 1);
  emit(q, ctx, nvars);
  fexpr_clear(normal);
  fmpz_mpoly_q_clear(q, ctx);
}

static void authored(fexpr_t out, unsigned recipe, const fexpr_vec_t vars) {
  fexpr_t a, b, t, u, one, exponent, op;
  fexpr_init(a); fexpr_init(b); fexpr_init(t); fexpr_init(u);
  fexpr_init(one); fexpr_init(exponent); fexpr_init(op);
  fexpr_set_si(one, 1);
  fexpr_add(a, fexpr_vec_entry(vars, 0), one);
  fexpr_set_si(one, 2);
  fexpr_add(b, fexpr_vec_entry(vars, vars->length - 1), one);
  switch (recipe) {
  case 0: fexpr_div(out, a, a); break;
  case 1:
    fexpr_mul(t, a, a); fexpr_mul(u, b, b); fexpr_sub(t, t, u);
    fexpr_sub(u, a, b); fexpr_div(out, t, u); break;
  case 2:
    fexpr_div(t, a, b); fexpr_div(u, b, a); fexpr_mul(out, t, u); break;
  case 3:
    fexpr_add(t, a, b); fexpr_neg(out, t); break;
  case 4:
    fexpr_div(t, a, b); fexpr_set_symbol_str(op, "Pos");
    fexpr_call1(out, op, t); break;
  case 5:
    fexpr_set_symbol_str(op, "Add"); fexpr_call3(out, op, a, b, a); break;
  case 6:
    fexpr_set_symbol_str(op, "Mul"); fexpr_call3(out, op, a, b, a); break;
  case 7:
    fexpr_div(t, a, b); fexpr_set_si(exponent, -3);
    fexpr_pow(out, t, exponent); break;
  case 8:
    fexpr_sub(t, a, a); fexpr_div(out, t, b); break;
  case 9:
    fexpr_add(t, a, b); fexpr_mul(out, t, t);
    fexpr_mul(t, a, a); fexpr_mul(u, b, b); fexpr_add(t, t, u);
    fexpr_mul(u, a, b); fexpr_mul(u, u, one); fexpr_add(t, t, u);
    fexpr_sub(out, out, t); break;
  default: assert(0);
  }
  fexpr_clear(a); fexpr_clear(b); fexpr_clear(t); fexpr_clear(u);
  fexpr_clear(one); fexpr_clear(exponent); fexpr_clear(op);
}

static void bridge_context(unsigned id, slong nvars, ordering_t order) {
  const char *names[] = {"v0", "v1", "v2", "v3"};
  const slong powers[] = {-3, -1, 0, 1, 2, 3};
  fmpz_mpoly_ctx_t ctx;
  gr_ctx_t ring;
  fexpr_vec_t vars;
  fexpr_t expr, base, exponent;
  fmpz_t power;
  fmpz_mpoly_struct fs[5];
  fmpz_mpoly_q_struct input[20];
  fmpz_mpoly_q_t z;
  fmpz_mpoly_ctx_init(ctx, nvars, order);
  gr_ctx_init_fmpz_mpoly_q(ring, nvars, order);
  fexpr_vec_init(vars, nvars);
  for (slong j = 0; j < nvars; j++)
    fexpr_set_symbol_str(fexpr_vec_entry(vars, j), names[j]);
  fexpr_init(expr); fexpr_init(base); fexpr_init(exponent);
  fmpz_init(power); fmpz_mpoly_q_init(z, ctx);
  for (unsigned j = 0; j < 5; j++) fmpz_mpoly_init(fs + j, ctx);
  fmpz_mpoly_gen(fs, 0, ctx); fmpz_mpoly_add_si(fs, fs, 1, ctx);
  fmpz_mpoly_gen(fs + 1, nvars - 1, ctx);
  fmpz_mpoly_add_si(fs + 1, fs + 1, 2, ctx);
  fmpz_mpoly_add(fs + 2, fs, fs + 1, ctx);
  fmpz_mpoly_gen(fs + 3, 0, ctx);
  fmpz_mpoly_sub(fs + 4, fs, fs + 1, ctx);
  for (unsigned i = 0; i < 20; i++) {
    fmpz_mpoly_q_init(input + i, ctx); make(input + i, i, fs, ctx);
    fmpz_mpoly_q_canonicalise(input + i, ctx);
    printf("{\"kind\":\"input\",\"ctx\":%u,\"i\":%u,\"value\":", id, i);
    emit(input + i, ctx, nvars); printf("}\n"); rows++;
  }
  for (unsigned i = 0; i < 20; i++) {
    fexpr_set_fmpz_mpoly_q(base, input + i, vars, ctx);
    printf("{\"kind\":\"roundtrip\",\"ctx\":%u,\"i\":%u,\"values\":[", id, i);
    converted(base, vars, ctx, nvars); printf("]}\n"); rows++;
    for (unsigned p = 0; p < 6; p++) {
      slong e = powers[p];
      if (i == 0 && e < 0) continue;
      fmpz_set_si(power, e); fexpr_set_si(exponent, e);
      printf("{\"kind\":\"power\",\"ctx\":%u,\"i\":%u,\"e\":%ld,\"values\":[", id, i, e);
      for (unsigned alias = 0; alias < 2; alias++) {
        if (alias) { printf(","); fmpz_mpoly_q_set(z, input + i, ctx); aliases++; }
        else fmpz_mpoly_q_set_si(z, 7, ctx);
        assert(gr_pow_fmpz(z, alias ? z : input + i, power, ring) == GR_SUCCESS);
        emit(z, ctx, nvars);
      }
      printf(","); fexpr_pow(expr, base, exponent);
      converted(expr, vars, ctx, nvars);
      if (e >= 0) for (unsigned alias = 0; alias < 2; alias++) {
        printf(",");
        if (alias) { fmpz_mpoly_q_set(z, input + i, ctx); aliases++; }
        else fmpz_mpoly_q_set_si(z, 7, ctx);
        assert(gr_pow_ui(z, alias ? z : input + i, (ulong)e, ring) == GR_SUCCESS);
        emit(z, ctx, nvars);
      }
      printf("]}\n"); rows++;
    }
    for (unsigned part = 0; part < 2; part++) {
      printf("{\"kind\":\"part\",\"ctx\":%u,\"i\":%u,\"part\":%u,\"values\":[", id, i, part);
      for (unsigned alias = 0; alias < 2; alias++) {
        if (alias) { printf(","); fmpz_mpoly_q_set(z, input + i, ctx); aliases++; }
        else fmpz_mpoly_q_set_si(z, 7, ctx);
        int status = part ? gr_denominator(z, alias ? z : input + i, ring)
                          : gr_numerator(z, alias ? z : input + i, ring);
        assert(status == GR_SUCCESS); emit(z, ctx, nvars);
      }
      printf("]}\n"); rows++;
    }
  }
  for (unsigned i = 0; i < 10; i++) {
    authored(expr, i, vars);
    printf("{\"kind\":\"authored\",\"ctx\":%u,\"i\":%u,\"values\":[", id, i);
    converted(expr, vars, ctx, nvars); printf("]}\n"); rows++;
  }
  for (unsigned i = 0; i < 20; i++) {
    printf("{\"kind\":\"preserved\",\"ctx\":%u,\"i\":%u,\"value\":", id, i);
    emit(input + i, ctx, nvars); printf("}\n"); rows++;
    fmpz_mpoly_q_clear(input + i, ctx);
  }
  fmpz_mpoly_q_clear(z, ctx); fmpz_clear(power);
  for (unsigned j = 0; j < 5; j++) fmpz_mpoly_clear(fs + j, ctx);
  fexpr_clear(expr); fexpr_clear(base); fexpr_clear(exponent);
  fexpr_vec_clear(vars); gr_ctx_clear(ring); fmpz_mpoly_ctx_clear(ctx);
}
int main(void) {
  assert(FLINT_BITS == 64);
  const slong nvars[] = {1, 2, 4};
  const ordering_t orders[] = {ORD_LEX, ORD_DEGLEX, ORD_DEGREVLEX};
  for (unsigned n = 0; n < 3; n++) for (unsigned o = 0; o < 3; o++)
    bridge_context(n * 3 + o, nvars[n], orders[o]);
  flint_cleanup();
  printf("{\"summary\":true,\"contexts\":9,\"rows\":%lu,\"values\":%lu,\"aliases\":%lu,\"failures\":%lu}\n",
         rows, values, aliases, failures);
  return failures ? 1 : 0;
}
