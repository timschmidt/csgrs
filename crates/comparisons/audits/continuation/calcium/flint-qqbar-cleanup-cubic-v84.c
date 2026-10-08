#define main symbolic_boundary_unused_main
#include "flint-symbolic-boundary-fixed-v83.c"
#undef main
#include "fmpz_mpoly.h"
#include "gr.h"
#include "gr_generic.h"
#include "gr_poly.h"
#include "gr_vec.h"
#include "fmpz_vec.h"

static int monomial(int generic, int exponent, int terms)
{
    fmpz_mpoly_ctx_t mctx; fmpz_mpoly_t p; fmpz_t e;
    fmpz *ep[1]; gr_ctx_t ctx; qqbar_t x, y; int ok;
    fmpz_mpoly_ctx_init(mctx, 1, ORD_LEX); fmpz_mpoly_init(p, mctx);
    fmpz_init(e); fmpz_one(e); fmpz_mul_2exp(e, e, exponent); ep[0] = e;
    fmpz_mpoly_set_coeff_si_fmpz(p, 1, ep, mctx);
    if (terms == 2) { fmpz_zero(e); fmpz_mpoly_set_coeff_si_fmpz(p, 1, ep, mctx); }
    qqbar_init(x); qqbar_init(y); qqbar_one(x); qqbar_set_si(y, 7);
    gr_ctx_init_complex_qqbar(ctx); _gr_ctx_qqbar_set_limits(ctx, 8, 1000);
    fprintf(stderr, "monomial generic=%d exponent=%d terms=%d packed_bits=%lu\n", generic, exponent, terms, (unsigned long)p->bits); fflush(stderr);
    if (generic) ok = gr_fmpz_mpoly_evaluate(y, p, x, mctx, ctx) == GR_SUCCESS;
    else ok = qqbar_evaluate_fmpz_mpoly(y, p, x, 8, 1000, mctx);
    printf("{\"family\":\"monomial\",\"generic\":%d,\"exponent\":%d,\"terms\":%d,\"packedBits\":%lu,\"ok\":%d", generic, exponent, terms, (unsigned long)p->bits, ok);
    if (ok) value(y);
    puts("}");
    gr_ctx_clear(ctx); qqbar_clear(x); qqbar_clear(y); fmpz_clear(e);
    fmpz_mpoly_clear(p, mctx); fmpz_mpoly_ctx_clear(mctx); flint_cleanup(); return 0;
}
static int root_limit(int degree, int bounded)
{
    qqbar_ptr coeffs = _qqbar_vec_init(degree + 1), roots = _qqbar_vec_init(degree);
    int i, ok; slong limit = bounded ? 200 : -1;
    qqbar_set_ui(coeffs, 2); qqbar_sqrt(coeffs, coeffs);
    for (i = 1; i <= degree; i++) qqbar_set(coeffs + i, coeffs);
    fprintf(stderr, "root-limit degree=%d limit=%ld coefficient_degree=2\n", degree, (long)limit); fflush(stderr);
    ok = _qqbar_roots_poly_squarefree(roots, coeffs, degree + 1, limit, 1000);
    printf("{\"family\":\"root-limit\",\"degree\":%d,\"bounded\":%d,\"ok\":%d}\n", degree, bounded, ok);
    _qqbar_vec_clear(coeffs, degree + 1); _qqbar_vec_clear(roots, degree); flint_cleanup(); return 0;
}
static int root_cleanup(int limit, int repetitions)
{
    gr_ctx_t ctx; gr_poly_t p; gr_vec_t roots; fmpz_vec_t mult; qqbar_t c;
    int i, status; slong j;
    gr_ctx_init_complex_qqbar(ctx); gr_poly_init(p, ctx); gr_vec_init(roots, 0, ctx); fmpz_vec_init(mult, 0);
    qqbar_init(c); qqbar_set_ui(c, 2); qqbar_sqrt(c, c); qqbar_neg(c, c);
    status = gr_poly_set_scalar(p, c, ctx); status |= gr_poly_set_coeff_si(p, 3, 1, ctx);
    if (status != GR_SUCCESS) abort();
    _gr_ctx_qqbar_set_limits(ctx, limit, 1000);
    for (i = 0; i < repetitions; i++) {
        status = gr_poly_roots(roots, mult, p, 0, ctx);
        printf("{\"family\":\"root-cleanup\",\"limit\":%d,\"iteration\":%d,\"status\":%d", limit, i, status);
        if (status == GR_SUCCESS) {
            printf(",\"roots\":[");
            for (j = 0; j < roots->length; j++) {
                if (j) putchar(',');
                printf("{\"index\":%ld", (long)j); value((qqbar_ptr) roots->entries + j); putchar('}');
            }
            putchar(']');
        }
        puts("}");
    }
    gr_poly_clear(p, ctx); gr_vec_clear(roots, ctx); fmpz_vec_clear(mult); qqbar_clear(c); gr_ctx_clear(ctx); flint_cleanup();
    printf("{\"terminal\":true,\"rows\":%d}\n", repetitions); return 0;
}
int main(int argc, char **argv)
{
    if (argc == 5 && strcmp(argv[1], "monomial") == 0) {
        int g = atoi(argv[2]), e = atoi(argv[3]), t = atoi(argv[4]);
        if ((g != 0 && g != 1) || e < 0 || e > 256 || (t != 1 && t != 2)) return 2;
        return monomial(g, e, t);
    }
    if (argc == 4 && strcmp(argv[1], "limit") == 0) {
        int d = atoi(argv[2]), b = atoi(argv[3]);
        if (d < 8 || d > 63 || (b != 0 && b != 1)) return 2;
        return root_limit(d, b);
    }
    if (argc == 4 && strcmp(argv[1], "cleanup") == 0) {
        int l = atoi(argv[2]), n = atoi(argv[3]);
        if (l < 1 || l > 8 || n < 1 || n > 256) return 2;
        return root_cleanup(l, n);
    }
    return 2;
}
