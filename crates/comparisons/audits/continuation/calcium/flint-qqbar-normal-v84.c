#define main symbolic_boundary_unused_main
#include "flint-symbolic-boundary-fixed-v83.c"
#undef main
#include "fmpz_mpoly.h"
#include "gr.h"
#include "gr_generic.h"
#include "fmpq_mat.h"
#include "fmpz_mat.h"

int main(void)
{
    int order, seed, alias, generic, i, j, ok, family, flags, n, k, dim;
    long rows = 0; ulong exps[2];
    gr_ctx_t ctx; fmpz_mpoly_ctx_t mctx; fmpz_mpoly_t p;
    qqbar_ptr x = _qqbar_vec_init(2), roots = _qqbar_vec_init(3);
    qqbar_t out, s2, s3; qqbar_ptr destination;
    fmpz_mat_t zm; fmpq_mat_t qm; fmpq_poly_t qp; fmpq_t q;
    gr_ctx_init_complex_qqbar(ctx); _gr_ctx_qqbar_set_limits(ctx, 64, 10000);
    qqbar_init(out); qqbar_init(s2); qqbar_init(s3);
    qqbar_set_ui(s2, 2); qqbar_sqrt(s2, s2);
    qqbar_set_ui(s3, 3); qqbar_sqrt(s3, s3);
    for (order = 0; order < 3; order++) {
        fmpz_mpoly_ctx_init(mctx, 2, order == 0 ? ORD_LEX : order == 1 ? ORD_DEGLEX : ORD_DEGREVLEX);
        fmpz_mpoly_init(p, mctx);
        for (seed = 0; seed < 10; seed++) {
            fmpz_mpoly_zero(p, mctx);
            for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
                int coefficient = seed < 8 ? (seed + 3*i + 5*j) % 7 - 3 : seed == 9 && i == 0 && j == 0 ? 7 : 0;
                exps[0] = i; exps[1] = j;
                fmpz_mpoly_set_coeff_si_ui(p, coefficient, exps, mctx);
            }
            for (alias = 0; alias < 3; alias++) for (generic = 0; generic < 2; generic++) {
                qqbar_add_si(x, s2, seed - 4); qqbar_add_si(x + 1, s3, 1 - seed);
                destination = alias == 0 ? out : x + alias - 1;
                if (generic) ok = gr_fmpz_mpoly_evaluate(destination, p, x, mctx, ctx) == GR_SUCCESS;
                else ok = qqbar_evaluate_fmpz_mpoly(destination, p, x, 64, 10000, mctx);
                printf("{\"family\":\"normal-mpoly\",\"order\":%d,\"seed\":%d,\"alias\":%d,\"generic\":%d,\"ok\":%d", order, seed, alias, generic, ok);
                if (ok) value(destination);
                puts("}"); rows++;
            }
        }
        fmpz_mpoly_clear(p, mctx); fmpz_mpoly_ctx_clear(mctx);
    }
    fmpq_init(q); fmpq_poly_init(qp);
    for (family = 0; family < 3; family++) for (flags = 0; flags < 4; flags++) for (n = 2; n <= 3; n++) {
        fmpz_mat_init(zm, 2, 2); fmpq_mat_init(qm, 2, 2);
        fmpz_set_si(fmpz_mat_entry(zm, 0, 1), n); fmpz_one(fmpz_mat_entry(zm, 1, 0));
        fmpq_set_si(fmpq_mat_entry(qm, 0, 1), n, 3); fmpq_set_si(fmpq_mat_entry(qm, 1, 0), 1, 3);
        fmpq_poly_zero(qp); fmpq_set_si(q, -n, 7); fmpq_poly_set_coeff_fmpq(qp, 0, q);
        fmpq_set_si(q, 1, 7); fmpq_poly_set_coeff_fmpq(qp, 2, q);
        if (family == 0) qqbar_eigenvalues_fmpz_mat(roots, zm, flags);
        if (family == 1) qqbar_eigenvalues_fmpq_mat(roots, qm, flags);
        if (family == 2) qqbar_roots_fmpq_poly(roots, qp, flags);
        for (k = 0; k < 2; k++) {
            printf("{\"family\":\"root-wrapper\",\"wrapper\":%d,\"flags\":%d,\"n\":%d,\"index\":%d", family, flags, n, k);
            value(roots + k); puts("}"); rows++;
        }
        fmpz_mat_clear(zm); fmpq_mat_clear(qm);
    }
    for (family = 0; family < 2; family++) for (flags = 0; flags <= 2; flags += 2) for (dim = 0; dim <= 3; dim++) {
        fmpz_mat_init(zm, dim, dim); fmpq_mat_init(qm, dim, dim);
        for (i = 0; i < dim; i++) for (j = i; j < dim; j++) {
            int entry = i == j ? (i == 0 ? -2 : 1) : 5 + i + j;
            fmpz_set_si(fmpz_mat_entry(zm, i, j), entry); fmpq_set_si(fmpq_mat_entry(qm, i, j), entry, 3);
        }
        if (family == 0) qqbar_eigenvalues_fmpz_mat(roots, zm, flags);
        else qqbar_eigenvalues_fmpq_mat(roots, qm, flags);
        printf("{\"family\":\"triangular\",\"wrapper\":%d,\"flags\":%d,\"dim\":%d,\"roots\":[", family, flags, dim);
        for (k = 0; k < dim; k++) { if (k) putchar(','); printf("{\"index\":%d", k); value(roots + k); putchar('}'); }
        puts("]}"); rows++;
        fmpz_mat_clear(zm); fmpq_mat_clear(qm);
    }
    fmpq_clear(q); fmpq_poly_clear(qp); qqbar_clear(out); qqbar_clear(s2); qqbar_clear(s3);
    _qqbar_vec_clear(x, 2); _qqbar_vec_clear(roots, 3); gr_ctx_clear(ctx); flint_cleanup();
    printf("{\"terminal\":true,\"rows\":%ld}\n", rows); return 0;
}
