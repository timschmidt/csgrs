#include <stdio.h>
#include "ca.h"
#include "ca_mat.h"
#include "perm.h"

static const char *truth(truth_t t)
{ return t == T_TRUE ? "True" : t == T_FALSE ? "False" : "Unknown"; }

int main(void)
{
    int cases = 0, failures = 0, default_unknown = 0, lu_unknown = 0;
    ca_ctx_t ctx; ca_ctx_init(ctx);
    puts("kind,n,shape,expected_zero,default,berkowitz,bareiss_success,bareiss,lu_success,lu,nonsingular_lu");
    for (int kind = 0; kind < 3; kind++) for (slong n = 0; n <= 10; n++)
    for (int shape = 0; shape < 6; shape++)
    {
        ca_mat_t a, lu;
        ca_mat_init(a, n, n, ctx); ca_mat_init(lu, n, n, ctx);
        ca_t base, expected, d, b, f, l;
        ca_init(base, ctx); ca_init(expected, ctx); ca_init(d, ctx);
        ca_init(b, ctx); ca_init(f, ctx); ca_init(l, ctx);
        if (kind == 0) ca_one(base, ctx);
        else if (kind == 1) ca_sqrt_ui(base, 2, ctx);
        else { ca_set_ui(base, 2, ctx); ca_log(base, base, ctx); }
        ca_one(expected, ctx);
        for (slong i = 0; i < n; i++)
        {
            ca_mul_ui(ca_mat_entry(a, i, i), base, (ulong)i + 1, ctx);
            ca_mul(expected, expected, ca_mat_entry(a, i, i), ctx);
        }
        int expected_zero = n > 0 && ((shape >= 1 && shape <= 3) || (shape == 5 && n > 1));
        if (shape >= 1 && shape <= 3 && n > 0)
        {
            slong index = shape == 1 ? 0 : shape == 2 ? n / 2 : n - 1;
            ca_zero(ca_mat_entry(a, index, index), ctx);
        }
        if (shape == 4 && n > 1)
        {
            _ca_mat_swap_rows(a, NULL, 0, n - 1);
            ca_neg(expected, expected, ctx);
        }
        if (shape == 5 && n > 1)
            for (slong j = 0; j < n; j++) ca_set(ca_mat_entry(a, n - 1, j), ca_mat_entry(a, 0, j), ctx);
        if (expected_zero) ca_zero(expected, ctx);
        ca_mat_det(d, a, ctx);
        ca_mat_det_berkowitz(b, a, ctx);
        int fs = ca_mat_det_bareiss(f, a, ctx), ls = ca_mat_det_lu(l, a, ctx);
        slong *perm = _perm_init(n);
        truth_t nonsingular = ca_mat_nonsingular_lu(perm, lu, a, ctx);
        _perm_clear(perm);
        truth_t de = ca_check_equal(d, expected, ctx), be = ca_check_equal(b, expected, ctx);
        truth_t fe = ca_check_equal(f, expected, ctx), le = ca_check_equal(l, expected, ctx);
        failures += de == T_FALSE || be != T_TRUE || fs != 1 || fe != T_TRUE || le == T_FALSE;
        failures += nonsingular != T_UNKNOWN && nonsingular != (expected_zero ? T_FALSE : T_TRUE);
        default_unknown += de == T_UNKNOWN; lu_unknown += le == T_UNKNOWN;
        printf("%d,%ld,%d,%d,%s,%s,%d,%s,%d,%s,%s\n", kind, n, shape, expected_zero,
            truth(de), truth(be), fs, truth(fe), ls, truth(le), truth(nonsingular));
        cases++;
        ca_mat_clear(a, ctx); ca_mat_clear(lu, ctx);
        ca_clear(base, ctx); ca_clear(expected, ctx); ca_clear(d, ctx);
        ca_clear(b, ctx); ca_clear(f, ctx); ca_clear(l, ctx);
    }
    ca_ctx_clear(ctx); flint_cleanup();
    printf("{\"suite\":\"matrix-pivots\",\"cases\":%d,\"failures\":%d,\"default_unknown\":%d,\"lu_unknown\":%d}\n",
        cases, failures, default_unknown, lu_unknown);
    return failures != 0;
}
