/* Numerical validation controls, not a reproducer for the earlier field-relation abort. */
#include <fenv.h>
#include <stdio.h>
#include "fmpz.h"
#include "fmpz_mat.h"
#include "fmpz_lll.h"
int main(void)
{
    const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
    const ulong shifts[] = {0, 24, 60, 500, 1200};
    unsigned rows = 0, failures = 0;
    int original = fegetround();
    puts("dimension,pattern,shift,representation,rounding,method,expected,result,round_preserved");
    for (slong n = 0; n <= 6; n++)
    for (int pattern = 0; pattern < 4; pattern++)
    for (unsigned si = 0; si < 5; si++)
    {
        fmpz_mat_t basis, gram;
        fmpz_mat_init(basis, n, n);
        fmpz_mat_init(gram, n, n);
        for (slong i = 0; i < n; i++)
        {
            ulong exp = shifts[si] + (pattern == 1 ? (ulong) i :
                                      pattern == 3 ? (ulong) (n - i) : 0);
            fmpz_one(fmpz_mat_entry(basis, i, i));
            fmpz_mul_2exp(fmpz_mat_entry(basis, i, i), fmpz_mat_entry(basis, i, i), exp);
        }
        if (pattern == 2 && n > 1)
        {
            fmpz_set_ui(fmpz_mat_entry(basis, n - 1, 0), 2);
            fmpz_mul_2exp(fmpz_mat_entry(basis, n - 1, 0), fmpz_mat_entry(basis, n - 1, 0), shifts[si]);
        }
        /* Explicit integer dots; no Gram/matrix-multiplication oracle. */
        for (slong i = 0; i < n; i++)
        for (slong j = 0; j < n; j++)
        for (slong k = 0; k < n; k++)
            fmpz_addmul(fmpz_mat_entry(gram, i, j), fmpz_mat_entry(basis, i, k), fmpz_mat_entry(basis, j, k));
        int expected = n <= 1 || pattern < 2;
        /* Increasing orthogonal norms satisfy Lovasz; a coefficient 2 violates
           size reduction, and decreasing adjacent squared norms have ratio 1/4. */
        for (int rep = 0; rep < 2; rep++)
        for (int rounding = 0; rounding < 4; rounding++)
        for (int method = 0; method < 3; method++)
        {
            fmpz_lll_t fl;
            fmpz_lll_context_init(fl, 0.75, 0.51, rep ? GRAM : Z_BASIS, APPROX);
            if (fesetround(modes[rounding]) != 0) return 2;
            const fmpz_mat_struct *input = rep ? gram : basis;
            int result = method == 0 ? fmpz_lll_is_reduced_d(input, fl) :
                         method == 1 ? fmpz_lll_is_reduced_mpfr(input, fl, 128) :
                                       fmpz_lll_is_reduced(input, fl, 128);
            int restored = fegetround() == modes[rounding];
            failures += !restored || (method == 2 ? result != expected : result && !expected);
            printf("%ld,%d,%lu,%d,%d,%d,%d,%d,%d\n", n, pattern, shifts[si],
                   rep, rounding, method, expected, result, restored);
            rows++;
        }
        fmpz_mat_clear(basis);
        fmpz_mat_clear(gram);
    }
    if (fesetround(original) != 0) return 2;
    flint_cleanup_master();
    printf("{\"suite\":\"lll-controls\",\"rows\":%u,\"failures\":%u}\n", rows, failures);
    return failures != 0;
}
