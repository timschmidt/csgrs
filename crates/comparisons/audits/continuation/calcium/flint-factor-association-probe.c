/* Distinct positive prime factors make exponent association independently exact. */
#include <stdio.h>
#include "ca.h"

int main(void)
{
    const int permutations[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    const int primes[3] = {2,3,5};
    ca_ctx_t ctx;
    ca_factor_t factors;
    ca_t base, exponent, expected, actual, term;
    int cases = 0, initial_failed = 0, updated_failed = 0;
    ca_ctx_init(ctx); ca_factor_init(factors, ctx);
    ca_init(base, ctx); ca_init(exponent, ctx); ca_init(expected, ctx);
    ca_init(actual, ctx); ca_init(term, ctx);
    puts("permutation,index,delta,initial_equal,updated_equal");
    for (int p = 0; p < 6; p++) for (int index = 0; index < 3; index++) for (int delta = -2; delta <= 2; delta++)
    {
        ca_factor_one(factors, ctx);
        ca_one(expected, ctx);
        for (int i = 0; i < 3; i++)
        {
            ca_set_si(base, primes[permutations[p][i]], ctx);
            ca_set_si(exponent, i+1, ctx);
            ca_factor_insert(factors, base, exponent, ctx);
            ca_pow(term, base, exponent, ctx); ca_mul(expected, expected, term, ctx);
        }
        ca_factor_get_ca(actual, factors, ctx);
        int initial_equal = ca_check_equal(actual, expected, ctx) == T_TRUE;
        ca_set_si(base, primes[permutations[p][index]], ctx);
        ca_set_si(exponent, delta, ctx);
        ca_factor_insert(factors, base, exponent, ctx);
        // Independent product update uses the requested base, not factor storage.
        ca_pow(term, base, exponent, ctx); ca_mul(expected, expected, term, ctx);
        ca_factor_get_ca(actual, factors, ctx);
        int updated_equal = ca_check_equal(actual, expected, ctx) == T_TRUE;
        initial_failed += !initial_equal; updated_failed += !updated_equal; cases++;
        printf("%d,%d,%d,%d,%d\n", p, index, delta, initial_equal, updated_equal);
    }
    printf("{\"suite\":\"factor-association\",\"cases\":%d,\"initial_failed\":%d,\"updated_failed\":%d}\n",
        cases, initial_failed, updated_failed);
    ca_clear(base, ctx); ca_clear(exponent, ctx); ca_clear(expected, ctx);
    ca_clear(actual, ctx); ca_clear(term, ctx); ca_factor_clear(factors, ctx);
    ca_ctx_clear(ctx); flint_cleanup();
    return initial_failed || updated_failed ? 1 : 0;
}
