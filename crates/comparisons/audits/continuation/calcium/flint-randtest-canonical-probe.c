#include <stdio.h>
#include "ca.h"

/* Public random constructor must still satisfy the rational carrier invariant.
   The independent control canonicalizes a copy, never the tested value. */
int main(void)
{
    flint_rand_t state;
    ca_ctx_t ctx;
    ca_t x, carrier, control;
    fmpq_t q;
    unsigned noncanonical = 0, wrong_one = 0, controls = 0;
    flint_rand_init(state);
    flint_rand_set_seed(state, 1729, 2718);
    ca_ctx_init(ctx);
    ca_init(x, ctx);
    ca_init(carrier, ctx);
    ca_init(control, ctx);
    fmpq_init(q);
    ca_one(carrier, ctx);
    for (unsigned i = 0; i < 10000; i++)
    {
        ca_randtest_same_nf(x, state, carrier, 4, 4, ctx);
        if (!CA_IS_QQ(x, ctx))
            return 2;
        if (!fmpq_is_canonical(CA_FMPQ(x)))
            noncanonical++;
        fmpq_set(q, CA_FMPQ(x));
        fmpq_canonicalise(q);
        ca_set_fmpq(control, q, ctx);
        truth_t want = fmpq_is_one(q) ? T_TRUE : T_FALSE;
        if (ca_check_is_one(control, ctx) != want)
            return 3;
        controls++;
        if (ca_check_is_one(x, ctx) != want)
        {
            wrong_one++;
            if (wrong_one <= 5)
            {
                printf("wrong_one input=");
                fmpq_print(CA_FMPQ(x));
                printf(" canonical=");
                fmpq_print(q);
                printf("\n");
            }
        }
    }
    printf("cases=10000 canonical_controls=%u noncanonical=%u wrong_one=%u\n",
        controls, noncanonical, wrong_one);
    fmpq_clear(q);
    ca_clear(x, ctx);
    ca_clear(carrier, ctx);
    ca_clear(control, ctx);
    ca_ctx_clear(ctx);
    flint_rand_clear(state);
    flint_cleanup();
    return noncanonical || wrong_one ? 1 : 0;
}
