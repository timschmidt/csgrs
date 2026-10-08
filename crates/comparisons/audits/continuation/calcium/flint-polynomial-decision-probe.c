#include <stdio.h>
#include "ca.h"
#include "ca_poly.h"

int main(void)
{
    ca_ctx_t ctx;
    ca_ctx_init(ctx);
    int failures = 0;
    puts("code,length,zero,one,proper,monic,correct");
    for (int code = 0; code < 81; code++)
    {
        ca_poly_t p, monic;
        ca_t value;
        ca_poly_init(p, ctx);
        ca_poly_init(monic, ctx);
        ca_init(value, ctx);
        int digits = code, any_one = 0, any_unknown = 0;
        int higher_one = 0, higher_unknown = 0, constant = code % 3;
        int length = 0, leading = 0;
        for (int i = 0; i < 4; i++)
        {
            int digit = digits % 3;
            digits /= 3;
            if (digit == 0) ca_zero(value, ctx);
            if (digit == 1) { ca_one(value, ctx); any_one = 1; }
            if (digit == 2) { ca_unknown(value, ctx); any_unknown = 1; }
            if (i > 0 && digit == 1) higher_one = 1;
            if (i > 0 && digit == 2) higher_unknown = 1;
            if (digit != 0) { length = i + 1; leading = digit; }
            ca_poly_set_coeff_ca(p, i, value, ctx);
        }
        truth_t expected_zero = any_one ? T_FALSE : any_unknown ? T_UNKNOWN : T_TRUE;
        truth_t expected_one = constant == 0 || higher_one ? T_FALSE
            : constant == 2 || higher_unknown ? T_UNKNOWN : T_TRUE;
        truth_t zero = ca_poly_check_is_zero(p, ctx);
        truth_t one = ca_poly_check_is_one(p, ctx);
        int proper = ca_poly_is_proper(p, ctx);
        int made_monic = ca_poly_make_monic(monic, p, ctx);
        int correct = p->length == length && zero == expected_zero && one == expected_one
            && proper == !any_unknown && made_monic == (length > 0 && leading == 1);
        failures += !correct;
        printf("%d,%ld,%d,%d,%d,%d,%d\n", code, (long)p->length,
            (int)zero, (int)one, proper, made_monic, correct);
        ca_clear(value, ctx);
        ca_poly_clear(p, ctx);
        ca_poly_clear(monic, ctx);
    }
    ca_ctx_clear(ctx);
    flint_cleanup();
    printf("{\"suite\":\"polynomial-decision\",\"cases\":81,\"checks\":405,\"failed_cases\":%d}\n", failures);
    return failures != 0;
}
