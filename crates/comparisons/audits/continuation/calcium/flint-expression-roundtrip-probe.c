/* Public, well-formed scalar round trips; parsing is not value preservation. */
#include <stdio.h>
#include "ca.h"
#include "fexpr.h"

static const char *outcome(truth_t t)
{
    return t == T_TRUE ? "Equal" : t == T_FALSE ? "NotEqual" : "Unknown";
}

int main(void)
{
    const char *inputs[] = {"two-thirds", "sqrt-two", "pi", "one-plus-i", "pi-plus-i", "one-plus-pi-i"};
    const char *operations[] = {"identity", "arg", "csgn", "exp"};
    int cases = 0, parse_failed = 0, lost_value = 0, unequal = 0;
    puts("input,operation,serialization,source_unknown,parsed,restored_unknown,equality");
    for (int input = 0; input < 6; input++) for (int operation = 0; operation < 4; operation++) for (int mode = 0; mode < 2; mode++)
    {
        ca_ctx_t ctx, restored_ctx;
        ca_t x, value, restored, back, t;
        fexpr_t expression;
        ca_ctx_init(ctx); ca_ctx_init(restored_ctx);
        ca_ctx_set_option(ctx, CA_OPT_PREC_LIMIT, 256);
        ca_ctx_set_option(restored_ctx, CA_OPT_PREC_LIMIT, 256);
        ca_init(x, ctx); ca_init(value, ctx); ca_init(t, ctx); ca_init(back, ctx);
        ca_init(restored, restored_ctx); fexpr_init(expression);
        switch (input)
        {
            case 0: ca_set_ui(x, 2, ctx); ca_div_ui(x, x, 3, ctx); break;
            case 1: ca_set_ui(x, 2, ctx); ca_sqrt(x, x, ctx); break;
            case 2: ca_pi(x, ctx); break;
            case 3: ca_set_d_d(x, 1.0, 1.0, ctx); break;
            case 4: ca_pi(x, ctx); ca_i(t, ctx); ca_add(x, x, t, ctx); break;
            default: ca_pi(x, ctx); ca_i(t, ctx); ca_mul(x, x, t, ctx); ca_add_ui(x, x, 1, ctx); break;
        }
        switch (operation)
        {
            case 0: ca_set(value, x, ctx); break;
            case 1: ca_arg(value, x, ctx); break;
            case 2: ca_csgn(value, x, ctx); break;
            default: ca_exp(value, x, ctx); break;
        }
        ca_get_fexpr(expression, value, mode ? CA_FEXPR_SERIALIZATION : 0, ctx);
        int parsed = ca_set_fexpr(restored, expression, restored_ctx);
        // Compare in the source context using the same independent expression;
        // no cross-context ca_t aliasing or access to private extension objects.
        int reparsed = ca_set_fexpr(back, expression, ctx);
        truth_t equal = reparsed ? ca_check_equal(value, back, ctx) : T_UNKNOWN;
        int source_unknown = ca_is_unknown(value, ctx);
        int restored_unknown = ca_is_unknown(restored, restored_ctx);
        cases++;
        parse_failed += !parsed || !reparsed;
        lost_value += !source_unknown && restored_unknown;
        unequal += equal == T_FALSE;
        printf("%s,%s,%d,%d,%d,%d,%s\n", inputs[input], operations[operation], mode,
            source_unknown, parsed, restored_unknown, outcome(equal));
        fexpr_clear(expression); ca_clear(restored, restored_ctx);
        ca_clear(x, ctx); ca_clear(value, ctx); ca_clear(t, ctx); ca_clear(back, ctx);
        ca_ctx_clear(restored_ctx); ca_ctx_clear(ctx);
    }
    printf("{\"suite\":\"expression-roundtrip\",\"cases\":%d,\"parse_failed\":%d,\"lost_value\":%d,\"unequal\":%d}\n",
        cases, parse_failed, lost_value, unequal);
    flint_cleanup();
    return parse_failed || lost_value || unequal ? 1 : 0;
}
