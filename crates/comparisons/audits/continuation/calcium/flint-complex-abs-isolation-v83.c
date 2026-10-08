#define main symbolic_boundary_unused_main
#include "flint-symbolic-boundary-fixed-v83.c"
#undef main

int main(void)
{
    qqbar_t x, y;
    qqbar_init(x); qqbar_init(y);
    qqbar_exp_pi_i(x, 1, 12);
    qqbar_mul_si(x, x, -7);
    qqbar_div_ui(x, x, 3);
    fputs("before abs\n", stderr); fflush(stderr);
    qqbar_abs(y, x);
    printf("{\"family\":\"isolated-abs\""); value(y); puts("}");
    qqbar_clear(x); qqbar_clear(y); flint_cleanup();
    puts("{\"terminal\":true,\"rows\":1}");
    return 0;
}
