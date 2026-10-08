/* Reuse the audited collector helpers; discard its entry point at link time. */
#define main bounded_corpus_main_v80
#include "flint-qqbar-inverse-v80.c"
#undef main

int main(void)
{
    qqbar_t x;
    acb_t z;
    slong p;
    ulong q;
    int ok;
    qqbar_init(x); acb_init(z);
    if (!qqbar_tan_pi(x,1,3360)) abort();
    qqbar_enclosure_raw(z,x,128);
    ok = qqbar_atan_pi(&p,&q,x);
    printf("{\"denominator\":3360,\"degree\":%ld,\"recognized\":%d",(long)qqbar_degree(x),ok);
    if (ok) printf(",\"p\":%ld,\"q\":%lu",(long)p,(unsigned long)q);
    printf(",\"poly\":"); polynomial(QQBAR_POLY(x));
    printf(",\"real\":"); endpoints(acb_realref(z));
    printf(",\"imag\":"); endpoints(acb_imagref(z)); proposal(x,2);
    puts("}");
    qqbar_clear(x); acb_clear(z); flint_cleanup(); return 0;
}
