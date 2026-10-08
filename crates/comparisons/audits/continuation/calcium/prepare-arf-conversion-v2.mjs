import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
export function correctedConversionSource() {
 const source=readFileSync('flint-arf-conversion-controls.c','utf8');
 const old=String.raw`  const slong le = !mpz_sgn(m) ? 0 : top - (mpz_cmpabs_ui(scratch, 1) == 0);
  arf_abs_bound_le_2exp_fmpz(fe, x);
  check(fmpz_equal_si(fe, le), "nonstrict exponent bound");
  printf("%ld,", (long)fmpz_get_si(fe));
  arf_abs_bound_lt_2exp_fmpz(fe, x);
  check(fmpz_equal_si(fe, top), "strict exponent bound");
  printf("%ld,", (long)fmpz_get_si(fe));
  slong bsi = arf_abs_bound_lt_2exp_si(x);
  check(bsi == top, "bounded signed exponent");
  printf("%ld],\"powers\":[", (long)bsi);`;
 const replacement=String.raw`  /* The fmpz bounds leave zero unspecified. The signed bound documents
   * -ARF_PREC_EXACT for zero. Preserve its full integer in JSON as text. */
  if (mpz_sgn(m)) {
    const slong le = top - (mpz_cmpabs_ui(scratch, 1) == 0);
    arf_abs_bound_le_2exp_fmpz(fe, x);
    check(fmpz_equal_si(fe, le), "nonstrict exponent bound");
    printf("%ld,", (long)fmpz_get_si(fe));
    arf_abs_bound_lt_2exp_fmpz(fe, x);
    check(fmpz_equal_si(fe, top), "strict exponent bound");
    printf("%ld,", (long)fmpz_get_si(fe));
  } else {
    printf("null,null,");
  }
  slong bsi = arf_abs_bound_lt_2exp_si(x);
  check(bsi == (mpz_sgn(m) ? top : -ARF_PREC_EXACT), "bounded signed exponent");
  printf("\"%ld\"],\"powers\":[", (long)bsi);`;
 assert.equal(source.split(old).length,2);
 return source.replace(old,replacement);
}
if(process.argv.includes('--prepare')) {
 const source=correctedConversionSource();
 writeFileSync('flint-arf-conversion-v2-controls.c',source,{flag:'wx'});
 console.log(JSON.stringify({generated:'flint-arf-conversion-v2-controls.c',bytes:Buffer.byteLength(source),
  changes:'One bounds block: skip unspecified zero fmpz bounds; check documented signed zero sentinel; emit signed bound as exact decimal text. Original source/executable/failed gate remain unchanged.'}));
}
