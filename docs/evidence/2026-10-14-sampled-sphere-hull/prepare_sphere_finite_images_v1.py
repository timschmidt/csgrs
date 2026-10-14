"""Exhaustively validate small finite-field images for all 64 admitted orders."""
import hashlib
import json
from math import isqrt
from pathlib import Path

from check_sphere_trace_cyclotomic_v1 import cyclotomic

E = Path(__file__).resolve().parent
OUT = E / 'sphere-finite-images-v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prime(value):
    return value >= 2 and all(value % divisor for divisor in range(2, isqrt(value) + 1))


def main():
    assert not OUT.exists()
    OUT.mkdir()
    rows = []
    for order in range(4, 257, 4):
        modulus = cyclotomic(order)
        start = 65536 // order + 1
        candidates = [(k * order + 1) for k in range(start, start + 256)]
        p = next(value for value in candidates if prime(value))
        assert 65536 < p < 131072
        divisors = [d for d in range(1, order) if order % d == 0]
        root = next(value for generator in range(2, 258)
                    if (value := pow(generator, (p - 1) // order, p))
                    and pow(value, order, p) == 1
                    and all(pow(value, divisor, p) != 1 for divisor in divisors))
        evaluated = sum(coefficient * pow(root, exponent, p)
                        for exponent, coefficient in enumerate(modulus)) % p
        assert evaluated == 0
        rows.append({'order': order, 'prime': p, 'root': root,
                     'cyclotomic_polynomial_low_to_high': modulus,
                     'exact_integer_polynomial_evaluation_mod_prime': evaluated,
                     'proper_divisor_powers': {str(d): pow(root, d, p) for d in divisors}})
    table = '// Generated and independently checked by prepare_sphere_finite_images_v1.py.\n'
    table += '// Entry i supplies a prime and a primitive root of order 4*(i+1).\n'
    table += 'const PRIME_IMAGES: [(u32, u32); 64] = [\n'
    table += ''.join(f"    ({row['prime']}, {row['root']}), // {row['order']}\n" for row in rows)
    table += '];\n'
    (OUT / 'images.rs').write_text(table)
    value = {'complete': True, 'generator_sha256': sha(__file__),
             'independent_integer_oracle_sha256': sha(E / 'check_sphere_trace_cyclotomic_v1.py'),
             'images_sha256': sha(OUT / 'images.rs'), 'orders_checked': 64, 'rows': rows,
             'interpretation': 'Each prime is checked by exhaustive trial division; each root has the specified exact multiplicative order and directly annihilates the independently constructed integer cyclotomic polynomial. This validates the finite constant table, not a production expression evaluator or its resource bounds.',
             'runtime_admitted': False, 'plan_complete': False}
    (OUT / 'result.json').write_text(json.dumps(value, indent=2) + '\n')
    print(json.dumps({'complete': True, 'orders_checked': 64,
                      'maximum_prime': max(row['prime'] for row in rows),
                      'result_sha256': sha(OUT / 'result.json')}))


if __name__ == '__main__':
    main()
