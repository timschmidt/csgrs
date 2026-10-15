"""Independent exact-rational checks of retained scalar-probe responses."""
from fractions import Fraction
import re


ENCODING = 'signed-numerator/positive-denominator-v1'


def fraction(text):
    if not isinstance(text, str) or not re.fullmatch(r'-?(?:0|[1-9][0-9]*)/[1-9][0-9]*', text):
        raise ValueError('invalid signed-numerator/positive-denominator fraction')
    return Fraction(text)


def validate_response(response, input_value, cases):
    if response.get('status') != 'ok' or response.get('schema') != 1:
        raise ValueError('probe did not report a successful schema-1 response')
    if response.get('rational_encoding') != ENCODING:
        raise ValueError('probe rational wire encoding is missing or unsupported')
    if response['recipe']['input'] != input_value:
        raise ValueError('probe changed the retained input identity')
    lo, hi = map(fraction, response['oracle']['interval'])
    if not lo <= hi or not (lo > 0 or hi < 0):
        raise ValueError('independent oracle did not isolate a nonzero sign')
    expected_sign = 'Positive' if lo > 0 else 'Negative'
    if response['oracle']['sign'] != expected_sign:
        raise ValueError('oracle sign disagrees with its exact rational bounds')
    if response['oracle']['precision_bits'] != 1024 or response.get('cache_contract_checked') is not True:
        raise ValueError('probe omitted a declared correctness check')
    if [row['query'] for row in response['queries']] != list(cases):
        raise ValueError('probe query inventory changed')
    scores = {}
    for row in response['queries']:
        samples = row['samples']
        if len(samples) != 3 or [sample['sample'] for sample in samples] != [0, 1, 2]:
            raise ValueError('missing or reordered timing samples')
        times = [sample['elapsed_ns'] for sample in samples]
        if any(type(value) is not int or value < 0 for value in times):
            raise ValueError('invalid timing sample')
        if row['score_ns'] != sorted(times)[1]:
            raise ValueError('fitness is not the retained three-sample median')
        scores[row['query']] = row['score_ns']
        for sample in samples:
            answer = sample['answer']
            expected_key = ('normal_form' if row['query'] == 'normal_form' else
                            'interval' if row['query'].startswith('interval_') else 'sign')
            if set(answer) != {expected_key}:
                raise ValueError('answer does not match its declared query')
            if 'sign' in answer and answer['sign'] not in [None, expected_sign]:
                raise ValueError('certificate contradicts the independent sign')
            if answer.get('normal_form') is not None and not lo <= fraction(answer['normal_form']) <= hi:
                raise ValueError('normal form contradicts the independent interval')
            if 'interval' in answer:
                a, b = map(fraction, answer['interval'])
                if a > b or b < lo or a > hi:
                    raise ValueError('certified interval contradicts independent enclosure')
            for trace in sample['dispatch']:
                if type(trace['count']) is not int or trace['count'] < 0:
                    raise ValueError('invalid dispatch count')
    cache = response['cache_contract']
    for name in ['normal', 'affine', 'sign']:
        cold, warm = cache['cold'][name], cache['warm'][name]
        if cold is not None and warm != cold:
            raise ValueError('available cold fact was weakened or contradicted')
        if name == 'normal':
            if warm is not None and not lo <= fraction(warm) <= hi:
                raise ValueError('warm normal form contradicts independent enclosure')
        elif warm not in [None, expected_sign]:
            raise ValueError('warm sign contradicts independent enclosure')
    fine = cache['refined_sign']
    if cache['cold']['sign'] is not None and fine != cache['cold']['sign']:
        raise ValueError('refinement discarded a known sign')
    if fine is not None and cache['warm']['sign'] != fine:
        raise ValueError('zero-refinement query discarded a prior finer proof')
    return scores


if __name__ == '__main__':
    import argparse
    import json
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--response', type=Path, required=True)
    parser.add_argument('--inventory', type=Path, required=True)
    args = parser.parse_args()
    scores = validate_response(json.loads(args.response.read_text()),
                               json.loads(args.input.read_text()),
                               json.loads(args.inventory.read_text())['queries'])
    print(json.dumps({'verified': True, 'query_scores': scores}))
