import copy
from fractions import Fraction
import json
from pathlib import Path
import unittest

from evidence import ENCODING, fraction, validate_response


INPUT = {'bucket': 'One', 'genome': [0] * 8}
CASES = ['normal_form', 'affine_det3', 'sign_zero', 'sign_256', 'interval_64', 'interval_256']


def response():
    queries = []
    for case in CASES:
        answer = ({'normal_form': '1/1'} if case == 'normal_form' else
                  {'interval': ['1/2', '3/2']} if case.startswith('interval_') else
                  {'sign': None if case == 'sign_zero' else 'Positive'})
        queries.append({'query': case, 'score_ns': 7,
                        'samples': [{'sample': i, 'elapsed_ns': ns, 'answer': copy.deepcopy(answer),
                                     'dispatch': [{'layer': 'real', 'operation': case, 'path': 'fixture', 'count': 1}]}
                                    for i, ns in enumerate([1, 11, 7])]})
    return {'schema': 1, 'status': 'ok', 'recipe': {'input': copy.deepcopy(INPUT)},
            'rational_encoding': ENCODING,
            'oracle': {'precision_bits': 1024, 'interval': ['3/4', '5/4'], 'sign': 'Positive'},
            'queries': queries, 'cache_contract_checked': True,
            'cache_contract': {'cold': {'normal': None, 'affine': None, 'sign': None},
                               'refined_sign': 'Positive',
                               'warm': {'normal': '1/1', 'affine': 'Positive', 'sign': 'Positive'}}}


class EvidenceTests(unittest.TestCase):
    def test_fraction_wire_format_has_explicit_sign_and_denominator(self):
        for text in ['0/1', '-1/1', '7/3', '-7/3', '-10000000000000000000000000000000000000001/3']:
            self.assertEqual(fraction(text), Fraction(text))
        for text in ['-2 1/3', '1', '1/-3', '1/0', '1.5', '1 / 3', '+1/3', '01/3', None]:
            with self.assertRaises(ValueError):
                fraction(text)

    def test_full_original_mixed_number_response_is_preserved(self):
        base = Path(__file__).resolve().parents[2] / 'examples/directed_scalar_probe/regressions'
        original = json.loads((base / 'mixed_number_v2.response.json').read_text())
        payload = json.loads((base / 'mixed_number_v2.input.json').read_text())
        self.assertEqual(payload['genome'], [47, 128, 223, 71, 133, 255, 155, 107])
        self.assertEqual(original['recipe']['input'], payload)
        self.assertIn(' ', original['oracle']['interval'][0])

        def convert(text):
            if text is None:
                return None
            if ' ' in text:
                whole, remainder = text.lstrip('-').split(' ')
                value = int(whole) + Fraction(remainder)
                if text.startswith('-'):
                    value = -value
            else:
                value = Fraction(text)
            return f'{value.numerator}/{value.denominator}'

        converted = copy.deepcopy(original)
        converted['rational_encoding'] = ENCODING
        converted['oracle']['interval'] = list(map(convert, original['oracle']['interval']))
        for query in converted['queries']:
            for sample in query['samples']:
                answer = sample['answer']
                if 'normal_form' in answer:
                    answer['normal_form'] = convert(answer['normal_form'])
                if 'interval' in answer:
                    answer['interval'] = list(map(convert, answer['interval']))
        for state in ['cold', 'warm']:
            cache = converted['cache_contract'][state]
            cache['normal'] = convert(cache['normal'])
        self.assertEqual(validate_response(converted, payload, CASES),
                         {query['query']: query['score_ns'] for query in original['queries']})

    def test_unknown_may_become_certified(self):
        self.assertEqual(validate_response(response(), INPUT, CASES), dict.fromkeys(CASES, 7))

    def test_wrong_sign_and_disjoint_interval_are_rejected(self):
        wrong = response(); wrong['queries'][1]['samples'][0]['answer']['sign'] = 'Negative'
        with self.assertRaises(ValueError):
            validate_response(wrong, INPUT, CASES)
        wrong = response(); wrong['queries'][4]['samples'][0]['answer']['interval'] = ['2/1', '3/1']
        with self.assertRaises(ValueError):
            validate_response(wrong, INPUT, CASES)

    def test_available_facts_cannot_be_dropped(self):
        for field, value in [('normal', '1/1'), ('affine', 'Positive'), ('sign', 'Positive')]:
            wrong = response()
            wrong['cache_contract']['cold'][field] = value
            wrong['cache_contract']['warm'][field] = None
            with self.assertRaises(ValueError):
                validate_response(wrong, INPUT, CASES)

    def test_fine_proof_survives_a_zero_refinement_query(self):
        wrong = response(); wrong['cache_contract']['warm']['sign'] = None
        with self.assertRaises(ValueError):
            validate_response(wrong, INPUT, CASES)

    def test_timing_samples_cannot_be_replaced_by_a_best_case(self):
        wrong = response(); wrong['queries'][0]['score_ns'] = 1
        with self.assertRaises(ValueError):
            validate_response(wrong, INPUT, CASES)

    def test_identity_query_inventory_and_answer_types_are_checked(self):
        wrong = response(); wrong['recipe']['input']['genome'][0] = 1
        with self.assertRaises(ValueError):
            validate_response(wrong, INPUT, CASES)
        wrong = response(); wrong['queries'].pop()
        with self.assertRaises(ValueError):
            validate_response(wrong, INPUT, CASES)
        wrong = response(); wrong['queries'][1]['samples'][0]['answer'] = {'normal_form': '1/1'}
        with self.assertRaises(ValueError):
            validate_response(wrong, INPUT, CASES)


if __name__ == '__main__':
    unittest.main()
