import copy
import itertools
import random
import unittest

from lexicase import select, select_buckets, verify_selection


def candidate(name, scores, bucket='Pi'):
    return {'id': name, 'bucket': bucket, 'scores': dict(zip(['left', 'right'], scores))}


class FixedOrder:
    def __init__(self, reverse=False):
        self.reverse = reverse

    def shuffle(self, values):
        if self.reverse:
            values.reverse()

    @staticmethod
    def choice(values):
        return values[0]


class LexicaseTests(unittest.TestCase):
    def test_specialists_survive_instead_of_the_average_winner(self):
        rows = [candidate('left-specialist', [10, 0]), candidate('right-specialist', [0, 10]),
                candidate('average-winner', [9, 9])]
        self.assertEqual(select(rows, ['left', 'right'], FixedOrder())['selected'], 'left-specialist')
        self.assertEqual(select(rows, ['left', 'right'], FixedOrder(True))['selected'], 'right-specialist')

    def test_later_case_breaks_a_tie_among_current_survivors(self):
        rows = [candidate('a', [5, 2]), candidate('b', [5, 3]), candidate('c', [4, 9])]
        record = select(rows, ['left', 'right'], FixedOrder())
        self.assertEqual(record['selected'], 'b')
        self.assertEqual([step['survivors'] for step in record['steps']], [['a', 'b'], ['b']])
        verify_selection(rows, ['left', 'right'], record)

    def test_exhaustive_small_matrices_match_lexicographic_maxima(self):
        cases = ['left', 'right']
        for scores in itertools.product(range(3), repeat=6):
            rows = [candidate(str(i), scores[2*i:2*i+2]) for i in range(3)]
            for reverse in [False, True]:
                record = select(rows, cases, FixedOrder(reverse))
                ordered = cases[::-1] if reverse else cases
                vectors = {row['id']: tuple(row['scores'][case] for case in ordered) for row in rows}
                self.assertEqual(vectors[record['selected']], max(vectors.values()))
                verify_selection(rows, cases, record)

    def test_independent_representation_buckets_have_independent_parents(self):
        populations = {'Pi': [candidate('pi', [1, 2])],
                       'Exp': [candidate('exp', [1000000, 2000000], 'Exp')]}
        selected = select_buckets(populations, ['left', 'right'], random.Random(7), 3)
        self.assertEqual([x['selected'] for x in selected['Pi']], ['pi'] * 3)
        self.assertEqual([x['selected'] for x in selected['Exp']], ['exp'] * 3)
        with self.assertRaises(ValueError):
            select(populations['Pi'] + populations['Exp'], ['left', 'right'], random.Random(7))

    def test_seeded_ties_and_population_permutations_replay(self):
        rows = [candidate('a', [0, 0]), candidate('b', [0, 0]), candidate('c', [0, 0])]
        first = select(rows, ['left', 'right'], random.Random(91))
        self.assertEqual(first, select(rows[::-1], ['left', 'right'], random.Random(91)))
        self.assertEqual(len(first['steps']), 2)
        verify_selection(rows, ['left', 'right'], first)

    def test_invalid_scores_and_incomplete_records_are_rejected(self):
        for invalid in [-1, 1.5, True, None, '1']:
            with self.subTest(score=invalid), self.assertRaises(ValueError):
                select([candidate('a', [invalid, 0])], ['left', 'right'], random.Random(0))
        for rows, cases in [([], ['left', 'right']), ([candidate('a', [1, 2])], []),
                            ([candidate('a', [1, 2])], ['left', 'left']),
                            ([candidate('a', [1, 2]), candidate('a', [3, 4])], ['left', 'right']),
                            ([candidate('a', [1])], ['left', 'right'])]:
            with self.assertRaises(ValueError):
                select(rows, cases, random.Random(0))

    def test_tampered_selection_evidence_is_rejected(self):
        rows = [candidate('a', [5, 1]), candidate('b', [4, 8])]
        original = select(rows, ['left', 'right'], FixedOrder())
        variants = []
        wrong = copy.deepcopy(original); wrong['selected'] = 'b'; variants.append(wrong)
        wrong = copy.deepcopy(original); wrong['steps'][0]['worst_score'] += 1; variants.append(wrong)
        wrong = copy.deepcopy(original); wrong['case_order'] = ['left', 'left']; variants.append(wrong)
        wrong = copy.deepcopy(original); wrong['steps'][0]['survivors'].append('b'); variants.append(wrong)
        wrong = copy.deepcopy(original); wrong['bucket'] = 'Exp'; variants.append(wrong)
        for wrong in variants:
            with self.assertRaises(ValueError):
                verify_selection(rows, ['left', 'right'], wrong)


if __name__ == '__main__':
    unittest.main()
