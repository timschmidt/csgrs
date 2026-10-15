"""Worst-case lexicase selection with independent, replayable representation buckets."""


def validate(population, cases):
    if not population or not isinstance(cases, (list, tuple)) or not cases:
        raise ValueError('population and cases must be nonempty sequences')
    if any(not isinstance(case, str) or not case for case in cases):
        raise ValueError('case names must be nonempty strings')
    if len(set(cases)) != len(cases):
        raise ValueError('case names must be distinct')
    if any(not isinstance(row, dict) or not {'id', 'bucket', 'scores'} <= set(row)
           or not isinstance(row['scores'], dict) for row in population):
        raise ValueError('malformed candidate record')
    if any(not isinstance(row['id'], str) or not row['id'] for row in population):
        raise ValueError('candidate identifiers must be nonempty strings')
    rows = sorted(population, key=lambda row: row['id'])
    if len({row['id'] for row in rows}) != len(rows):
        raise ValueError('duplicate candidate identifier')
    if len({row['bucket'] for row in rows}) != 1:
        raise ValueError('representation buckets cannot share a selection pool')
    for row in rows:
        if not isinstance(row['bucket'], str) or not row['bucket']:
            raise ValueError('bucket names must be nonempty strings')
        if set(row['scores']) != set(cases):
            raise ValueError('every candidate needs every declared case score')
        if any(type(score) is not int or score < 0 for score in row['scores'].values()):
            raise ValueError('scores must be nonnegative integers')
    return rows


def select(population, cases, random):
    """Maximize observed cost on successive shuffled cases, then break ties."""
    survivors = validate(population, cases)
    order = list(cases)
    random.shuffle(order)
    steps = []
    for case in order:
        worst = max(row['scores'][case] for row in survivors)
        survivors = [row for row in survivors if row['scores'][case] == worst]
        steps.append({'case': case, 'worst_score': worst,
                      'survivors': [row['id'] for row in survivors]})
        if len(survivors) == 1:
            break
    chosen = random.choice(survivors)
    return {'bucket': chosen['bucket'], 'case_order': order, 'steps': steps,
            'selected': chosen['id']}


def select_buckets(populations, cases, random, parents_per_bucket):
    if type(parents_per_bucket) is not int or parents_per_bucket <= 0:
        raise ValueError('parent count must be a positive integer')
    if not populations:
        raise ValueError('representation populations cannot be empty')
    for bucket, rows in populations.items():
        validate(rows, cases)
        if any(row['bucket'] != bucket for row in rows):
            raise ValueError('candidate is stored in the wrong representation bucket')
    return {bucket: [select(rows, cases, random) for _ in range(parents_per_bucket)]
            for bucket, rows in sorted(populations.items())}


def verify_selection(population, cases, record):
    """Replay all filtering steps from retained scores, independent of PRNG state."""
    survivors = validate(population, cases)
    if record['bucket'] != survivors[0]['bucket']:
        raise ValueError('selection bucket mismatch')
    order = record['case_order']
    if len(order) != len(cases) or set(order) != set(cases):
        raise ValueError('case order is not a complete permutation')
    expected = []
    for case in order:
        worst = max(row['scores'][case] for row in survivors)
        survivors = [row for row in survivors if row['scores'][case] == worst]
        expected.append({'case': case, 'worst_score': worst,
                         'survivors': [row['id'] for row in survivors]})
        if len(survivors) == 1:
            break
    if expected != record['steps']:
        raise ValueError('selection filtering trace does not replay')
    if record['selected'] not in {row['id'] for row in survivors}:
        raise ValueError('selected candidate did not survive')
