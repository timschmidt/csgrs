"""Reconstruct exact integer winding constraints from diagnostic merge events."""
from collections import deque
import json
from pathlib import Path
import sys

source, destination = map(Path, sys.argv[1:])
assert not destination.exists()
records = []
current = None
active = False
edge = None
for line in source.read_text().splitlines():
    if line.startswith('TRACE arrangement '):
        current = {'operands': int(line.split()[2]), 'points': {}, 'facets': {},
                   'sheets': {}, 'merges': [], 'native_failures': []}
        records.append(current)
    elif line.startswith('TRACE point '):
        _, _, index, value = line.split(' ', 3)
        current['points'][int(index)] = json.loads(value)
    elif line.startswith('TRACE facet '):
        _, _, index, value = line.split(' ', 3)
        current['facets'][int(index)] = json.loads(value)
    elif line.startswith('TRACE sheet '):
        _, _, index, face, value = line.split(' ', 4)
        current['sheets'][int(index)] = {'face': int(face), 'delta': json.loads(value)}
    elif line == 'TRACE unions_start':
        active = True
    elif line == 'TRACE unions_end':
        active = False
    elif line.startswith('TRACE edge '):
        edge = json.loads(line.removeprefix('TRACE edge '))
    elif active and line.startswith('TRACE merge '):
        _, _, left, right = line.split()
        current['merges'].append({'left': int(left), 'right': int(right), 'edge': edge})
    elif line.startswith('TRACE inconsistent '):
        current['native_failures'].append(line)

assert len(records) == 2
results = []
for record in records:
    assert len(record['native_failures']) == 1
    dimension = record['operands']
    count = 2 * len(record['sheets'])
    zero = [0] * dimension
    parents = list(range(count))
    weights = [zero.copy() for _ in range(count)]
    graph = [[] for _ in range(count)]

    def add(a, b): return [x+y for x, y in zip(a, b)]
    def sub(a, b): return [x-y for x, y in zip(a, b)]

    def find(node):
        path = []
        while parents[node] != node:
            path.append(node)
            node = parents[node]
        root = node
        total = zero.copy()
        for node in reversed(path):
            total = add(weights[node], total)
            parents[node], weights[node] = root, total
        return root, total

    def relation(left, right, delta, witness):
        a, wa = find(left)
        b, wb = find(right)
        if a == b:
            if sub(wb, wa) != delta: return sub(wb, wa)
        else:
            parents[b], weights[b] = a, sub(add(delta, wa), wb)
        graph[left].append((right, delta, witness))
        graph[right].append((left, [-x for x in delta], witness))
        return None

    for sheet, row in record['sheets'].items():
        assert relation(2*sheet, 2*sheet+1, row['delta'], {'sheet': sheet, **row}) is None
    failure = None
    for index, merge in enumerate(record['merges']):
        difference = relation(merge['left'], merge['right'], zero, {'merge': index, **merge})
        if difference is None: continue
        queue = deque([merge['left']])
        previous = {merge['left']: None}
        while queue and merge['right'] not in previous:
            node = queue.popleft()
            for other, delta, witness in graph[node]:
                if other not in previous:
                    previous[other] = (node, delta, witness)
                    queue.append(other)
        assert merge['right'] in previous
        path = []
        total = zero.copy()
        node = merge['right']
        while node != merge['left']:
            before, delta, witness = previous[node]
            path.append({'from': before, 'to': node, 'delta': delta, 'witness': witness})
            total = add(total, delta)
            node = before
        path.reverse()
        assert total == difference and any(total)
        failure = {'merge_index': index, **merge, 'existing_difference': difference,
                   'cycle_path': path,
                   'left_sheet': record['sheets'][merge['left']//2],
                   'right_sheet': record['sheets'][merge['right']//2],
                   'edge_points': [record['points'][point] for point in merge['edge']]}
        break
    results.append({'operand_count': dimension, 'sheet_count': len(record['sheets']),
                    'merge_count': len(record['merges']), 'first_conflict': failure,
                    'native_failures': record['native_failures']})
destination.write_text(json.dumps({'results': results}, indent=2) + '\n')
for result in results:
    failure = result['first_conflict']
    print(json.dumps({'operand_count': result['operand_count'],
                      'first_conflict': None if failure is None else {
                          key: value for key, value in failure.items() if key != 'cycle_path'}}))
