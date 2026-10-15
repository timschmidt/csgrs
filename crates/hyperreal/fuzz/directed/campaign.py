"""Bounded adaptive scalar search; every probe, selection, and mutation is retained."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import random
import re
import subprocess
import sys
import time

from evidence import validate_response
from lexicase import select_buckets, verify_selection


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    with path.open('x') as output:
        output.write(json.dumps(value, indent=2, sort_keys=True) + '\n')


def identity(input_value):
    return hashlib.sha256(json.dumps(input_value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def execute(binary, arguments, prefix, timeout, expected_hash):
    if digest(binary) != expected_hash:
        raise ValueError('probe executable changed during the campaign')
    stdout, stderr = prefix.with_suffix('.stdout'), prefix.with_suffix('.stderr')
    start = time.monotonic_ns()
    with stdout.open('xb') as out, stderr.open('xb') as err:
        try:
            code = subprocess.run([str(binary), *map(str, arguments)], stdout=out, stderr=err,
                                  timeout=timeout, check=False).returncode
            status = 'exited'
        except subprocess.TimeoutExpired:
            code, status = None, 'timeout'
    record = {'command': [str(binary), *map(str, arguments)], 'status': status, 'exit_code': code,
              'process_elapsed_ns': time.monotonic_ns() - start,
              'stdout': stdout.name, 'stderr': stderr.name,
              'stdout_sha256': digest(stdout), 'stderr_sha256': digest(stderr)}
    write(prefix.with_suffix('.process.json'), record)
    if code != 0 or status != 'exited':
        raise ValueError(f'probe {status} with exit code {code}; retained at {prefix}')
    if stderr.stat().st_size:
        raise ValueError(f'probe emitted diagnostics; retained at {stderr}')
    if stdout.stat().st_size > 32 * 1024 * 1024:
        raise ValueError('probe response exceeds 32 MiB')
    return json.loads(stdout.read_text()), record


def next_population(bucket, rows, selections, rng, size):
    by_id = {row['id']: row for row in rows}
    chosen = [by_id[selection['selected']] for selection in selections]
    output = {}
    for parent in chosen:
        payload = {'bucket': bucket, 'genome': parent['genome']}
        output[parent['id']] = {'input': payload, 'origin': 'retained', 'parent_id': parent['id']}
    attempts = 0
    while len(output) < size:
        attempts += 1
        if attempts > 128 * size:
            raise ValueError('distinct mutation budget exhausted')
        parent = rng.choice(chosen)
        genome = parent['genome'].copy()
        index, mask = rng.randrange(8), rng.randrange(1, 256)
        genome[index] ^= mask
        payload = {'bucket': bucket, 'genome': genome}
        key = identity(payload)
        if key not in output:
            output[key] = {'input': payload, 'origin': 'mutation', 'parent_id': parent['id'],
                           'mutation': {'index': index, 'xor_mask': mask}}
    return list(output.values())


def initial_populations(buckets, rngs, size):
    populations = {}
    boundary = [[0] * 8, [255] * 8, [0, 1, 127, 3, 5, 2, 128, 77], [4, 31, 0, 0, 0, 1, 0, 0]]
    for bucket in buckets:
        values = {tuple(genome) for genome in boundary}
        attempts = 0
        while len(values) < size:
            attempts += 1
            if attempts > 128 * size:
                raise ValueError('distinct initial-population budget exhausted')
            values.add(tuple(rngs[bucket].randrange(256) for _ in range(8)))
        populations[bucket] = [{'input': {'bucket': bucket, 'genome': list(genome)},
                                'origin': 'initial', 'parent_id': None} for genome in sorted(values)]
    return populations


def run(args):
    if not 4 <= args.population <= 1024 or not 1 <= args.parents < args.population:
        raise ValueError('population must be 4..1024 and parents must be 1..population-1')
    if not 1 <= args.rounds <= 1000 or not 1 <= args.timeout <= 3600:
        raise ValueError('rounds must be 1..1000 and timeout 1..3600 seconds')
    binary, output = args.binary.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    binary_hash = digest(binary)
    protocol = {'schema': 1, 'binary': str(binary), 'binary_sha256': binary_hash,
                'python': sys.version, 'seed': args.seed, 'rounds': args.rounds,
                'population_per_representation': args.population, 'parents_per_representation': args.parents,
                'process_timeout_seconds': args.timeout,
                'sanitizer_environment': {name: os.environ.get(name) for name in ['ASAN_OPTIONS', 'LSAN_OPTIONS']},
                'sources_sha256': {name: digest(Path(__file__).with_name(name))
                                   for name in ['campaign.py', 'lexicase.py', 'evidence.py']},
                'score_contract': 'Maximize median query-body elapsed nanoseconds within each independent representation population.',
                'no_resumption_or_extension': True, 'competitive_benchmark': False,
                'P38_complete': False, 'plan_complete': False}
    write(output / 'protocol.json', protocol)
    completed, generations = 0, []
    try:
        listing, _ = execute(binary, ['--list'], output / 'inventory', args.timeout, binary_hash)
        cases = listing['queries']
        buckets = sorted(row['name'] for row in listing['representations'])
        if len(set(buckets)) != 22 or any(not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', b) for b in buckets):
            raise ValueError('expected all 22 distinct representation buckets')
        if any(row['name'] != row['actual_class'] for row in listing['representations']):
            raise ValueError('fixture inventory did not reach its declared private classes')
        seeds = {bucket: int(hashlib.sha256(f'{args.seed}:{bucket}'.encode()).hexdigest(), 16) for bucket in buckets}
        rngs = {bucket: random.Random(seed) for bucket, seed in seeds.items()}
        write(output / 'bucket-seeds.json', seeds)
        populations = initial_populations(buckets, rngs, args.population)
        for generation in range(args.rounds):
            write(output / f'round-{generation:03}-planned.json', populations)
            measured, selection = {}, {}
            for bucket in buckets:
                measured[bucket] = []
                for member in populations[bucket]:
                    payload = member['input']
                    key = identity(payload)
                    stem = output / f'round-{generation:03}-{bucket}-{key}'
                    input_path = stem.with_suffix('.input.json')
                    write(input_path, payload)
                    response, process = execute(binary, [input_path], stem, args.timeout, binary_hash)
                    scores = validate_response(response, payload, cases)
                    row = {'id': key, 'bucket': bucket, 'genome': payload['genome'], 'scores': scores,
                           'input_file': input_path.name, 'input_sha256': digest(input_path),
                           'response_file': process['stdout'], 'response_sha256': process['stdout_sha256']}
                    measured[bucket].append(row)
                    completed += 1
                    with (output / 'completed-probes.jsonl').open('a') as journal:
                        journal.write(json.dumps({'round': generation, **row}, sort_keys=True) + '\n')
                selection[bucket] = select_buckets({bucket: measured[bucket]}, cases, rngs[bucket], args.parents)[bucket]
                for record in selection[bucket]:
                    verify_selection(measured[bucket], cases, record)
            record = {'round': generation, 'populations': measured, 'selections': selection}
            write(output / f'round-{generation:03}-selection.json', record)
            generations.append(record)
            if generation + 1 < args.rounds:
                populations = {bucket: next_population(bucket, measured[bucket], selection[bucket], rngs[bucket], args.population)
                               for bucket in buckets}
        final = generations[-1]
        retained = {}
        for bucket in buckets:
            rows = {row['id']: row for row in final['populations'][bucket]}
            retained[bucket] = [{'bucket': bucket, 'genome': rows[key]['genome']}
                                for key in sorted({r['selected'] for r in final['selections'][bucket]})]
        write(output / 'selected-inputs.json', retained)
        result = {'complete': True, 'rounds': args.rounds, 'representations': len(buckets),
                  'completed_probes': completed, 'selection_events': args.rounds * len(buckets) * args.parents,
                  'all_selection_traces_replayed': True,
                  'artifacts_sha256': {p.name: digest(p) for p in output.iterdir() if p.is_file()},
                  'competitive_benchmark_complete': False, 'P38_complete': False, 'plan_complete': False}
        write(output / 'result.json', result)
        print(json.dumps({key: value for key, value in result.items() if key != 'artifacts_sha256'}))
    except Exception as error:
        write(output / 'failure.json', {'complete': False, 'error': str(error), 'completed_probes': completed,
                                       'artifacts_sha256': {p.name: digest(p) for p in output.iterdir() if p.is_file()},
                                       'P38_complete': False, 'plan_complete': False})
        raise


def verify(output):
    result = json.loads((output / 'result.json').read_text())
    if not result['complete'] or (output / 'failure.json').exists():
        raise ValueError('campaign did not complete')
    actual_artifacts = {p.name for p in output.iterdir() if p.is_file() and p.name != 'result.json'}
    if set(result['artifacts_sha256']) != actual_artifacts:
        raise ValueError('campaign artifact inventory changed')
    for name, expected in result['artifacts_sha256'].items():
        if digest(output / name) != expected:
            raise ValueError(f'changed campaign artifact: {name}')
    protocol = json.loads((output / 'protocol.json').read_text())
    listing = json.loads((output / 'inventory.stdout').read_text())
    cases = listing['queries']
    buckets = sorted(row['name'] for row in listing['representations'])
    if len(buckets) != 22 or len(set(buckets)) != 22 or any(
            row['name'] != row['actual_class'] for row in listing['representations']):
        raise ValueError('representation inventory changed')
    if result['rounds'] != protocol['rounds'] or result['representations'] != 22:
        raise ValueError('declared campaign dimensions changed')
    for name, expected_hash in protocol['sources_sha256'].items():
        if digest(Path(__file__).with_name(name)) != expected_hash:
            raise ValueError('campaign source differs from its recorded version')
    seeds = {bucket: int(hashlib.sha256(f"{protocol['seed']}:{bucket}".encode()).hexdigest(), 16)
             for bucket in buckets}
    if json.loads((output / 'bucket-seeds.json').read_text()) != seeds:
        raise ValueError('independent representation seeds changed')
    rngs = {bucket: random.Random(seed) for bucket, seed in seeds.items()}
    expected_plan = initial_populations(buckets, rngs, protocol['population_per_representation'])
    count = 0
    selection_count = 0
    journal = []
    prior = None
    for generation in range(protocol['rounds']):
        planned = json.loads((output / f'round-{generation:03}-planned.json').read_text())
        record = json.loads((output / f'round-{generation:03}-selection.json').read_text())
        if planned != expected_plan:
            raise ValueError('planned population does not replay from independent random streams')
        if record['round'] != generation or any(set(value) != set(buckets) for value in [
                planned, record['populations'], record['selections']]):
            raise ValueError('generation omitted or replaced a representation bucket')
        for bucket, rows in record['populations'].items():
            if len(rows) != protocol['population_per_representation']:
                raise ValueError('population was shrunk')
            for member, row in zip(planned[bucket], rows, strict=True):
                payload = json.loads((output / row['input_file']).read_text())
                if payload != member['input'] or identity(payload) != row['id']:
                    raise ValueError('input identity changed')
                if row['bucket'] != bucket or row['genome'] != payload['genome']:
                    raise ValueError('measured candidate identity differs from its input')
                if digest(output / row['input_file']) != row['input_sha256'] or digest(
                        output / row['response_file']) != row['response_sha256']:
                    raise ValueError('candidate input or response digest changed')
                stem = row['input_file'].removesuffix('.input.json')
                process = json.loads((output / (stem + '.process.json')).read_text())
                if process['status'] != 'exited' or process['exit_code'] != 0:
                    raise ValueError('retained probe did not exit successfully')
                if process['stdout'] != row['response_file'] or process['stdout_sha256'] != row['response_sha256']:
                    raise ValueError('probe process output differs from its recorded response')
                if (output / process['stderr']).read_bytes() or digest(output / process['stderr']) != process['stderr_sha256']:
                    raise ValueError('retained probe diagnostics changed or were ignored')
                response = json.loads((output / row['response_file']).read_text())
                if validate_response(response, payload, cases) != row['scores']:
                    raise ValueError('retained fitness differs from raw samples')
                if prior is not None:
                    parent = {r['id']: r for r in prior['populations'][bucket]}[member['parent_id']]
                    selected = {r['selected'] for r in prior['selections'][bucket]}
                    if parent['id'] not in selected:
                        raise ValueError('mutation or retention ignored selected parents')
                    genome = parent['genome'].copy()
                    if member['origin'] == 'mutation':
                        mutation = member['mutation']
                        if not 0 <= mutation['index'] < 8 or not 1 <= mutation['xor_mask'] <= 255:
                            raise ValueError('mutation is outside its declared domain')
                        genome[mutation['index']] ^= mutation['xor_mask']
                    elif member['origin'] != 'retained':
                        raise ValueError('unknown descendant origin')
                    if genome != payload['genome']:
                        raise ValueError('descendant does not replay from selected parent')
                count += 1
                journal.append({'round': generation, **row})
            if len(record['selections'][bucket]) != protocol['parents_per_representation']:
                raise ValueError('parent-selection budget was not completed exactly')
            for selection in record['selections'][bucket]:
                verify_selection(rows, cases, selection)
                selection_count += 1
            expected_selection = select_buckets({bucket: rows}, cases, rngs[bucket],
                                                protocol['parents_per_representation'])[bucket]
            if record['selections'][bucket] != expected_selection:
                raise ValueError('selection does not replay from its independent random stream')
        prior = record
        if generation + 1 < protocol['rounds']:
            expected_plan = {bucket: next_population(bucket, record['populations'][bucket],
                             record['selections'][bucket], rngs[bucket], protocol['population_per_representation'])
                             for bucket in buckets}
    expected = protocol['rounds'] * 22 * protocol['population_per_representation']
    if count != expected or result['completed_probes'] != count:
        raise ValueError('campaign budget was not completed exactly')
    if selection_count != protocol['rounds'] * 22 * protocol['parents_per_representation'] or result['selection_events'] != selection_count:
        raise ValueError('selection-event inventory changed')
    actual_journal = [json.loads(line) for line in (output / 'completed-probes.jsonl').read_text().splitlines()]
    if actual_journal != journal:
        raise ValueError('completed-probe journal differs from the generation records')
    selected = {}
    for bucket in buckets:
        rows = {row['id']: row for row in prior['populations'][bucket]}
        selected[bucket] = [{'bucket': bucket, 'genome': rows[key]['genome']}
                            for key in sorted({r['selected'] for r in prior['selections'][bucket]})]
    if json.loads((output / 'selected-inputs.json').read_text()) != selected:
        raise ValueError('retained input inventory differs from final selected parents')
    print(json.dumps({'verified': True, 'completed_probes': count,
                      'selection_events': selection_count, 'rounds': protocol['rounds']}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    run_parser = sub.add_parser('run')
    run_parser.add_argument('--binary', type=Path, required=True)
    run_parser.add_argument('--output', type=Path, required=True)
    run_parser.add_argument('--seed', type=int, required=True)
    run_parser.add_argument('--rounds', type=int, required=True)
    run_parser.add_argument('--population', type=int, required=True)
    run_parser.add_argument('--parents', type=int, required=True)
    run_parser.add_argument('--timeout', type=int, required=True)
    verify_parser = sub.add_parser('verify')
    verify_parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.command == 'run':
        run(args)
    else:
        verify(args.output)
