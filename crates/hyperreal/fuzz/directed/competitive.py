"""Retain native enclosure comparisons for every final selected scalar input."""
import argparse
from fractions import Fraction
import json
import os
from pathlib import Path
import platform
import random
import statistics
import sys

from campaign import digest, execute, identity, verify as verify_campaign, write
from evidence import ENCODING, fraction

BACKENDS = ('hyperreal', 'mpfr')
TARGET = Fraction(1, 2 ** 256)
PRECISIONS = list(range(256, 2049, 128))


def validate(response, payload, backend):
    if response.get('schema') != 1 or response.get('status') != 'ok' or response.get('backend') != backend:
        raise ValueError('benchmark backend or response status changed')
    if response.get('rational_encoding') != ENCODING or response['recipe']['input'] != payload:
        raise ValueError('benchmark encoding or exact input identity changed')
    if response['actual_class'] != payload['bucket']:
        raise ValueError('benchmark changed the representation')
    if response['dispatch_trace_enabled'] is not False or response['debug_assertions'] is not False:
        raise ValueError('benchmark was instrumented or not a release build')
    if fraction(response['maximum_absolute_width']) != TARGET:
        raise ValueError('benchmark weakened the absolute-width contract')
    lo, hi = map(fraction, response['oracle']['interval'])
    if lo > hi or not (lo > 0 or hi < 0) or response['oracle']['precision_bits'] != 1024:
        raise ValueError('reference interval failed to isolate the expected nonzero value')
    if response['oracle']['sign'] != ('Positive' if lo > 0 else 'Negative'):
        raise ValueError('reference sign contradicts its interval')
    steps = response['steps']
    if not 1 <= len(steps) <= len(PRECISIONS) or [step['precision_bits'] for step in steps] != PRECISIONS[:len(steps)]:
        raise ValueError('precision schedule changed or was not completed in order')
    for i, step in enumerate(steps):
        a, b = map(fraction, step['interval'])
        if a > b or b < lo or a > hi:
            raise ValueError('native interval contradicts the reference')
        if fraction(step['width']) != b - a:
            raise ValueError('reported width differs from exact endpoint subtraction')
        if (b - a <= TARGET) != (i == len(steps) - 1):
            raise ValueError('adaptive evaluation did not stop at the required width')
    elapsed = response['elapsed_ns']
    if type(elapsed) is not int or elapsed <= 0:
        raise ValueError('invalid native timing sample')
    return elapsed


def make_schedule(inputs, samples, seed):
    if type(samples) is not int or not 3 <= samples <= 101 or samples % 2 != 1:
        raise ValueError('samples must be an odd count in 3..101')
    rng = random.Random(seed)
    keys = sorted(inputs)
    indices = {key: i for i, key in enumerate(keys)}
    schedule = []
    for sample in range(samples):
        order = keys.copy()
        rng.shuffle(order)
        for key in order:
            backends = BACKENDS if (sample + indices[key]) % 2 == 0 else tuple(reversed(BACKENDS))
            for backend in backends:
                schedule.append({'sequence': len(schedule), 'sample': sample, 'id': key, 'backend': backend})
    return schedule


def summarize(inputs, journal, samples):
    output = []
    for key, payload in sorted(inputs.items()):
        row = {'id': key, 'input': payload, 'samples': samples, 'backends': {}}
        for backend in BACKENDS:
            selected = sorted((item for item in journal if item['id'] == key and item['backend'] == backend),
                              key=lambda item: item['sample'])
            if [item['sample'] for item in selected] != list(range(samples)):
                raise ValueError('missing or duplicate native timing samples')
            values = [item['elapsed_ns'] for item in selected]
            row['backends'][backend] = {'raw_elapsed_ns': values, 'median_ns': statistics.median(values),
                                       'minimum_ns': min(values), 'maximum_ns': max(values)}
        row['hyperreal_over_mpfr_median_ratio'] = row['backends']['hyperreal']['median_ns'] / row['backends']['mpfr']['median_ns']
        output.append(row)
    return output


def selected_inputs(campaign):
    selected = json.loads((campaign / 'selected-inputs.json').read_text())
    if len(selected) != 22 or any(not rows for rows in selected.values()):
        raise ValueError('all 22 representation buckets must retain selected inputs')
    inputs = {}
    for bucket, rows in selected.items():
        for payload in rows:
            if payload['bucket'] != bucket:
                raise ValueError('selected input is in the wrong representation bucket')
            key = identity(payload)
            if key in inputs:
                raise ValueError('duplicate final selected input')
            inputs[key] = payload
    return inputs


def artifacts(output):
    return {str(p.relative_to(output)): digest(p) for p in output.rglob('*') if p.is_file()}


def run(args):
    make_schedule({}, args.samples, args.seed)
    if not 1 <= args.timeout <= 3600:
        raise ValueError('timeout must be 1..3600 seconds')
    output, campaign, binary = args.output.resolve(), args.campaign.resolve(), args.binary.resolve()
    output.mkdir(parents=True, exist_ok=False)
    original_affinity = os.sched_getaffinity(0)
    journal = []
    try:
        verify_campaign(campaign)
        inputs = selected_inputs(campaign)
        cpu = min(original_affinity)
        protocol = {'schema': 1, 'campaign': str(campaign), 'campaign_result_sha256': digest(campaign / 'result.json'),
                    'selected_inputs_sha256': digest(campaign / 'selected-inputs.json'),
                    'binary': str(binary), 'binary_sha256': digest(binary), 'python': sys.version,
                    'platform': platform.platform(), 'cpu': cpu, 'original_cpu_affinity': sorted(original_affinity),
                    'samples_per_backend': args.samples, 'seed': args.seed, 'process_timeout_seconds': args.timeout,
                    'input_count': len(inputs), 'representations': 22, 'maximum_absolute_width': str(TARGET),
                    'sources_sha256': {name: digest(Path(__file__).with_name(name)) for name in [
                        'competitive.py', 'campaign.py', 'lexicase.py', 'evidence.py']},
                    'no_resumption_extension_filtering_or_pooling': True, 'P38_complete': False, 'plan_complete': False}
        write(output / 'protocol.json', protocol)
        (output / 'cpuinfo.txt').write_text(Path('/proc/cpuinfo').read_text())
        write(output / 'inputs.json', inputs)
        (output / 'inputs').mkdir()
        for key, payload in inputs.items():
            write(output / 'inputs' / (key + '.json'), payload)
        schedule = make_schedule(inputs, args.samples, args.seed)
        write(output / 'schedule.json', schedule)
        os.sched_setaffinity(0, {cpu})
        for planned in schedule:
            key, backend = planned['id'], planned['backend']
            stem = f"case-{key}-sample-{planned['sample']:03}-{backend}"
            response, process = execute(binary, [backend, output / 'inputs' / (key + '.json')],
                                        output / stem, args.timeout, protocol['binary_sha256'])
            elapsed = validate(response, inputs[key], backend)
            record = planned | {'elapsed_ns': elapsed, 'response_file': process['stdout'],
                                'response_sha256': process['stdout_sha256'], 'process_file': stem + '.process.json'}
            journal.append(record)
            with (output / 'samples.jsonl').open('a') as log:
                log.write(json.dumps(record, sort_keys=True) + '\n')
        if selected_inputs(campaign) != inputs or digest(campaign / 'result.json') != protocol['campaign_result_sha256']:
            raise ValueError('source campaign changed during benchmarking')
        write(output / 'summary.json', summarize(inputs, journal, args.samples))
        result = {'complete': True, 'input_count': len(inputs), 'representations': 22,
                  'samples_per_backend_per_input': args.samples, 'completed_samples': len(journal),
                  'artifacts_sha256': artifacts(output), 'interval_comparison_complete': True,
                  'structural_certificate_api_comparison': False, 'P38_complete': False, 'plan_complete': False}
        write(output / 'result.json', result)
        print(json.dumps({key: value for key, value in result.items() if key != 'artifacts_sha256'}))
    except Exception as error:
        write(output / 'failure.json', {'complete': False, 'error': str(error), 'completed_samples': len(journal),
                                       'artifacts_sha256': artifacts(output), 'P38_complete': False, 'plan_complete': False})
        raise
    finally:
        os.sched_setaffinity(0, original_affinity)


def verify(output):
    result = json.loads((output / 'result.json').read_text())
    if not result['complete'] or (output / 'failure.json').exists():
        raise ValueError('benchmark did not complete')
    actual = artifacts(output)
    actual.pop('result.json')
    if actual != result['artifacts_sha256']:
        raise ValueError('benchmark artifacts changed')
    protocol = json.loads((output / 'protocol.json').read_text())
    if (protocol['representations'] != 22 or result['representations'] != 22
            or result['input_count'] != protocol['input_count']
            or result['samples_per_backend_per_input'] != protocol['samples_per_backend']
            or fraction(protocol['maximum_absolute_width']) != TARGET):
        raise ValueError('benchmark result dimensions or accuracy contract changed')
    campaign = Path(protocol['campaign'])
    verify_campaign(campaign)
    if digest(campaign / 'result.json') != protocol['campaign_result_sha256'] or digest(
            campaign / 'selected-inputs.json') != protocol['selected_inputs_sha256']:
        raise ValueError('source campaign digest changed')
    inputs = json.loads((output / 'inputs.json').read_text())
    if inputs != selected_inputs(campaign) or len(inputs) != protocol['input_count']:
        raise ValueError('benchmark omitted or changed a final selected input')
    for name, expected in protocol['sources_sha256'].items():
        if digest(Path(__file__).with_name(name)) != expected:
            raise ValueError('benchmark source differs from its frozen version')
    for key, payload in inputs.items():
        if identity(payload) != key or json.loads((output / 'inputs' / (key + '.json')).read_text()) != payload:
            raise ValueError('benchmark input identity changed')
    schedule = make_schedule(inputs, protocol['samples_per_backend'], protocol['seed'])
    if json.loads((output / 'schedule.json').read_text()) != schedule:
        raise ValueError('native execution order changed')
    journal = [json.loads(line) for line in (output / 'samples.jsonl').read_text().splitlines()]
    if len(journal) != len(schedule) or result['completed_samples'] != len(schedule):
        raise ValueError('native sample budget was not completed exactly')
    for planned, actual in zip(schedule, journal, strict=True):
        if {key: actual[key] for key in planned} != planned:
            raise ValueError('native sample journal changed order or identity')
        process = json.loads((output / actual['process_file']).read_text())
        if process['status'] != 'exited' or process['exit_code'] != 0:
            raise ValueError('native probe did not exit successfully')
        if (output / process['stderr']).read_bytes() or digest(output / process['stderr']) != process['stderr_sha256']:
            raise ValueError('native diagnostics were ignored or changed')
        if actual['response_file'] != process['stdout'] or actual['response_sha256'] != process['stdout_sha256'] or digest(
                output / actual['response_file']) != actual['response_sha256']:
            raise ValueError('native response hash changed')
        response = json.loads((output / actual['response_file']).read_text())
        if validate(response, inputs[planned['id']], planned['backend']) != actual['elapsed_ns']:
            raise ValueError('native sample differs from its raw response')
    summary = summarize(inputs, journal, protocol['samples_per_backend'])
    if json.loads((output / 'summary.json').read_text()) != summary:
        raise ValueError('native statistics differ from the complete raw samples')
    print(json.dumps({'verified': True, 'inputs': len(inputs), 'samples': len(journal)}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    run_parser = sub.add_parser('run')
    run_parser.add_argument('--binary', type=Path, required=True)
    run_parser.add_argument('--campaign', type=Path, required=True)
    run_parser.add_argument('--output', type=Path, required=True)
    run_parser.add_argument('--seed', type=int, required=True)
    run_parser.add_argument('--samples', type=int, default=11)
    run_parser.add_argument('--timeout', type=int, default=30)
    verify_parser = sub.add_parser('verify')
    verify_parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.command == 'run':
        run(args)
    else:
        verify(args.output)
