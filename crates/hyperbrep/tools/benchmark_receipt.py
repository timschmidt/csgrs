#!/usr/bin/env python3
"""Check the named rows emitted by HyperBRep's kernel benchmark.

This verifies fixture coverage, iteration counts, units and numeric checksums.
It does not establish successful execution, geometric equivalence beyond the
harness assertions, or statistically significant timing differences.
"""

import argparse
from dataclasses import asdict, dataclass
from decimal import Decimal
import json
from pathlib import Path
import re
import sys


LABEL = r'[^\s:"]+/[^\s:"]+'
SOURCE_LABEL = re.compile(r'"(' + LABEL + r'):')
ROW_START = re.compile(r'^(' + LABEL + r'):')
ROW = re.compile(
    r'^(' + LABEL + r'): (\d+) (paired )?iterations in (\S+) '
    r'\((\S+)/(iter|pair)\), checksum=(\d+)$'
)
DURATION = re.compile(r'(\d+(?:\.\d+)?)(ns|µs|ms|s)')


@dataclass(frozen=True)
class Case:
    name: str
    iterations: int
    unit: str
    checksum: int
    elapsed_ns: int
    per_iteration_ns: int


def expected_cases(source: str) -> list[str]:
    names = SOURCE_LABEL.findall(source)
    if not names or len(names) != len(set(names)):
        raise ValueError('benchmark source must contain unique static fixture labels')
    return names


def nanoseconds(value: str) -> int:
    match = DURATION.fullmatch(value)
    if match is None:
        raise ValueError(f'invalid benchmark duration: {value}')
    scale = {'ns': 1, 'µs': 1000, 'ms': 1_000_000, 's': 1_000_000_000}
    result = Decimal(match[1]) * scale[match[2]]
    if result != result.to_integral_value():
        raise ValueError(f'duration is not an integral nanosecond count: {value}')
    return int(result)


def read_receipt(text: str, expected: list[str]) -> list[Case]:
    cases = []
    for line in text.splitlines():
        if ROW_START.match(line) is None:
            continue
        match = ROW.fullmatch(line)
        if match is None:
            raise ValueError(f'malformed benchmark row: {line}')
        name, count, paired, elapsed, per_iteration, unit, checksum = match.groups()
        if (paired is not None) != (unit == 'pair'):
            raise ValueError(f'inconsistent iteration unit: {name}')
        case = Case(name, int(count), unit, int(checksum), nanoseconds(elapsed), nanoseconds(per_iteration))
        if case.iterations == 0:
            raise ValueError(f'zero benchmark iterations: {name}')
        if case.elapsed_ns // case.iterations != case.per_iteration_ns:
            raise ValueError(f'inconsistent per-iteration duration: {name}')
        cases.append(case)
    names = [case.name for case in cases]
    if names != expected:
        missing = sorted(set(expected) - set(names))
        extra = sorted(set(names) - set(expected))
        duplicates = sorted({name for name in names if names.count(name) > 1})
        raise ValueError(
            f'benchmark rows differ from source order/coverage: '
            f'missing={missing}, extra={extra}, duplicates={duplicates}'
        )
    return cases


def compare(baseline: list[Case], candidate: list[Case]) -> None:
    if len(baseline) != len(candidate):
        raise ValueError('benchmark row counts differ')
    for first, second in zip(baseline, candidate):
        for field in ('name', 'iterations', 'unit', 'checksum'):
            if getattr(first, field) != getattr(second, field):
                raise ValueError(f'benchmark {field} changed: {first.name}')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1] / 'benches/kernel.rs')
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--candidate', required=True, type=Path)
    parser.add_argument('--output', type=Path, help='Optional complete parsed receipt as JSON')
    args = parser.parse_args()
    try:
        expected = expected_cases(args.source.read_text())
        baseline = read_receipt(args.baseline.read_text(), expected)
        candidate = read_receipt(args.candidate.read_text(), expected)
        compare(baseline, candidate)
    except (OSError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1
    if args.output is not None:
        args.output.write_text(json.dumps({
            'schema_version': 1,
            'rows': len(expected),
            'preserved': ['source order', 'fixture names', 'iteration counts', 'iteration units', 'checksums'],
            'baseline': [asdict(case) for case in baseline],
            'candidate': [asdict(case) for case in candidate],
        }, indent=2) + '\n')
    print(f'{len(expected)} benchmark rows verified against source and baseline.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
