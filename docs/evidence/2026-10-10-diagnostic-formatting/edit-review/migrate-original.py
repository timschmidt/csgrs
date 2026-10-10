"""Add explicit diagnostic scalar views to reviewed human-output format sites.

Run only while validation/measurement is stopped. The artifact-analysis file
is excluded because its three rounded strings are used as comparison keys.
"""
import json
from pathlib import Path
import re
import sys

root = Path(sys.argv[1]).resolve()
inventory = []


def skip_literal(text, start):
    quote = text[start]
    index = start + 1
    while index < len(text):
        if text[index] == '\\':
            index += 2
        elif text[index] == quote:
            return index + 1
        else:
            index += 1
    raise AssertionError('unterminated literal')


def tokens(text, start, stop):
    index = start
    while index < stop:
        if text[index] == '"':
            index = skip_literal(text, index)
        elif text[index] == "'" and re.match(r"'(?:[^'\\]|\\.)'", text[index:]):
            index = skip_literal(text, index)
        elif text.startswith('//', index):
            end = text.find('\n', index)
            index = stop if end < 0 else end + 1
        else:
            yield index, text[index]
            index += 1


for path in sorted((root / 'crates/hyperdrc/src').rglob('*.rs')):
    if path.name == 'artifacts.rs':
        continue
    text = path.read_text()
    edits = []
    for match in re.finditer(r'(?:format!|log::trace!)\s*\(', text):
        body_start = match.end()
        literal_start = body_start
        while text[literal_start].isspace():
            literal_start += 1
        if text[literal_start] != '"':
            continue
        literal_end = skip_literal(text, literal_start)
        literal = text[literal_start:literal_end]
        if '#.' not in literal:
            continue
        assert 'diagnostic_decimal' not in literal
        depth = 0
        separators = []
        body_end = None
        for index, char in tokens(text, literal_end, len(text)):
            if char in '([{':
                depth += 1
            elif char in ')]}':
                if char == ')' and depth == 0:
                    body_end = index
                    break
                depth -= 1
            elif char == ',' and depth == 0:
                separators.append(index)
        assert body_end is not None and depth == 0
        arguments = []
        if separators:
            assert not text[literal_end:separators[0]].strip()
            for begin, end in zip(separators, separators[1:] + [body_end]):
                if text[begin + 1:end].strip():
                    arguments.append((begin + 1, end))
        positional = 0
        names = []
        fields = []
        for field in re.finditer(r'\{([^{}]*)\}', literal):
            contents = field.group(1)
            name, _, spec = contents.partition(':')
            if not name:
                arg = positional
                positional += 1
            elif name.isdigit():
                arg = int(name)
            else:
                arg = None
            if not spec.startswith('#.'):
                continue
            assert re.fullmatch(r'#\.\d+', spec), contents
            if arg is not None:
                begin, end = arguments[arg]
                expression = text[begin:end].strip()
                edits.append((begin, end, '\n(' + expression + ').diagnostic_decimal(-256)\n'))
                fields.append({'positional': arg, 'expression': expression})
            else:
                assert re.fullmatch(r'[A-Za-z_]\w*', name), name
                if name not in names:
                    names.append(name)
                fields.append({'named': name})
        assert fields
        if names:
            existing = text[literal_end:body_end].rstrip()
            prefix = '' if existing.endswith(',') else ','
            additions = ',\n'.join(name + ' = ' + name + '.diagnostic_decimal(-256)' for name in names)
            edits.append((body_end, body_end, prefix + '\n' + additions + ',\n'))
        inventory.append({'path': str(path.relative_to(root)), 'line_before': text.count('\n', 0, match.start()) + 1, 'format': literal, 'fields': fields})
    for begin, end, replacement in sorted(edits, reverse=True):
        text = text[:begin] + replacement + text[end:]
    if edits:
        path.write_text(text)

Path(sys.argv[2]).write_text(json.dumps(inventory, indent=2) + '\n')
print('Migrated', len(inventory), 'human-output format sites in', len({row['path'] for row in inventory}), 'files.')
