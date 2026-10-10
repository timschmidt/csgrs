"""Align migrated macro arguments where rustfmt preserves a long enclosing block."""
import json
from pathlib import Path
import re
import sys

root = Path(sys.argv[1]).resolve()
source = Path(__file__).with_name('migrate_diagnostics.py').read_text()
namespace = {}
exec(source[source.index('def skip_literal'):source.index('\nfor path in sorted')], {'re': re}, namespace)
skip_literal = namespace['skip_literal']
tokens = namespace['tokens']
tokens.__globals__['skip_literal'] = skip_literal
inventory = json.loads(Path(__file__).with_name('production-migration.json').read_text())
for name in sorted({row['path'] for row in inventory}):
    path = root / name
    text = path.read_text()
    edits = []
    for match in re.finditer(r'(?:format!|log::trace!)\s*\(', text):
        start = match.end()
        depth = 0
        commas = []
        for index, char in tokens(text, start, len(text)):
            if char in '([{':
                depth += 1
            elif char in ')]}':
                if char == ')' and depth == 0:
                    end = index
                    break
                depth -= 1
            elif char == ',' and depth == 0:
                commas.append(index)
        else:
            raise AssertionError('unclosed macro')
        if 'diagnostic_decimal' not in text[start:end]:
            continue
        parts = [text[a + 1:b].strip() for a, b in zip([start - 1] + commas, commas + [end])]
        parts = [part for part in parts if part]
        prefix = text[text.rfind('\n', 0, match.start()) + 1:match.start()]
        indent = re.match(r'\s*', prefix)[0]
        replacement = '\n' + ''.join(indent + '    ' + part + ',\n' for part in parts) + indent
        # Moving whitespace and a trailing comma does not change arguments.
        edits.append((start, end, replacement))
    for start, end, replacement in reversed(edits):
        text = text[:start] + replacement + text[end:]
    path.write_text(text)
print('Aligned diagnostic macro arguments in', len({row['path'] for row in inventory}), 'files.')
