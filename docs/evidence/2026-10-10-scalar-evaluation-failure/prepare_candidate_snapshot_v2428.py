"""Fork complete scalar input snapshots without changing completed v2 phases."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

CAPTURE = '''def source_hashes(source, original):
    """Hash original workspace inputs and every introduced scalar input file."""
    result = {name: digest(source / name) for name in original}
    for directory, children, names in os.walk(source / "crates/hyperreal"):
        children[:] = sorted(name for name in children
                             if name not in {"target", ".git", "__pycache__"})
        for name in [*children, *names]:
            path = Path(directory) / name
            if path.is_symlink():
                raise RuntimeError(f"Symbolic-link scalar input is not supported: {path}")
        for name in sorted(names):
            path = Path(directory) / name
            if path.is_file():
                result[str(path.relative_to(source))] = digest(path)
    return result


'''


def main():
    parent = E / 'validate_candidate_v2.py'
    successor = E / 'validate_candidate_v3.py'
    record = E / 'candidate-snapshot-preparation-v2428.json'
    assert not successor.exists() and not record.exists()
    code = parent.read_text()
    begin = code.index('    source = {name: digest(SOURCE / name) for name in original}\n')
    end = code.index('    for name, value in source.items():\n', begin)
    previous = code[begin:end]
    assert 'if path.suffix in ' in previous
    code = code[:begin] + '    source = source_hashes(SOURCE, original)\n' + code[end:]
    assert code.count('def main():\n') == 1
    code = code.replace('def main():\n', CAPTURE + 'def main():\n')
    before = '    source_unchanged = all(digest(SOURCE / name) == value for name, value in source.items())'
    assert code.count(before) == 1
    code = code.replace(before, '''    source_snapshot_error = None
    try:
        source_unchanged = source_hashes(SOURCE, original) == source
    except (OSError, RuntimeError) as error:
        source_unchanged = False
        source_snapshot_error = str(error)''')
    before = '        "source_unchanged": source_unchanged, "main_unchanged": main_unchanged,'
    assert code.count(before) == 1
    code = code.replace(before, before + '\n        "source_catalog_rechecked": True, "source_snapshot_error": source_snapshot_error,')
    code = code.replace('Freeze each isolated prototype, including added proof/check inputs.',
                        'Freeze each isolated prototype, including all introduced scalar input types.')
    ast.parse(code)
    successor.write_text(code)
    record.write_text(json.dumps({
        'status': 'Snapshot wrapper forked before unit or candidate validation',
        'preparation_sha256': sha(Path(__file__)), 'parent_sha256': sha(parent),
        'successor': successor.name, 'successor_sha256': sha(successor),
        'removed_collection_block_sha256': hashlib.sha256(previous.encode()).hexdigest(),
        'changes': [
            'Keep every original workspace input, and remove extension filtering from introduced scalar inputs. Licenses, original manifests, binary tables and hidden files are included.',
            'Prune the same generated target, Git and Python cache directories before traversal; reject scalar source symlinks that would hide external input trees.',
            'Recollect and compare the complete source catalogue after each command, detecting added files as well as changed/deleted inputs. Preserve the existing main guard, archived changed inputs, command environment and failure assertion.'
        ],
        'scope': 'The wrapper still bounds new-file discovery to the scalar candidate subtree. It does not claim to discover arbitrary new files throughout all workspace crates. Completed v2 phases and retained source/coverage counts are unchanged.',
    }, indent=2) + '\n')
    print(json.dumps({'successor': successor.name, 'extension_filter_removed': True, 'post_command_catalog_rechecked': True}))


if __name__ == '__main__':
    main()
