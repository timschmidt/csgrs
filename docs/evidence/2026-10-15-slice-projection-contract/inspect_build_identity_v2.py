"""Inspect actual compiled dependency metadata before any cost measurements."""
import gzip, hashlib, json, os, re, subprocess
from pathlib import Path
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p,value):p.write_text(json.dumps(value,indent=2)+'\n')
def main():
 out=E/'build-identity-v2';out.mkdir(exist_ok=False)
 old=ROOT/'docs/evidence/2026-10-15-planar-operation-failures/cost-build-v1'
 new=E/'native-v1'
 protocols=[json.loads((p/'protocol.json').read_text()) for p in [old,new]]
 assert protocols[0]['rustc']==protocols[1]['rustc']
 assert protocols[0]['commands'][0][1]==next(c for n,c in protocols[1]['commands'] if n=='renderer-build')
 sources=[json.loads(gzip.decompress((p/'source-hashes.json.gz').read_bytes())) for p in [old,new]]
 core=['hyperreal','hyperlattice','hyperlimit','hypercurve','hypertri','hypersolve']
 names={n for s in sources for n in s if any(n.startswith('crates/'+crate+'/') for crate in core) or n in ['Cargo.toml','Cargo.lock','.cargo/config.toml']}
 assert all(sources[0].get(n)==sources[1].get(n) for n in names)
 targets={'baseline':ROOT/'target/planar-operation-failures/cost-build-v1/release','candidate':ROOT/'target/slice-projection-contract/native-v1/release'}
 libraries=['libcsgrs.rlib','libhyperlattice-64c0cb923a8aa061.rlib','libhypercurve-65c0df1f5cff6824.rlib']
 commands=[[label+'-'+lib,['rustc','-Zls=root',str(target/'deps'/lib)]] for label,target in targets.items() for lib in libraries]
 dump(out/'protocol.json',{'commands':commands,'environment_override':{'RUSTC_BOOTSTRAP':'1'},'runner_sha256':sha(Path(__file__)),'toolchain':protocols[0]['rustc'],'source_records':[str(p/'source-hashes.json.gz') for p in [old,new]],'purpose':'Read actual compiled crate identity and dependency graphs. This does not assume that matching Cargo fingerprint file names imply matching resolved dependencies.'})
 rows=[];metadata={}
 for name,cmd in commands:
  result=subprocess.run(cmd,cwd=ROOT,env=os.environ|{'RUSTC_BOOTSTRAP':'1'},capture_output=True)
  for suffix,data in [('stdout',result.stdout),('stderr',result.stderr)]: (out/(name+'.'+suffix)).write_bytes(data)
  rows.append({'name':name,'exit_code':result.returncode,'stdout_sha256':sha(out/(name+'.stdout')),'stderr_sha256':sha(out/(name+'.stderr'))})
  dump(out/'result.json',{'commands':rows,'passed':False})
  assert result.returncode==0
  metadata[name]=result.stdout.decode()
 comparisons={}
 for lib in libraries:
  a,b=[metadata[label+'-'+lib] for label in targets]
  # Preserve all raw hashes above; compare actual crate names, stable IDs,
  # target, ABI and complete resolved dependency names/visibility/kinds.
  normalized=lambda s:re.sub(r'\bhash [0-9a-f]+', 'hash <recorded>',s)
  assert normalized(a)==normalized(b),lib
  comparisons[lib]={'same_actual_dependency_names_and_abi':True,'baseline_sha256':sha(targets['baseline']/'deps'/lib),'candidate_sha256':sha(targets['candidate']/'deps'/lib),'same_compiled_crate_hashes':a==b}
 fingerprints={}
 for lib in libraries[1:]:
  unit=lib.removeprefix('lib').removesuffix('.rlib');name=unit.rsplit('-',1)[0]
  paths=[target/'.fingerprint'/unit/('lib-'+name+'.json') for target in targets.values()]
  assert json.loads(paths[0].read_text())==json.loads(paths[1].read_text())
  fingerprints[unit]=[{'path':str(p),'sha256':sha(p)} for p in paths]
 dump(out/'result.json',{'commands':rows,'passed':True,'core_sources_and_locked_configuration_identical':len(names),'same_toolchain_and_build_command':True,'actual_compiled_graphs':comparisons,'core_cargo_compilation_units':fingerprints,'qualification':'Fresh and seeded builds have different encoded crate hashes despite identical sources, toolchain, Cargo compilation units and actual resolved dependency names/ABI. The cause is not established. Archive bytes are not claimed identical. The cost probe must link each revision against its own exact libraries and validate every completed output. This is a rebuild-controlled before/after probe, not evidence of a performance optimization or a bit-identical dependency binary experiment.'})
 print(json.dumps({'passed':True,'compared_source_files':len(names),'actual_compiled_graphs':len(comparisons)}))
if __name__=='__main__':main()
