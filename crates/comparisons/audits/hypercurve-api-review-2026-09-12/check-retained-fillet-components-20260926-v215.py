from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent; W=A.parent
prefix="retained-fillet-components-20260926-v215"
manifest=json.loads((A/f"{prefix}-sources.json").read_text())
archive=A/"source-archives"/prefix; build=A/"build-workspace-20260925"
env=dict(os.environ,**json.loads((A/"opposed-endpoint-contact-full1-build-settings.json").read_text()))
parents={name:subprocess.check_output(["git","rev-parse","HEAD"],cwd=W/name,text=True).strip() for name in ("hypercurve","hypersolve","hyperreal")}
def verify():
 for name,sha in manifest.items():
  for root in (W,archive,build): assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for name,head in parents.items():
  assert subprocess.check_output(["git","rev-parse","HEAD"],cwd=W/name,text=True).strip()==head
  assert not subprocess.check_output(["git","diff","--cached","--name-only"],cwd=W/name)
for name in manifest:
 source=archive/name; target=build/name
 if not target.exists() or source.read_bytes()!=target.read_bytes():
  target.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(source,target); os.utime(target,None)
 assert (source.stat().st_dev,source.stat().st_ino)!=(target.stat().st_dev,target.stat().st_ino)
verify()
command=["/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo","check","--all-targets","--all-features","--locked","--offline"]
log=A/f"{prefix}-check.log"
with log.open("w") as out:
 code=subprocess.run(command,cwd=build/"hypercurve",env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
verify()
(A/f"{prefix}-terminal.json").write_text(json.dumps(dict(parents=parents,command=command,returncode=code,log=log.name,source_manifest=f"{prefix}-sources.json",all_sources_unchanged=True,all_processes_reaped=True),indent=2)+"\n")
print(log.read_text()[-15000:],flush=True)
raise SystemExit(code)
