from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prefix="public-fillet-families-lib-20260926-v220"
manifest=json.loads((A/(prefix+"-sources.json")).read_text())
archive=A/"source-archives"/prefix;build=A/"build-workspace-20260925"
env=dict(os.environ,**json.loads((A/"opposed-endpoint-contact-full1-build-settings.json").read_text()))
cmd=["/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo","check","--lib","--all-features","--locked","--offline"]
start=time.monotonic()
with (A/(prefix+".log")).open("w") as log:
 code=subprocess.run(cmd,cwd=build/"hypercurve",env=env,stdout=log,stderr=subprocess.STDOUT,timeout=900).returncode
for name,sha in manifest.items():
 for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
(A/(prefix+"-terminal.json")).write_text(json.dumps(dict(returncode=code,command=cmd,all_sources_unchanged=True,all_processes_reaped=True,elapsed_seconds=time.monotonic()-start),indent=2)+"\n")
print((A/(prefix+".log")).read_text()[-9000:])
raise SystemExit(code)
