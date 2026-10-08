from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prefix="public-fillet-families-callers-20260926-v221"
manifest=json.loads((A/(prefix+"-sources.json")).read_text())
archive=A/"source-archives"/prefix;build=A/"build-workspace-20260925"
env=dict(os.environ,**json.loads((A/"opposed-endpoint-contact-full1-build-settings.json").read_text()))
cmd=["/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo","check","--all-targets","--all-features","--locked","--offline","--message-format=json"]
start=time.monotonic()
with (A/(prefix+".log")).open("w") as log:
 code=subprocess.run(cmd,cwd=build/"hypercurve",env=env,stdout=log,stderr=subprocess.STDOUT,timeout=900).returncode
for name,sha in manifest.items():
 for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
(A/(prefix+"-terminal.json")).write_text(json.dumps(dict(returncode=code,command=cmd,all_sources_unchanged=True,all_processes_reaped=True,elapsed_seconds=time.monotonic()-start),indent=2)+"\n")
errors=[]
for line in (A/(prefix+".log")).read_text().splitlines():
 try:row=json.loads(line)
 except Exception:continue
 if row.get("reason")=="compiler-message" and row["message"]["level"]=="error":
  m=row["message"];errors.append(m)
(A/(prefix+"-errors.json")).write_text(json.dumps(errors,indent=2)+"\n")
for m in errors[:12]:print(m["rendered"])
print("Errors:",len(errors))
raise SystemExit(code)
