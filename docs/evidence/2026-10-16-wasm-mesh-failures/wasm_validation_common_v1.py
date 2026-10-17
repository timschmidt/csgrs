"""Frozen per-command containment, source verification and evidence retention."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import shutil
import subprocess
import time

ROOT=Path('/home/tim/Documents/GitHub/workspace/csgrs')
E=ROOT/'target/properties/wasm-mesh-failure-review'
WORK=ROOT/'target/properties/wasm-mesh-failure-review'
SOURCE=WORK/'candidate-source-v1'
OLD=ROOT/'target/properties/export-projection-contracts'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def dump(path,value):
    Path(path).write_text(json.dumps(value,indent=2)+'\n')


def read(path):
    return json.loads(Path(path).read_text())


class Validation:
    def __init__(self,kind,runner):
        self.kind,self.runner=kind,Path(runner)
        self.out=WORK/(kind+'-validation-v1')
        self.target=WORK/(kind+'-target-v1')
        self.scratch=WORK/(kind+'-scratch-v1')
        self.retained=WORK/(kind+'-executables-v1')
        for directory in [self.out,self.target,self.scratch,self.retained]:
            directory.mkdir(exist_ok=False)
        prep=read(WORK/'preparation-v1/result.json')
        assert prep['complete']
        self.manifest=WORK/'preparation-v1/source-hashes.json.gz'
        assert sha(self.manifest)==prep['source_hashes_sha256']
        with gzip.open(self.manifest,'rt') as stream:
            self.inputs=json.load(stream)
        self.verify_sources()
        self.env=os.environ.copy()
        for key in ['RUSTFLAGS','CARGO_ENCODED_RUSTFLAGS','RUSTC_WRAPPER','RUSTC_WORKSPACE_WRAPPER','RUSTDOCFLAGS','NODE_OPTIONS']:
            self.env.pop(key,None)
        self.env.update({'CARGO_HOME':'/home/tim/.cargo','CARGO_TARGET_DIR':str(self.target),
                         'CARGO_BUILD_JOBS':'2','CARGO_NET_OFFLINE':'true','CARGO_INCREMENTAL':'0',
                         'TMPDIR':str(self.scratch),'CARGO_TERM_COLOR':'never','RUST_BACKTRACE':'0'})
        self.tools={}
        if kind=='wasm':
            tool=read(OLD/'wasm-tools-v1/result.json')
            assert tool['exit_code']==0 and tool['version']=='wasm-bindgen 0.2.126'
            assert sha(tool['binary'])==tool['binary_sha256']
            typescript=OLD/'wasm-source-v1/tests/wasm/node_modules/typescript'
            assert read(typescript/'package.json')['version']=='5.7.3'
            self.tools={str(path):sha(path) for path in typescript.rglob('*') if path.is_file()}
            self.tools[tool['binary']]=tool['binary_sha256']
            self.env.update({'WASM_BINDGEN':tool['binary'],'TSC':str(typescript/'bin/tsc')})
        self.address_space=(64 if kind=='wasm' else 8)*1024**3
        self.protocol={'runner_sha256':sha(self.runner),'common_sha256':sha(__file__),
                       'source':str(SOURCE),'source_hashes_sha256':sha(self.manifest),
                       'preparation_result_sha256':sha(WORK/'preparation-v1/result.json'),
                       'target':str(self.target),'target_initially_empty':not list(self.target.iterdir()),
                       'mount':subprocess.check_output(['findmnt','-T',str(self.target),'-n','-o','TARGET,FSTYPE,OPTIONS'],text=True),
                       'rustc':subprocess.check_output(['rustc','-vV'],text=True),
                       'cargo':subprocess.check_output(['cargo','-V'],text=True),
                       'node':subprocess.check_output(['node','--version'],text=True).strip(),
                       'tools_read_only_sha256':self.tools,'environment':{key:self.env[key] for key in ['CARGO_HOME','CARGO_TARGET_DIR','CARGO_BUILD_JOBS','CARGO_NET_OFFLINE','CARGO_INCREMENTAL','TMPDIR']},
                       'elapsed_limit_seconds_per_command':720,'address_space_limit_bytes_per_process':self.address_space,
                       'containment':'External elapsed/address-space limits only; V8 requires large virtual reservations. No internal heap/work/cancellation guarantee.',
                       'base_commit':'99b88f707b04053de7f40a86d6d029e3c07460d9','main_sources_modified':False}
        assert 'ext4' in self.protocol['mount'] and 'fuse' not in self.protocol['mount']
        dump(self.out/'protocol.json',self.protocol)
        self.rows=[]

    def verify_sources(self):
        assert all(sha(SOURCE/path)==digest for path,digest in self.inputs.items())

    def limits(self):
        resource.setrlimit(resource.RLIMIT_AS,(self.address_space,self.address_space))
        resource.setrlimit(resource.RLIMIT_CORE,(0,0))

    def run(self,name,command,*,docs=False):
        self.verify_sources()
        assert sha(self.runner)==self.protocol['runner_sha256'] and sha(__file__)==self.protocol['common_sha256']
        assert all(sha(path)==digest for path,digest in self.tools.items())
        print('START '+self.kind+' '+name,flush=True)
        start=time.monotonic()
        env=self.env|({'RUSTDOCFLAGS':'-D warnings'} if docs else {})
        with (self.out/(name+'.stdout')).open('wb') as stdout,(self.out/(name+'.stderr')).open('wb') as stderr:
            p=subprocess.run(['/usr/bin/time','-v','-o',str(self.out/(name+'.resources')),
                              'timeout','--signal=TERM','--kill-after=20s','720s',*command],
                             cwd=SOURCE,env=env,stdout=stdout,stderr=stderr,
                             preexec_fn=self.limits,start_new_session=True)
        self.verify_sources()
        assert all(sha(path)==digest for path,digest in self.tools.items())
        paths={Path(path) for path in re.findall(r'Running[^\n]*\(([^)]+)\)',(self.out/(name+'.stderr')).read_text(errors='replace'))}
        paths.update(Path(path) for path in re.findall(r'Running `([^`]+)`',(self.out/(name+'.stderr')).read_text(errors='replace')))
        binaries={}
        for path in paths:
            path=path if path.is_absolute() else SOURCE/path
            digest=sha(path)
            copy=self.retained/digest/path.name
            copy.parent.mkdir(exist_ok=True)
            if not copy.exists():shutil.copy2(path,copy)
            assert sha(copy)==digest
            binaries[str(path)]={'sha256':digest,'retained':str(copy),'bytes':path.stat().st_size}
        row={'name':name,'command':command,'exit_code':p.returncode,'expected_exit_code':0,
             'elapsed_seconds':time.monotonic()-start,'sources_unchanged':True,'binaries':binaries,
             'rustdocflags':env.get('RUSTDOCFLAGS'),
             **{suffix+'_sha256':sha(self.out/(name+'.'+suffix)) for suffix in ['stdout','stderr','resources']}}
        self.rows.append(row)
        dump(self.out/'commands.json',self.rows)
        print(json.dumps({key:row[key] for key in ['name','exit_code','elapsed_seconds']}),flush=True)
        return p.returncode

    def finish(self,extra=None):
        self.verify_sources()
        assert sha(self.runner)==self.protocol['runner_sha256'] and sha(__file__)==self.protocol['common_sha256']
        record={'complete':True,'commands':len(self.rows),'unexpected_exits':[row['name'] for row in self.rows if row['exit_code']!=0],
                'sources_unchanged':True,'read_only_tools_unchanged':all(sha(path)==digest for path,digest in self.tools.items()),
                'native_runtime_admitted':False,'full_plan_complete':False}|(extra or {})
        dump(self.out/'result.json',record)
        print(json.dumps(record),flush=True)
        return 101 if record['unexpected_exits'] else 0
