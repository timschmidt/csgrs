"""Frozen source/command identity and external containment for native disposition."""
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
E=ROOT/'docs/evidence/2026-10-16-native-change-disposition'
WORK=ROOT/'target/properties/native-change-disposition'


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')
def gz(path):
    with gzip.open(path,'rt') as stream:return json.load(stream)


class Validation:
    def __init__(self,variant,runner):
        self.variant,self.runner=variant,Path(runner)
        self.source=WORK/(variant+'-source-v1')
        self.target=WORK/(variant+'-target-v1')
        self.out=WORK/(variant+'-validation-v1')
        self.scratch=WORK/(variant+'-scratch-v1')
        self.retained=WORK/(variant+'-executables-v1')
        for directory in [self.target,self.out,self.scratch,self.retained]:directory.mkdir(exist_ok=False)
        self.manifest=WORK/'preparation-v1'/(variant+'-source-hashes.json.gz')
        self.inputs=gz(self.manifest)
        preparation=read(WORK/'preparation-v1/result.json')
        assert sha(self.manifest)==preparation['variants'][variant]['source_hashes_sha256']
        self.main_inputs=gz(WORK/'preparation-v1/candidate-source-hashes.json.gz')
        self.verify()
        self.env=os.environ.copy()
        for key in ['RUSTFLAGS','CARGO_ENCODED_RUSTFLAGS','RUSTC_WRAPPER','RUSTC_WORKSPACE_WRAPPER','RUSTDOCFLAGS']:
            self.env.pop(key,None)
        self.env.update({'CARGO_HOME':'/home/tim/.cargo','CARGO_TARGET_DIR':str(self.target),
                         'CARGO_BUILD_JOBS':'2','CARGO_NET_OFFLINE':'true','CARGO_INCREMENTAL':'0',
                         'TMPDIR':str(self.scratch),'CARGO_TERM_COLOR':'never','RUST_BACKTRACE':'0'})
        self.protocol={'base_commit':preparation['base_commit'],'variant':variant,
                       'runner_sha256':sha(self.runner),'common_sha256':sha(__file__),
                       'source':str(self.source),'source_hashes_sha256':sha(self.manifest),
                       'preparation_sha256':sha(WORK/'preparation-v1/result.json'),
                       'target':str(self.target),'target_initially_empty':not list(self.target.iterdir()),
                       'mount':subprocess.check_output(['findmnt','-T',str(self.target),'-n','-o','TARGET,FSTYPE,OPTIONS'],text=True),
                       'rustc':subprocess.check_output(['rustc','-vV'],text=True),'cargo':subprocess.check_output(['cargo','-V'],text=True),
                       'hardware':subprocess.check_output(['lscpu'],text=True),'fixtures':preparation['fixtures'],
                       'environment':{key:self.env[key] for key in ['CARGO_HOME','CARGO_TARGET_DIR','CARGO_BUILD_JOBS','CARGO_NET_OFFLINE','CARGO_INCREMENTAL','TMPDIR']},
                       'elapsed_limit_seconds_per_command':1800,'address_space_limit_bytes_per_process':8*1024**3,
                       'internal_resource_bound_claimed':False,'full_plan_complete':False}
        assert 'ext4' in self.protocol['mount'] and 'fuse' not in self.protocol['mount']
        dump(self.out/'protocol.json',self.protocol)
        self.rows=[]

    def verify(self):
        assert all(sha(self.source/path)==digest for path,digest in self.inputs.items())
        assert all(sha(ROOT/path)==digest for path,digest in self.main_inputs.items())

    def limits(self):
        resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3))
        resource.setrlimit(resource.RLIMIT_CORE,(0,0))

    def run(self,name,command,*,docs=False):
        self.verify()
        assert sha(self.runner)==self.protocol['runner_sha256'] and sha(__file__)==self.protocol['common_sha256']
        print('START '+self.variant+' '+name,flush=True)
        start=time.monotonic()
        env=self.env|({'RUSTDOCFLAGS':'-D warnings'} if docs else {})
        with (self.out/(name+'.stdout')).open('wb') as stdout,(self.out/(name+'.stderr')).open('wb') as stderr:
            p=subprocess.run(['/usr/bin/time','-v','-o',str(self.out/(name+'.resources')),
                              'timeout','--signal=TERM','--kill-after=20s','1800s',*command],
                             cwd=self.source,env=env,stdout=stdout,stderr=stderr,
                             preexec_fn=self.limits,start_new_session=True)
        self.verify()
        text=(self.out/(name+'.stderr')).read_text(errors='replace')
        paths={Path(path) for path in re.findall(r'(?:Running|Executable)[^\n]*\(([^)]+)\)',text)}
        binaries={}
        for path in paths:
            path=path if path.is_absolute() else self.source/path
            assert path.is_relative_to(self.target) and path.is_file(),path
            digest=sha(path)
            copy=self.retained/digest/path.name
            copy.parent.mkdir(exist_ok=True)
            if not copy.exists():shutil.copy2(path,copy)
            assert sha(copy)==digest
            binaries[str(path)]={'sha256':digest,'retained':str(copy),'bytes':path.stat().st_size}
        row={'name':name,'command':command,'exit_code':p.returncode,'elapsed_seconds':time.monotonic()-start,
             'sources_unchanged':True,'binaries':binaries,'rustdocflags':env.get('RUSTDOCFLAGS'),
             **{suffix+'_sha256':sha(self.out/(name+'.'+suffix)) for suffix in ['stdout','stderr','resources']}}
        self.rows.append(row)
        dump(self.out/'commands.json',self.rows)
        print(json.dumps({key:row[key] for key in ['name','exit_code','elapsed_seconds']}),flush=True)
        return p.returncode

    def finish(self):
        self.verify()
        assert sha(self.runner)==self.protocol['runner_sha256'] and sha(__file__)==self.protocol['common_sha256']
        result={'complete':True,'variant':self.variant,'commands':len(self.rows),
                'failed_commands':[row['name'] for row in self.rows if row['exit_code']!=0],
                'sources_unchanged':True,'original_assertions_unchanged':True,'full_plan_complete':False}
        dump(self.out/'result.json',result)
        print(json.dumps(result),flush=True)
        return 101 if result['failed_commands'] else 0
