"""Independently identify exact coincident source tracks with Python Fractions."""
import hashlib,json,re,zipfile
from fractions import Fraction
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'source-disks-v1'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
O.mkdir(exist_ok=False);fixtures={}
ordinary='(kicad_pcb (net 1 signal) '+''.join(f'(segment (start {i*4} 0) (end {i*4+2} 0) (width 2) (layer F.Cu) (net 1))' for i in range(64))+')'
(O/'ordinary64.kicad_pcb').write_text(ordinary);fixtures['ordinary64']=O/'ordinary64.kicad_pcb'
archive_pins={}
for f in read(W/'baseline-preparation-v1/result.json')['fixtures']:
    archive=Path(f['snapshot_archive']);assert sha(archive)==f['sha256'];archive_pins[str(archive)]=sha(archive)
    with zipfile.ZipFile(archive) as z:data=z.read(f['entry'])
    assert len(data)==f['entry_bytes'] and hashlib.sha256(data).hexdigest()==f['entry_sha256']
    name='cparti' if f['entry'].startswith('CPArti') else 'hvp109a';p=O/(name+'.kicad_pcb');p.write_bytes(data);fixtures[name]=p
summaries={}
for name,path in fixtures.items():
    root=None;stack=[]
    for token in re.findall(r'"(?:\\.|[^"\\])*"|[()]|[^\s()]+',path.read_text()):
        if token=='(':
            node=[]
            if stack:stack[-1].append(node)
            else:assert root is None;root=node
            stack.append(node)
        elif token==')':assert stack;stack.pop()
        else:assert stack;stack[-1].append(token)
    assert not stack and root[0]=='kicad_pcb'
    positive=0;zero=0;disks=[];total=0
    for record in root[1:]:
        if not isinstance(record,list) or not record or record[0]!='segment':continue
        fields={}
        for field,arity in [('start',2),('end',2),('width',1)]:
            values=[r for r in record[1:] if isinstance(r,list) and r and r[0]==field]
            assert len(values)==1 and len(values[0])==arity+1
            fields[field]=[Fraction(token) for token in values[0][1:]]
        width=fields['width'][0];assert width>=0
        if width==0:zero+=1
        else:
            if fields['start']==fields['end']:
                disks.append({'source_segment_index':total,'positive_segment_index':positive,
                    'center':[str(v) for v in fields['start']],'width':str(width),'radius':str(width/2),
                    'original_source_record':record})
            positive+=1
        total+=1
    value={'source':str(path),'source_sha256':sha(path),'segments':total,'zero_width':zero,
        'positive_segments':positive,'disks':disks,'independent_arithmetic':'Python fractions.Fraction directly from source tokens, no binary64 conversion'}
    dump(O/(name+'.json'),value);summaries[name]={'segments':total,'zero_width':zero,'positive_segments':positive,'disks':len(disks),'manifest_sha256':sha(O/(name+'.json'))}
assert all(sha(p)==h for p,h in archive_pins.items())
dump(O/'result.json',{'complete':True,'runner_sha256':sha(__file__),'archive_pins':archive_pins,'fixtures':summaries})
print(json.dumps(summaries),flush=True)
