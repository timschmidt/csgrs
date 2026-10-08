from pathlib import Path
import collections, hashlib, json, re, sys
root=Path('/home/tim/Documents/GitHub/workspace')
repo=root/sys.argv[1]
source=Path(sys.argv[2])
allowed_defs={file:set(names) for file,names in json.loads(Path('bezier-bounds-policy-declarations.json').read_text()).items()}
allowed_values={'policy','&policy','attempt','&attempt','&CurveContext::STRICT','&CurveContext::APPROXIMATE_512','&policy()','&self.data.policy','strict','&strict'}
edits={}
for line in source.read_text().splitlines():
 obj=json.loads(line)
 if obj.get('reason')!='compiler-message':continue
 message=obj['message']
 if (message.get('code') or {}).get('code')!='E0061':continue
 notes=[s for c in message['children'] if c['level']=='note' and 'defined here' in c['message'] for s in c['spans']]
 assert notes,message['rendered']
 for note in notes:
  path=(repo/note['file_name']).resolve()
  key=str(path.relative_to(root/'hypercurve'))
  declaration=path.read_bytes()[note['byte_start']:note['byte_end']].decode()
  assert declaration in allowed_defs.get(key,set()),(key,declaration)
 arguments=[s for s in message['spans'] if (s.get('label') or '').startswith('unexpected argument')]
 assert len(arguments)==1,message['rendered']
 arg=arguments[0]
 assert 'CurveContext' in arg['label'] or (arg['label']=='unexpected argument' and arg['file_name']=='src/rational_bezier_general.rs' and (repo/arg['file_name']).read_bytes()[arg['byte_start']:arg['byte_end']]==b'policy'),message['rendered']
 for child in message['children']:
  for span in child['spans']:
   replacement=span.get('suggested_replacement')
   if replacement is None:continue
   assert replacement=='',message['rendered']
   path=(repo/span['file_name']).resolve()
   assert path.is_relative_to(repo.resolve()),path
   a,b=span['byte_start'],span['byte_end']
   old=path.read_bytes()[a:b].decode()
   assert old.strip().strip(',').strip() in allowed_values,old
   arg=arguments[0]
   assert arg['file_name']==span['file_name'] and a<=arg['byte_start']<arg['byte_end']<=b,(arg,span)
   edits[(path,a,b)]=old
by_file=collections.defaultdict(list)
for (path,a,b),old in edits.items():by_file[path].append((a,b,old))
audit=[]
for path,changes in sorted(by_file.items()):
 before=path.read_bytes(); after=before
 previous=0
 for a,b,old in sorted(changes):
  assert previous<=a<b and before[a:b].decode()==old
  previous=b
 for a,b,old in sorted(changes,reverse=True):after=after[:a]+after[b:]
 audit.append({'file':str(path.relative_to(root)),'before_sha256':hashlib.sha256(before).hexdigest(),'after_sha256':hashlib.sha256(after).hexdigest(),'deletions':[{'byte_start':a,'byte_end':b,'old':old} for a,b,old in sorted(changes)]})
# Validation completes for every file before writing any source.
for record in audit:
 path=root/record['file']; data=path.read_bytes()
 for edit in reversed(record['deletions']):data=data[:edit['byte_start']]+data[edit['byte_end']:]
 assert hashlib.sha256(data).hexdigest()==record['after_sha256']
 path.write_bytes(data)
Path(source.stem+'-applied.json').write_text(json.dumps(audit,indent=2)+'\n')
print('removed',len(edits),'arguments across',len(audit),'files')
