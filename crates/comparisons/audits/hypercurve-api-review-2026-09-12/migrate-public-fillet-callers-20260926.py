from pathlib import Path
import re,json,sys,collections
A=Path(__file__).resolve().parent;W=A.parent;H=W/'hypercurve'
errors=json.loads((A/sys.argv[1]).read_text())
byfile=collections.defaultdict(list)
for e in errors:
 for sp in e['spans']:
  if sp['is_primary']:
   while not sp['file_name'].startswith(('src/','tests/','benches/')) and sp.get('expansion'):sp=sp['expansion']['span']
   if sp['file_name'].startswith(('src/','tests/','benches/')):byfile[sp['file_name']].append((e,sp))
def mask(s):
 pattern=r'//[^\n]*|/\*[\s\S]*?\*/|r(#+)"[\s\S]*?"\1|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])\''
 return re.sub(pattern,lambda m:' '*len(m.group()),s)
def close(s,start):
 m=mask(s);kind=m[start];end={'(':')','{':'}','[':']'}[kind];n=1
 for i in range(start+1,len(s)):
  if m[i]==kind:n+=1
  if m[i]==end:n-=1
  if n==0:return i+1
 raise Exception('not closed')
for file,rows in byfile.items():
 p=H/file;s=p.read_text();edits=[];done=set()
 def add(a,b,t):
  if (a,b) not in done:
   if any(a<d and b>c for c,d,_ in edits):return False
   done.add((a,b));edits.append((a,b,t));return True
  return False
 for e,sp in rows:
  a=len(s.encode()[:sp['byte_start']].decode());b=len(s.encode()[:sp['byte_end']].decode());code=e.get('code',{}).get('code');selected=s[a:b]
  if code=='E0599' and 'candidate_count' in e['message']:
   m=next((m for m in re.finditer(r'([A-Za-z_]\w*(?:\.\w+)*)\.candidate_count\(\)',s[max(0,a-100):b+3]) if max(0,a-100)+m.start()<=a<max(0,a-100)+m.end()),None)
   if m is None:continue
   base=max(0,a-100);expr=m[1];add(base+m.start(),base+m.end(),'{ assert!('+expr+'.families().is_empty()); '+expr+'.isolated_solutions().len() }');continue
  if code=='E0369':
   start=s.rfind('assert_eq!(',0,a+12)
   if start<0:continue
   end=close(s,start+10);txt=s[start:end]
   m=re.search(r',\s*(?:\w+::)*CurveCornerSolutions2::NoSolution\(',txt)
   if not m:continue
   expr=txt[11:m.start()].strip();pos=m.end()-1;stop=close(txt,pos)
   new='assert_eq!(('+expr+').no_solution_reason(), Some('+txt[pos+1:stop-1]+')'+txt[stop:]
   add(start,end,new);continue
  if code!='E0308':continue
  line=s[s.rfind('\n',0,a)+1:s.find('\n',b)]
  if file=='src/bezier_region.rs' and sp.get('label','').startswith('expected `&[CurveRegion2]`'):
   fn='fillet_regions' if 'CurveFilletSolutions2' in sp['label'] else 'corner_regions'
   add(a,b,fn+'('+selected+')');continue
  if 'for_each_corner_region' in line:
   start=s.rfind('for_each_corner_region',0,a+len('for_each_corner_region'))
   add(start,start+len('for_each_corner_region'),'for_each_fillet_region');continue
  if 'CurveCornerSolutions2' not in selected:continue
  start=s.rfind('let ',0,a+1)
  if start>=0 and re.fullmatch(r'let\s+(?:\w+::)*',s[start:a]):
   m=re.match(r'let\s+(?:\w+::)*CurveCornerSolutions2::(Unique|Multiple)\((\w+)\)\s*=\s*',s[start:])
   if not m:continue
   expr_start=start+m.end();em=re.search(r'\belse\s*\{',mask(s[expr_start:]))
   assert em;es=expr_start+em.start();brace=expr_start+em.end()-1;end=close(s,brace)
   rhs=s[expr_start:es].strip();var=m[2];unique=m[1]=='Unique'
   body='let solutions = '+rhs+';\nassert!(solutions.families().is_empty(), "expected isolated fillets");\nlet ('+('mut ' if unique else '')+'candidates, _) = solutions.into_parts();\n'
   body+=('assert_eq!(candidates.len(), 1, "expected one isolated fillet");\ncandidates.pop().unwrap()' if unique else 'assert!(candidates.len() > 1, "expected multiple isolated fillets");\ncandidates')
   if file.startswith('benches/') and unique:
    add(start,end,'let solutions = '+rhs+';\nassert!(solutions.families().is_empty());\nlet ['+var+'] = solutions.isolated_solutions() else { panic!("expected one isolated fillet"); }');continue
   add(start,end,'let '+var+' = {\n'+body+'\n}');continue
  # Simple vec collection matches, including zero-allowed finite queries.
  start=mask(s).rfind('match ',max(0,a-500),a)
  if start>=0:
   brace=mask(s).find('{',start);end=close(s,brace);txt=s[brace+1:end-1]
   pat=r'\s*(?:\w+::)*CurveCornerSolutions2::Unique\((\w+)\)\s*=>\s*vec!\[\1\],\s*(?:\w+::)*CurveCornerSolutions2::Multiple\((\w+)\)\s*=>\s*\2,'
   if re.match(pat,txt):
    expr=s[start+6:brace].strip();allow_empty='Vec::new()' in txt
    body='{ let solutions = '+expr+'; assert!(solutions.families().is_empty(), "expected isolated fillets"); let (candidates, _) = solutions.into_parts(); '
    if not allow_empty:body+='assert!(!candidates.is_empty(), "expected at least one isolated fillet"); '
    add(start,end,body+'candidates }');continue
   pat=r'\s*(?:\w+::)*CurveCornerSolutions2::Unique\((\w+)\)\s*=>\s*(\w+)\(\1\),\s*(?:\w+::)*CurveCornerSolutions2::Multiple\((\w+)\)\s*=>\s*\{\s*\3\.iter\(\)\.any\(\2\)\s*\}\s*(?:\w+::)*CurveCornerSolutions2::NoSolution\(_\)\s*=>\s*false,?\s*'
   m=re.fullmatch(pat,txt)
   if m:
    expr=s[start+6:brace].strip().removeprefix('&');add(start,end,'{ assert!('+expr+'.families().is_empty()); '+expr+'.isolated_solutions().iter().any('+m[2]+') }');continue
 for a,b,t in sorted(edits,reverse=True):s=s[:a]+t+s[b:]
 if edits:p.write_text(s);print(file,len(edits))
