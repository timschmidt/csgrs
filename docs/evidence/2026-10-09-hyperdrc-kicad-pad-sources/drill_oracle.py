"""Independent exact source drill centers and clearance pairs for both full boards.

Oval/rectangular drill declarations retain the existing conservative circular
proxy (larger axis); this does not certify routed-slot geometry.
"""
from full_board_oracle import *

def sign(x):
    a,b=x
    if not b:return (a>0)-(a<0)
    if not a:return (b>0)-(b<0)
    if (a>0)==(b>0):return (a>0)-(a<0)
    order=(a*a>2*b*b)-(a*a<2*b*b)
    return order if a>0 else -order

def diameter(drill):return max(F(drill[1]),F(drill[2])) if drill[0] in ['oval','rect'] else F(drill[0])
def enclosure(x):return x[0]-2*abs(x[1]),x[0]+2*abs(x[1])
def run(path):
    root=parse(path.read_text());drills=[]
    for fi,fp in enumerate(children(root,'footprint')):
        at=child(fp,'at',['0','0','0']);angle=at[2] if len(at)>2 else '0'
        for pi,pad in enumerate(children(fp,'pad')):
            drill=child(pad,'drill',[])
            if not drill:continue
            at_pad=child(pad,'at',['0','0','0']);pad_angle=at_pad[2] if len(at_pad)>2 else '0'
            x,y=rotate(real(at_pad[0]),real(at_pad[1]),angle);center=[add(real(at[0]),x),add(real(at[1]),y)]
            offset=child(['drill',*drill],'offset',['0','0']);x,y=rotate(real(offset[0]),real(offset[1]),pad_angle);center=[add(center[0],x),add(center[1],y)]
            drills.append(dict(source=['pad',fi,pi],center=center,diameter=diameter(drill),plated=pad[2]!='np_thru_hole'))
    for i,via in enumerate(children(root,'via')):
        at=child(via,'at',[]);drill=child(via,'drill',child(via,'size',[]));assert drill
        drills.append(dict(source=['via',i],center=[real(at[0]),real(at[1])],diameter=diameter(drill),plated=True))
    clearance=F(1,4);largest=max(d['diameter'] for d in drills);intervals=[tuple(enclosure(v) for v in d['center']) for d in drills]
    ordered=sorted(range(len(drills)),key=lambda i:intervals[i][0][0]);pairs=[];tested=0
    for k,i in enumerate(ordered):
        left=drills[i];left_x,left_y=intervals[i]
        for j in ordered[k+1:]:
            right=drills[j];right_x,right_y=intervals[j]
            if right_x[0]-left_x[1]>=(left['diameter']+largest)/2+clearance:break
            radius=(left['diameter']+right['diameter'])/2+clearance
            if right_y[0]-left_y[1]>=radius or left_y[0]-right_y[1]>=radius:continue
            dx=add(left['center'][0],neg(right['center'][0]));dy=add(left['center'][1],neg(right['center'][1]));tested+=1
            if sign(add(add(mul(dx,dx),mul(dy,dy)),real(-radius*radius)))<0:pairs.append(sorted([i,j]))
    return dict(source=str(path),input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),clearance='1/4',drills=[dict(**{k:v for k,v in d.items() if k not in ['center','diameter']},center=[encoded(v) for v in d['center']],diameter=encoded((d['diameter'],0))) for d in drills],pairs=sorted(pairs),exact_pair_tests=tested,scope='Exact rational/quadratic-surd source centers and STRICT clearance comparison; conservative larger-axis circular proxy for oval drills; independent rational sweep and squared-distance formula, not the production index or distance routine.')

if __name__=='__main__':
    inputs,out=map(Path,sys.argv[1:3]);out.mkdir(parents=True,exist_ok=False)
    for label,name in [('cparti-fpga','CPArti FPGA dev board.kicad_pcb'),('hvp109a','HVP109A.kicad_pcb')]:
        result=run(inputs/label/name);(out/(label+'.json')).write_text(json.dumps(result,separators=(',',':'))+'\n');print(label,len(result['drills']),len(result['pairs']),flush=True)
