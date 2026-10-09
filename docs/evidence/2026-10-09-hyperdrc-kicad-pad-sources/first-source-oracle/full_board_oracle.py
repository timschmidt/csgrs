"""Independent rational/√2 formulas for the complete fixtures' pad placement and primitive envelopes.

This is a source-contract oracle, not a KiCad executable differential. It does
not reproduce KiCad integer-unit rounding. Custom primitives validate placement
only; their local geometry remains outside this exact primitive audit.
"""
from fractions import Fraction as F
from pathlib import Path
import hashlib
import json
import re
import sys

def parse(text):
    stack = [[]]
    for match in re.finditer(r'"(?:\\.|[^"\\])*"|[^\s()]+|[()]', text):
        token = match.group()
        if token == '(':
            stack.append([])
        elif token == ')':
            stack[-2].append(stack.pop())
        else:
            stack[-1].append(json.loads(token) if token.startswith('"') else token)
    assert len(stack) == 1 and len(stack[0]) == 1
    return stack[0][0]

def children(node, key):
    return [v for v in node if isinstance(v, list) and v and v[0] == key]
def child(node, key, default):
    values = children(node, key)
    assert len(values) <= 1
    return values[0][1:] if values else default

def real(a=0, b=0): return (F(a), F(b))
def add(x, y): return (x[0]+y[0], x[1]+y[1])
def neg(x): return (-x[0], -x[1])
def mul(x, y): return (x[0]*y[0]+2*x[1]*y[1], x[0]*y[1]+x[1]*y[0])
def absolute(x):
    assert not x[0] or not x[1]
    return abs(x[0]), abs(x[1])
def trig(angle):
    angle = F(angle) % 360
    assert angle % 45 == 0
    c = [real(1), real(0,F(1,2)),real(),real(0,-F(1,2)),real(-1),real(0,-F(1,2)),real(),real(0,F(1,2))][int(angle/45)]
    s = [real(),real(0,F(1,2)),real(1),real(0,F(1,2)),real(),real(0,-F(1,2)),real(-1),real(0,-F(1,2))][int(angle/45)]
    return c, s

def rotate(x, y, angle):
    c,s = trig(angle)
    return add(mul(x,c),mul(y,s)), add(mul(y,c),neg(mul(x,s)))
def encoded(x): return [[str(v.numerator),str(v.denominator)] for v in x]
def rectangle(center,w,h,angle,r=F(0)):
    c,s = map(absolute,trig(angle))
    ex=add(add(mul(real(w/2-r),c),mul(real(h/2-r),s)),real(r))
    ey=add(add(mul(real(w/2-r),s),mul(real(h/2-r),c)),real(r))
    return dict(center=[encoded(v) for v in center],bounds=[encoded(add(center[0],neg(ex))),encoded(add(center[1],neg(ey))),encoded(add(center[0],ex)),encoded(add(center[1],ey))],area=encoded((w*h-4*r*r,r*r)))
def circle(center,r): return dict(center=[encoded(v) for v in center],bounds=[encoded(add(center[0],real(-r))),encoded(add(center[1],real(-r))),encoded(add(center[0],real(r))),encoded(add(center[1],real(r)))],area=encoded((0,r*r)))

def expected(path):
    data=path.read_bytes();root=parse(data.decode());rows=[];shapes={};angles={}
    declared=[item[1] for item in child(root,'layers',[]) if item[1].endswith('.Cu')]
    nets={v[1]:v[2] for v in children(root,'net')}
    for fp_index,fp in enumerate(children(root,'footprint')):
        fp_at=child(fp,'at',['0','0','0']);fp_angle=fp_at[2] if len(fp_at)>2 else '0'
        for pad_index,pad in enumerate(children(fp,'pad')):
            layers=child(pad,'layers',[]);expanded=[]
            for layer in layers:
                for layer in (declared if layer=='*.Cu' else ['F.Cu','B.Cu'] if layer=='F&B.Cu' else [layer]):
                    if layer.endswith('.Cu') and layer not in expanded:expanded.append(layer)
            if not expanded:continue
            shape=pad[3];shapes[shape]=shapes.get(shape,0)+1
            at=child(pad,'at',['0','0','0']);angle=at[2] if len(at)>2 else '0';angles[angle]=angles.get(angle,0)+1
            local=rotate(real(at[0]),real(at[1]),fp_angle);center=[add(real(fp_at[i]),local[i]) for i in range(2)]
            w,h=map(lambda x:abs(F(x)),child(pad,'size',[]));net=child(pad,'net',[]);net=net[1] if len(net)>1 else nets.get(net[0]) if net else None
            if shape=='circle': parts=[circle(center,max(w,h)/2)]
            elif shape in ['rect','roundrect']:
                ratio=F(child(pad,'roundrect_rratio',['0.25'])[0]) if shape=='roundrect' else F(0)
                assert not children(pad,'chamfer')
                parts=[rectangle(center,w,h,angle,min(w,h)*min(F(1,2),max(F(0),ratio)))]
            elif shape=='oval':
                r=min(w,h)/2
                if w==h:parts=[circle(center,r)]
                else:
                    parts=[rectangle(center,abs(w-h) if w>h else w,h if w>h else abs(w-h),angle)]
                    for sign in [-1,1]:
                        delta=sign*abs(w-h)/2;dx,dy=rotate(real(delta if w>h else 0),real(delta if h>w else 0),angle)
                        parts.append(circle([add(center[0],dx),add(center[1],dy)],r))
            elif shape=='custom':parts=[dict(center=[encoded(v) for v in center],bounds=None,area=None)]
            else:raise AssertionError(shape)
            for layer in expanded:
                for part in parts:rows.append(dict(footprint=fp_index,pad=pad_index,shape=shape,layer=layer,net=net,anchor=[encoded(v) for v in center],**part))
    return dict(source=str(path),input_sha256=hashlib.sha256(data).hexdigest(),scope='All copper pads; exact position and native area/envelope of rect, roundrect, circle and oval components. Custom local geometry excluded.',shapes=shapes,angles=angles,rows=rows)

if __name__=='__main__':
    inputs,out=map(Path,sys.argv[1:3]);out.mkdir(parents=True,exist_ok=False)
    for label,name in [('cparti-fpga','CPArti FPGA dev board.kicad_pcb'),('hvp109a','HVP109A.kicad_pcb')]:
        result=expected(inputs/label/name);(out/(label+'.json')).write_text(json.dumps(result,separators=(',',':'))+'\n');print(label,len(result['rows']),result['shapes'])
