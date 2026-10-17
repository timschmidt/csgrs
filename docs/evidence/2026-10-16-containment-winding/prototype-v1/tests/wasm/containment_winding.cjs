const assert = require('node:assert/strict');
const { test } = require('node:test');
const path = require('node:path');
const { MeshJs, Point3Js, GeometryCertaintyJs, GeometryQueryStatusJs } = require(path.resolve(process.argv[2]));

function source(boxes) {
  const unit = MeshJs.cube(1);
  let template;
  try { template = unit.toArrays(); } finally { unit.free(); }
  const positions=[],indices=[];
  for (const [min,max,sign] of boxes) {
    const base=positions.length/3;
    positions.push(...Array.from(template.positions,(v,i)=>min[i%3]+v*(max[i%3]-min[i%3])));
    for (let i=0;i<template.indices.length;i+=3) {
      const face=Array.from(template.indices.slice(i,i+3));
      if (sign<0) face.reverse();
      indices.push(...face.map(v=>base+v));
    }
  }
  return MeshJs.fromArrays(Float64Array.from(positions),Uint32Array.from(indices));
}
function expectedInterior(boxes,twice) {
  for (const dx of [-1,1]) for (const dy of [-1,1]) for (const dz of [-1,1]) {
    const sample=twice.map((v,i)=>4*v+[dx,dy,dz][i]);
    const winding=boxes.reduce((sum,[min,max,sign])=>sum+(sample.every((v,i)=>8*min[i]<v && v<8*max[i])?sign:0),0);
    if (winding===0) return false;
  }
  return true;
}
const cases=[
  ['nested material',[[[0,0,0],[3,3,3],1],[[1,1,1],[2,2,2],1]],[[3,3,3],[2,3,3],[2,2,3],[2,2,2],[0,3,3]]],
  ['coincident cancellation',[[[0,0,0],[2,2,2],1],[[0,0,0],[2,2,2],-1]],[[1,1,1],[0,1,1],[0,0,0]]],
  ['overlapping material',[[[0,0,0],[2,2,2],1],[[1,0,0],[3,2,2],1]],[[3,1,1],[2,1,1],[4,1,1],[1,1,1]]],
  ['opposite overlap',[[[0,0,0],[2,2,2],1],[[1,0,0],[3,2,2],-1]],[[3,1,1],[1,1,1],[5,1,1],[2,1,1]]],
  ['shared internal face',[[[0,0,0],[1,2,2],1],[[1,0,0],[2,2,2],1]],[[2,1,1],[2,2,2],[2,0,1]]],
  ['cavity boundary',[[[0,0,0],[3,3,3],1],[[1,1,1],[2,2,2],-1]],[[3,3,3],[1,1,1],[2,3,3],[2,2,2]]],
  ['tangent ray contact',[[[16,6,4],[17,7,5],1],[[-2,-2,-2],[-1,-1,-1],1]],[[0,0,0],[33,13,9],[-3,-3,-3]]],
  ['reversed multiplicity',[[[0,0,0],[2,2,2],-1],[[0,0,0],[2,2,2],-1]],[[1,1,1],[0,1,1],[5,1,1]]],
];
for (const [name,boxes,points] of cases) {
  test('nonzero set containment: '+name,()=>{
    const mesh=source(boxes),empty=new MeshJs();
    let composed;
    try {
      const outcome=mesh.unionWithContext(empty,false);
      assert.equal(outcome.certainty,GeometryCertaintyJs.Certified);
      composed=outcome.intoMesh();
      for (const twice of points) {
        const expected=expectedInterior(boxes,twice),point=new Point3Js(...twice.map(v=>v/2));
        try {
          for (const input of [mesh,composed]) {
            assert.equal(input.containsPoint(point),expected,JSON.stringify(twice));
            for (const approximate of [false,true,false]) {
              const result=input.containsPointWithContext(point,approximate);
              try {
                assert.equal(result.status,GeometryQueryStatusJs.Decided);
                assert.equal(result.certainty,GeometryCertaintyJs.Certified);
                assert.equal(result.value,expected,JSON.stringify(twice));
              } finally { result.free(); }
            }
          }
        } finally { point.free(); }
      }
    } finally { if (composed) composed.free();empty.free();mesh.free(); }
  });
}
