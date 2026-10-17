const assert = require('node:assert/strict');
const path = require('node:path');
const { MeshJs, Point3Js } = require(path.resolve(process.argv[2]));
function cubeRows(scale, offset) {
  const cube = MeshJs.cube(1);
  try {
    const arrays = cube.toArrays();
    return {
      positions: Array.from(arrays.positions, value => value * scale + offset),
      indices: Array.from(arrays.indices)
    };
  } finally { cube.free(); }
}
function join(left,right,reverse) {
  const offset = left.positions.length / 3;
  let indices = right.indices.slice();
  if (reverse) for (let i=0;i<indices.length;i+=3) [indices[i],indices[i+2]]=[indices[i+2],indices[i]];
  return MeshJs.fromArrays(Float64Array.from([...left.positions,...right.positions]),
    Uint32Array.from([...left.indices,...indices.map(value=>value+offset)]));
}
const cases = [
  ['nested_same_orientation', cubeRows(3,0),cubeRows(1,1),false,[1.5,1.5,1.5],true,2],
  ['coincident_opposite_orientation', cubeRows(1,0),cubeRows(1,0),true,[0.5,0.5,0.5],false,0],
];
const rows = [];
for (const [name,left,right,reverse,xyz,expected,winding] of cases) {
  const mesh=join(left,right,reverse),point=new Point3Js(...xyz);
  try {
    for (const approximate of [false,true,false]) {
      const result=mesh.containsPointWithContext(point,approximate);
      try {
        rows.push({name,approximate,expected_nonzero_winding_membership:expected,independently_derived_winding_magnitude:winding,
          observed:result.value,status:result.status,certainty:result.certainty,reason:result.reason});
      } finally { result.free(); }
    }
  } finally { point.free();mesh.free(); }
}
assert.equal(rows.length,6);
console.log(JSON.stringify(rows,null,2));
