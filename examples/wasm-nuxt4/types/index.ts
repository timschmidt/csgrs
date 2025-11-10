import { BufferAttribute } from 'three';

export interface CsgRsMeshArrays {
  positions: Float64Array; // vertex positions
  normals: Float64Array;   // vertex normals
  indices: Uint32Array;    // face indices
}

export interface TresBufferGeometryAttributes 
{
    position: [ArrayLike<number>, number];
    normal: [ArrayLike<number>, number];
    index: [ArrayLike<number>, number];
}