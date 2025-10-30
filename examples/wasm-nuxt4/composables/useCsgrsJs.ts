// Make sure we get the typing from the csgrs WASM module
import type { 
  MeshJs, 
  SketchJs, 
  PlaneJs, 
  PolygonJs, 
  VertexJs, 
  Matrix4Js,
  ChromaSampling 
} from '../../../pkg/csgrs.js';

type CsgrsModule = {
  MeshJs: typeof MeshJs;
  SketchJs: typeof SketchJs;
  PlaneJs: typeof PlaneJs;
  PolygonJs: typeof PolygonJs;
  VertexJs: typeof VertexJs;
  Matrix4Js: typeof Matrix4Js;
  ChromaSampling: typeof ChromaSampling;
};

let wasmCache: CsgrsModule | null = null;
let wasmPromise: Promise<CsgrsModule> | null = null;

export const useCsgrsJs = () => 
{
  const loadWasm = async (): Promise<CsgrsModule> => 
  {
    // If already loaded, return cached instance
    if (wasmCache) {
      return wasmCache;
    }

    // If loading is in progress, return the same promise
    if (wasmPromise) {
      return wasmPromise;
    }

    // Start loading
    wasmPromise = (async () => {
      const timeStartLoad = performance.now();
      //const csgrs = await import('../../../pkg/csgrs.js');
      const csgrs = await import('csgrs-js/csgrs.js');
      console.log(`WASM load time: ${performance.now() - timeStartLoad} ms`);
      wasmCache = csgrs;
      wasmPromise = null; // Reset promise after loading
      return csgrs;
    })();

    return wasmPromise;
  };

  return {
    loadWasm,
    isLoaded: () => wasmCache !== null
  };
};
