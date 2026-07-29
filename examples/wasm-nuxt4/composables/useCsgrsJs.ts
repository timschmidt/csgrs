// Make sure we get the typing from the csgrs WASM module
import type { 
  MeshJs, 
  CurveRegionJs,
  PlaneJs, 
  Matrix4Js,
} from '../../../pkg/csgrs.js';

type CsgrsModule = {
  MeshJs: typeof MeshJs;
  CurveRegionJs: typeof CurveRegionJs;
  PlaneJs: typeof PlaneJs;
  Matrix4Js: typeof Matrix4Js;
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
      //const csgrs = await import('../../../pkg/csgrs.js'); // use this is you build locally
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
