<template>
  <div>
    <Viewer :meshes="meshes" :metrics="metrics"></Viewer>
  </div>
</template>

<script setup lang="ts">

import { ref } from 'vue';

import type { CsgRsMeshArrays } from '../../types';
import { useCsgrsJs } from '../../composables/useCsgrsJs';
const { loadWasm } = useCsgrsJs();

import Viewer from '../../components/Viewer.vue';

const meshes = ref<Array<CsgRsMeshArrays>>();
const metrics = ref<Record<string, number>>({});

// Ensure this code runs only on the client side
if (import.meta.client)
{
  await loadWasm().then((csgrs) => 
  {
    const timeStartGeneration = performance.now();

    const smallBox = csgrs.MeshJs.cuboid(100,5,5).translate(-50,-2.5,-2.5);
    const smallBoxColl = smallBox.distributeLinear(100, 0, 10, 0, 10);
    const subBox = csgrs.MeshJs.cuboid(100,1500,10)
                  .rotate(0,0,5)
                  .translate(50,-20,-5);   

    const diffShape = smallBoxColl
                        .difference(subBox);

    meshes.value = [
                    //smallBoxColl.to_arrays() as CsgRsMeshArrays, 
                    //subBox.to_arrays() as CsgRsMeshArrays, 
                    diffShape.to_arrays() as CsgRsMeshArrays
                    ];

    metrics.value['generation'] = performance.now() - timeStartGeneration;
  });
}
  

</script>