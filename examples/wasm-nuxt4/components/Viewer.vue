<template>
    <div>
        <ClientOnly>
            <TresCanvas window-size clear-color="#EEE">
                <TresPerspectiveCamera :position="[0, -50, 50]" :up="[0, 0, 1]" :look-at="[0, 0, 0]" />
                <TresAmbientLight :intensity="0.5" />
                <TresDirectionalLight :position="[1000, 500, 1000]" :intensity="5" />
                <OrbitControls />
                <!-- Make Z-axis up -->
                <TresAxesHelper :args="[50]" :rotation="[Math.PI / 2, 0, 0]"/>
                <TresGridHelper :args="[100, 10]" :rotation="[Math.PI / 2, 0, 0]"/>
                <!-- render the meshes two times
                    the first time with solid shaded faces
                -->
                
                <TresMesh
                    v-for="(meshAttributes, index) in meshesTres" :key="index">
                    <TresBufferGeometry 
                        :position="meshAttributes.position"
                        :normal="meshAttributes.normal"
                        :index="meshAttributes.index"
                    />
                    <TresMeshStandardMaterial 
                        color="#1565C0" 
                        :metalness="0.5" 
                        :roughness="0.5" 
                        />
                </TresMesh>
                
                <!-- wireframe -->
                <TresMesh
                    :visible="showWireframe"
                    v-for="(meshAttributes, index) in meshesTres" :key="index">
                    <TresBufferGeometry 
                        :position="meshAttributes.position"
                        :normal="meshAttributes.normal"
                        :index="meshAttributes.index"
                    />
                    <TresMeshStandardMaterial 
                        :wireframe="true"
                        color="#FFFFFF" 
                        :metalness="0.5" 
                        :roughness="0.5" 
                        :depthTest="false"
                    />

                </TresMesh>
                
            </TresCanvas>
            <div id="debug-controls">
                <input type="checkbox" v-model="showWireframe">Show Wireframe</input>
            </div>
        </ClientOnly>
    </div>
</template>

<script setup lang="ts">

import { TresCanvas } from '@tresjs/core'
import { OrbitControls } from '@tresjs/cientos'

import { ref, defineProps  } from 'vue'
import type { CsgRsMeshArrays, TresBufferGeometryAttributes } from '../types';

const showWireframe = ref(true);

const props = defineProps<{
  meshes?: Array<CsgRsMeshArrays>
}>();

// Convert CsgRsMeshArrays to TresBufferGeometryAttributes
const meshesTres = computed(() => 
{
  if(!props || !props?.meshes) return [];
  return props.meshes.map(mesh => {
    console.log(mesh);
    return   {
        position: [new Float32Array(Array.from(mesh.positions)), 3],
        normal: [new Float32Array(Array.from(mesh.normals)), 3],
        index: [new Uint32Array(Array.from(mesh.indices)), 1]
      }  as TresBufferGeometryAttributes
  })
})



</script>

<style scoped>

h1 {
    color: #333;
}

#debug-controls
{
    font-family: Arial, sans-serif;
    position: absolute;
    bottom: 10px;
    left: 10px;
    background: rgba(0,0,0, 0.5);
    padding: 10px;
    border-radius: 5px;
    color: white;
    font-size: 0.8em;
}

#debug-controls input 
{
    margin-right: 1em;
}

</style>