<template>
    <div>
        <ClientOnly>
            <TresCanvas window-size clear-color="#EEE">
                <TresPerspectiveCamera :position="[0, -50, 50]" :up="[0, 0, 1]" :look-at="[0, 0, 0]" />
                <TresAmbientLight :intensity="2" />
                <OrbitControls />
                <!-- Make Z-axis up -->
                <TresAxesHelper :args="[50]" :rotation="[Math.PI / 2, 0, 0]"/>
                <TresGridHelper :args="[100, 10]" :rotation="[Math.PI / 2, 0, 0]"/>
                <TresDirectionalLight :position="[1000, 500, 1000]" :intensity="5" />
                <TresDirectionalLight :position="[-1000, -500, 1000]" :intensity="5" />
                
                <!-- solid faces -->
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
                        :side="2"
                        />
                    <!-- NOTE: side does not seem to work to fix normal problems-->
                </TresMesh>
                
                <!-- wireframe -->
                <TresMesh
                    :visible="showWireframe"
                    v-for="(meshAttributes, index) in meshesTres" :key="index">
                    <TresBufferGeometry 
                        :position="meshAttributes.position"
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
                <span class="metric" v-for="(value, key) in metrics" :key="key">{{ key }}: {{ Math.round(value) }} ms</span>
            </div>
        </ClientOnly>
    </div>
</template>

<script setup lang="ts">

import { TresCanvas } from '@tresjs/core'
import { OrbitControls } from '@tresjs/cientos'
import { DoubleSide } from 'three'

import { ref, defineProps  } from 'vue'
import type { CsgRsMeshArrays, TresBufferGeometryAttributes } from '../types';

const showWireframe = ref<boolean>(false);

const props = defineProps<{
  meshes?: Array<CsgRsMeshArrays>,
  metrics?: Record<string, number> // some misc performance metrics
}>();

// Convert CsgRsMeshArrays to TresBufferGeometryAttributes
const meshesTres = computed(() => 
{
  if(!props || !props?.meshes) return [];
  return toRaw(props.meshes).map(mesh => {
    return   {
        position: [new Float32Array(Array.from(mesh.positions)), 3],
        normal: [new Float32Array(Array.from(mesh.normals)), 3],
        index: [new Uint32Array(Array.from(mesh.indices)), 1] // TODO: why is Tres complaining about this? 
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

.metric {
    display: inline-block;
    margin-left: 1em;
    color: black;
    opacity:0.8;
}

</style>