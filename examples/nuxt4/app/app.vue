<template>
  <div>
    csgrs-wasm
  </div>
</template>

<script setup>

const loadCsgrs = async () => 
{
  if (process.client) 
  {
    const csgrs = (await import('../../../bin/csgrs_wasm.js'));

    console.log(csgrs)

    const vertex = new csgrs.VertexJs(1.0, 2.0, 3.0)
    console.log(vertex)
    console.log('Vertex array:', vertex.to_array())
    
    return csgrs;
  }
}

await loadCsgrs();

</script>