<template>
  <div >
    
  </div>
</template>

<script setup>

import { ref } from 'vue';

const stl = ref('');

const loadWasm = async () => 
{
  
  const csgrs = (await import('../../../pkg/csgrs.js'));    
  return csgrs;
}

// Ensure this code runs only on the client side
if (process.client)
{
  

  await loadWasm().then((csgrs) => 
  {
    /*
    // DEBUG
    console.log(csgrs)
    console.log(csgrs.PlaneJs);
    */

    // Create a triangle plane mesh from 3 vertices on XY plane
    // No need for decimal points
    const plane = new csgrs.PlaneJs(
      -50, 50, 0,
      50, 0, 0,
      0, 50, 0 
    );

    // Make a 2D Sketch
    const sketch = new csgrs.SketchJs();
    console.log(sketch.isEmpty()); // true
    
    /*
    sketch.polygon([
      -50, 50,
      50, 0,
      0, 50
    ]);
    */

    const circle = csgrs.SketchJs.circle(10, 32);
    // console.log(circle.center()); // Expect the center point, is center operation
    console.log(circle.boundingBox()); // { min: [-10,-10,0], max: [10,10,0] }
    const translatedCircle = circle.translate(100,0,0); // Expect in-place translation
    console.log(translatedCircle.boundingBox()); // { min: [90,-10,0], max: [110,10,0] }
    console.log(circle.toArrays().indices.length); // number of vertices for circle

    const rectangle = csgrs.SketchJs.rectangle(50,100);
    console.log(rectangle.boundingBox()); // { min: [0,0,0], max: [10,100,0] }
    console.log(rectangle.toArrays().indices.length); // number of vertices for rectangle


    const unionShape = circle.union(rectangle);
    console.log(unionShape.toArrays().indices.length); // number of vertices after union

    const extruded = unionShape.extrude(20);
    stl.value = extruded.toSTLBinary();


  });
}
  

</script>