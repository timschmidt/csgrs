<template>
  <div>
    <header>
        <span id="title"><a href="https://github.com/timschmidt/csgrs/tree/main/examples/nuxt4">csgrs-js</a></span>      
          <select @change="goToPage($event)">
            
            <option 
              v-for="(description, pagePath) in examplePagesOrdered" 
              :key="pagePath" 
              :value="pagePath">
              {{ (currentPathPage === pagePath) ? '>>' : '' }}{{ description.label }}
            </option>
          </select>
          {{ examplePages[currentPathPage]?.description }}
    </header> 
    <slot />
  </div>
</template>

<script setup lang="ts">

const examplePages = ref(
  {
     '/': { label: 'Hello Boolean', description: 'With CSGs you can subtract, add/union and intersect meshes to create complex shapes.' },
     primitives: { label: 'Primitives', description: 'Create basic 3D shapes like boxes, spheres, cylinders, tori and gyroids.' },  
     'animated-booleans' : { label: 'Animated booleans', description: 'Booleans can be fast enough to animate smoothly!' },
     'array-boolean' : { label: 'Array boolean', description: 'An array of a lot of boxes with subtracted big box' },
     'bigger-boolean' : { label: 'Bigger boolean', description: 'More complex shape boolean operations.' },
     sketch : { label: 'Sketch 2D shapes', description: 'Create 2D sketches and extrude them into 3D meshes.' },
  }
)

// Get current page from route, removing leading slash
const route = useRoute()
const currentPathPage = computed(() => {
  return (route.path === '/') ? '/' : route.path.slice(1) // Remove leading '/'
})

// Place current path first
const examplePagesOrdered = computed(() => 
{
   const currentPageEntry = Object.entries(examplePages.value).find(([key, value]) => key === currentPathPage.value);
   const otherPageEntries = Object.entries(examplePages.value).filter(([key, value]) => key !== currentPathPage.value);
   
   const orderedEntries = currentPageEntry ? [currentPageEntry, ...otherPageEntries] : otherPageEntries;
   
   return Object.fromEntries(orderedEntries);
})

function goToPage(e: Event)
{
  const selectedPage = (e.target as HTMLSelectElement).value;
  if(currentPathPage.value !== selectedPage)
  {
    navigateTo(`/` + selectedPage);
  }
}
</script>

<style scoped>


header {
  display:block;
  position: fixed;
  background: #f5f5f5;
  z-index:1000;
  text-align: center;
}

#title {
  font-weight: bold;
  font-size: 24px;
  margin-right: 20px;
}

#title a {
  text-decoration: none;
  color: #333;
}

#title a:hover {
  text-decoration: underline;
  color: blue;
}

</style>