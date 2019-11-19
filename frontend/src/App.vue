<template>
  <div id="app">
    <MapComponent />
  </div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import HelloWorld from './components/HelloWorld.vue';
import MapComponent from './components/MapComponent.vue';

@Component({
  components: {
    HelloWorld,
    MapComponent,
  },
})
export default class App extends Vue {}
</script>

<style lang="scss">
body, html, #app {
  height: 100%;
  width: 100%;
  margin: 0;
  padding: 0;
}

#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
}
</style>
