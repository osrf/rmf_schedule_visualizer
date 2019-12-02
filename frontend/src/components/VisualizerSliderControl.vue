<template>
  <l-control class="visualizer-slider-control" position="bottomcenter">
    hi
  </l-control>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator';
import { LControl } from 'vue2-leaflet';

@Component({
  name: 'visualizer-slider-control',
  components: {
    LControl,
  },
})
export default class VisualizerSliderControl extends Vue {

}
</script>