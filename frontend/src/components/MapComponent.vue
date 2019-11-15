<template>
  <LMap ref="scheduleMap">
  </LMap>
</template>

<script lang="ts">

import axios from 'axios';
import * as L from 'leaflet';
import { Component, Vue } from 'vue-property-decorator';
import { LMap } from 'vue2-leaflet';

import { getFloors } from '../mock'
import { rawCompressedSvgToSvgElement } from '../util';

@Component({
  components: {
    LMap,
  },
})
export default class MapComponent extends Vue {
  private readonly _IMAGE_SCALE = 0.0125;

  private _maxBounds: L.LatLngBounds;

  public mounted = () => {
    this.$nextTick(async () => {
      const floors = await getFloors();

      console.log(floors);
      for (const floor of floors) {
        const { elevation, image } = floor;
        const { pose, scale } = image;
        const { x, y } = pose;
        const svgElement = rawCompressedSvgToSvgElement(image.data);
        const height = svgElement.height.baseVal.value * scale / this._IMAGE_SCALE;
        const width = svgElement.width.baseVal.value * scale / this._IMAGE_SCALE;

        const offsetPixelsX = x / this._IMAGE_SCALE;
        const offsetPixelsY = y / this._IMAGE_SCALE;

        const bounds = new L.LatLngBounds(
          new L.LatLng(offsetPixelsX, offsetPixelsY, elevation),
          new L.LatLng(
            offsetPixelsY - height,
            offsetPixelsX + width,
            elevation,
          ),
        );

        if (!this._maxBounds) {
          this._maxBounds = bounds;
        } else {
          this._maxBounds.extend(bounds);
        }
      }
      // this.$refs.scheduleMap.mapObject.LEAFLET_METHOD
    });
  };
}
</script>