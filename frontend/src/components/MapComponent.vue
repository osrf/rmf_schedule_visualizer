<template>
  <l-map
    ref="scheduleMap"
    class="map"
    :options="MAP_OPTIONS"
    :crs="MAP_CRS"
  >
    <l-control-attribution
      prefix="OSRC-SG"
      position="bottomright"
    />
    <server-date-control :date="currentDate" />
    <l-control-layers />
    <l-layer-group
      v-for="floor in floors"
      :key="floor.name"
      :name="floor.name"
      layer-type="base"
      ref="floorLayers"
    >
      <svg-overlay
        :svgImage="floor.svgElement"
        :bounds="floor.bounds"
      />
    </l-layer-group>
    <visualizer-slider-control />
  </l-map>
</template>

<script lang="ts">

import * as L from 'leaflet';
import axios from 'axios';
import produce from 'immer';
import { Component, Vue } from 'vue-property-decorator';
import {
  LControl,
  LControlAttribution,
  LControlLayers,
  LLayerGroup,
  LMap,
} from 'vue2-leaflet';

import ServerDateControl from '../components/ServerDateControl.vue';
import SVGOverlay from '../components/SVGOverlay.vue';
import VisualizerSliderControl from '../components/VisualizerSliderControl.vue';
import { getFloors } from '../mock'
import { IFloor as _IFloor } from '../models/Floor';
import { rawCompressedSvgToSvgElement } from '../util';

export interface IFloor extends _IFloor {
  svgElement?: SVGElement;
  bounds?: L.LatLngBounds;
}

@Component({
  components: {
    ServerDateControl,
    'svg-overlay': SVGOverlay,
    LControl,
    LControlAttribution,
    LControlLayers,
    LLayerGroup,
    LMap,
    VisualizerSliderControl,
  },
})
export default class MapComponent extends Vue {
  public readonly MAP_OPTIONS = {
    minZoom: -2,
    maxZoom: 2,
    attributionControl: false,
  };
  public readonly MAP_CRS = L.CRS.Simple;

  private readonly IMAGE_SCALE = 0.0125;
  private readonly DEFAULT_FLOOR = 'B2';
  // private readonly MAP_LAYER_CONTROL = new L.Control.Layers();
  private currentDate = new Date();

  // Initialized in mounted()
  private currentFloor!: string;
  private floors: IFloor[] = [];
  private floorLayers: LLayerGroup[] = [];
  // private floorLayers: { [key: string]: L.Layer } = {};
  private maxBounds!: L.LatLngBounds;
  private map!: L.Map;

  public async mounted() {
    this.startClock();
    this.maxBounds = new L.LatLngBounds([0, 0], [0, 0]);
    this.floors = produce(await getFloors(), (draft: IFloor[]) => {
      for (const floor of draft) {
        const { elevation, image } = floor;
        const { pose, scale } = image;
        const { x, y } = pose;

        const svgElement = rawCompressedSvgToSvgElement(image.data);
        const height = svgElement.height.baseVal.value * scale / this.IMAGE_SCALE;
        const width = svgElement.width.baseVal.value * scale / this.IMAGE_SCALE;

        const offsetPixelsX = x / this.IMAGE_SCALE;
        const offsetPixelsY = y / this.IMAGE_SCALE;

        floor.bounds = new L.LatLngBounds(
          new L.LatLng(offsetPixelsY, offsetPixelsX, elevation),
          new L.LatLng(
            offsetPixelsY - height,
            offsetPixelsX + width,
            elevation,
          ),
        );
        
        floor.svgElement = svgElement;

        if (!this.maxBounds) {
          this.maxBounds = floor.bounds;
        }  else {
          this.maxBounds.extend(floor.bounds);
        }
      }
    });

    await this.$nextTick();

    this.map = (this.$refs.scheduleMap as any).mapObject;
    
    this.floorLayers = produce(
      this.$refs.floorLayers,
      draft => draft
    ) as unknown as LLayerGroup[];

    this.map.fitBounds(this.maxBounds);
    this.map.setMaxBounds(this.maxBounds);
    this.map.eachLayer((layer) => layer.remove());
    this.map.addLayer(
      this.floorLayers[0].mapObject as unknown as L.LayerGroup
    );
  }

  private startClock() {
    setInterval(() => {
      this.currentDate = new Date();
    }, 1000)
  }

  private initializeMap(mapObject: L.Map) {
    this.map.on(
      'baselayerchange',
      this.onMapBaseLayerChange,
    )
  }


  private onMapBaseLayerChange(event: L.LayersControlEvent) {
    this.currentFloor = event.name;
  }
}
</script>

<style scoped lang="scss">
.map {
  height: 100%;
  width: 100%;
  margin: 0;
  padding: 0;
}
</style>