<template>
  <LMap
    style="height: 600px; width: 100vw"
    ref="scheduleMap"
    :options="MAP_OPTIONS"
    :crs="MAP_CRS"
  >
  <LControlAttribution
    prefix="OSRC-SG"
    position="bottomright"
  />
  </LMap>
</template>

<script lang="ts">

import * as L from 'leaflet';
import axios from 'axios';
import produce from 'immer';
import { Component, Vue } from 'vue-property-decorator';
import { LMap, LControlAttribution, LControlLayers } from 'vue2-leaflet';

import { getFloors } from '../mock'
import { rawCompressedSvgToSvgElement } from '../util';
import { IFloor } from '../models/Floor';

@Component({
  components: {
    LControlAttribution,
    LMap,
    LControlLayers,
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
  private readonly MAP_LAYER_CONTROL = new L.Control.Layers();

  // Initialized in mounted()
  private currentFloor!: string;
  private floors: IFloor[] = [];
  private floorLayers: { [key: string]: L.Layer } = {};
  private maxBounds!: L.LatLngBounds;
  private map!: L.Map;

  public mounted() {
    this.$nextTick(async () => {
      this.initializeMap((this.$refs.scheduleMap as any).mapObject as L.Map);
      await this.updateMap();
    });
  }

  private initializeMap(mapObject: L.Map) {
    if (this.map === mapObject) {
      return;
    }

    this.map = mapObject;
    this.map.addControl(this.MAP_LAYER_CONTROL);

    this.map.on(
      'baselayerchange',
      this.onMapBaseLayerChange,
    )
  }

  private async updateMap() {
    this.maxBounds = new L.LatLngBounds([0, 0], [0, 0]);
    this.floors = await produce(this.floors, async (draft) => draft = await getFloors());

    this.floorLayers = produce(this.floorLayers, (draftFloorLayers) => {
      for (const floor of this.floors) {
        const { elevation, image, name } = floor;
        const { pose, scale } = image;
        const { x, y } = pose;

        const svgElement = rawCompressedSvgToSvgElement(image.data);
        const height = svgElement.height.baseVal.value * scale / this.IMAGE_SCALE;
        const width = svgElement.width.baseVal.value * scale / this.IMAGE_SCALE;

        const offsetPixelsX = x / this.IMAGE_SCALE;
        const offsetPixelsY = y / this.IMAGE_SCALE;

        const bounds = new L.LatLngBounds(
          new L.LatLng(offsetPixelsY, offsetPixelsX, elevation),
          new L.LatLng(
            offsetPixelsY - height,
            offsetPixelsX + width,
            elevation,
          ),
        );

        if (!this.maxBounds) {
          this.maxBounds = bounds;
        } else {
          this.maxBounds.extend(bounds);
        }

        const layer = new L.LayerGroup([
          new L.SVGOverlay(svgElement, bounds)
        ]);

        draftFloorLayers[name] = layer;
        this.MAP_LAYER_CONTROL.addBaseLayer(layer, name);
      }
    });
    
    const defaultFloor = this.floors[0].name;
    this.currentFloor = defaultFloor;
    const layer = this.floorLayers[defaultFloor];

    this.map.eachLayer((layer) => this.map.removeLayer(layer));
    this.map.addLayer(layer);
    this.map.fitBounds(this.maxBounds);
    this.map.setMaxBounds(this.maxBounds);
    // this.$refs.scheduleMap.mapObject.LEAFLET_METHOD
  }

  private onMapBaseLayerChange(event: L.LayersControlEvent) {
    this.currentFloor = event.name;
  }
}
</script>