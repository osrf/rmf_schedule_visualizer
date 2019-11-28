<template>
  <div />
</template>

<script lang="ts">

import {
  DomEvent,
  LatLngBoundsExpression,
  LatLngBounds,
  SVGOverlay as LSVGOverlay,
} from 'leaflet';
import { LLayerGroup, findRealParent, propsBinder } from 'vue2-leaflet';
import { Component, Prop, Vue } from 'vue-property-decorator';

@Component({
  name: 'svg-overlay'
})
export default class SVGOverlay extends Vue {

  @Prop({ type: [SVGElement, String]}) public svgImage!: SVGElement | string;
  @Prop({ type: [Array, LatLngBounds]}) public bounds!: LatLngBoundsExpression;
  @Prop({ type: String }) public name!: string;
  public mapObject!: LSVGOverlay;
  public parentContainer!: LLayerGroup;

  async mounted() {
    this.mapObject = new LSVGOverlay(this.svgImage, this.bounds);

    // FIXME: type
    DomEvent.on(this.mapObject as any, this.$listeners as any);

    // FIXME: type
    propsBinder(this, this.mapObject as any, this.$options.props as any);

    this.parentContainer = findRealParent(this.$parent);
    this.parentContainer.addLayer(this);

    await this.$nextTick();
    this.$emit('ready', this.mapObject);
  }
}
</script>