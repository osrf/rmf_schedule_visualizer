<template>
  <l-control class="server-date-control" position="topright">
    <div title="Server Time" class="server-date">
      {{ date.toLocaleString() }}
    </div>
  </l-control>
</template>

<script lang="ts">

import { Component, Prop, Vue } from 'vue-property-decorator';
import { LControl } from 'vue2-leaflet';
import { ControlOptions } from 'leaflet';

@Component({
  name: 'server-date-control',
  components: {
    LControl,
  }
})
export default class ServerTimeControl extends Vue {
  @Prop({ type: Date, default: () => new Date() }) public date!: Date;
  @Prop({ type: String, default: () => 'topright' }) public position!: ControlOptions;
}
</script>

<style scoped lang="scss">
.server-date-control {
  float: left;
  background: #fff;
  border-radius: 5px 5px 5px 5px;
  background-clip: padding-box;
  border: 2px solid rgba(0, 0, 0, 0.2);
  height: 44px;
  vertical-align: center;
}

.server-date {
  padding: 3px 5px 0 5px;
  font-size: 2em;
}
</style>

<style lang="scss">
.server-date-control + div.leaflet-control {
  clear: none;
}
</style>