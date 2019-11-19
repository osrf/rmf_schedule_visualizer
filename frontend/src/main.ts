import Vue from 'vue'
import { Icon } from 'leaflet';
import 'leaflet/dist/leaflet.css';

import App from './App.vue'
import store from './store'

// Needed by vue2-leaflet: this part resolve an issue where the markers would not appear
delete (<any>Icon.Default.prototype)._getIconUrl;
Icon.Default.mergeOptions({
  iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png'),
  iconUrl: require('leaflet/dist/images/marker-icon.png'),
  shadowUrl: require('leaflet/dist/images/marker-shadow.png')
});

Vue.config.productionTip = false

new Vue({
  store,
  render: h => h(App)
}).$mount('#app')
