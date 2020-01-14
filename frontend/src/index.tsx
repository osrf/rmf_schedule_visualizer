import React from 'react';
import ReactDOM from 'react-dom';


import './index.css';
import 'leaflet/dist/leaflet.css'
import { extendControlPositions } from './leaflet/control-positions';

import App from './App';
import * as serviceWorker from './serviceWorker';
import { ClockSource } from './clock';
import { WebSocketManager } from './util/websocket';

extendControlPositions();
export const clockSource = new ClockSource()
clockSource.start()

export const webSocketManager = new WebSocketManager('ws://localhost:8006')
webSocketManager.connect()

window.addEventListener('unload', (_event) => {
  webSocketManager.disconnect()
  clockSource.stop()
})

ReactDOM.render(<App />, document.getElementById('root'));

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();
