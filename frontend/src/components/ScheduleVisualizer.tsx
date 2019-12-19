import React from 'react'
import {
  AttributionControl,
  ImageOverlay,
  LayersControl,
  Map as _Map,
} from 'react-leaflet'
import produce from 'immer'
import * as L from 'leaflet'
import styled from 'styled-components'

import 'leaflet/dist/leaflet.css'

import { IFloor as _IFloor } from '../models/Floor'
import { IAffineImage as _IAffineImage } from '../models/AffineImage'
import { getFloors } from '../mock'
import { rawCompressedSVGToSVGSVGElement, SVGSVGElementToDataURI } from '../util'

import ScheduleVisualizerServerDateControl from './ScheduleVisualizerServerDateControl'
import ScheduleVisualizerSliderControl from './ScheduleVisualizerSliderControl'

const { BaseLayer } = LayersControl

const Map = styled(_Map)`
  height: 100%;
  width: 100%;
  margin: 0;
  padding: 0;
`

const IMAGE_SCALE = 0.0125

export interface IAffineImage extends Omit<_IAffineImage, 'data'> {
  data: string
}

export interface IFloor extends Omit<_IFloor, 'image'> {
  image: IAffineImage
  bounds?: L.LatLngBounds
}

export interface State {
  floors: IFloor[]
  date: Date
  maxBounds: L.LatLngBounds
}

export interface Props {}

export default function ScheduleVisualizer() {
  const mapRef = React.useRef<_Map>(null)
  const { current: mapElement } = mapRef
  const [floors, setFloors] = React.useState<IFloor[]>([])
  const [maxBounds, setMaxBounds] = React.useState(new L.LatLngBounds([0, 0], [0, 0]))
  const [date, setDate] = React.useState(new Date())

  React.useEffect(() => {
    const clockIntervalID = setInterval(() => {
      setDate(new Date())
    }, 1000)

    return function cleanup() {
      clearInterval(clockIntervalID);
    }
  }, []);

  React.useEffect(() => {
    getFloors().then((floors) => {
      setFloors(produce<_IFloor[], any, IFloor[]>(floors, (draft) => {
        for (const floor of draft) {
          const { elevation, image } = floor
          const { pose, scale } = image
          const { x, y } = pose

          const svgElement = rawCompressedSVGToSVGSVGElement(image.data)
          const height = svgElement.height.baseVal.value * scale / IMAGE_SCALE
          const width = svgElement.width.baseVal.value * scale / IMAGE_SCALE

          const offsetPixelsX = x / IMAGE_SCALE
          const offsetPixelsY = y / IMAGE_SCALE

          floor.bounds = new L.LatLngBounds(
            new L.LatLng(offsetPixelsY, offsetPixelsX, elevation),
            new L.LatLng(
              offsetPixelsY - height,
              offsetPixelsX + width,
              elevation,
            ),
          )
          
          floor.image.data = SVGSVGElementToDataURI(svgElement)
      
          setMaxBounds(maxBounds.extend(floor.bounds))
        }
        return draft as IFloor[]
      }))
    })

    if (mapElement) {
      mapElement.leafletElement.fitBounds(maxBounds);
    }
  }, [maxBounds, mapElement])

  return (
    <Map
      ref={mapRef}
      attributionControl={false}
      crs={L.CRS.Simple}
      minZoom={-2}
      maxZoom={2}
      maxBounds={maxBounds}
    >
      <AttributionControl position="bottomright" prefix="OSRC-SG" />
      <ScheduleVisualizerServerDateControl date={date} position="topright" />
      <LayersControl position="topright">
        {
          floors.map((floor, i) => (
            <BaseLayer checked={i === 0} name={floor.name} key={floor.name}>
              <ImageOverlay bounds={floor.bounds} url={floor.image.data} />
            </BaseLayer> 
          ))
        }
      </LayersControl>
      <ScheduleVisualizerSliderControl />
    </Map>
  )
}