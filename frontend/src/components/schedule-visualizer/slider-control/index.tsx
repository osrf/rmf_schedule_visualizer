import React from 'react'
import Control from 'react-leaflet-control'

import styled from 'styled-components/macro'

const DURATION_MINS = 120
const NOW_POSITION_MINS = 90
const DURATION = DURATION_MINS * 60 * 1000
const NOW_POSITION_PERCENT = NOW_POSITION_MINS / DURATION_MINS * 100
const MARKER_MINUTE_WIDTH = 1
const MARKER_MINUTE_HEIGHT = 20
const MARKER_NOW_WIDTH = 2
const MARKER_NOW_HEIGHT = 50
const CONTROL_BUTTON_WIDTH = 30
const CONTROL_BUTTON_HEIGHT = 30
const CONTROL_BUTTON_BOTTOM = 15
const CONTROL_BUTTON_Z_INDEX = 4
const CONTROL_BUTTON_BORDER_WIDTH = 2
const SLIDER_WIDTH_VW = 40
const SLIDER_HEIGHT = 4
const SLIDER_BORDER_WIDTH = 2
const KNOB_WIDTH = 10
const KNOB_HEIGHT = 20
const KNOB_WRAPPER_HEIGHT = 8

const ControlButton = styled.div`
  position: absolute;
  background: #fff;
  border-radius: 5px 5px 5px 5px;
  background-clip: padding-box;
  border: ${CONTROL_BUTTON_BORDER_WIDTH}px solid rgba(0, 0, 0, 0.2);
  width: ${CONTROL_BUTTON_WIDTH}px;
  height: ${CONTROL_BUTTON_HEIGHT}px;
  vertical-align: center;
  cursor: pointer;
  background-position: 50% 50%;
  bottom: ${CONTROL_BUTTON_BOTTOM}px;
  z-index: ${CONTROL_BUTTON_Z_INDEX};
`

const RewindControlButton = styled(ControlButton)`
  left: calc(-${SLIDER_WIDTH_VW / 2}vw - ${KNOB_WIDTH + CONTROL_BUTTON_WIDTH + CONTROL_BUTTON_BORDER_WIDTH}px);
  background-image: url(/fast_rewind-24px.svg);
`

const FastForwardControlButton = styled(ControlButton)`
  left: calc(${SLIDER_WIDTH_VW / 2}vw + ${KNOB_WIDTH - CONTROL_BUTTON_BORDER_WIDTH}px);
  background-image: url(/fast_forward-24px.svg);
`

const PlayPauseControl = styled(ControlButton)`
  left: calc(${SLIDER_WIDTH_VW / 2}vw + ${KNOB_WIDTH + CONTROL_BUTTON_WIDTH + CONTROL_BUTTON_BORDER_WIDTH / 2}px);
`

const SliderBase = styled.div`
  background: #fff;
  bottom: ${30 - SLIDER_BORDER_WIDTH}px;
  border-top: ${SLIDER_BORDER_WIDTH}px solid rgba(0, 0, 0, 0.2);
  border-bottom: ${SLIDER_BORDER_WIDTH}px solid rgba(0, 0, 0, 0.2);
  height: ${SLIDER_HEIGHT}px;
`

const Slider = styled(SliderBase)`
  position: absolute;
  width: calc(${SLIDER_WIDTH_VW}vw);
  vertical-align: center;
  bottom: ${30 - SLIDER_BORDER_WIDTH}px;
  left: calc(-${SLIDER_WIDTH_VW / 2}vw);
`

const SliderExtensionBase = styled(SliderBase)`
  position: absolute;
  width: ${KNOB_WIDTH}px;
`

const SliderExtensionLeft = styled(SliderExtensionBase)`
  left: calc(-${SLIDER_WIDTH_VW / 2}vw - ${KNOB_WIDTH}px);
`

const SliderExtensionRight = styled(SliderExtensionBase)`
  left: ${SLIDER_WIDTH_VW / 2}vw;
`

const Knob = styled.div`
  position: absolute;
  background: #fff;
  box-shadow: 0 0 2px;
  cursor: pointer;
  width: ${KNOB_WIDTH}px;
  height: ${KNOB_HEIGHT}px;
  top: calc(${KNOB_WRAPPER_HEIGHT - KNOB_HEIGHT}px / 2);
  z-index: 3;
`

const KnobWrapper = styled.div`
  position: absolute;
  background: #fff;
  box-shadow: 0 0 2px;
  cursor: pointer;
  left: 0%;
  width: 100%;
  height: ${KNOB_WRAPPER_HEIGHT}px;
  top: calc(${SLIDER_HEIGHT - KNOB_WRAPPER_HEIGHT}px / 2);
  z-index: 3;
`

const LeftKnob = styled(Knob)`
  left: calc(0% - ${KNOB_WIDTH}px);
`

const RightKnob = styled(Knob)`
  left: 100%;
`

const MarkersWrapper = styled.div`
  position: absolute;
  width: 100%;
  top: calc(${SLIDER_HEIGHT}px / 2);
`

const MarkerBase = styled.div`
  position: absolute;
  background: grey;
  z-index: -1;
`

const MarkerMinute = styled(MarkerBase)`
  width: ${MARKER_MINUTE_WIDTH}px;
  height: ${MARKER_MINUTE_HEIGHT}px;
  top: calc(-${MARKER_MINUTE_HEIGHT}px / 2);
`

const MarkerNow = styled(MarkerBase)`
  background: red;
  left: ${NOW_POSITION_PERCENT}%;
  width: ${MARKER_NOW_WIDTH}px;
  height: ${MARKER_NOW_HEIGHT}px;
  top: calc(-${MARKER_NOW_HEIGHT}px / 2);
`

export interface Props {
}

export interface State {
  playing: boolean
  knobStartPercent: number
  knobEndPercent: number
}

export interface KnobControlDimensions {
  maxWidth: number
  viewportMinX: number
  viewportMaxX: number
}

export interface SliderInfo {
  min: number
  max: number
}

export interface KnobRefs {
  left: React.RefObject<HTMLDivElement>
  center: React.RefObject<HTMLDivElement>,
  right: React.RefObject<HTMLDivElement>
}

function useSlider(
  dimensionsRef: React.RefObject<KnobControlDimensions>
): [KnobRefs, SliderInfo] {
  const knobRefs: KnobRefs = {
    left: React.useRef<HTMLDivElement>(null),
    center: React.useRef<HTMLDivElement>(null),
    right: React.useRef<HTMLDivElement>(null),
  }

  // TODO: Make center knob work
  const { current: leftKnobElement } = knobRefs.left
  const { current: rightKnobElement } = knobRefs.right
  const { current: dimensions } = dimensionsRef
  const cachedMin = React.useRef(0)
  const cachedMax = React.useRef(100)
  const [min, setMin] = React.useState(0)
  const [max, setMax] = React.useState(100)

  const onLeftKnobAdjust = React.useCallback((event: MouseEvent) => {
    if (!dimensions) return

    let { clientX: mouseCurrentX } = event
    const { viewportMinX, maxWidth } = dimensions
    mouseCurrentX += KNOB_WIDTH / 2

    let min = ((mouseCurrentX - viewportMinX) / maxWidth) * 100

    if (min < 0) {
      min = 0
    } else if (min > cachedMax.current) {
      min = cachedMax.current
    }

    cachedMin.current = min
    setMin(min)
  }, [dimensions, cachedMax])

  const onMouseMoveWhileLeftKnobDown = React.useCallback((event: MouseEvent) => {
    event.preventDefault()
    onLeftKnobAdjust(event)
  }, [onLeftKnobAdjust])

  const onLeftKnobMouseUpOrBlur = React.useCallback((event: FocusEvent | MouseEvent) => {
    event.preventDefault()

    if (event instanceof FocusEvent) {
      window.addEventListener('mousemove', function onMouseMove(event) {
        onLeftKnobAdjust(event)
        window.removeEventListener('mousemove', onMouseMove)
      })
    } else {
      onLeftKnobAdjust(event)
    }
    window.removeEventListener('mousemove', onMouseMoveWhileLeftKnobDown)
    window.removeEventListener('mouseup', onLeftKnobMouseUpOrBlur)
    window.removeEventListener('blur', onLeftKnobMouseUpOrBlur)
  }, [onLeftKnobAdjust, onMouseMoveWhileLeftKnobDown])

  const onLeftKnobMouseDown = React.useCallback((event: MouseEvent) => {
    event.stopPropagation()
    event.preventDefault()
    onLeftKnobAdjust(event)
    window.addEventListener('mousemove', onMouseMoveWhileLeftKnobDown)
    window.addEventListener('mouseup', onLeftKnobMouseUpOrBlur)
    window.addEventListener('blur', onLeftKnobMouseUpOrBlur)
  }, [onMouseMoveWhileLeftKnobDown, onLeftKnobMouseUpOrBlur, onLeftKnobAdjust])

  const onRightKnobAdjust = React.useCallback((event: MouseEvent) => {
    if (!dimensions) return

    let { clientX: mouseCurrentX } = event
    const { viewportMinX, maxWidth } = dimensions
    mouseCurrentX -= KNOB_WIDTH / 2

    let max = ((mouseCurrentX - viewportMinX) / maxWidth) * 100

    if (max > 100) {
      max = 100
    } else if (max < cachedMin.current) {
      max = cachedMin.current 
    }

    cachedMax.current = max
    setMax(max)
  }, [dimensions, cachedMin])

  const onMouseMoveWhileRightKnobDown = React.useCallback((event: MouseEvent) => {
    event.preventDefault()
    onRightKnobAdjust(event)
  }, [onRightKnobAdjust])

  const onRightKnobMouseUpOrBlur = React.useCallback((event: FocusEvent | MouseEvent) => {
    event.preventDefault()

    if (event instanceof FocusEvent) {
      window.addEventListener('mousemove', function onMouseMove(event) {
        onRightKnobAdjust(event)
        window.removeEventListener('mousemove', onMouseMove)
      })
    } else {
      onRightKnobAdjust(event)
    }
    window.removeEventListener('mousemove', onMouseMoveWhileRightKnobDown)
    window.removeEventListener('mouseup', onRightKnobMouseUpOrBlur)
    window.removeEventListener('blur', onRightKnobMouseUpOrBlur)
  }, [onMouseMoveWhileRightKnobDown, onRightKnobAdjust])

  const onRightKnobMouseDown = React.useCallback((event: MouseEvent) => {
    event.stopPropagation()
    event.preventDefault()
    onRightKnobAdjust(event)
    window.addEventListener('mousemove', onMouseMoveWhileRightKnobDown)
    window.addEventListener('mouseup', onRightKnobMouseUpOrBlur)
    window.addEventListener('blur', onRightKnobMouseUpOrBlur)
  }, [onMouseMoveWhileRightKnobDown, onRightKnobMouseUpOrBlur, onRightKnobAdjust])

  React.useEffect(() => {
    if (leftKnobElement) {
      leftKnobElement.addEventListener('mousedown', onLeftKnobMouseDown)
    }
  }, [leftKnobElement, onLeftKnobMouseDown])

  React.useEffect(() => {
    if (rightKnobElement) {
      rightKnobElement.addEventListener('mousedown', onRightKnobMouseDown)
    }
  }, [rightKnobElement, onRightKnobMouseDown])

  return [knobRefs, {min, max}]
}

function useKnobControlDimensions(
): [React.RefObject<HTMLDivElement>, React.RefObject<KnobControlDimensions>] {
  const sliderRef = React.useRef<HTMLDivElement>(null)
  const { current: sliderElement } = sliderRef
  const dimensionsRef = React.useRef<KnobControlDimensions>({
    maxWidth: 0,
    viewportMinX: 0,
    viewportMaxX: 0,
  })

  const onWindowResize = React.useCallback(() => {
    if (sliderElement) {
      const {x, width} = sliderElement.getBoundingClientRect()
      if (dimensionsRef.current) {
        dimensionsRef.current.maxWidth = width
        dimensionsRef.current.viewportMinX = x
        dimensionsRef.current.viewportMaxX = x + width
      }
    }
  }, [sliderElement])

  React.useEffect(() => {
    onWindowResize();
    window.addEventListener('resize', onWindowResize);
    return function cleanup() {
      window.removeEventListener('resize', onWindowResize);
    }
  }, [onWindowResize])

  return [sliderRef, dimensionsRef]
}

export default function SliderControl() {
  const [playing, setPlaying] = React.useState(false)
  const markerMinuteRef = React.useRef<JSX.Element[]>([])

  const [sliderRef, dimensionsRef] = useKnobControlDimensions()
  const [knobRefs, sliderInfo] = useSlider(dimensionsRef)
  const {min, max} = sliderInfo

  function getPlayPauseControlURL() {
    if (playing) {
      return '/pause-24px.svg'
    }
    return '/play_arrow-24px.svg'
  }

  function togglePlayPauseControl() {
    setPlaying(!playing)
  }

  function renderMarkerMinutes() {
    const { current: markers } = markerMinuteRef
    if (!markers.length) {
      for (let i = 0; i <= DURATION_MINS; ++i) {
        const leftPercent = i / DURATION_MINS * 100
        markers.push(<MarkerMinute key={i} style={{ left: `${leftPercent}%`}}></MarkerMinute>)
      }
    }
    return markers
  }

  return (
    <Control position="bottomcenter">
      <RewindControlButton />
      <SliderExtensionLeft />
      <Slider ref={sliderRef}>
        <MarkersWrapper>
          {
            renderMarkerMinutes()
          }
          <MarkerNow title="Now"/>
        </MarkersWrapper>
        <KnobWrapper
          ref={knobRefs.center}
          style={{
            left: `${min}%`,
            width: `${max - min}%`,
          }}
        >
          <LeftKnob
            ref={knobRefs.left}
          />
          <RightKnob
            ref={knobRefs.right}
          />
        </KnobWrapper>
      </Slider>
      <SliderExtensionRight />
      <FastForwardControlButton />
      <PlayPauseControl
        style={{ backgroundImage: `url(${getPlayPauseControlURL()})` }}
        onClick={togglePlayPauseControl}
      />
    </Control>
  )
}