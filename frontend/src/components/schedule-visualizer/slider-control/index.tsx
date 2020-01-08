import React, { DetailedHTMLProps, HTMLAttributes } from 'react'
import Control from 'react-leaflet-control'

import styled, { ThemedStyledProps } from 'styled-components/macro'
import { ClockSecondEvent } from '../../../clock'

const DURATION_MINS = 120
const NOW_POSITION_MINS = 90
const DURATION = DURATION_MINS * 60 * 1000
const NOW_POSITION_PERCENT = (NOW_POSITION_MINS / DURATION_MINS) * 100
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
const KNOB_LABEL_WIDTH = 100
const KNOB_MIN_INIT_PERCENT = 20
const KNOB_MAX_INIT_PERCENT = 80

console.log(DURATION)

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
  cursor: pointer;
  width: ${KNOB_WIDTH}px;
  height: ${KNOB_HEIGHT}px;
  top: calc(${KNOB_WRAPPER_HEIGHT - KNOB_HEIGHT}px / 2);
  z-index: 3;
`

const KnobWrapper = styled.div`
  position: absolute;
  background: #888;
  box-shadow: 0 0 2px;
  cursor: pointer;
  left: 0%;
  width: 100%;
  height: ${KNOB_WRAPPER_HEIGHT}px;
  top: calc(${SLIDER_HEIGHT - KNOB_WRAPPER_HEIGHT}px / 2);
  z-index: 3;
`

const LeftKnob = styled(Knob)`
  width: 0;
  height: 0;
  border-top: ${KNOB_WIDTH}px solid transparent;
  border-bottom: ${KNOB_WIDTH}px solid transparent;
  border-right: calc(${KNOB_HEIGHT}px / 2) solid #888;
  left: calc(0% - ${KNOB_WIDTH}px);
`

const RightKnob = styled(Knob)`
  width: 0;
  height: 0;
  border-top: ${KNOB_WIDTH}px solid transparent;
  border-bottom: ${KNOB_WIDTH}px solid transparent;
  border-left: calc(${KNOB_HEIGHT}px / 2) solid #888;
  left: 100%;
`

const KnobLabel = styled.div.attrs((props: KnobLabelProps) => ({
  'data-fade': props['data-fade'] || false,
}))`
  text-align: center;
  position: absolute;
  border-radius: 5px;
  padding: 0 5px 0 5px;
  width: ${KNOB_LABEL_WIDTH}px;
  background: #aaa;
  top: -60px;
  opacity: ${props => props['data-fade'] ? "0.5": "1"};
`

const LeftKnobLabel = styled(KnobLabel)`
  left: calc((${KNOB_WIDTH}px / 2) - (${KNOB_LABEL_WIDTH}px / 2));
`

const RightKnobLabel = styled(KnobLabel)`
  right: calc((${KNOB_WIDTH}px / 2) - (${KNOB_LABEL_WIDTH}px / 2));
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

const MarkerNow = styled(MarkerBase)`
  background: red;
  left: ${NOW_POSITION_PERCENT}%;
  width: ${MARKER_NOW_WIDTH}px;
  height: ${MARKER_NOW_HEIGHT}px;
  top: calc(-${MARKER_NOW_HEIGHT}px / 2);
`

export interface KnobLabelProps extends ThemedStyledProps<DetailedHTMLProps<HTMLAttributes<HTMLDivElement>, HTMLDivElement>, any> {
  'data-fade': boolean,
}

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

export interface CachedSliderInfo {
  min: React.RefObject<number>
}

export interface KnobRefs {
  left: React.RefObject<HTMLDivElement>
  center: React.RefObject<HTMLDivElement>
  right: React.RefObject<HTMLDivElement>
}

export interface KnobLabelInfo {
  leftHidden: boolean
  rightHidden: boolean
}

export interface DateRange {
  min: Date
  max: Date
}

export interface TimeDiffs {
  min: number
  max: number
}

function useSlider(
  dimensionsRef: React.RefObject<KnobControlDimensions>
): [KnobRefs, SliderInfo, KnobLabelInfo] {
  const knobRefs: KnobRefs = {
    left: React.useRef<HTMLDivElement>(null),
    center: React.useRef<HTMLDivElement>(null),
    right: React.useRef<HTMLDivElement>(null),
  }

  // TODO: Make center knob work
  const { current: leftKnobElement } = knobRefs.left
  const { current: rightKnobElement } = knobRefs.right
  const { current: dimensions } = dimensionsRef
  const cachedMin = React.useRef(KNOB_MIN_INIT_PERCENT)
  const cachedMax = React.useRef(KNOB_MAX_INIT_PERCENT)
  const [min, setMin] = React.useState(KNOB_MIN_INIT_PERCENT)
  const [max, setMax] = React.useState(KNOB_MAX_INIT_PERCENT)
  const [leftKnobLabelHidden, setLeftKnobLabelHidden] = React.useState(true);
  const [rightKnobLabelHidden, setRightKnobLabelHidden] = React.useState(true);

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
    setLeftKnobLabelHidden(false);
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
    setLeftKnobLabelHidden(true);
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
    setRightKnobLabelHidden(false)
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
    setRightKnobLabelHidden(true)
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

  return [knobRefs, {min, max}, {leftHidden: leftKnobLabelHidden, rightHidden: rightKnobLabelHidden}]
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

function percentDiffToDurationDiff(percentDiff: number) {
  return ((percentDiff - NOW_POSITION_PERCENT) / 100) * DURATION
}

function useDateRange(sliderInfo: SliderInfo): [DateRange, TimeDiffs] {
  const {min: minPercent, max: maxPercent} = sliderInfo;
  const [minDate, setMinDate] = React.useState<Date>(new Date())
  const [maxDate, setMaxDate] = React.useState<Date>(new Date())
  const cachedMinDate = React.useRef(new Date())
  const cachedMaxDate = React.useRef(new Date())
  const cachedTimeNow = React.useRef(cachedMinDate.current.getTime())
  const cachedMinTimeDiffMS = React.useRef(percentDiffToDurationDiff(KNOB_MIN_INIT_PERCENT))
  const cachedMaxTimeDiffMS = React.useRef(percentDiffToDurationDiff(KNOB_MAX_INIT_PERCENT))
  const [minTimeDiffMS, setMinTimeDiffMS] = React.useState(cachedMinTimeDiffMS.current)
  const [maxTimeDiffMS, setMaxTimeDiffMS] = React.useState(cachedMaxTimeDiffMS.current)

  React.useEffect(() => {
    window.addEventListener('onclocksecond', (event: ClockSecondEvent) => {
      if (!event.date) return
      cachedTimeNow.current = event.date.getTime()

      cachedMinDate.current.setTime(cachedTimeNow.current + cachedMinTimeDiffMS.current)
      cachedMaxDate.current.setTime(cachedTimeNow.current + cachedMaxTimeDiffMS.current)

      setMinDate(cachedMinDate.current)
      setMaxDate(cachedMaxDate.current)
    })
  }, [])

  React.useEffect(() => {
    cachedMinTimeDiffMS.current = percentDiffToDurationDiff(minPercent)
    cachedMaxTimeDiffMS.current = percentDiffToDurationDiff(maxPercent)
    setMinTimeDiffMS(cachedMinTimeDiffMS.current)
    setMaxTimeDiffMS(cachedMaxTimeDiffMS.current)

    cachedMinDate.current.setTime(cachedTimeNow.current + cachedMinTimeDiffMS.current)
    cachedMaxDate.current.setTime(cachedTimeNow.current + cachedMaxTimeDiffMS.current)
    setMinDate(cachedMinDate.current)
    setMaxDate(cachedMaxDate.current)
  }, [minPercent, maxPercent])

  return [{min: minDate, max: maxDate}, {min: minTimeDiffMS, max: maxTimeDiffMS}]
}

function useTimeDiffsFormat(timeDiffs: TimeDiffs, decimalPlaces: number) {
  const {min: minDiff, max: maxDiff} = timeDiffs;
  const [minTimeString, setMinTimeString] = React.useState(minDiff.toFixed(decimalPlaces))
  const [maxTimeString, setMaxTimeString] = React.useState(maxDiff.toFixed(decimalPlaces))

  React.useEffect(() => {
    setMinTimeString((minDiff / 100).toFixed(decimalPlaces))
    setMaxTimeString((maxDiff / 100).toFixed(decimalPlaces))
  }, [minDiff, maxDiff, decimalPlaces])

  return [minTimeString, maxTimeString]
}

export default function SliderControl() {
  const [playing, setPlaying] = React.useState(false)

  const [sliderRef, dimensionsRef] = useKnobControlDimensions()
  const [knobRefs, sliderInfo, knobLabelInfo] = useSlider(dimensionsRef)
  const [dateRange, timeDiffs] = useDateRange(sliderInfo)
  const [minDiffTimeString, maxDiffTimeString] = useTimeDiffsFormat(timeDiffs, 2)
  const {leftHidden: leftKnobLabelHidden, rightHidden: rightKnobLabelHidden} = knobLabelInfo;
  const {min, max} = sliderInfo
  const {min: minDate, max: maxDate} = dateRange

  function getPlayPauseControlURL() {
    if (playing) {
      return '/pause-24px.svg'
    }
    return '/play_arrow-24px.svg'
  }

  function togglePlayPauseControl() {
    setPlaying(!playing)
  }

  return (
    <Control position="bottomcenter">
      <RewindControlButton />
      <SliderExtensionLeft />
      <Slider ref={sliderRef}>
        <MarkersWrapper>
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
          >
            <LeftKnobLabel data-fade={leftKnobLabelHidden}>
              {minDate.toLocaleTimeString()} <br />
              {minDiffTimeString} secs
            </LeftKnobLabel>
          </LeftKnob>
          <RightKnob
            ref={knobRefs.right}
          >
            <RightKnobLabel data-fade={rightKnobLabelHidden}>
              {maxDate.toLocaleTimeString()} <br />
              {maxDiffTimeString} secs
            </RightKnobLabel>
          </RightKnob>
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