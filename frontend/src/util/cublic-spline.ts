import Big from "big.js";

export interface Segment {
  initialPosition: Big;
  finalPosition: Big;
  initialVelocity: Big;
  finalVelocity: Big;
  initialTime: Big;
  finalTime: Big;
}

export interface CoefficientSet {
  a: Big;
  b: Big;
  c: Big;
  d: Big;
}

export interface Pose2D {
  x: Big;
  y: Big;
  theta: Big;
}

export interface Velocity {
  x: Big;
  y: Big;
  theta: Big;
}

export interface Knot {
  pose: Pose2D;
  velocity: Velocity;
  time: Big;
}

export interface SegmentCoefficients {
  x: CoefficientSet;
  y: CoefficientSet;
  theta: CoefficientSet;
  initialTime: Big;
  finalTime: Big;
}

export function segmentToCoefficientSet(segment: Segment): CoefficientSet {
  const {
    initialPosition: x0,
    finalPosition: x1,
    initialVelocity: v0,
    finalVelocity: v1,
    initialTime,
    finalTime,
  } = segment;

  const dt = finalTime.minus(initialTime);
  const w0 = v0.div(dt);
  const w1 = v1.div(dt);

  const a = w1.plus(w0).minus(x1.times(2)).plus(x0.times(2));
  const b = Big(0).minus(w1).minus(w0.times(2)).plus(x1.times(3)).minus(x0.times(3));

  return {
    a,
    b,
    c: w0,
    d: x0,
  };
}

export function assignKnotsToSegment(knot: Knot, nextKnot: Knot, forCoordinate: keyof Pose2D) {
  return {
    initialPosition: knot.pose[forCoordinate],
    finalPosition: nextKnot.pose[forCoordinate],
    initialVelocity: knot.velocity[forCoordinate],
    finalVelocity: nextKnot.velocity[forCoordinate],
    initialTime: knot.time,
    finalTime: nextKnot.time,
  };
}

export function knotsToSegmentCoefficientsArray(knots: Knot[]): SegmentCoefficients[] {
  const scs: SegmentCoefficients[] = [];

  for (let i = 0; i < knots.length - 1; ++i) {
    const knot = knots[i];
    const nextKnot = knots[i + 1];

    const sc: SegmentCoefficients = {
      x: segmentToCoefficientSet(assignKnotsToSegment(knot, nextKnot, 'x')),
      y: segmentToCoefficientSet(assignKnotsToSegment(knot, nextKnot, 'y')),
      theta: segmentToCoefficientSet(assignKnotsToSegment(knot, nextKnot, 'theta')),
      initialTime: knot.time,
      finalTime: nextKnot.time,
    };

    scs.push(sc);
  }

  return scs;
}

export function getInterpolatedTime(initialTime: Big, finalTime: Big, time: Big) {
  return time.minus(initialTime).div(finalTime.minus(initialTime));
}

export function resolveSpline(
  coefficientSet: CoefficientSet,
  interpolatedTime: Big,
  interpolatedTimePow2 = interpolatedTime.pow(2),
  interpolatedTimePow3 = interpolatedTime.pow(3),
): Big {
  return coefficientSet.a.times(interpolatedTimePow3)
    .plus(coefficientSet.b.times(interpolatedTimePow2))
    .plus(coefficientSet.c.times(interpolatedTime))
    .plus(coefficientSet.d);
}

export function getPositionFromSegmentCoefficientsArray(time: Big, scs: SegmentCoefficients[]): Pose2D | null {
  let sc: SegmentCoefficients | undefined;

  for (sc of scs) {
    if (time >= sc.initialTime && time <= sc.finalTime) {
      break;
    }
  } 

  if (!sc) {
    return null;
  }

  const {
    x: xCoeff,
    y: yCoeff,
    theta: thetaCoeff,
    initialTime,
    finalTime
  } = sc;

  const interpolatedTime = getInterpolatedTime(initialTime, finalTime, time);
  const interpolatedTimePow2 = interpolatedTime.pow(2);
  const interpolatedTimePow3 = interpolatedTime.pow(3);

  return {
    x: resolveSpline(xCoeff, interpolatedTime, interpolatedTimePow2, interpolatedTimePow3),
    y: resolveSpline(yCoeff, interpolatedTime, interpolatedTimePow2, interpolatedTimePow3),
    theta: resolveSpline(thetaCoeff, interpolatedTime, interpolatedTimePow2, interpolatedTimePow3),
  };
}
