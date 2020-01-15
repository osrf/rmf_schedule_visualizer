export interface TrajectoryRequestParam {
  map_name: string
  start_time: string
  finish_time: string
}

export interface TrajectoryRequest {
  request: 'trajectory'
  param: TrajectoryRequestParam
}

export function trajectoryRequest(param: TrajectoryRequestParam): string {
  const requestObject: TrajectoryRequest = {
    request: 'trajectory',
    param,
  }
  return JSON.stringify(requestObject)
}

// RawVelocity received from server is in this format (x, y, theta)
export type RawVelocity = [number, number, number]

// RawPose2D received from server is in this format (x, y, theta)
export type RawPose2D = [number, number, number]

export interface TrajectoryKnot {
  t: string // nanoseconds
  v: RawVelocity
  x: RawPose2D
}

export interface TrajectoryResponseValue {
  dimensions: number[] 
  segments: TrajectoryKnot[] 
  shape: string
}

export interface TrajectoryResponse {
  response: 'trajectory'
  values: TrajectoryResponseValue[]
}