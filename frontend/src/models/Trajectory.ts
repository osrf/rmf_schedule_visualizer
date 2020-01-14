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