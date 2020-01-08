export class ClockSecondEvent extends Event {
  date?: Date
  constructor() {
    super('onclocksecond')
  }
}

export function startClock() {
  const event = new ClockSecondEvent()

  return setInterval(() => {
    event.date = new Date()
    const { date } = event
    date.setSeconds(date.getSeconds() + 1)
    const milliseconds = date.getMilliseconds()
    if (milliseconds !== 0) {
      setTimeout(() => {
        window.dispatchEvent(event)
      }, 1000 - milliseconds)    
    } else {
      window.dispatchEvent(event)
    }
  }, 1000)
}