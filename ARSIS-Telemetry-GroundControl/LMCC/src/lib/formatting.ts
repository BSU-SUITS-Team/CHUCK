// this is a random collection of different formatting functions to make stuff easier


export function formatTime(seconds: number) {
  const date = new Date(seconds * 1000); // Convert seconds to milliseconds
  const hours = date.getUTCHours();
  const minutes = date.getUTCMinutes();
  const secs = date.getUTCSeconds();

  let timeString = '';

  if (hours > 0) {
    timeString += `${hours.toString().padStart(2, '0')}:`;
  }

  if (minutes > 0 || hours > 0) {
    timeString += `${minutes.toString().padStart(2, '0')}:`;
  }

  timeString += secs.toString().padStart(2, '0');

  return timeString;
}
