
import * as pako from 'pako';

export function rawCompressedSvgToSvgElement(data: ArrayBuffer | Uint8Array) {
  if (data instanceof ArrayBuffer) {
    data = new Uint8Array(data);
  }

  const inflated = pako.inflate(<Uint8Array>data);
  const inflatedText: string = (new TextDecoder('utf-8')).decode(inflated);

  const svgDoc: XMLDocument = (new DOMParser()).parseFromString(inflatedText, 'image/svg+xml');

  return <any>svgDoc.documentElement;
}