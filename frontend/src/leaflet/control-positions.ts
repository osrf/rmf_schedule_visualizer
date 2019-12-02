import { ControlPosition, DomUtil, Map } from 'leaflet';

export type ControlPosition = (
  ControlPosition | 'topcenter' | 'bottomcenter' | 'leftcenter' | 'rightcenter' | 'center'
);

export function extendControlPositions() {
  // Implement feature from this PR: https://github.com/Leaflet/Leaflet/pull/5554
  const origInitControlPos = (Map as any).prototype._initControlPos;
  Map.include({
    _initControlPos() {
      origInitControlPos.call(this);
      this._controlCorners['topcenter'] = DomUtil.create(
        'div',
        'leaflet-top leaflet-horizontal-center',
        this._controlContainer,
      );
      this._controlCorners['bottomcenter'] = DomUtil.create(
        'div',
        'leaflet-bottom leaflet-horizontal-center',
        this._controlContainer,
      );
      this._controlCorners['leftcenter'] = DomUtil.create(
        'div',
        'leaflet-left leaflet-vertical-center',
        this._controlContainer,
      );
      this._controlCorners['rightcenter'] = DomUtil.create(
        'div',
        'leaflet-right leaflet-vertical-center',
        this._controlContainer,
      );
      this._controlCorners['center'] = DomUtil.create(
        'div',
        'leaflet-horizontal-center leaflet-vertical-center',
        this._controlContainer,
      );
    },
  });
}