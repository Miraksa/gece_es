import React, { useRef, useEffect } from 'react';
import "@maptiler/sdk/dist/maptiler-sdk.css";
import * as maptilersdk from '@maptiler/sdk';
import * as maptiler3d from '@maptiler/3d';
import './Map.css';
import { GUI } from 'lil-gui';

export default function Map() {
  const mapContainer = useRef(null);
  const map = useRef(null);
  maptilersdk.config.apiKey = 'ETdmY9zrqqxWh6KiMb9C';

  useEffect(() => {
    if (map.current) return; // stops map from initializing more than once

    map.current = new maptilersdk.Map({
      container: mapContainer.current,
      style: maptilersdk.MapStyle.SATELLITE,
      center: [-7.22, 80.18],
      zoom: 11,
      pitch: 60,
      maxPitch: 85,
      terrainControl: true,
      terrain: true,
      maptilerLogo: true,
    });

    (async () => {
      await map.current.onReadyAsync();

      map.current.setSky({
        "sky-color": "#0C2E4B",
        "horizon-color": "#09112F",
        "fog-color": "#09112F",
        "fog-ground-blend": 0.5,
        "horizon-fog-blend": 0.1,
        "sky-horizon-blend": 1.0,
        "atmosphere-blend": 0.5,
      });

      const layer3D = new maptiler3d.Layer3D("custom-3D-layer");
      map.current.addLayer(layer3D);

      const guiObj = {
        heading: 0,
        scale: 1,
        altitude: 3000,
        opacity: 1,
        wireframe: false,
        altitudeReference: "GROUND_LEVEL",
        removePlane: () => {
          layer3D.removeMesh(originalPlaneID);
        }
      };

      const originalPlaneID = "plane";
      try {
        await layer3D.addMeshFromURL(
          originalPlaneID,
          "/model/plane_a340.glb",
          {
            scale: guiObj.scale,
            altitude: guiObj.altitude,
            altitudeReference: maptiler3d.AltitudeReference.MEAN_SEA_LEVEL,
            wireframe: guiObj.wireframe,
            lngLat: [-7.22, 80.18]
          }
        );
      } catch (error) {
        console.error("Error loading the 3D model:", error);
      }

      let planeCanMove = false;

      map.current.on("mousemove", (e) => {
        if (!planeCanMove) return;
        layer3D.modifyMesh(originalPlaneID, { lngLat: e.lngLat });
      });

      map.current.on("click", () => {
        planeCanMove = !planeCanMove;
      });

      const gui = new GUI();
      gui.add(guiObj, "heading", 0, 360).onChange((heading) => {
        layer3D.modifyMesh(originalPlaneID, { heading });
      });
      gui.add(guiObj, "scale", 0.1, 10).onChange((scale) => {
        layer3D.modifyMesh(originalPlaneID, { scale });
      });
      gui.add(guiObj, "altitude", 0, 10000).onChange((altitude) => {
        layer3D.modifyMesh(originalPlaneID, { altitude });
      });
      gui.add(guiObj, "opacity", 0, 1).onChange((opacity) => {
        layer3D.modifyMesh(originalPlaneID, { opacity });
      });
      gui.add(guiObj, "wireframe").onChange((wireframe) => {
        layer3D.modifyMesh(originalPlaneID, { wireframe });
      });
      gui.add(guiObj, "removePlane");

    })();

  }, []);

  return (
    <div className="map-wrap">
      <div ref={mapContainer} className="map" />
      <div id="info">
        <p>
          Click on the map to move the plane. You can then click again to fix its position.
        </p>
      </div>
    </div>
  );
}