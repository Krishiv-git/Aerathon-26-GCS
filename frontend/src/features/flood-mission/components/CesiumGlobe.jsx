"use client";

import React, { useCallback, useEffect, useRef } from "react";
import Script from "next/script";
import { useCesium } from "@/context/CesiumContext";

export default function CesiumGlobe() {
  const cesiumContainer = useRef(null);
  const viewerRef = useRef(null);
  const { registerViewer } = useCesium();

  const initializeViewer = useCallback(() => {
    if (!window.Cesium || !cesiumContainer.current || viewerRef.current) return;

    const Cesium = window.Cesium;
    const token = process.env.NEXT_PUBLIC_CESIUM_ION_ACCESS_TOKEN;
    if (token) Cesium.Ion.defaultAccessToken = token;

    const viewer = new Cesium.Viewer(cesiumContainer.current, {
      timeline: false,
      animation: false,
      baseLayerPicker: false,
      fullscreenButton: false,
      geocoder: false,
      homeButton: false,
      navigationHelpButton: false,
      sceneModePicker: false,
      selectionIndicator: false,
      infoBox: false,
    });

    viewer.scene.globe.show = true;
    viewer.scene.globe.baseColor = Cesium.Color.fromCssColorString("#101820");
    viewer.terrainProvider = new Cesium.EllipsoidTerrainProvider();
    viewer.imageryLayers.removeAll();
    viewer.imageryLayers.addImageryProvider(
      new Cesium.UrlTemplateImageryProvider({
        url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        tilingScheme: new Cesium.WebMercatorTilingScheme(),
        // Cap at the level with consistent global coverage. Esri returns
        // opaque "Map data not yet available" tiles for many remote areas
        // when requested at higher zoom levels; Cesium will upscale level 16.
        maximumLevel: 16,
        credit: "Imagery © Esri, Maxar, Earthstar Geographics, and the GIS User Community",
      }),
    );
    viewer.camera.setView({
      destination: Cesium.Cartesian3.fromDegrees(77.5946, 12.9716, 24000000),
      orientation: { heading: 0, pitch: -Math.PI / 2, roll: 0 },
    });

    viewerRef.current = viewer;
    registerViewer(viewer);
  }, [registerViewer]);

  useEffect(() => {
    initializeViewer();

    return () => {
      if (viewerRef.current) {
        registerViewer(null);
        viewerRef.current.destroy();
        viewerRef.current = null;
      }
    };
  }, [initializeViewer, registerViewer]);

  return (
    <>
      <Script
        src="https://cesium.com/downloads/cesiumjs/releases/1.108/Build/Cesium/Cesium.js"
        strategy="afterInteractive"
        onLoad={initializeViewer}
      />
      <link
        rel="stylesheet"
        href="https://cesium.com/downloads/cesiumjs/releases/1.108/Build/Cesium/Widgets/widgets.css"
      />
      <div ref={cesiumContainer} className="w-full h-full" />
    </>
  );
}
