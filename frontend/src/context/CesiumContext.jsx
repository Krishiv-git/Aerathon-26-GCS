"use client";

import React, { createContext, useContext, useState, useCallback } from "react";

const CesiumContext = createContext(null);

export function CesiumProvider({ children }) {
  const [viewer, setViewer] = useState(null);
  const [kmlData, setKmlData] = useState(null);
  const [kmlUploaded, setKmlUploaded] = useState(false);
  const [manualWaypointsCount, setManualWaypointsCount] = useState(0);

  const registerViewer = useCallback((viewerInstance) => {
    setViewer(viewerInstance);
  }, []);

  const loadKML = useCallback((parsedKmlData) => {
    setKmlData(parsedKmlData);
  }, []);

  const value = {
    viewer,
    registerViewer,
    kmlData,
    loadKML,
    // Shared UI state for blocking manual vs KML planners
    kmlUploaded,
    setKmlUploaded,
    manualWaypointsCount,
    setManualWaypointsCount,
  };

  return (
    <CesiumContext.Provider value={value}>{children}</CesiumContext.Provider>
  );
}

export function useCesium() {
  const context = useContext(CesiumContext);
  if (!context) {
    throw new Error("useCesium must be used within a CesiumProvider");
  }
  return context;
}
