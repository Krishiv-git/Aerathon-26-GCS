"use client";

import React from "react";
import { CesiumProvider } from "@/context/CesiumContext";
import FloodMissionGCS from "./FloodMissionGCS";

export default function HomeApp() {
  return (
    <CesiumProvider>
      <FloodMissionGCS />
    </CesiumProvider>
  );
}
