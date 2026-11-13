"use client";

import React, { useEffect, useRef, useState } from "react";
import Head from "next/head";
import { Maximize2, Minimize2 } from "lucide-react";

const API = process.env.NEXT_PUBLIC_BACKEND_URL || "http://127.0.0.1:8000";
const MISSION_ALTITUDE_M = 5;
const MISSION_SPEED_MPS = 3;
const PHASE_LABELS = {
  INITIALIZING: "INITIALIZING",
  TAKEOFF: "TAKEOFF",
  SURVEY: "SEARCHING INDOOR ZONE",
  RETURN: "ABORTING MISSION",
  LAND: "LANDING",
  COMPLETE: "MISSION COMPLETE",
  STOPPED: "MISSION ABORTED",
  ERROR: "SIMULATION ERROR",
};

function prettyTime(seconds = 0) {
  const total = Math.max(0, Math.floor(seconds));
  return `${String(Math.floor(total / 60)).padStart(2, "0")}:${String(total % 60).padStart(2, "0")}`;
}

function CameraPanel({ title, camera, stamp, sessionId, active, detail, expanded, onToggle }) {
  const src = sessionId ? `${API}/simulation/flood/${sessionId}/camera/${camera}?frame=${stamp}` : null;
  const feedText = active ? "SIM FEED" : sessionId ? "LAST FRAME" : "STANDBY";
  return (
    <section className={`camera-card ${expanded ? "camera-expanded" : ""}`} aria-label={`${title} panel`}>
      <div className="camera-head">
        <div className="camera-title"><span className={`camera-indicator camera-${camera}`} /><span>{title}</span></div>
        <div className="camera-head-actions">
          <span className={`feed-state ${active ? "feed-live" : sessionId ? "feed-last" : "feed-idle"}`}><i />{feedText}</span>
          <button className="camera-expand" type="button" onClick={onToggle} aria-label={expanded ? `Restore ${title.toLowerCase()}` : `Expand ${title.toLowerCase()}`} title={expanded ? "Restore camera size" : "Expand camera"}>
            {expanded ? <Minimize2 size={15} /> : <Maximize2 size={15} />}
          </button>
        </div>
      </div>
      <div className="camera-image">
        {src ? <img src={src} alt={`${title} simulation feed`} /> : (
          <div className="camera-empty">
            <span className="empty-feed-mark" />
            <span>AWAITING SIMULATION FEED</span>
          </div>
        )}
        {active && <div className="camera-hud"><span>SIMULATION</span><span>{detail}</span></div>}
      </div>
    </section>
  );
}

export default function FloodMissionGCS() {
  const [mission, setMission] = useState(null);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const [cameraStamp, setCameraStamp] = useState(Date.now());
  const [expandedCamera, setExpandedCamera] = useState(false);
  const [groundSpeed, setGroundSpeed] = useState(null);
  const speedSample = useRef(null);

  const active = mission && ["starting", "running", "returning"].includes(mission.status);
  const phase = mission?.state?.phase || (mission?.status === "starting" ? "INITIALIZING" : "STANDBY");
  const survivorCount = mission?.state?.people?.length || 0;
  const phaseLabel = PHASE_LABELS[phase] || phase;
  const pose = mission?.state?.pose;
  const headingDegrees = Number.isFinite(pose?.yaw)
    ? ((Math.round((90 - (pose.yaw * 180) / Math.PI) % 360) + 360) % 360)
    : null;
  const heading = headingDegrees === null ? "—" : `${String(headingDegrees).padStart(3, "0")}°`;
  const battery = Number.isFinite(mission?.state?.battery_percent)
    ? `${Math.round(mission.state.battery_percent)}%`
    : "N/A";

  useEffect(() => {
    let cancelled = false;
    fetch(`${API}/simulation/flood/active`, { cache: "no-store" })
      .then(async (response) => {
        if (!response.ok) return null;
        return response.json();
      })
      .then((data) => {
        if (cancelled || !data?.session_id) return;
        speedSample.current = null;
        setGroundSpeed(null);
        setMission((current) => current?.session_id ? current : data);
      })
      .catch(() => {});
    return () => { cancelled = true; };
  }, []);

  const startMission = async () => {
    setBusy(true);
    setError("");
    try {
      const form = new FormData();
      form.append("altitude", String(MISSION_ALTITUDE_M));
      form.append("speed", String(MISSION_SPEED_MPS));
      form.append("realtime", "true");
      const response = await fetch(`${API}/simulation/flood/start`, { method: "POST", body: form });
      const data = await response.json();
      if (!response.ok) throw new Error(data.detail || "Could not start the simulation");
      setGroundSpeed(null);
      speedSample.current = null;
      setMission({ ...data, state: { phase: "INITIALIZING", trail: [], people: [] } });
    } catch (e) {
      const message = e.message || "Could not connect to the simulation backend";
      if (/already active/i.test(message)) {
        try {
          const response = await fetch(`${API}/simulation/flood/active`, { cache: "no-store" });
          const activeMission = response.ok ? await response.json() : null;
          if (activeMission?.session_id) {
            speedSample.current = null;
            setGroundSpeed(null);
            setMission(activeMission);
            setError("");
            return;
          }
        } catch {
          // Keep original error if active session cannot be recovered.
        }
      }
      setError(message);
    } finally {
      setBusy(false);
    }
  };

  const abortMission = async () => {
    if (!mission?.session_id) return;
    try {
      const response = await fetch(`${API}/simulation/flood/${mission.session_id}/stop`, { method: "POST" });
      const data = await response.json();
      if (!response.ok) throw new Error(data.detail || "Could not abort mission");
      setMission((current) => ({ ...current, status: data.status, state: { ...current.state, phase: "STOPPED" } }));
    } catch (e) {
      setError(e.message || "Could not abort mission");
    }
  };

  useEffect(() => {
    if (!mission?.session_id) return undefined;
    if (!["starting", "running", "returning"].includes(mission.status)) return undefined;
    let cancelled = false;
    const poll = async () => {
      try {
        const response = await fetch(`${API}/simulation/flood/${mission.session_id}/state`, { cache: "no-store" });
        const data = await response.json();
        if (!response.ok) throw new Error(data.detail || "Lost simulation connection");
        if (!cancelled) {
          if (data.status === "failed" && data.error) setError(data.error);
          const state = data.state;
          const currentPose = state?.pose;
          if (currentPose && Number.isFinite(state?.sim_time) && Number.isFinite(currentPose.x) && Number.isFinite(currentPose.y)) {
            const previous = speedSample.current;
            const currentSample = { sessionId: data.session_id, simTime: state.sim_time, x: currentPose.x, y: currentPose.y };
            if (previous?.sessionId !== data.session_id) {
              setGroundSpeed(null);
              speedSample.current = currentSample;
            } else if (state.sim_time > previous.simTime) {
              const deltaTime = state.sim_time - previous.simTime;
              if (deltaTime >= 0.05) {
                setGroundSpeed(Math.hypot(currentPose.x - previous.x, currentPose.y - previous.y) / deltaTime);
                speedSample.current = currentSample;
              }
            }
          }
          setMission((current) => ({ ...current, ...data }));
        }
      } catch (e) {
        if (!cancelled && /session not found/i.test(e.message || "")) {
          try {
            const response = await fetch(`${API}/simulation/flood/active`, { cache: "no-store" });
            const activeMission = response.ok ? await response.json() : null;
            if (cancelled) return;
            if (activeMission?.session_id) {
              speedSample.current = null;
              setGroundSpeed(null);
              setMission((current) => current?.session_id === mission.session_id ? activeMission : current);
              setError("");
            } else {
              setMission((current) => current?.session_id === mission.session_id ? null : current);
              setGroundSpeed(null);
              speedSample.current = null;
              setError("The previous simulation session was cleared. Start the mission again.");
            }
          } catch {
            if (!cancelled) setError("Lost simulation connection. Check that the backend is running, then retry.");
          }
        } else if (!cancelled) {
          setError(e.message || "Lost simulation connection");
        }
      }
      if (!cancelled) setCameraStamp(Date.now());
    };
    poll();
    const timer = window.setInterval(poll, 600);
    return () => { cancelled = true; window.clearInterval(timer); };
  }, [mission?.session_id, mission?.status]);

  useEffect(() => {
    if (!expandedCamera) return undefined;
    const closeOnEscape = (event) => {
      if (event.key === "Escape") setExpandedCamera(false);
    };
    window.addEventListener("keydown", closeOnEscape);
    return () => window.removeEventListener("keydown", closeOnEscape);
  }, [expandedCamera]);

  return (
    <main className="flood-gcs">
      <Head>
        <title>PHOENIX GCS · Mission Control</title>
        <meta name="description" content="Phoenix GCS autonomous indoor search and mapping mission control." />
      </Head>

      {/* 1. Header */}
      <header className="gcs-header">
        <div className="phoenix-brand">
          <img className="phoenix-mark" src="/phoenix-bird.png" alt="Phoenix bird emblem" />
          <div className="phoenix-wordmark"><strong>PHOENIX <span>GCS</span></strong><small>GROUND CONTROL STATION</small></div>
        </div>
        <img className="dhruva-logo" src="/dhruva-logo.png" alt="DHRUVA · Disaster Handling, Rescue & Unmanned Vigilance Aircraft" />
      </header>

      {/* 2. Split Body: Left (Start/Abort Button + Single Camera Feed) & Right (2D Map) */}
      <div className="gcs-body">
        <div className="gcs-left-pane">
          <div className="mission-action-bar">
            {active ? (
              <button className="abort-button" type="button" onClick={abortMission}>
                ABORT MISSION
              </button>
            ) : (
              <button className="start-button" type="button" onClick={startMission} disabled={busy}>
                {busy ? <span className="spinner" /> : null}
                {busy ? "STARTING MISSION..." : "START MISSION"}
              </button>
            )}
          </div>

          {expandedCamera && <button className="camera-expanded-backdrop" type="button" aria-label="Close expanded camera" onClick={() => setExpandedCamera(false)} />}
          <CameraPanel
            title="LIVE CAMERA FEED"
            camera="front"
            stamp={cameraStamp}
            sessionId={mission?.session_id}
            active={active}
            detail={phase === "SURVEY" ? "SURVIVOR SEARCH" : "FLIGHT VIEW"}
            expanded={expandedCamera}
            onToggle={() => setExpandedCamera((current) => !current)}
          />
        </div>

        {/* Right Half: Blank 2D Map Container */}
        <div className="gcs-right-pane">
          <section className="map-2d-panel" aria-label="2D Indoor Map">
            <div className="map-2d-head">
              <div className="camera-title">
                <span className="camera-indicator" />
                <span>2D MAP</span>
              </div>
              <span className={`feed-state ${active ? "feed-live" : "feed-idle"}`}>
                <i />
                {active ? "MAPPING" : "STANDBY"}
              </span>
            </div>
            <div className="map-2d-canvas" />
          </section>
        </div>
      </div>

      {error && <div className="error-banner" role="alert"><span className="error-mark" /><span>{error}</span><button onClick={() => setError("")}>DISMISS</button></div>}

      {/* 3 & 4. Bottom Telemetry Bar: GPS Removed & SURVIVORS Centered (5th of 9) */}
      <section className="telemetry-strip" aria-label="Live telemetry">
        <div className="telemetry-cell aircraft-cell"><span><small>AIRCRAFT</small><strong>DHRUVA · RESCUE-01</strong></span></div>
        <div className="telemetry-cell phase-cell"><span><small>FLIGHT PHASE</small><strong>{phaseLabel}</strong></span></div>
        <div className="telemetry-cell"><span><small>MISSION TIME</small><strong className="mono">{prettyTime(mission?.state?.sim_time)}</strong></span></div>
        <div className="telemetry-cell"><span><small>ALTITUDE</small><strong className="mono">{mission?.state?.drone ? `${mission.state.drone.altitude.toFixed(1)} m` : "—"}</strong></span></div>
        <div className="telemetry-cell people-cell"><span><small>SURVIVORS</small><strong>{String(survivorCount).padStart(2, "0")} confirmed</strong></span></div>
        <div className="telemetry-cell"><span><small>GROUND SPEED</small><strong className="mono">{active && Number.isFinite(groundSpeed) ? `${groundSpeed.toFixed(1)} m/s` : "—"}</strong></span></div>
        <div className="telemetry-cell"><span><small>HEADING</small><strong className="mono">{heading}</strong></span></div>
        <div className="telemetry-cell"><span><small>BATTERY</small><strong className="mono muted-value">{battery}</strong></span></div>
        <div className="telemetry-cell link-cell"><span className={`link-indicator ${active ? "link-active" : ""}`} /><span><small>SIM LINK</small><strong>{active ? "STREAMING" : mission?.status === "failed" ? "ERROR" : "STANDBY"}</strong></span></div>
      </section>
    </main>
  );
}