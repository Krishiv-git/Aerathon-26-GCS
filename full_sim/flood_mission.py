#!/usr/bin/env python3
"""Flood survey simulation using the local PyBullet flight and camera adapters.

Three live panels: downward camera, forward detections, flown trail/map.
RGB clothing detector is specific to flood_world's synthetic people, not a
trained general-purpose human detector. No person coordinates or segmentation
IDs are used by the detector/tracker. This is a PyBullet kinematic simulation.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import pybullet as p

try:
    from . import flood_support as master, flood_world as world
except ImportError:
    import flood_support as master
    import flood_world as world

DT = .1
W, H = master.CAMERA_WIDTH, master.CAMERA_HEIGHT


# Clearance includes the square drone's half-diagonal plus 0.4 m stand-off.
OBSTACLE_BUFFER = math.hypot(master.COLLISION_LENGTH_M, master.COLLISION_WIDTH_M)/2 + .4


def safe_xy(x, y, margin=0.):
    if not (.75 <= x <= 39.25 and -14.25 <= y <= 14.25):
        return False
    for bx, by, w, d, _ in world.BUILDING_SPECS:
        # Include roof overhang; deliberately route around even the short roof.
        if abs(x-bx) <= w/2+.12+OBSTACLE_BUFFER+margin and abs(y-by) <= d/2+.12+OBSTACLE_BUFFER+margin:
            return False
    return all(math.hypot(x-tx,y-ty) > 2.2+OBSTACLE_BUFFER+margin for tx,ty in world.TREE_POSITIONS)


def safe_leg(a, b):
    steps=max(1, math.ceil(math.dist(a[:2], b[:2])/.1))
    # Reserve 12 cm for waypoint acceptance and sampled-segment error, above
    # the required 40 cm obstacle stand-off checked against actual telemetry.
    return all(safe_xy(a[0]+(b[0]-a[0])*t/steps, a[1]+(b[1]-a[1])*t/steps, margin=.12)
               for t in range(steps+1))


# User-drawn loop, in world metres, starting at the bottom launch pad.
# Left lobe -> rear trees -> right lobe -> bottom junction -> home.
# Rounded bends in the sketch are represented by short, safe straight legs.
DRAWN_LOOP_XY = (
    world.DRONE_START_POS,
    (20., -10.), (17., -10.), (14., -9.6), (11., -9.4),
    (10.8, -7.), (10.2, -5.4), (7., -5.4), (2., -5.4),
    (1.4, -4.8), (1.4, -1.), (1.7, 1.2), (2.2, 1.8),
    (6., 2.4), (10.8, 2.4), (12.25, 2.8),
    (12., 4.), (12., 7.), (13.7, 7.8), (14.9, 9.5),
    (15.3, 11.5), (16., 13.), (18., 13.4), (21., 13.5),
    (24., 13.5), (24.5, 13.1), (24.5, 11.5),
    (24., 9.5), (24., 5.), (24., 2.6), (25.5, 2.),
    (29., 2.), (32., 2.4), (35., 2.6), (38., 2.6),
    (38.4, 2.1), (38.1, 1.), (38.5, -1.5), (38.7, -4.8),
    (38.3, -5.6), (34., -5.6), (30., -5.7), (28.2, -5.7),
    (27.7, -6.6), (27.2, -8.), (27.2, -10.5),
    (26.7, -10.8), (25., -9.9), (23., -9.8), (20., -10.),
    world.DRONE_START_POS,
)


def survey_path(altitude):
    """Follow the supplied sketch once, with fixed altitude and safe clearance."""
    if not math.isfinite(altitude) or altitude < 4:
        raise ValueError('Survey altitude must be at least 4 m above original ground')
    route=[(x,y,altitude) for x,y in DRAWN_LOOP_XY]
    for index,(start,end) in enumerate(zip(route,route[1:])):
        if not safe_leg(start,end):
            raise ValueError(f'Drawn route leg {index} does not clear obstacles: {start} -> {end}')
    return route


FORWARD_VFOV = 55.
FORWARD_PITCH = math.atan2(4., 8.)


def forward_basis(pose):
    c,s=math.cos(pose.yaw),math.sin(pose.yaw)
    cp,sp=math.cos(FORWARD_PITCH),math.sin(FORWARD_PITCH)
    eye=np.array((pose.x+.35*c,pose.y+.35*s,pose.z+.02))
    forward=np.array((cp*c,cp*s,-sp))
    right=np.array((s,-c,0.))
    up=np.array((sp*c,sp*s,cp))
    return eye,forward,right,up


def pixel_to_world(box, pose):
    """Project the forward observation to the visible raised-arm height plane."""
    u,v=(box[0]+box[2])/2,(box[1]+box[3])/2
    eye,forward,right,up=forward_basis(pose)
    focal=(H/2)/math.tan(math.radians(FORWARD_VFOV/2))
    ray=forward+(u-W/2)/focal*right-(v-H/2)/focal*up
    if ray[2]>=-1e-6:
        return None
    t=(1.83-eye[2])/ray[2]
    if t<=0 or np.linalg.norm(t*ray)>22:
        return None
    return (eye+t*ray)[:2]


def detect_people(rgb, pose, depth=None, return_positions=False):
    """Group forward RGB clothing fragments by estimated spatial proximity.

    Candidate colors come from the image; no world person locations, object
    identities or segmentation masks are read here. Optional rendered camera
    depth localizes image candidates; it does not supply their identities.
    """
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    # Rescue shirts are more saturated than the yellow arena boundary.
    warm=cv2.inRange(hsv,(0,215,110),(38,255,255))
    purple=cv2.inRange(hsv,(135,130,55),(175,255,255))
    mask=cv2.bitwise_or(warm,purple)
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask)
    parts=[]
    for i in range(1,count):
        x,y,w,h,area=map(int,stats[i])
        if area<2 or area>1800 or max(w,h)>105 or min(x,y)<2 or x+w>=W-2 or y+h>=H-2:
            continue
        box=(x,y,x+w-1,y+h-1)
        xy=pixel_to_world(box,pose)
        if depth is not None:
            yy,xx=np.where(labels[y:y+h,x:x+w]==i)
            xx=xx+x; yy=yy+y
            eye,forward,right,up=forward_basis(pose)
            focal=(H/2)/math.tan(math.radians(FORWARD_VFOV/2))
            rays=forward+(xx[:,None]-W/2)/focal*right-(yy[:,None]-H/2)/focal*up
            points=eye+depth[yy,xx,None]*rays
            points=points[np.isfinite(points).all(axis=1)]
            # Visible raised arms must be above the water, not on a building.
            points=points[(points[:,2]>1.53)&(points[:,2]<2.02)]
            xy=np.median(points[:,:2],axis=0) if len(points) else None
        if xy is not None:
            parts.append((box,xy))
    groups=[]
    # Connected groups of arm fragments, with a distance threshold in metres.
    while parts:
        group=[parts.pop()]
        changed=True
        while changed:
            changed=False
            for part in parts[:]:
                if any(np.linalg.norm(part[1]-other[1])<1.0 for other in group):
                    group.append(part)
                    parts.remove(part)
                    changed=True
        groups.append(group)
    boxes=[]
    positions=[]
    for group in groups:
        x0=min(q[0][0] for q in group); y0=min(q[0][1] for q in group)
        x1=max(q[0][2] for q in group); y1=max(q[0][3] for q in group)
        if x1-x0>150 or y1-y0>100:
            continue
        boxes.append((max(0,x0-3),max(0,y0-4),min(W-1,x1+3),min(H-1,y1+4)))
        positions.append(np.mean([q[1] for q in group],axis=0))
    return (boxes,positions) if return_positions else boxes


class Tracker:
    def __init__(self):
        self.tracks = []

    def update(self, boxes, pose, now, positions=None):
        assignments, used = [], set()
        for box_index,box in enumerate(boxes):
            xy = positions[box_index] if positions is not None else pixel_to_world(box, pose)
            if xy is None:
                continue
            if not (0.6 < xy[0] < 39.4 and -14.4 < xy[1] < 14.4):
                continue
            distances = [np.linalg.norm(xy-t['xy']) if i not in used else float('inf')
                         for i,t in enumerate(self.tracks)]
            if distances and min(distances) < 1.1:
                index = int(np.argmin(distances))
                track = self.tracks[index]
                track['hits'] += 1
                track['xy'] = .8 * track['xy'] + .2 * xy
            else:
                index = len(self.tracks)
                track = {'xy': xy, 'hits': 1, 'first_seen': now, 'last_seen': now}
                self.tracks.append(track)
            used.add(index)
            track['last_seen'] = now
            assignments.append((box, index + 1, track['hits'] >= 3))
        return assignments

    def confirmed(self):
        return [(i+1,t) for i,t in enumerate(self.tracks) if t['hits'] >= 3]


def render_forward(pose, include_depth=False):
    eye,forward,right,up=forward_basis(pose)
    view=p.computeViewMatrix(eye.tolist(),(eye+8*forward).tolist(),up.tolist())
    proj=p.computeProjectionMatrixFOV(FORWARD_VFOV,W/H,.05,100)
    result=p.getCameraImage(W,H,view,proj,renderer=p.ER_TINY_RENDERER)
    rgb=np.asarray(result[2],dtype=np.uint8).reshape(H,W,4)[:,:,:3]
    if not include_depth:
        return rgb
    buffer=np.asarray(result[3],dtype=float).reshape(H,W)
    depth=100*.05/(100-(100-.05)*buffer)
    return rgb,depth


class Coverage:
    """Track camera-visible accessible water cells; excludes solid footprints."""
    def __init__(self):
        cells = []
        for x in np.arange(.5, 40, 1):
            for y in np.arange(-14.5, 15, 1):
                if any(abs(x-bx) < w/2+.15 and abs(y-by) < d/2+.15 for bx,by,w,d,_ in world.BUILDING_SPECS):
                    continue
                if any(math.hypot(x-tx,y-ty) < 2.2 for tx,ty in world.TREE_POSITIONS):
                    continue
                if abs(x-20)<1.4 and abs(y+13)<1.4:
                    continue
                cells.append((x,y))
        self.cells = np.array(cells)
        self.seen = np.zeros(len(cells), dtype=bool)

    def update(self, pose):
        c,s = math.cos(pose.yaw), math.sin(pose.yaw)
        eye = np.array((pose.x+.15*c,pose.y+.15*s,pose.z-.1))
        delta = self.cells - eye[:2]
        forward = delta[:,0]*c + delta[:,1]*s
        right = delta[:,0]*s - delta[:,1]*c
        half_h = (eye[2]-1.51)*math.tan(math.radians(master.CAMERA_VFOV_DEG/2))
        candidates = np.flatnonzero(~self.seen & (abs(forward)<half_h*.98) & (abs(right)<half_h*W/H*.98))
        if not len(candidates):
            return
        ends = [(float(self.cells[i,0]),float(self.cells[i,1]),1.51) for i in candidates]
        hits = p.rayTestBatch([eye.tolist()]*len(ends), ends)
        for index,hit in zip(candidates,hits):
            if hit[0] == -1:
                self.seen[index] = True

    @property
    def percent(self):
        return float(self.seen.mean()*100)


def text(image, label, xy, color=(240,240,240), size=.5):
    cv2.putText(image,label,xy,cv2.FONT_HERSHEY_SIMPLEX,size,color,1,cv2.LINE_AA)


def dashboard(down, front, pose, phase, now, trail, tracker, coverage):
    panel = np.full((H, 480, 3), (33,28,22), np.uint8)
    def point(x,y):
        return (int(30+x*10.5), int(350-(y+15)*10.5))
    cv2.rectangle(panel,point(0,15),point(40,-15),(130,110,50),-1)
    for cell in coverage.cells[coverage.seen]:
        cv2.circle(panel,point(*cell),3,(115,100,40),-1)
    for x,y,w,d,h in world.BUILDING_SPECS:
        cv2.rectangle(panel,point(x-w/2,y+d/2),point(x+w/2,y-d/2),(125,125,125),-1)
        text(panel,f'{h:g}m',point(x-w/2,y),size=.35)
    for x,y in world.TREE_POSITIONS:
        cv2.circle(panel,point(x,y),20,(35,100,35),-1)
    # Only measured historical positions are supplied here. No future route.
    if len(trail)>1:
        cv2.polylines(panel,[np.array([point(x,y) for x,y in trail])],False,(0,230,245),2)
    for identity,track in tracker.confirmed():
        uv=point(*track['xy'])
        cv2.circle(panel,uv,5,(40,40,255),-1)
        text(panel,f'P{identity}',(uv[0]+6,uv[1]-5),(150,190,255),.4)
    uv=point(pose.x,pose.y)
    cv2.circle(panel,uv,5,(255,255,255),-1)
    cv2.arrowedLine(panel,uv,point(pose.x+1.5*math.cos(pose.yaw),pose.y+1.5*math.sin(pose.yaw)),(255,255,255),2)
    text(panel,'LIVE MAP - FLOWN TRAIL ONLY',(30,22))
    text(panel,f'{phase} | t={now:.1f}s | z={pose.z:.1f}m',(20,385))
    text(panel,f'People tagged: {len(tracker.confirmed())}',(20,410))
    text(panel,f'Visible open-water coverage: {coverage.percent:.1f}%',(20,435),size=.45)
    text(panel,'Yellow: flown | Red: confirmed person',(20,461),size=.43)
    down=cv2.cvtColor(down,cv2.COLOR_RGB2BGR)
    front=cv2.cvtColor(front,cv2.COLOR_RGB2BGR)
    for image,label in ((down,'DOWNWARD CAMERA'),(front,'FORWARD - PERSON DETECTIONS')):
        cv2.rectangle(image,(0,0),(W,30),(24,24,24),-1)
        text(image,label,(12,21))
    return np.hstack((down,front,panel))


def run(args, *, on_state=None, on_frames=None, stop_event=None):
    route=survey_path(args.altitude)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'route_waypoints.json').write_text(json.dumps({
        'altitude_m':args.altitude,
        'description':'User-drawn loop, left side first',
        'waypoints':route,
    },indent=2)+'\n')
    client,solids=world.build_arena_only(p.GUI if args.gui else p.DIRECT)
    writer=None
    vehicle=None
    tracker,coverage=Tracker(),Coverage()
    trace,trail=[],[]
    phase='TAKEOFF'
    contacts=0
    complete=False
    frame=None
    best_detection_score=0
    try:
        # The shared adapter's landing datum must match the raised launch pad.
        master.GROUND_CENTER_Z=world.TAKEOFF_PAD_TOP_M+master.COLLISION_HEIGHT_M/2+.03
        start=(*world.DRONE_START_POS,master.GROUND_CENTER_Z)
        drone=master.DroneVisual(start,world.DRONE_START_YAW)
        vehicle=master.KinematicVehicle(drone.body_id,start,world.DRONE_START_YAW,latency_s=0)
        camera=master.SimulationCamera(drone.body_id)
        vehicle.arm()
        vehicle.takeoff(args.altitude)
        if args.gui:
            p.resetDebugVisualizerCamera(45,35,-45,(20,0,3))
        target_index=0
        next_render=0.
        stop_reroute_done=False
        while vehicle.sim_time < args.max_seconds:
            wall_start=time.monotonic()
            pose=vehicle.get_pose()
            if phase=='TAKEOFF' and abs(pose.z-args.altitude)<.025:
                phase='SURVEY'

            # On operator stop, retrace the already-flown track at cruise
            # altitude. This produces a safe, deterministic return before
            # landing instead of abandoning the vehicle mid-route.
            if stop_event is not None and stop_event.is_set() and not stop_reroute_done:
                stop_reroute_done=True
                if phase in ('SURVEY','RETURN'):
                    reverse_trail=[]
                    last_xy=None
                    for x,y in reversed(trail):
                        if last_xy is None or math.dist((x,y),last_xy)>=1.5:
                            reverse_trail.append((x,y,args.altitude))
                            last_xy=(x,y)
                    route=[(pose.x,pose.y,args.altitude),*reverse_trail,
                           (*world.DRONE_START_POS,args.altitude)]
                    target_index=1
                    phase='RETURN'
                elif phase=='TAKEOFF':
                    # Finish the vertical climb, then land at the launch pad.
                    route=[(*world.DRONE_START_POS,args.altitude)]
                    target_index=0
            if phase in ('SURVEY','RETURN'):
                target=np.array(route[target_index])
                delta=target-np.array((pose.x,pose.y,pose.z))
                if np.linalg.norm(delta)<.08:
                    vehicle.flush_velocity_commands()
                    target_index+=1
                    if target_index==len(route):
                        phase='LAND'
                        vehicle.land()
                    elif target_index==len(route)-1:
                        phase='RETURN'
                else:
                    distance=max(np.linalg.norm(delta),1e-6)
                    # Cruise at the operator-selected survey speed, then
                    # ease off inside the final meter before each waypoint.
                    velocity=delta*min(args.speed,args.speed/distance)
                    c,s=math.cos(pose.yaw),math.sin(pose.yaw)
                    vehicle.send_velocity_body(c*velocity[0]+s*velocity[1],-s*velocity[0]+c*velocity[1],-velocity[2])
            vehicle.step(DT)
            if hasattr(vehicle,'sync_sim_pose'):
                vehicle.sync_sim_pose()
            pose=vehicle.get_pose()
            p.performCollisionDetection()
            if any(cp[8]<-1e-5 for cp in p.getContactPoints(bodyA=drone.body_id)):
                contacts+=1
                raise RuntimeError('Drone obstacle contact; simulation stopped')
            trail.append((pose.x,pose.y))
            trace.append((vehicle.sim_time,phase,pose.x,pose.y,pose.z,pose.yaw))
            landed = phase=='LAND' and not vehicle.armed
            if landed:
                phase='COMPLETE'
                complete=True
            if vehicle.sim_time+1e-6>=next_render or landed:
                next_render+=1/args.fps
                down=camera.render_downward().copy()
                front,depth=render_forward(pose,include_depth=True)
                front=front.copy()
                assignments=[]
                if phase in ('SURVEY','RETURN'):
                    boxes,positions=detect_people(front,pose,depth,return_positions=True)
                    assignments=tracker.update(boxes,pose,vehicle.sim_time,positions)
                    coverage.update(pose)
                for box,identity,confirmed in assignments:
                    color=(40,255,70) if confirmed else (255,200,30)
                    cv2.rectangle(front,box[:2],box[2:],color,1)
                    cv2.putText(front,f'P{identity}' if confirmed else 'person?',(box[0],max(12,box[1]-3)),cv2.FONT_HERSHEY_SIMPLEX,.36,color,1)
                frame=dashboard(down,front,pose,phase,vehicle.sim_time,trail,tracker,coverage)
                score=len(tracker.confirmed())*100+sum(confirmed for _,_,confirmed in assignments)
                if assignments and score>best_detection_score:
                    best_detection_score=score
                    cv2.imwrite(str(args.output/'detections_frame.png'),frame)
                if on_frames is not None:
                    on_frames(down.copy(),front.copy(),phase,vehicle.sim_time)
                if not args.no_video:
                    if writer is None:
                        writer=cv2.VideoWriter(str(args.output/'flood_mission.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),args.fps,(frame.shape[1],frame.shape[0]))
                        if not writer.isOpened():
                            raise RuntimeError('Could not open MP4 writer')
                    writer.write(frame)
                if not args.headless:
                    cv2.imshow('Flood mission | downward / forward / flown map',frame)
                    if cv2.waitKey(1)&255 in (27,ord('q')):
                        break
            if complete:
                break
            if on_state is not None:
                on_state({
                    'phase':phase,
                    'sim_time':vehicle.sim_time,
                    'pose':{'x':pose.x,'y':pose.y,'z':pose.z,'yaw':pose.yaw},
                    'armed':vehicle.armed,
                    'target_index':target_index,
                    'waypoint_count':len(route),
                    'coverage_percent':coverage.percent,
                    'people':[{'id':i,'x':float(t['xy'][0]),'y':float(t['xy'][1]),
                               'observations':t['hits'],'first_seen_s':t['first_seen']}
                              for i,t in tracker.confirmed()],
                })
            if args.realtime:
                time.sleep(max(0,DT-(time.monotonic()-wall_start)))
        summary={'complete':complete,'phase':phase,'simulation_seconds':vehicle.sim_time,
                 'altitude_m':args.altitude,'contacts':contacts,'coverage_percent':coverage.percent,
                 'obstacle_buffer_m':OBSTACLE_BUFFER,'planned_waypoints':len(route),
                 'planned_distance_m':sum(math.dist(a,b) for a,b in zip(route,route[1:])),
                 'coverage_definition':'1 m visible open-water cells, excluding buildings, canopy footprints and launch pad',
                 'detector':'Forward RGB rescue clothing; synthetic world only',
                 'detection_camera':'forward','fixed_yaw_rad':world.DRONE_START_YAW,
                 'localization':'forward rendered depth at RGB clothing pixels',
                 'people':[{'id':i,'x':float(t['xy'][0]),'y':float(t['xy'][1]),'observations':t['hits'],'first_seen_s':t['first_seen']} for i,t in tracker.confirmed()]}
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        with (args.output/'flight_trace.csv').open('w',newline='') as stream:
            csvwriter=csv.writer(stream)
            csvwriter.writerow(('time_s','phase','x','y','z','yaw_rad'))
            csvwriter.writerows(trace)
        if frame is not None:
            cv2.imwrite(str(args.output/'last_frame.png'),frame)
        print(json.dumps(summary,indent=2))
        return 0 if complete else 2
    finally:
        if writer is not None:
            writer.release()
        if vehicle:
            vehicle.close()
        if not args.headless:
            cv2.destroyAllWindows()
        p.disconnect(client)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true',help='Run without camera dashboard window')
    parser.add_argument('--gui',action='store_true',help='Also open the PyBullet 3D viewer')
    parser.add_argument('--realtime',action='store_true',help='Pace flight to real time')
    parser.add_argument('--altitude',type=float,default=5.,help='Metres above original ground (3.5 m above water); default 5')
    parser.add_argument('--speed',type=float,default=3.,help='Maximum kinematic survey speed, m/s')
    parser.add_argument('--fps',type=float,default=5.,help='Camera/detector/video rate (1-10 Hz)')
    parser.add_argument('--max-seconds',type=float,default=1800.)
    parser.add_argument('--no-video',action='store_true')
    parser.add_argument('--output',type=Path,default=Path(__file__).with_name('flood_run'))
    args=parser.parse_args()
    if not (0<args.speed<=5 and 1<=args.fps<=10 and args.max_seconds>0):
        parser.error('Require 0 < speed <= 5, 1 <= fps <= 10, max-seconds > 0')
    return run(args)


if __name__=='__main__':
    raise SystemExit(main())
