
#########################################################################
# PROJECT: PARKPILOT - AUTONOMOUS DRONE INSPECTION SYSTEM               #
# BY lama abdullah aldraim                                              #
# github/ yellowHUBcom                                                  #
# Tuwaiq drone programming                                              #
# Group: Batch 2 (The Second Cohort) - Drone Bootcamp 2025              #
#                                                                       #
#                                                                       #
#                                                                       #
#                                                                       #
#########################################################################

import asyncio
import cv2
import rclpy
import pyttsx3
import tkinter as tk
from tkinter import ttk
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
import gz_camera_to_frame as cam_lib
import numpy as np
import os

save_path = "/home/yellowlmo/Desktop/x500parkpilot/parkpilot_mission.avi"
engine = pyttsx3.init()

def speak(text):
    print(f"[VOICE]: {text}")
    engine.say(text)
    engine.runAndWait()

#########################################################################
#                                                                       #
#                                                                       #
#                                                                       #
#                        PARKPILOT MONITOR                              #
#                                                                       #
#                                                                       #
#                                                                       #
#########################################################################
class ParkPilotApp:
    def __init__(self, drone):
        self.drone = drone
        self.root = tk.Tk()
        self.root.title("PARKPILOT | Mission Control")
        self.root.geometry("450x650")
        self.root.configure(bg="#0D1B2A") 
        
        self.violation_count = 0
        self.pass_count = 0
        self.current_spot = 0

        # Header
        header = tk.Frame(self.root, bg="#1B263B", height=80)
        header.pack(fill="x")
        tk.Label(header, text="PARK PILOT DASHBOARD", font=("Segoe UI", 16, "bold"), 
                 bg="#1B263B", fg="white").pack(pady=25)
        
        # Stats Container
        stats_container = tk.Frame(self.root, bg="#0D1B2A")
        stats_container.pack(pady=30)

        # Authorized Box (Purple)
        self.p_box = tk.Frame(stats_container, bg="#7B2CBF", width=160, height=130)
        self.p_box.pack_propagate(False)
        self.p_box.grid(row=0, column=0, padx=15)
        tk.Label(self.p_box, text="AUTHORIZED", font=("Arial", 10, "bold"), bg="#7B2CBF", fg="white").pack(pady=10)
        self.p_label = tk.Label(self.p_box, text="0", font=("Arial", 40, "bold"), bg="#7B2CBF", fg="white")
        self.p_label.pack()

        # Violation Box (Dark Purple)
        self.v_box = tk.Frame(stats_container, bg="#5A189A", width=160, height=130)
        self.v_box.pack_propagate(False)
        self.v_box.grid(row=0, column=1, padx=15)
        tk.Label(self.v_box, text="VIOLATIONS", font=("Arial", 10, "bold"), bg="#5A189A", fg="white").pack(pady=10)
        self.v_label = tk.Label(self.v_box, text="0", font=("Arial", 40, "bold"), bg="#5A189A", fg="white")
        self.v_label.pack()

        self.prog_label = tk.Label(self.root, text="Waiting for mission...", font=("Arial", 11), bg="#0D1B2A", fg="#E0E1DD")
        self.prog_label.pack(pady=20)

        self.btn = tk.Button(self.root, text="⚠️ EMERGENCY RTL", bg="#C53030", fg="white", 
                             font=("Arial", 12, "bold"), relief="flat", command=self.emergency_land)
        self.btn.pack(pady=20, padx=50, fill="x")

    def emergency_land(self):
        asyncio.ensure_future(self.drone.action.return_to_launch())

    def update_ui(self, v=0, p=0, spot=None, total=13):
        if p: self.pass_count += p
        if v: self.violation_count += v
        self.p_label.config(text=str(self.pass_count))
        self.v_label.config(text=str(self.violation_count))
        if spot is not None:
            self.prog_label.config(text=f"Spot: {spot} / {total}")
        self.root.update()

#########################################################################
#                                                                       #
#                                                                       #
#                                                                       #
#                     PARKPILOT MISSION                                 #
#                                                                       #
#                                                                       #
#                                                                       #
#########################################################################

async def run():
    if not rclpy.ok(): rclpy.init()
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    app = ParkPilotApp(drone)
    app.root.update()

    print("[INFO] Connecting to ParkPilot Drone...")
    async for state in drone.core.connection_state():
        if state.is_connected: break

    camera = cam_lib.AsyncCameraInterface("/world/parkpilotworld/model/x500_gimbal_0/link/camera_link/sensor/camera/image")
    camera.start()
    
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(save_path, fourcc, 20.0, (640, 480))

    async def update_telemetry():
        async for battery in drone.telemetry.battery():
            app.update_ui(battery=int(battery.remaining_percent * 100))
        async for progress in drone.mission.mission_progress():
            app.update_ui(spot=progress.current, total=progress.total, mode="MISSION")

    asyncio.create_task(update_telemetry())



    coords = [(2.41, -74.46), (2.41, -80.96), (2.41, -96.84), (2.41, -100.18), (2.41, -103.24)]
    base_lat, base_lon = 24.8747139, 46.6382076
    mission_items = []

    print("Fetching home location...")
    async for terrain_info in drone.telemetry.home():
        launch_lat = terrain_info.latitude_deg
        launch_lon = terrain_info.longitude_deg
        break

    mission_items.append(MissionItem(
        launch_lat, launch_lon, 4.5, 1.0, False, -90.0, 0.0, 
        MissionItem.CameraAction.NONE, 5.0, float('nan'), 0.5, 
        float('nan'), float('nan'), MissionItem.VehicleAction.TAKEOFF
    ))

    for x, y in coords:
        mission_items.append(MissionItem(
            base_lat + (y * 1e-5), base_lon + (x * 1e-5), 4.5, 0.5, False, -90.0, 0.0, 
            MissionItem.CameraAction.NONE, 7.0, float('nan'), 0.5, 
            0.0, float('nan'), MissionItem.VehicleAction.NONE
        ))

    mission_items.append(MissionItem(
        launch_lat, launch_lon, 4.5, 1.0, False, -90.0, 0.0, 
        MissionItem.CameraAction.NONE, 5.0, float('nan'), 0.5, 
        float('nan'), float('nan'), MissionItem.VehicleAction.LAND
    ))

    print("Uploading mission plan...")
    await drone.mission.upload_mission(MissionPlan(mission_items))
    await drone.action.arm()
    await drone.mission.start_mission()
    engine.say("Mission active.")
    
    await drone.mission.upload_mission(MissionPlan(mission_items))
    await drone.action.arm()
    await drone.mission.start_mission()
    speak("System is on, Launching Dashboard....")

    last_processed_spot = -1
    barcode_scanned = False

    while True:
        app.root.update()
        
        frame = await camera.get_frame()
        if frame is not None:
            proc, barcode, status = cam_lib.analyze_frame(frame)
            
            cv2.imshow("ParkPilot Live Feed", proc)
            out.write(cv2.resize(proc, (640, 480)))
            
            if barcode and not barcode_scanned:
                barcode_scanned = True
                app.update_ui(p=1)
                speak(f"Verified.")

        if last_processed_spot != -1 and last_processed_spot != (app.root.children.get('!label') or 0): 
            pass 

        
        if cv2.waitKey(1) & 0xFF == ord('q'): break
        await asyncio.sleep(0.01)

    out.release()
    cv2.destroyAllWindows()
    app.root.destroy()

  
if __name__ == "__main__":
    asyncio.run(run())
