
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

import os
import asyncio
import cv2
import rclpy
import pyttsx3
import logging
import tkinter as tk
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.gimbal import GimbalMode 
import gz_camera_to_frame as cam_lib

os.environ["QT_QPA_PLATFORM"] = "xcb"
logging.getLogger('mavsdk').setLevel(logging.ERROR)

desktop_path = os.path.join(os.path.expanduser("~"), "Desktop", "x500parkpilot")
if not os.path.exists(desktop_path): os.makedirs(desktop_path)

SAVE_PATH = os.path.join(desktop_path, "parkpilot_mission.mp4")
engine = pyttsx3.init()


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
        self.root.geometry("450x600") 
        self.root.configure(bg="#F5F7FA")
        self.violation_count = 0
        self.pass_count = 0
        self.total_spots = 5
        self._build_ui()

    def _build_ui(self):
        header = tk.Frame(self.root, bg="#102A43", height=80)
        header.pack(fill="x")
        tk.Label(header, text="PARK PILOT DASHBOARD", font=("Segoe UI", 14, "bold"), bg="#102A43", fg="white").pack(pady=25)
        
        self.bat_label = tk.Label(self.root, text="Battery: --%", font=("Arial", 12, "bold"), bg="#F5F7FA", fg="#2D3748")
        self.bat_label.pack(pady=10)
        
        self.bat_bar = tk.Canvas(self.root, width=200, height=20, bg="#E2E8F0", highlightthickness=0)
        self.bat_bar.pack(pady=5)
        self.bat_fill = self.bat_bar.create_rectangle(0, 0, 0, 20, fill="#48BB78")

        stats_container = tk.Frame(self.root, bg="#F5F7FA")
        stats_container.pack(pady=20)
        self.p_label = tk.Label(stats_container, text="0", font=("Arial", 35, "bold"), bg="#102A43", fg="white", width=6)
        self.p_label.grid(row=0, column=0, padx=10)
        self.v_label = tk.Label(stats_container, text="0", font=("Arial", 35, "bold"), bg="#C53030", fg="white", width=6)
        self.v_label.grid(row=0, column=1, padx=10)
        self.prog_label = tk.Label(self.root, text="System Ready", font=("Arial", 10), bg="#F5F7FA")
        self.prog_label.pack(pady=10)
        self.btn = tk.Button(self.root, text="⚠️ EMERGENCY Return to Station", bg="#D32F2F", fg="white", 
                             font=("Arial", 11, "bold"), relief="flat", command=self.emergency_land)
        self.btn.pack(pady=25, padx=40, fill="x")

    def emergency_land(self):
        asyncio.ensure_future(self.drone.action.return_to_launch())

    def update_ui(self, v=0, p=0, spot=None, battery=None):
        if v: self.violation_count += v
        if p: self.pass_count += p
        self.p_label.config(text=str(self.pass_count))
        self.v_label.config(text=str(self.violation_count))
        if spot is not None: self.prog_label.config(text=f"Progress: {spot} / {self.total_spots}")
        
         #update monitor battery 
        if battery is not None:
            pct = int(battery * 100)
            self.bat_label.config(text=f"Battery: {pct}%")
            color = "#48BB78" if pct > 40 else "#ECC94B" if pct > 20 else "#F56565"
            self.bat_bar.coords(self.bat_fill, 0, 0, pct * 2, 20)
            self.bat_bar.itemconfig(self.bat_fill, fill=color)
        
        self.root.update_idletasks()

async def run():
    if not rclpy.ok(): rclpy.init()
    drone = System()
    await drone.connect(system_address="udp://:14540")
    app = ParkPilotApp(drone)
    
    print("Waiting for Drone Telemetry...")
    async for state in drone.core.connection_state():
        if state.is_connected: break

    async def watch_battery():
        async for battery in drone.telemetry.battery():
            app.update_ui(battery=battery.remaining_percent)
            if battery.remaining_percent < 0.20: 
                engine.say("Low battery alert. Returning to base.")
                await drone.mission.pause_mission()
                await drone.action.return_to_launch()
                break

    asyncio.ensure_future(watch_battery())

    async for pos in drone.telemetry.position():
        launch_lat, launch_lon = pos.latitude_deg, pos.longitude_deg
        break

    camera = cam_lib.AsyncCameraInterface("/world/parkpilotworld/model/x500_gimbal_0/link/camera_link/sensor/camera/image")
    camera.start()
    out = cv2.VideoWriter(SAVE_PATH, cv2.VideoWriter_fourcc(*'mp4v'), 20.0, (640, 480))


#########################################################################
#                                                                       #
#                                                                       #
#                                                                       #
#                     PARKPILOT MISSION                                 #
#                                                                       #
#                                                                       #
#                                                                       #
#########################################################################


    coords = [(2.41, -74.46), (2.41, -80.96), (2.41, -96.84), (2.41, -100.18), (2.41, -103.24)]
    base_lat, base_lon = 24.8747139, 46.6382076
    mission_items = []

    mission_items.append(MissionItem(launch_lat, launch_lon, 4.5, 1.0, False, float('nan'), float('nan'), MissionItem.CameraAction.NONE, 5.0, float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.TAKEOFF))
    for x, y in coords:
        mission_items.append(MissionItem(base_lat + (y * 1e-5), base_lon + (x * 1e-5), 4.5, 0.7, False, float('nan'), float('nan'), MissionItem.CameraAction.NONE, 7.0, float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE))
    mission_items.append(MissionItem(launch_lat, launch_lon, 5.0, 1.0, False, float('nan'), float('nan'), MissionItem.CameraAction.NONE, 5.0, float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE))
    mission_items.append(MissionItem(launch_lat, launch_lon, 0.0, 1.0, False, float('nan'), float('nan'), MissionItem.CameraAction.NONE, 5.0, float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.LAND))

    await drone.mission.upload_mission(MissionPlan(mission_items))
    await drone.action.arm()
    await asyncio.sleep(2)
    await drone.mission.start_mission()
    engine.say("Mission active with battery monitoring.")

    last_spot = -1
    scanned = False
    mission_finished = False

    try:
        while not mission_finished:
            app.root.update()
            frame = await camera.get_frame()
            if frame is not None:
                proc, barcode, _ = cam_lib.analyze_frame(frame)
                cv2.imshow("ParkPilot Live Feed", proc)
                out.write(cv2.resize(proc, (640, 480)))
                if 0 < last_spot <= len(coords) and barcode and not scanned:
                    scanned = True
                    app.update_ui(p=1)
                    engine.say("Verified.")

            try:
                prog = await drone.mission.get_mission_progress()
                if prog.current != last_spot:
                    if 0 < last_spot <= len(coords) and not scanned:
                        app.update_ui(v=1)
                        engine.say("Violation.")
                    last_spot = prog.current
                    scanned = False
                    app.update_ui(spot=prog.current)
                if prog.current == prog.total and prog.total != 0:
                    mission_finished = True
            except: pass

            if cv2.waitKey(1) & 0xFF == ord('q'): break
            await asyncio.sleep(0.01)
    finally:
        out.release() 
        cv2.destroyAllWindows()
        try: app.root.destroy()
        except: pass
        engine.say("Mission Finished.")

if __name__ == "__main__":
    asyncio.run(run())
    
