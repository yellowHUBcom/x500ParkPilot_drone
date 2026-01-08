import asyncio
import cv2
import rclpy
import pyttsx3
import tkinter as tk
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
import gz_camera_to_frame as cam_lib

# إعداد الصوت
engine = pyttsx3.init()
def speak(text):
    print(f"[VOICE]: {text}")
    engine.say(text)
    engine.runAndWait()

class ParkPilotApp:
    def __init__(self, drone):
        self.drone = drone
        self.root = tk.Tk()
        self.root.title("PARKPILOT REAL-TIME MONITOR")
        self.root.geometry("350x300")
        self.root.attributes("-topmost", True)
        
        self.violation_count = 0
        self.pass_count = 0
        self.current_spot = 0

        self.batt_label = tk.Label(self.root, text="Battery: --%", font=("Arial", 12, "bold"), fg="blue")
        self.batt_label.pack(pady=5)
        
        self.prog_label = tk.Label(self.root, text="Progress: Waiting for start...", font=("Arial", 11), fg="purple")
        self.prog_label.pack(pady=5)
        
        self.v_label = tk.Label(self.root, text="Violations: 0", font=("Arial", 14), fg="red")
        self.v_label.pack()
        
        self.p_label = tk.Label(self.root, text="Valid Passes: 0", font=("Arial", 14), fg="green")
        self.p_label.pack()

        self.btn = tk.Button(self.root, text="EMERGENCY LAND", bg="red", fg="white", 
                             font=("Arial", 12, "bold"), command=self.emergency_land)
        self.btn.pack(pady=15)

    def emergency_land(self):
        print("[EMERGENCY] RTL Triggered!")
        asyncio.ensure_future(self.drone.action.return_to_launch())

    def update_stats(self, v=0, p=0, spot=0, total=5):
        self.violation_count += v
        self.pass_count += p
        self.current_spot = spot
        self.v_label.config(text=f"Violations: {self.violation_count}")
        self.p_label.config(text=f"Valid Passes: {self.pass_count}")
        self.prog_label.config(text=f"Current Spot: {spot} / {total}")
        self.root.update()

async def monitor_progress(drone, app):
    async for progress in drone.mission.mission_progress():
        print(f"[MISSION UPDATE] Item {progress.current} of {progress.total}")
        app.update_stats(spot=progress.current, total=progress.total)

async def battery_checker(drone, app):
    async for battery in drone.telemetry.battery():
        percent = int(battery.remaining_percent * 100)
        app.batt_label.config(text=f"Battery: {percent}%")
        app.root.update()
        if percent < 20:
            speak("Emergency! Battery low.")
            await drone.action.return_to_launch()
            break

async def run():
    if not rclpy.ok(): rclpy.init()
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("[INFO] Initializing Interface...")
    app = ParkPilotApp(drone)
    app.root.update() 

    print("[INFO] Connecting to Drone...")
    async for state in drone.core.connection_state():
        if state.is_connected: break

    camera = cam_lib.AsyncCameraInterface("/world/parkpilotworld/model/x500_gimbal/link/camera_link/sensor/camera/image")
    camera.start()

    asyncio.create_task(monitor_progress(drone, app))
    asyncio.create_task(battery_checker(drone, app))

    coords = [(2.41, -74.46), (2.41, -80.96), (2.41, -96.84), (2.41, -100.18), (2.41, -103.24)]
    mission_items = []
    base_lat, base_lon = 24.8747139, 46.6382076
    for x, y in coords:
        mission_items.append(MissionItem(base_lat + (y * 1e-5), base_lon + (x * 1e-5), 2.0, 0.7, False, float('nan'), float('nan'), MissionItem.CameraAction.NONE, 7.0, float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE))
    
    await drone.mission.set_return_to_launch_after_mission(True)
    await drone.mission.upload_mission(MissionPlan(mission_items))
    
    await drone.action.arm()
    await drone.mission.start_mission()
    speak("System Online. Starting Southbound inspection.")

    last_processed_spot = -1
    barcode_scanned = False

    while True:
        app.root.update()
    
        frame = await camera.get_frame()
        if frame is not None:
            proc, barcode, status = cam_lib.analyze_frame(frame)
            cv2.imshow("ParkPilot Live Scan", proc)
            
            if barcode and not barcode_scanned:
                barcode_scanned = True
                app.update_stats(p=1, spot=app.current_spot)
                speak(f"Pass verified.")

        current = app.current_spot
        if current != last_processed_spot:
            if last_processed_spot != -1 and not barcode_scanned:
                app.update_stats(v=1, spot=current)
                speak(f"Violation detected.")
            last_processed_spot = current
            barcode_scanned = False

        if cv2.waitKey(1) & 0xFF == ord('q'): break
        await asyncio.sleep(0.05)

    cv2.destroyAllWindows()
    app.root.destroy()

if __name__ == "__main__":
    asyncio.run(run())