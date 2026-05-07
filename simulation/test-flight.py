import airsim
import time

def run_test_flight():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # --- FIX: Proper GeoPoint Initialization ---
    print("Setting Home Location via API...")
    home_gps = airsim.GeoPoint()
    home_gps.latitude = 47.641468
    home_gps.longitude = -122.140165
    home_gps.altitude = 121.0
    
    # This aligns the simulator's coordinate origin
    client.setHomeLocation(home_gps)

    print("Requesting control...")
    client.enableApiControl(True)

    try:
        print("Attempting to arm...")
        # This will now work ONLY if COM_ARM_WO_GPS is set in settings.json
        client.armDisarm(True)
        time.sleep(2)

        print("Taking off...")
        # Note: takeoffAsync() requires a position lock. 
        # If you truly have NO GPS, you must use moveByVelocityAsync instead.
        client.takeoffAsync().join()

        print("Hovering...")
        time.sleep(5)

        client.landAsync().join()

    except Exception as e:
        print(f"❌ Flight Command Failed: {e}")
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)
        print("Test concluded.")

if __name__ == "__main__":
    run_test_flight()