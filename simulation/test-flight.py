import airsim
import time

def run_test_flight():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # --- API-CORRECT HOME SET ---
    print("Setting Home Location via simSetHomeLocation...")
    home_gps = airsim.GeoPoint()
    home_gps.latitude = 47.641468
    home_gps.longitude = -122.140165
    home_gps.altitude = 121.0
    
    # Correct API method name
    client.simSetHomeLocation(home_gps)

    client.enableApiControl(True)
    
    # --- FPS & ARMING LOGIC ---
    last_ts = 0
    try:
        print("Waiting for drone to report ready...")
        while True:
            state = client.getMultirotorState()
            
            # Detect FPS from API timestamps (nanoseconds)
            if last_ts > 0:
                dt = (state.timestamp - last_ts) / 1e9
                if dt > 0:
                    fps = 1.0 / dt
                    print(f"Current API FPS: {fps:.2f}", end='\r')
            last_ts = state.timestamp

            # Check if we can arm
            # If you set COM_ARM_WO_GPS in settings.json, this loop can be shorter
            if state.landed_state == airsim.LandedState.Landed:
                print("\nDrone is ready. Arming...")
                client.armDisarm(True)
                break
            time.sleep(0.5)

        print("Taking off...")
        client.takeoffAsync().join()
        
        print("Hovering...")
        time.sleep(5)
        
        client.landAsync().join()

    except Exception as e:
        print(f"\n❌ Flight Command Failed: {e}")
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)
        print("Test concluded.")

if __name__ == "__main__":
    run_test_flight()