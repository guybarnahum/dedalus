import airsim
import time

def run_test_flight():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # Note: No setParam or setHomeLocation here. 
    # They must be in settings.json to work.

    client.enableApiControl(True)
    
    last_ts = 0
    try:
        print("Monitoring Simulation State...")
        while True:
            state = client.getMultirotorState()
            
            # --- FPS DETECTION ---
            # state.timestamp is in nanoseconds. 
            # This calculates the true physics-ticking rate.
            if last_ts > 0:
                dt = (state.timestamp - last_ts) / 1e9
                if dt > 0:
                    fps = 1.0 / dt
                    print(f"Simulation FPS: {fps:.2f} | State: {state.landed_state}", end='\r')
            last_ts = state.timestamp

            # --- ARMING ---
            # 0 = Landed. With COM_ARM_WO_GPS=1 in settings.json, this will succeed.
            if state.landed_state == 0: 
                print("\nStatus: Landed. Attempting Arm...")
                client.armDisarm(True)
                break
            time.sleep(0.1)

        print("Status: Taking off...")
        # If takeoffAsync fails (it requires a position lock), 
        # use moveByVelocityAsync(0, 0, -5, 5) for a local manual climb.
        client.takeoffAsync().join()
        
        print("Status: Hovering...")
        time.sleep(5)
        
        print("Status: Landing...")
        client.landAsync().join()

    except Exception as e:
        print(f"\n❌ RPC Error: {e}")
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)
        print("Test Concluded.")

if __name__ == "__main__":
    run_test_flight()