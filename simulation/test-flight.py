import airsim
import time

def run_test_flight():
    # MultirotorClient inherits from VehicleClient
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # --- 1. FORCE ARMING (Bypass GPS) ---
    # We use the valid 'setParam' API to tell PX4 to ignore GPS for arming.
    # Parameter names must be strings, values are typically floats in AirSim RPC.
    print("Setting PX4 parameter: COM_ARM_WO_GPS = 1.0")
    client.setParam("COM_ARM_WO_GPS", 1.0)

    client.enableApiControl(True)
    
    last_ts = 0
    try:
        print("Monitoring Telemetry & FPS...")
        while True:
            # getMultirotorState() is the valid method
            state = client.getMultirotorState()
            
            # --- 2. DETECT FPS FROM API ---
            # state.timestamp is a uint64 in nanoseconds
            if last_ts > 0:
                dt_nano = state.timestamp - last_ts
                if dt_nano > 0:
                    # Convert nanoseconds to seconds for FPS
                    actual_fps = 1.0 / (dt_nano / 1e9)
                    print(f"API Detect FPS: {actual_fps:.2f} | Landed: {state.landed_state}", end='\r')
            last_ts = state.timestamp

            # --- 3. ARMING LOGIC ---
            # Check if drone is ready to arm (landed_state 0 is Landed)
            if state.landed_state == airsim.LandedState.Landed:
                print("\nAttempting to Arm...")
                # armDisarm(True) is the valid method
                success = client.armDisarm(True)
                if success:
                    print("✅ Armed successfully!")
                    break
            
            time.sleep(0.1)

        print("Taking off...")
        # takeoffAsync is valid, but may still fail if PX4 has NO position estimate.
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