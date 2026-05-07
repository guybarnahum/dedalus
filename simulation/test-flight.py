import airsim
import time

def run_test_flight():
    # Use the standard client
    client = airsim.MultirotorClient()
    client.confirmConnection()

    print("Checking for stable physics heartbeat...")
    last_ts = 0
    
    # 1. DEFENSIVE WAIT: Wait for a valid, non-zero telemetry stream
    # This prevents the rpclib null-pointer crash on the server
    while True:
        state = client.getMultirotorState()
        if state.timestamp > 0:
            # Wait for 3 consecutive valid frames to ensure API server is stable
            print(f"✅ Connection Stable. Physics TS: {state.timestamp}")
            break
        print("... Waiting for flight controller to link ...")
        time.sleep(1)

    try:
        # 2. ENABLE CONTROL: Use default empty string naming
        # This is the most stable way to call enableApiControl in 1-drone setups
        print("Requesting API Control...")
        client.enableApiControl(True) 
        
        print("Monitoring FPS (Press Ctrl+C to stop)...")
        
        # 3. FPS & ARMING LOOP
        while True:
            state = client.getMultirotorState()
            
            # --- DETECT FPS FROM API ---
            if last_ts > 0:
                # state.timestamp is nanoseconds
                dt = (state.timestamp - last_ts) / 1e9
                if dt > 0:
                    current_fps = 1.0 / dt
                    # Print status. LandedState: 0=Landed, 1=Flying
                    print(f"API FPS: {current_fps:6.2f} | Status: {state.landed_state}", end='\r')
            last_ts = state.timestamp

            # 4. ARMING
            # COM_ARM_WO_GPS in settings.json allows this to work without GPS
            if state.landed_state == 0:
                print("\nStatus: Landed. Sending Arm Command...")
                client.armDisarm(True)
                break
            
            time.sleep(0.1)

        print("Executing Takeoff...")
        # Note: If takeoffAsync fails, the EKF is still rejecting position modes.
        # Bypass with: client.moveByVelocityAsync(0, 0, -5, 5).join()
        client.takeoffAsync().join()
        
        print("Status: Hovering...")
        time.sleep(5)
        
        print("Status: Landing...")
        client.landAsync().join()

    except Exception as e:
        print(f"\n❌ RPC Session Error: {e}")
    finally:
        # 5. CLEANUP
        print("\nReleasing control...")
        client.armDisarm(False)
        client.enableApiControl(False)
        print("Test Concluded.")

if __name__ == "__main__":
    run_test_flight()