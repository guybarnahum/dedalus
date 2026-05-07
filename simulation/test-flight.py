import airsim
import time

def run_test_flight():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # 1. Use the name exactly as defined in settings.json
    v_name = "PX4"
    
    print(f"Waiting for PX4 SITL to initialize fully...")
    
    # We must wait until the drone is actually reporting a valid 
    # orientation/position before we can 'take control' of the API.
    while True:
        state = client.getMultirotorState(vehicle_name=v_name)
        # Check for non-zero timestamp AND valid kinematic data
        if state.timestamp > 0 and abs(state.kinematics_estimated.orientation.w_val) > 0.1:
            print(f"✅ PX4 Link Established. TS: {state.timestamp}")
            break
        print("... PX4 is still booting (checking sensors) ...")
        time.sleep(2)

    try:
        # 2. ENABLE CONTROL: Use a raw call to avoid wrapper argument bugs
        print(f"Requesting control of '{v_name}'...")
        # We call the underlying msgpack-rpc directly to ensure no extra args are sent
        client.client.call('enableApiControl', True, v_name)
        
        last_ts = 0
        print("Monitoring Telemetry & Performance...")

        # 3. FPS & ARMING LOOP
        # We'll loop for 10 seconds to see the FPS before arming
        start_wait = time.time()
        while time.time() - start_wait < 10:
            state = client.getMultirotorState(vehicle_name=v_name)
            
            # --- API FPS DETECTION ---
            if last_ts > 0:
                dt_nano = state.timestamp - last_ts
                if dt_nano > 0:
                    fps = 1.0 / (dt_nano / 1e9)
                    print(f"API FPS: {fps:6.2f} | PX4 Status: {state.landed_state}", end='\r')
            last_ts = state.timestamp
            time.sleep(0.1)

        # 4. FORCE ARMING
        # Because of COM_ARM_WO_GPS: 1 in your settings.json, this bypasses the GPS lock.
        print("\nSending Arm Command...")
        client.client.call('armDisarm', True, v_name)
        
        print("Executing Takeoff...")
        # If this still fails, your EKF is still 'pre-flight checking'. 
        # Bypass by using a direct velocity command:
        # client.moveByVelocityAsync(0, 0, -5, 5, vehicle_name=v_name).join()
        client.takeoffAsync(vehicle_name=v_name).join()
        
        print("Hovering...")
        time.sleep(5)
        
        client.landAsync(vehicle_name=v_name).join()

    except Exception as e:
        print(f"\n❌ RPC Session Error: {e}")
    finally:
        # Cleanup
        try:
            client.client.call('armDisarm', False, v_name)
            client.client.call('enableApiControl', False, v_name)
        except:
            pass
        print("Test Concluded.")

if __name__ == "__main__":
    run_test_flight()