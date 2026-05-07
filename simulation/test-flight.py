import airsim
import time

def run_test_flight():
    # Connect to the AirSim simulator 
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # 1. Resolve Vehicle Name
    # Your settings.json defines it as "PX4"
    v_name = "PX4"
    
    print(f"Waiting for telemetry heartbeat from '{v_name}'...")
    while True:
        state = client.getMultirotorState(vehicle_name=v_name)
        if state.timestamp > 0:
            print(f"✅ Connection verified. Initial TS: {state.timestamp}")
            break
        time.sleep(0.5)

    try:
        # 2. Enable API Control
        print(f"Requesting control of '{v_name}'...")
        client.enableApiControl(True, vehicle_name=v_name)
        
        last_ts = 0
        print("Monitoring performance... (Press Ctrl+C to abort)")

        # 3. Arming & FPS Loop
        while True:
            state = client.getMultirotorState(vehicle_name=v_name)
            
            # --- DETECT FPS FROM API ---
            # state.timestamp is in nanoseconds. 
            if last_ts > 0:
                dt = (state.timestamp - last_ts) / 1e9
                if dt > 0:
                    fps = 1.0 / dt
                    # Print status line (LandedState: 0=Landed, 1=Flying)
                    print(f"Current API FPS: {fps:6.2f} | State: {state.landed_state}", end='\r')
            
            last_ts = state.timestamp

            # 4. The Arming Trigger
            if state.landed_state == 0: # LandedState.Landed
                print(f"\nVehicle '{v_name}' reported Landed. Sending Arm command...")
                # This succeeds immediately because of COM_ARM_WO_GPS in settings.json
                client.armDisarm(True, vehicle_name=v_name)
                break
            
            time.sleep(0.1)

        # 5. Flight Sequence
        print("Executing Takeoff...")
        # Note: If this fails, use moveByVelocityAsync(0, 0, -5, 5).join() 
        # as a manual bypass for the high-level takeoff command.
        client.takeoffAsync(vehicle_name=v_name).join()
        
        print("Hovering at altitude...")
        time.sleep(5)
        
        print("Landing...")
        client.landAsync(vehicle_name=v_name).join()

    except Exception as e:
        print(f"\n❌ Flight Error: {e}")
    finally:
        # 6. Safety Cleanup
        print("Cleaning up session...")
        client.armDisarm(False, vehicle_name=v_name)
        client.enableApiControl(False, vehicle_name=v_name)
        print("Test Concluded.")

if __name__ == "__main__":
    run_test_flight()