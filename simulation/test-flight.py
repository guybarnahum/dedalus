import airsim
import time

def run_test_flight():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # Get the vehicle name (verified as 'PX4' in your logs)
    vehicles = client.listVehicles()
    if not vehicles:
        print("❌ No vehicles found!")
        return
    v_name = vehicles[0]

    print(f"Waiting for telemetry from '{v_name}'...")
    
    # 1. LOOP: Wait for valid state before enabling API
    # This prevents the rpclib crash by ensuring the vehicle is fully initialized
    while True:
        state = client.getMultirotorState(vehicle_name=v_name)
        if state.timestamp > 0:
            print(f"✅ Telemetry received. Simulation Timestamp: {state.timestamp}")
            break
        print("... Waiting for initial heartbeat ...")
        time.sleep(1)

    # 2. Enable Control
    # Passing an empty string often bypasses naming bugs in single-vehicle sims
    print(f"Enabling API control for '{v_name}'...")
    client.enableApiControl(True, vehicle_name=v_name)
    
    last_ts = 0
    try:
        # 3. Telemetry & FPS Loop
        for _ in range(50):
            state = client.getMultirotorState(vehicle_name=v_name)
            
            # FPS Calculation: 1 / (Delta Time in seconds)
            if last_ts > 0:
                dt = (state.timestamp - last_ts) / 1e9
                if dt > 0:
                    fps = 1.0 / dt
                    print(f"FPS: {fps:6.2f} | Landed State: {state.landed_state}", end='\r')
            last_ts = state.timestamp

            # 4. Arming Logic
            # Note: Requires COM_ARM_WO_GPS: 1 in settings.json
            if state.landed_state == 0: # LandedState.Landed
                print(f"\nArming '{v_name}'...")
                client.armDisarm(True, vehicle_name=v_name)
                break
            time.sleep(0.1)

        print("Taking off...")
        client.takeoffAsync(vehicle_name=v_name).join()
        
        print("Hovering...")
        time.sleep(5)
        
        client.landAsync(vehicle_name=v_name).join()

    except Exception as e:
        print(f"\n❌ Error during flight: {e}")
    finally:
        client.armDisarm(False, vehicle_name=v_name)
        client.enableApiControl(False, vehicle_name=v_name)
        print("Test concluded.")

if __name__ == "__main__":
    run_test_flight()