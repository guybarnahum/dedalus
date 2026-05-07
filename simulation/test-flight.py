import airsim
import time

def run_test_flight():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # 1. Get the correct vehicle name from the server
    vehicles = client.listVehicles()
    if not vehicles:
        print("❌ No vehicles found in simulation!")
        return
    
    vehicle_name = vehicles[0]
    print(f"Found vehicle: '{vehicle_name}'. Enabling control...")

    # 2. Enable Control specifically for this name
    client.enableApiControl(True, vehicle_name=vehicle_name)
    
    last_ts = 0
    try:
        print("Monitoring Telemetry (Press Ctrl+C to stop)...")
        # We'll run a loop to see the FPS and arm status
        while True:
            # getMultirotorState is the standard method for telemetry
            state = client.getMultirotorState(vehicle_name=vehicle_name)
            
            # FPS Calculation from Hardware Timestamps
            if last_ts > 0:
                dt_seconds = (state.timestamp - last_ts) / 1e9
                if dt_seconds > 0:
                    fps = 1.0 / dt_seconds
                    # 0 = Landed, 1 = Flying
                    print(f"Sim FPS: {fps:.2f} | Landed State: {state.landed_state}", end='\r')
            
            last_ts = state.timestamp

            # 3. Arming Logic
            # Note: This will only work if COM_ARM_WO_GPS is set in settings.json
            if state.landed_state == 0: # LandedState.Landed
                print(f"\nVehicle '{vehicle_name}' is Landed. Attempting Arm...")
                client.armDisarm(True, vehicle_name=vehicle_name)
                break
            
            time.sleep(0.1)

        print("Taking off...")
        # Note: If this fails, the EKF still doesn't have a position lock.
        client.takeoffAsync(vehicle_name=vehicle_name).join()
        
        print("Hovering...")
        time.sleep(5)
        
        client.landAsync(vehicle_name=vehicle_name).join()

    except Exception as e:
        print(f"\n❌ Flight Command Failed: {e}")
    finally:
        client.armDisarm(False, vehicle_name=vehicle_name)
        client.enableApiControl(False, vehicle_name=vehicle_name)
        print("Test concluded.")

if __name__ == "__main__":
    run_test_flight()