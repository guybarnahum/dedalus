import airsim
import time

def run_test_flight():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    print("Bypassing wrapper naming to avoid rpclib signature crash...")
    
    try:
        # 1. Enable API Control (Direct RPC call with 1 argument)
        # This bypasses the Python wrapper that forces a second 'vehicle_name' argument
        client.client.call('enableApiControl', True)
        print("✅ API Control Enabled.")

        last_ts = 0
        start_time = time.time()
        
        # 2. Telemetry & FPS Loop
        while time.time() - start_time < 15:  # Monitor for 15 seconds
            state = client.getMultirotorState()
            
            # FPS Detection via API Timestamps
            if last_ts > 0:
                dt_seconds = (state.timestamp - last_ts) / 1e9
                if dt_seconds > 0:
                    fps = 1.0 / dt_seconds
                    # Print stats. LandedState: 0=Landed, 1=Flying
                    print(f"API FPS: {fps:6.2f} | LandedState: {state.landed_state}", end='\r')
            last_ts = state.timestamp

            # 3. Arming Logic
            # Note: COM_ARM_WO_GPS=1 in settings.json handles the "No GPS" requirement
            if state.landed_state == 0: 
                print("\nVehicle is Landed. Sending Direct Arm Command...")
                # Direct RPC call with 1 argument
                client.client.call('armDisarm', True)
                print("🚀 Arm command sent. Check DCV window for propeller spin.")
                break
            
            time.sleep(0.1)

        print("\nExecuting Takeoff...")
        # takeoffAsync usually only takes 1 arg (vehicle_name) in the wrapper,
        # but if it fails, we use the direct call: client.client.call('takeoff', '')
        client.takeoffAsync().join()
        
        print("Hovering...")
        time.sleep(5)
        
        print("Landing...")
        client.landAsync().join()

    except Exception as e:
        print(f"\n❌ RPC Direct Call Error: {e}")
    finally:
        # Clean up
        try:
            client.client.call('armDisarm', False)
            client.client.call('enableApiControl', False)
        except:
            pass
        print("\nTest Concluded.")

if __name__ == "__main__":
    run_test_flight()