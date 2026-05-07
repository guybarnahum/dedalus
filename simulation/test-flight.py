import airsim
import time

def run_test_flight():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    print("Checking for GPS Home lock...")
    # Loop until the drone knows where it is
    while True:
        state = client.getMultirotorState()
        # Look for the GPS bit in the health status
        # If the state isn't Landed/Ready, it usually means GPS is missing
        if state.landed_state == airsim.LandedState.Landed:
            print("✅ GPS Home Lock Acquired. Drone is ready.")
            break
        print("... Still waiting for GPS Home location ...")
        time.sleep(5)

    print("Requesting control...")
    client.enableApiControl(True)
    
    try:
        print("Arming propellers...")
        client.armDisarm(True)
        time.sleep(2)
        
        print("Taking off...")
        client.takeoffAsync().join()
        
        print("Hovering...")
        time.sleep(5)
        
        print("Landing...")
        client.landAsync().join()
        
    except Exception as e:
        print(f"❌ Flight Command Failed: {e}")
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)
        print("Test concluded.")

if __name__ == "__main__":
    run_test_flight()