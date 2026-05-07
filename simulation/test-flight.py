import airsim
import time
import sys

def run_test_flight():
    # Connect to the AirSim RPC server
    client = airsim.MultirotorClient()
    
    try:
        client.confirmConnection()
        print("Connected to AirSim. Requesting control...")
    except Exception as e:
        print(f"FAILED to connect: {e}")
        sys.exit(1)

    # 1. Enable API Control (Required for PX4 to accept external commands)
    client.enableApiControl(True)
    
    # 2. Arm the drone
    print("Arming propellers...")
    client.armDisarm(True)
    time.sleep(2)

    # 3. Takeoff
    # Note: For PX4, this sends a MAVLink takeoff command via AirSim
    print("Taking off...")
    f1 = client.takeoffAsync()
    f1.join() # Wait for completion

    print("Hovering at 5 meters. Observe in DCV window.")
    time.sleep(5)

    # 4. Land
    print("Landing...")
    f2 = client.landAsync()
    f2.join()

    # 5. Cleanup
    client.armDisarm(False)
    client.enableApiControl(False)
    print("Test Complete. Drone Disarmed.")

if __name__ == "__main__":
    run_test_flight()