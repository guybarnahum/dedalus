import airsim
import time

def run_test_flight():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # 1. Set Home Location via Native API
    # This aligns the simulator's coordinate origin.
    print("Setting Home Location...")
    home_gps = airsim.GeoPoint(latitude=47.641468, longitude=-122.140165, altitude=121.0)
    client.setHomeLocation(home_gps)

    # 2. Force Arming (Bypass GPS Check)
    # We set the PX4 parameter COM_ARM_WO_GPS (Allow arming without GPS) to 1.
    print("Configuring PX4 to allow arming without GPS...")
    client.setParam("COM_ARM_WO_GPS", 1)

    print("Requesting control...")
    client.enableApiControl(True)

    try:
        print("Arming propellers...")
        # This sends the arm command. If the EKF hasn't finished initializing 
        # local sensors (accel/gyro), it may still return an RPC error.
        client.armDisarm(True)
        time.sleep(2)

        print("Taking off...")
        # CRITICAL: Automated commands like takeoffAsync() require a position lock.
        # While the drone is now ARMED, this specific command may still fail 
        # if the firmware doesn't have a valid EKF position yet.
        client.takeoffAsync().join()

        print("Hovering at 5m...")
        time.sleep(5)

        print("Landing...")
        client.landAsync().join()

    except Exception as e:
        print(f"❌ Flight Command Failed: {e}")
        print("Note: If arming succeeded but takeoff failed, PX4 is still rejecting ")
        print("position-based commands until the EKF converges.")
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)
        print("Test concluded.")

if __name__ == "__main__":
    run_test_flight()