import time
import os
import math
from pymavlink import mavutil

# --- Function to clear the console screen
def clear_console():
    os.system('clear' if os.name == 'posix' else 'cls')

# --- Create the connection
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)

# --- Wait for the first heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)!" %
      (master.target_system, master.target_component))
print("-" * 30)
time.sleep(1)

# --- Request all desired data streams
# Use a dictionary to set the frequency of each stream
# (0 to disable, -1 to leave unchanged)
# For a list of streams, see: https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
stream_rates = {
    mavutil.mavlink.MAV_DATA_STREAM_ALL: 0,                # Disable all streams first
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS: 2,    # For SYS_STATUS (Battery)
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1: 5,             # For ATTITUDE
    mavutil.mavlink.MAV_DATA_STREAM_POSITION: 2,           # For GLOBAL_POSITION_INT (Altitude, GPS)
    mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS: 0,        # Optional: Raw sensors
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS: 2         # For SERVO_OUTPUT_RAW (Motor Speed)
}

print("Requesting data streams...")
for stream_id, rate in stream_rates.items():
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        stream_id,
        rate,
        1  # 1 to start sending
    )
    time.sleep(0.1)
print("Data streams requested.")
print("-" * 30)
time.sleep(2)


# --- Main loop to receive and display data
try:
    while True:
        # We use a non-blocking recv_match to keep the loop running
        msg = master.recv_match(blocking=False)
        if not msg:
            # No new message, continue to the next iteration
            continue

        msg_type = msg.get_type()
        
        # Create a formatted dashboard string
        dashboard = []
        
        # --- System & Armed Status (from HEARTBEAT) ---
        armed_status = "ARMED" if master.motors_armed() else "DISARMED"
        dashboard.append(f"Status: {master.flightmode} | {armed_status}")

        # --- Battery Info (from SYS_STATUS) ---
        if msg_type == 'SYS_STATUS':
            voltage = msg.voltage_battery / 1000.0  # mV to V
            current = msg.current_battery / 100.0  # cA to A
            remaining = msg.battery_remaining # %
            dashboard.append(f"Battery: {voltage:.2f}V | {current:.2f}A | {remaining}%")
        
        # --- Attitude (from ATTITUDE) ---
        if msg_type == 'ATTITUDE':
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)
            dashboard.append(f"Attitude: R={roll:6.1f}° P={pitch:6.1f}° Y={yaw:6.1f}°")

        # --- Altitude & GPS (from GLOBAL_POSITION_INT and GPS_RAW_INT) ---
        if msg_type == 'GLOBAL_POSITION_INT':
            alt = msg.relative_alt / 1000.0 # mm to m
            dashboard.append(f"Altitude: {alt:.2f} m (rel)")
            
        if msg_type == 'GPS_RAW_INT':
            sats = msg.satellites_visible
            fix = msg.fix_type
            dashboard.append(f"GPS:      {sats} satellites | Fix Type: {fix}")

        # --- Motor/Servo Speeds (from SERVO_OUTPUT_RAW) ---
        if msg_type == 'SERVO_OUTPUT_RAW':
            # This shows the PWM values sent to the first 4 motors
            m1, m2, m3, m4 = msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw
            dashboard.append(f"Motors:   M1={m1} M2={m2} M3={m3} M4={m4}")
            
        # --- Update the display ---
        if dashboard:
#            clear_console()
            print("--- VEHICLE STATUS ---")
            # This part is a bit tricky to avoid flickering. We store last known values.
            # For this simple script, we just print the latest received data.
            # In a real GCS, you would update a state dictionary.
            # This example prints whatever new data came in. A full dashboard would be more complex.
            print("\n".join(dashboard))
            print("\n(Press Ctrl+C to exit)")
            
        # Control the loop rate to about 10 Hz to reduce CPU load
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nScript terminated by user.")
