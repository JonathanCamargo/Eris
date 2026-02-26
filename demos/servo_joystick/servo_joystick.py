"""
Joystick-to-servo demo for ErisServo firmware.

Maps joystick axes to servo angles and sends smooth movement commands
using the ErisServo 'Y' command (rate-limited stepping on the MCU).

Usage:
    python joystick_eris.py [PORT]

Requires: pygame, pyserial
"""

import pygame
import serial
import time
import sys

# --- Config ---
SERIAL_PORT = "COM6"       # Change to your Arduino's port (or pass as argument)
BAUD_RATE = 115200         # ErisServo console baud
UPDATE_HZ = 20             # Send rate
DEADBAND = 2               # Ignore angle changes smaller than this (degrees)
ANGLE_MIN = 0
ANGLE_MAX = 180

# Servo channel mapping (PCA9685 channels)
SERVO_CH_X = 0             # Joystick left-right axis
SERVO_CH_Y = 1             # Joystick up-down axis


def joystick_to_angle(axis_value):
    """Map joystick axis (-1.0 to 1.0) to servo angle (0 to 180)."""
    return int((axis_value + 1.0) / 2.0 * (ANGLE_MAX - ANGLE_MIN) + ANGLE_MIN)


def send_command(ser, command):
    """Send a text command to Eris (newline-terminated)."""
    ser.write((command + "\n").encode())
    ser.flush()


def read_response(ser):
    """Read and print any available text from Eris."""
    while ser.in_waiting:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            print(f"  <- {line}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT

    # --- Initialize pygame ---
    # A display surface is required on Windows for the event loop
    # to deliver joystick events
    pygame.init()
    pygame.display.set_mode((200, 200))
    pygame.display.set_caption("ErisServo Joystick")
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick found. Connect one and try again.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Using joystick: {joystick.get_name()}")

    # --- Connect to ErisServo ---
    ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    time.sleep(2)  # Wait for MCU reset after serial connection
    print(f"Connected to {port}")

    # Read the Eris startup messages (HELLO, Servos ready, etc.)
    read_response(ser)

    # Query firmware info
    send_command(ser, "INFO")
    time.sleep(0.2)
    read_response(ser)

    print(f"\nControls:")
    print(f"  Joystick X axis -> Servo channel {SERVO_CH_X}")
    print(f"  Joystick Y axis -> Servo channel {SERVO_CH_Y}")
    print(f"  Ctrl+C to quit\n")

    last_x = -1
    last_y = -1
    interval = 1.0 / UPDATE_HZ

    try:
        while True:
            pygame.event.pump()

            angle_x = joystick_to_angle(-joystick.get_axis(1))  # Up-down left stick
            angle_y = joystick_to_angle(joystick.get_axis(3))  # Up-down right stick

            print(f"Joystick: X={joystick.get_axis(0):.2f} Y={joystick.get_axis(1):.2f} -> Angles: X={angle_x} Y={angle_y}", end="\r")
            # Only send if angle changed beyond deadband
            if abs(angle_x - last_x) >= DEADBAND or abs(angle_y - last_y) >= DEADBAND:
                send_command(ser, f"Y {SERVO_CH_X} {angle_x}")
                send_command(ser, f"Y {SERVO_CH_Y} {angle_y}")
                last_x = angle_x
                last_y = angle_y
                print(f"Y {SERVO_CH_X} {angle_x}  |  Y {SERVO_CH_Y} {angle_y}")

            # Print any responses from Eris
            read_response(ser)

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        ser.close()
        pygame.quit()


if __name__ == "__main__":
    main()
