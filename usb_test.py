import serial
import time

def send_velocities(linear_velocity, angular_velocity):
    try:
        # Open a serial connection to your Arduino
        ser = serial.Serial('/dev/serial/by-id/usb-www.freetronics.com_0003_754393134373510162F0-if00', 115200)  # Adjust 'COM3' based on your Arduino's serial port

        hex_linear_velocity = format(linear_velocity, '08x')
        hex_angular_velocity = format(angular_velocity, '08x')
        # Format the data as a string and send it to the Arduino
        data = f"{{S8{hex_linear_velocity}{hex_angular_velocity}}}"
        ser.write(data.encode())

        # Close the serial connection
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Example linear and angular velocities
    linear_velocity = int(0.5*1000)
    angular_velocity = int(1.0*1000)

    # Send velocities
    send_velocities(linear_velocity, angular_velocity)
