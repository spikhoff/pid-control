#!/usr/bin/env python3
"""
A simple Python program that implements a PID controller, reads data from an
input file, and writes the control outputs to an output file.
"""

import sys

class PIDController:
    """
    A simple PID controller class.
    
    Attributes:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        setpoint (float): Desired target value.
        integral (float): Accumulated integral term.
        prev_error (float): Previous error term for the derivative calculation.
    """

    def __init__(self, Kp, Ki, Kd, setpoint):
        """
        Initialize the PID controller with tuning parameters and setpoint.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        
        # Internal states
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, current_value, dt):
        """
        Calculate the PID output based on the current measurement.
        
        Args:
            current_value (float): The current process value (measurement).
            dt (float): Time elapsed between updates, in seconds.

        Returns:
            float: The control signal (output of the PID controller).
        """
        # Calculate error (difference between setpoint and current measurement)
        error = self.setpoint - current_value

        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        D = self.Kd * derivative

        # Update previous error
        self.prev_error = error

        # Control output
        output = P + I + D
        return output


def main():
    """
    Main function to demonstrate reading from a file, computing PID outputs,
    and writing results to another file.
    """
    # PID parameters
    Kp = 1.0   # Proportional gain
    Ki = 0.1   # Integral gain
    Kd = 0.05  # Derivative gain
    
    # Desired setpoint for the process variable
    setpoint = 20.0

    # Time step (seconds) between measurements
    dt = 1.0

    # Initialize PID controller
    pid = PIDController(Kp, Ki, Kd, setpoint)

    # Define input and output file names
    input_file_name = "input_values.txt"
    output_file_name = "output_values.txt"

    try:
        with open(input_file_name, "r") as infile, open(output_file_name, "w") as outfile:
            # Read each line from the input file
            for line in infile:
                line = line.strip()
                if not line:
                    continue  # skip any empty lines

                # Convert the current measurement from string to float
                try:
                    current_value = float(line)
                except ValueError:
                    # Skip invalid lines that aren't numbers
                    print(f"Warning: '{line}' is not a valid float. Skipping.")
                    continue

                # Compute PID output
                control_signal = pid.update(current_value, dt)

                # Write the result to the output file
                outfile.write(f"{control_signal:.4f}\n")

        print(f"PID control outputs successfully written to '{output_file_name}'.")
    except FileNotFoundError:
        print(f"Error: Could not find file '{input_file_name}'. Please create it or specify the correct file name.")
        sys.exit(1)
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
