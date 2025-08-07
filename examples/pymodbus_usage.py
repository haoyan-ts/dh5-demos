#!/usr/bin/env python3
"""
Example demonstrating the complete refactored DH5ModbusAPI usage

This example shows how to use the comprehensive API with:
- Context manager support
- Initialization methods
- Individual axis control
- Configuration methods
- Status monitoring
"""

import time
from dh5_api import DH5ModbusAPI, DH5Registers

PORT = "COM4"  # Replace with your actual port


def initialization_example():
    """Example of robot initialization"""
    print("=== Robot Initialization Example ===")

    try:
        with DH5ModbusAPI(port=PORT, timeout=2.0) as robot:
            print("Connected to DH5 robot successfully!")

            # Check current initialization status
            print("\nChecking initialization status...")
            status = robot.check_initialization()
            if isinstance(status, dict):
                for axis, state in status.items():
                    print(f"{axis}: {state}")
            else:
                print(f"Error reading initialization status: {status}")

            # Initialize all axes to find total stroke
            print("\nInitializing all axes to find total stroke...")
            result = robot.initialize(DH5Registers.INIT_MODE_FIND_STROKE)
            if result == robot.SUCCESS:
                print("Initialization command sent successfully")

                # Wait for initialization to complete
                print("Waiting for initialization to complete...")
                while True:
                    status = robot.check_initialization()
                    if isinstance(status, dict):
                        initializing = any(
                            state == "initializing" for state in status.values()
                        )
                        if not initializing:
                            print("All axes initialization completed")
                            break
                        print("Still initializing...")
                        time.sleep(1)
                    else:
                        print(f"Error checking status: {status}")
                        break
            else:
                print(f"Error sending initialization command: {result}")

    except Exception as e:
        print(f"Error during initialization: {e}")


def individual_axis_control_example():
    """Example of individual axis control"""
    print("\n=== Individual Axis Control Example ===")

    try:
        with DH5ModbusAPI(port=PORT) as robot:
            # Set individual axis parameters
            axis = 1
            print(f"\nConfiguring axis {axis}...")

            # Set position, speed, and force for axis 1
            result = robot.set_axis_position(axis, 1000)
            if result == robot.SUCCESS:
                print(f"Axis {axis} position set to 1000")
            else:
                print(f"Error setting axis {axis} position: {result}")

            result = robot.set_axis_speed(axis, 500)
            if result == robot.SUCCESS:
                print(f"Axis {axis} speed set to 500")
            else:
                print(f"Error setting axis {axis} speed: {result}")

            result = robot.set_axis_force(axis, 100)
            if result == robot.SUCCESS:
                print(f"Axis {axis} force set to 100")
            else:
                print(f"Error setting axis {axis} force: {result}")

            # Read back current values
            print(f"\nReading current values for axis {axis}...")

            position = robot.get_axis_position(axis)
            if isinstance(position, list):
                print(f"Axis {axis} current position: {position[0]}")
            else:
                print(f"Error reading axis {axis} position: {position}")

            speed = robot.get_axis_speed(axis)
            if isinstance(speed, list):
                print(f"Axis {axis} current speed: {speed[0]}")
            else:
                print(f"Error reading axis {axis} speed: {speed}")

            current = robot.get_axis_current(axis)
            if isinstance(current, list):
                print(f"Axis {axis} current: {current[0]}")
            else:
                print(f"Error reading axis {axis} current: {current}")

    except Exception as e:
        print(f"Error during axis control: {e}")


def comprehensive_status_monitoring():
    """Example of comprehensive status monitoring"""
    print("\n=== Comprehensive Status Monitoring ===")

    try:
        with DH5ModbusAPI(port=PORT) as robot:
            print("Monitoring robot status...")

            # Check if robot is busy
            if robot.is_busy:
                print("Robot is currently busy")
            else:
                print("Robot is idle")

            # Get all positions
            positions = robot.get_all_positions()
            if isinstance(positions, list):
                print(f"All axis positions: {positions}")
            else:
                print(f"Error reading positions: {positions}")

            # Check current faults
            faults = robot.get_current_faults()
            if isinstance(faults, list):
                if faults[0] == 0:
                    print("No current faults")
                else:
                    print(f"Current faults: {faults[0]:016b}")
            else:
                print(f"Error reading faults: {faults}")

            # Monitor individual axes status
            print("\nIndividual axes status:")
            for axis in range(1, 7):
                try:
                    position = robot.get_axis_position(axis)
                    speed = robot.get_axis_speed(axis)
                    current = robot.get_axis_current(axis)

                    pos_val = position[0] if isinstance(position, list) else "Error"
                    speed_val = speed[0] if isinstance(speed, list) else "Error"
                    current_val = current[0] if isinstance(current, list) else "Error"

                    print(
                        f"  Axis {axis}: Position={pos_val}, Speed={speed_val}, Current={current_val}"
                    )
                except Exception as e:
                    print(f"  Axis {axis}: Error reading status - {e}")

    except Exception as e:
        print(f"Error during status monitoring: {e}")


def complete_workflow_example():
    """Example of a complete robot workflow"""
    print("\n=== Complete Workflow Example ===")

    try:
        with DH5ModbusAPI(port=PORT) as robot:
            print("Starting complete robot workflow...")

            # Step 1: Reset any existing faults
            print("1. Resetting faults...")
            result = robot.reset_faults()
            if result == robot.SUCCESS:
                print("   Faults reset successfully")

            # Step 2: Initialize specific axis
            print("2. Initializing axis 1...")
            result = robot.initialize_axis(1, DH5Registers.INIT_MODE_CLOSE)
            if result == robot.SUCCESS:
                print("   Axis 1 initialization started")

            # Step 3: Wait for initialization and monitor
            print("3. Monitoring initialization...")
            max_wait = 30  # Maximum wait time in seconds
            wait_time = 0
            while wait_time < max_wait:
                status = robot.check_initialization()
                if isinstance(status, dict):
                    if status.get("axis_F1") == "initialized":
                        print("   Axis 1 initialization completed")
                        break
                    elif status.get("axis_F1") == "initializing":
                        print("   Still initializing...")
                    else:
                        print("   Axis 1 not initialized")
                        break
                time.sleep(1)
                wait_time += 1

            # Step 4: Set axis parameters
            print("4. Setting axis parameters...")
            robot.set_axis_position(1, 2000)
            robot.set_axis_speed(1, 1000)
            robot.set_axis_force(1, 150)
            print("   Axis parameters set")

            # Step 5: Monitor final status
            print("5. Final status check...")
            position = robot.get_axis_position(1)
            if isinstance(position, list):
                print(f"   Final axis 1 position: {position[0]}")

            print("Workflow completed successfully!")

    except Exception as e:
        print(f"Error during workflow: {e}")


def main():
    """Main example function"""
    print("=== DH5 Robot API Complete Examples ===")

    # Run different examples
    try:
        initialization_example()
        individual_axis_control_example()
        comprehensive_status_monitoring()
        complete_workflow_example()
    except KeyboardInterrupt:
        print("\nExamples interrupted by user")
    except Exception as e:
        print(f"Unexpected error in examples: {e}")


if __name__ == "__main__":
    main()
