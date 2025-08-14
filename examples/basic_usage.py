"""
Basic usage example for DH5ModbusAPI with loop workflow.

This example demonstrates basic operations with the DH5 device:
- Opening/closing connections
- System management (restart, reset faults)
- Position control
- Basic motion operations
- Loop workflow with keyboard interrupt support
- Configurable loop cycle timing
"""

import time
import signal
import sys
from dh5_api import DH5ModbusAPI


class DH5LoopController:
    """Controller for DH5 device with loop workflow functionality."""

    def __init__(self, port="COM3", baud_rate=115200, loop_cycle=10.0):
        """
        Initialize the DH5 loop controller.

        Args:
            port: COM port for the device
            baud_rate: Baud rate for communication
            loop_cycle: Time in seconds for each loop cycle
        """
        self.dh5 = DH5ModbusAPI(port=port, baud_rate=baud_rate)
        self.loop_cycle = loop_cycle
        self.running = True
        self.loop_count = 0
        self.max_positions = None  # Will store maximum positions for each axis
        self.num_axes = 6  # Number of axes

        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle keyboard interrupt (Ctrl+C) for graceful shutdown."""
        print(
            f"\nKeyboard interrupt received. Stopping after {self.loop_count} loops..."
        )
        self.running = False

    def initialize_device(self):
        """Initialize the device connection and setup."""
        print("Initializing DH5 device...")

        # Open connection
        print("Opening connection...")
        result = self.dh5.open_connection()
        if result != DH5ModbusAPI.SUCCESS:
            print(f"Failed to open connection: {result}")
            return False
        print("Connection opened successfully")

        # Restart system
        print("Restarting system...")
        self.dh5.restart_system()
        time.sleep(2)

        # Reset faults
        print("Resetting faults...")
        self.dh5.reset_faults()
        time.sleep(2)

        # Initialize all axes
        print("Initializing all axes...")
        result = self.dh5.calibrate_max_positions()

        print("Device initialization completed successfully!")
        return True

    def run_loop_cycle(self):
        """Execute one cycle of the loop workflow."""
        self.loop_count += 1
        print(f"\n--- Loop Cycle {self.loop_count} ---")

        # Get current positions
        print("Getting current positions...")
        try:
            positions = self.dh5.get_all_positions()
            print(f"Current positions: {positions}")
        except Exception as e:
            print(f"Failed to get positions: {e}")
            return False

        # Open pose
        print("Moving to open pose...")
        open_pose_scalings = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        open_pose_positions = [
            int(max_pos * scaling)
            for max_pos, scaling in zip(self.dh5.max_positions, open_pose_scalings)
        ]

        result = self.dh5.set_all_positions_by_ratio(open_pose_scalings)
        if result == DH5ModbusAPI.SUCCESS:
            print(
                f"Positions set to scalings: {open_pose_scalings}, positions: {open_pose_positions}"
            )
        else:
            print(f"Failed to set positions: {result}")
            return False

        time.sleep(1)  # Wait for movement

        # fist pose
        print("Moving to fist pose...")
        fist_pose_scalings = [0.0] * self.num_axes
        fist_pose_scalings[0] = 1.0  # keep first axis at max position
        fist_pose_scalings[5] = 0.05  # set last axis to quarter position

        result = self.dh5.set_all_positions_by_ratio(fist_pose_scalings)
        if result == DH5ModbusAPI.SUCCESS:
            print("Moved to fist pose")
        else:
            print(f"Failed to move to fist pose: {result}")
            return False

        time.sleep(1)  # Wait for movement

        print(f"Loop cycle {self.loop_count} completed successfully!")
        return True

    def run_loop_workflow(self):
        """Run the main loop workflow until keyboard interrupt."""
        if not self.initialize_device():
            return

        print(f"\nStarting loop workflow with {self.loop_cycle}s cycle time...")
        print("Press Ctrl+C to stop the loop workflow gracefully.")

        try:
            while self.running:
                cycle_start_time = time.time()

                # Execute one loop cycle
                if not self.run_loop_cycle():
                    print("Loop cycle failed, stopping workflow...")
                    break

                # Wait for the remaining cycle time
                elapsed_time = time.time() - cycle_start_time
                remaining_time = self.loop_cycle - elapsed_time

                if remaining_time > 0 and self.running:
                    print(f"Waiting {remaining_time:.1f}s until next cycle...")
                    time.sleep(remaining_time)

        except KeyboardInterrupt:
            print("\nKeyboard interrupt in main loop...")

        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources and close connections."""
        print(f"\nLoop workflow stopped after {self.loop_count} cycles.")
        print("Closing connection...")
        self.dh5.close_connection()
        print("Connection closed. Goodbye!")


def main():
    """Main example function."""
    # Configuration
    COM_PORT = "COM4"
    BAUD_RATE = 115200
    LOOP_CYCLE_TIME = 3.0  # seconds - you can modify this value

    print("DH5 Modbus API - Loop Workflow Example")
    print("=" * 40)
    print(f"COM Port: {COM_PORT}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"Loop Cycle Time: {LOOP_CYCLE_TIME}s")
    print("=" * 40)

    try:
        # Create and run the loop controller
        controller = DH5LoopController(
            port=COM_PORT, baud_rate=BAUD_RATE, loop_cycle=LOOP_CYCLE_TIME
        )

        # Run the workflow
        controller.run_loop_workflow()

        # Show final position limits after initialization
        if controller.max_positions:
            print(f"\nPosition limits determined: {controller.max_positions}")
            print(
                "All target positions will be automatically clamped to stay within (0, max) range."
            )

    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
