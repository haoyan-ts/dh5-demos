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
        for axis in range(1, self.num_axes + 1):
            print(f"Initializing axis {axis}...")
            try:
                # Initialize each axis individually with open mode
                result = self.dh5.initialize_axis(axis, 2)  # 2 = open mode
                if result == DH5ModbusAPI.SUCCESS:
                    print(f"Axis {axis} initialized successfully")
                else:
                    print(f"Failed to initialize axis {axis}: {result}")
                time.sleep(1)  # Small delay between axis initializations
            except Exception as e:
                print(f"Error initializing axis {axis}: {e}")
                return False

        # Wait for all axes to complete initialization
        print("Waiting for all axes initialization to complete...")
        time.sleep(5)

        # Back to initial position
        print("Moving to initial position...")
        for axis in range(1, self.num_axes + 1):
            self.dh5.set_axis_position(axis, 1)  # Move each axis to position 1
        time.sleep(5)  # Wait for movement to complete

        # Read and store maximum positions for each axis
        print("Reading initialization positions (approximating maximum positions)...")
        try:
            positions = self.dh5.get_all_positions()
            if isinstance(positions, list) and len(positions) == self.num_axes:
                # Use the initialization positions as maximum values with some safety margin
                self.max_positions = [max(1000, int(pos * 0.95)) for pos in positions]
                print(f"Initialization positions: {positions}")
                print(f"Using maximum positions (95% of init): {self.max_positions}")
            else:
                raise Exception("Failed to get valid position readings")

        except Exception as e:
            print(f"Error reading initialization positions: {e}")
            # Use default safe maximum values if reading fails
            self.max_positions = [
                700,
                1500,
                1500,
                1500,
                1500,
                700,
            ]  # Conservative defaults
            print(f"Using default maximum positions: {self.max_positions}")

        print("Device initialization completed successfully!")
        return True

    def validate_and_clamp_positions(self, positions):
        """Validate and clamp positions to ensure they are within (0, max) range.

        Args:
            positions: List of target positions for all axes

        Returns:
            List of clamped positions within valid range
        """
        if not self.max_positions:
            print("Warning: Maximum positions not initialized, using positions as-is")
            return positions

        if len(positions) != len(self.max_positions):
            raise ValueError(
                f"Position list length {len(positions)} does not match number of axes {len(self.max_positions)}"
            )

        clamped_positions = []
        for i, (pos, max_pos) in enumerate(zip(positions, self.max_positions)):
            # Ensure position is within (0, max_pos) range
            clamped_pos = max(
                1, min(pos, max_pos - 1)
            )  # Keep at least 1 unit away from limits
            if clamped_pos != pos:
                print(
                    f"Warning: Axis {i+1} position {pos} clamped to {clamped_pos} (max: {max_pos})"
                )
            clamped_positions.append(clamped_pos)

        return clamped_positions

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

        # Set all positions to specific values
        print("Setting new positions...")
        target_positions = [874, 1600, 1700, 1700, 1700, 900]

        # Validate and clamp positions to ensure they are within safe limits
        safe_positions = self.validate_and_clamp_positions(target_positions)

        result = self.dh5.set_all_positions(safe_positions)
        if result == DH5ModbusAPI.SUCCESS:
            print(f"Positions set to: {safe_positions}")
        else:
            print(f"Failed to set positions: {result}")
            return False

        time.sleep(5)  # Wait for movement

        # Move back to zero positions
        print("Moving back to zero positions...")
        zero_positions = [1, 1, 1, 1, 1, 1]

        # Validate and clamp positions (though these should already be safe)
        safe_zero_positions = self.validate_and_clamp_positions(zero_positions)

        result = self.dh5.set_all_positions(safe_zero_positions)
        if result == DH5ModbusAPI.SUCCESS:
            print("Moved back to zero positions")
        else:
            print(f"Failed to move to zero positions: {result}")
            return False

        time.sleep(3)  # Wait for movement

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
    LOOP_CYCLE_TIME = 15.0  # seconds - you can modify this value

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

        # Initialize the Serial port
        controller.dh5.open_connection()

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
