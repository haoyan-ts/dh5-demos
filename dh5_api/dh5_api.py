import time
from enum import IntEnum
from typing import Any, List, Optional, Tuple, Union

from loguru import logger
from pymodbus.client.serial import ModbusSerialClient
from pymodbus.exceptions import ModbusException, ConnectionException
from pymodbus.pdu import ExceptionResponse, ModbusPDU


class ModbusFunction(IntEnum):
    """Modbus function codes"""

    READ_HOLDING_REGISTERS = 0x03
    WRITE_SINGLE_REGISTER = 0x06
    WRITE_MULTIPLE_REGISTERS = 0x10


class DH5Registers:
    """DH5 robot register addresses"""

    # Initialization registers
    RETURN_TO_ZERO = 0x0100
    RETURN_TO_ZERO_STATUS = 0x0200

    # Configuration registers
    SAVE_PARAM = 0x0300
    UART_CONFIG = 0x0302

    # Individual axis position registers (0x0101-0x0106)
    AXIS_POSITION_BASE = 0x0101

    # Individual axis force registers (0x0107-0x010C)
    AXIS_FORCE_BASE = 0x0107

    # Individual axis speed registers (0x010D-0x0112)
    AXIS_SPEED_BASE = 0x010D

    # Individual axis acceleration/deceleration registers (0x0113-0x0118)
    AXIS_ACCELERATION_BASE = 0x0113

    # Status registers - operating status (0x0201-0x0206)
    AXIS_OPERATION_STATUS_BASE = 0x0201

    # Status registers - current position readings (0x0207-0x020C)
    AXIS_CURRENT_POSITION_BASE = 0x0207

    # Status registers - current speed readings (0x020D-0x0212)
    AXIS_SPEED_STATUS_BASE = 0x020D

    # Status registers - current readings (0x0213-0x0218)
    AXIS_CURRENT_BASE = 0x0213

    # Command registers
    RESET_FAULTS = 0x0501
    RESTART_SYSTEM = 0x0503
    AGING_TEST = 0x0504

    # Data registers
    HISTORY_FAULTS = 0x0B00

    # Constants
    HISTORY_FAULTS_LENGTH = 0x3F
    AXIS_COUNT = 6

    # Initialization modes
    INIT_MODE_CLOSE = 0b01
    INIT_MODE_OPEN = 0b10
    INIT_MODE_FIND_STROKE = 0b11

    # Intialization status
    INIT_STATUS_NOT_INITIALIZED = 0b00
    INIT_STATUS_INITIALIZING = 0b01
    INIT_STATUS_INITIALIZED = 0b10


class DH5ModbusAPI:
    """Modbus API for DH5 Robot Controller

    Provides high-level interface for communicating with DH5 robot controller
    via Modbus RTU protocol.
    """

    SUCCESS = 0
    ERROR_CONNECTION_FAILED = 1
    ERROR_INVALID_RESPONSE = 2
    ERROR_CRC_CHECK_FAILED = 3
    ERROR_INVALID_COMMAND = 4

    max_positions = [500] * DH5Registers.AXIS_COUNT

    def __init__(
        self,
        port: str = "COM6",
        modbus_id: int = 1,
        baud_rate: int = 115200,
        stop_bits: int = 1,
        parity: str = "N",
        timeout: float = 1.0,
    ):
        """Initialize DH5 Modbus API

        Args:
            port: Serial port name (e.g., 'COM6', '/dev/ttyUSB0')
            modbus_id: Modbus device ID
            baud_rate: Serial communication baud rate
            stop_bits: Number of stop bits
            parity: Parity setting ('N', 'E', 'O')
            timeout: Communication timeout in seconds
        """
        self.port = port
        self.modbus_id = modbus_id
        self.baud_rate = baud_rate
        self.stop_bits = stop_bits
        self.parity = parity
        self.timeout = timeout
        self.client: Optional[ModbusSerialClient] = None

    def open_connection(self) -> int:
        """Open Modbus RTU connection

        Returns:
            SUCCESS if connection opened successfully, error code otherwise
        """
        try:
            self.client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baud_rate,
                stopbits=self.stop_bits,
                parity=self.parity,
                timeout=self.timeout,
            )
            if self.client.connect():
                logger.info(f"Modbus RTU connection opened successfully on {self.port}")
                return self.SUCCESS
            else:
                logger.error(f"Failed to open serial connection on {self.port}")
                return self.ERROR_CONNECTION_FAILED
        except Exception as e:
            logger.error(f"Failed to open serial connection on {self.port}: {str(e)}")
            return self.ERROR_CONNECTION_FAILED

    def close_connection(self) -> int:
        """Close Modbus RTU connection

        Returns:
            SUCCESS if connection closed successfully
        """
        if self.client:
            self.client.close()
            logger.info("Serial connection closed")
            self.client = None
        return self.SUCCESS

    @property
    def is_connected(self) -> bool:
        """Check if Modbus connection is active"""
        return self.client is not None and self.client.connected

    def _validate_connection(self) -> bool:
        """Validate that connection is active

        Returns:
            True if connection is valid, False otherwise
        """
        if not self.is_connected:
            logger.error("No active Modbus connection")
            return False
        return True

    def send_modbus_command(
        self,
        function_code: int,
        register_address: int,
        data: Optional[Union[int, List[int]]] = None,
        data_length: Optional[int] = None,
    ) -> Union[int, List[int]]:
        """Send Modbus command to device

        Args:
            function_code: Modbus function code
            register_address: Register address to read/write
            data: Data to write (for write operations)
            data_length: Number of registers to read (for read operations)

        Returns:
            Register values for read operations, SUCCESS for write operations,
            or error code on failure
        """
        if not self._validate_connection():
            return self.ERROR_CONNECTION_FAILED

        try:
            response = self._execute_modbus_function(
                function_code, register_address, data, data_length
            )

            return self._parse_response(response, function_code)

        except (ModbusException, ValueError, ConnectionError) as e:
            logger.error(f"Modbus operation failed: {str(e)}")
            return self.ERROR_INVALID_RESPONSE
        except Exception as e:
            logger.error(f"Unexpected error: {str(e)}")
            return self.ERROR_INVALID_RESPONSE

    def _clear_recv_buffer(self):
        """Clear the receive buffer of the Modbus client"""
        if self.client:
            self.client.recv(self.client._in_waiting())

    def _execute_modbus_function(
        self,
        function_code: int,
        register_address: int,
        data: Optional[Union[int, List[int]]],
        data_length: Optional[int],
    ) -> ModbusPDU:
        """Execute specific Modbus function

        Args:
            function_code: Modbus function code
            register_address: Register address
            data: Data for write operations
            data_length: Length for read operations

        Returns:
            Modbus response object

        Raises:
            ValueError: For invalid function codes or missing data
        """
        if not self.client:
            raise ConnectionError("Modbus client not initialized")

        if function_code == ModbusFunction.READ_HOLDING_REGISTERS:
            self._clear_recv_buffer()
            return self.client.read_holding_registers(
                register_address, count=data_length or 1, device_id=self.modbus_id
            )
        elif function_code == ModbusFunction.WRITE_SINGLE_REGISTER:
            if data is None:
                raise ValueError("Data is required for write single register")
            if isinstance(data, list):
                raise ValueError("Single register write requires int, not list")

            self._clear_recv_buffer()

            return self.client.write_register(
                register_address,
                data,
                device_id=self.modbus_id,
                no_response_expected=True,
            )
        elif function_code == ModbusFunction.WRITE_MULTIPLE_REGISTERS:
            if data is None:
                raise ValueError("Data is required for write multiple registers")
            if isinstance(data, int):
                data = [data]  # Convert single int to list

            self._clear_recv_buffer()

            return self.client.write_registers(
                register_address,
                data,
                device_id=self.modbus_id,
                no_response_expected=True,
            )
        else:
            raise ValueError(f"Unsupported function code: {function_code}")

    def _parse_response(
        self, response: ModbusPDU, function_code: int
    ) -> Union[int, List[int]]:
        """Parse Modbus response

        Args:
            response: Raw Modbus response
            function_code: Function code used for the request

        Returns:
            Parsed response data or error code
        """
        # Check if response is an exception
        if isinstance(response, ExceptionResponse) and response.function_code != 0xFF:
            logger.error(f"Modbus exception response: {response}")
            return self.ERROR_INVALID_RESPONSE

        # Check if response is valid
        # if response.isError():
        #     logger.error(f"Modbus error response: {response}")
        #     return self.ERROR_INVALID_RESPONSE

        if function_code == ModbusFunction.READ_HOLDING_REGISTERS:
            return response.registers
        elif function_code in [
            ModbusFunction.WRITE_SINGLE_REGISTER,
            ModbusFunction.WRITE_MULTIPLE_REGISTERS,
        ]:
            return self.SUCCESS

        return self.SUCCESS

    def calibrate_max_positions(self) -> int:
        if not self.client:
            raise ConnectionError("Modbus client not initialized")

        # Initialize to open pose
        self.initialize(2)
        time.sleep(2)

        while True:
            results = self.check_initialization()

            if isinstance(results, dict) and len(results) == DH5Registers.AXIS_COUNT:
                if all(status == "initialized" for status in results.values()):
                    break

            logger.debug(f"Initialization status: {results}")
            time.sleep(1)

        # get current positions
        results = self.get_all_positions()

        if isinstance(results, list) and len(results) == DH5Registers.AXIS_COUNT:
            self.max_positions = results
            logger.info(f"Max positions initialized: {self.max_positions}")

            return DH5ModbusAPI.SUCCESS
        else:
            logger.error("Failed to get valid position readings")
            return self.ERROR_INVALID_RESPONSE

    def _validate_and_clamp_positions(self, positions):
        if not self.max_positions:
            logger.warning(
                "Warning: Maximum positions not initialized, using positions as-is"
            )
            return positions

        if len(positions) != len(self.max_positions):
            raise ValueError(
                f"Position list length {len(positions)} does not match number of axes {len(self.max_positions)}"
            )

        clamped_positions = []
        for i, (pos, max_pos) in enumerate(zip(positions, self.max_positions)):
            # Ensure position is within (0, max_pos) range
            clamped_pos = max(
                1, min(pos, max_pos - 10)
            )  # Keep at least 10 units away from limits
            if clamped_pos != pos:
                logger.trace(
                    f"Axis {i+1} position {pos} clamped to {clamped_pos} (max: {max_pos})"
                )
            clamped_positions.append(clamped_pos)

        logger.debug(f"Clamped positions: {clamped_positions} (original: {positions})")

        return clamped_positions

    # Robot Status Methods
    def get_history_faults(self) -> Union[List[int], int]:
        """Get fault history

        Returns:
            List containing fault history registers, or error code
        """
        return self.send_modbus_command(
            function_code=ModbusFunction.READ_HOLDING_REGISTERS,
            register_address=DH5Registers.HISTORY_FAULTS,
            data_length=DH5Registers.HISTORY_FAULTS_LENGTH,
        )

    def get_all_positions(self) -> Union[List[int], int]:
        """Get current positions of all axes

        Returns:
            List of 6 position values for each axis, or error code
        """
        return self.send_modbus_command(
            function_code=ModbusFunction.READ_HOLDING_REGISTERS,
            register_address=DH5Registers.AXIS_CURRENT_POSITION_BASE,
            data_length=DH5Registers.AXIS_COUNT,
        )

    def check_initialization(self) -> Union[dict, int]:
        """Check the initialization status of all 6 axes

        Returns:
            Dictionary with axis status or error code
            Status values: "not initialized", "initialized", "initializing"
        """
        result = self.send_modbus_command(
            function_code=ModbusFunction.READ_HOLDING_REGISTERS,
            register_address=DH5Registers.RETURN_TO_ZERO_STATUS,
            data_length=1,
        )

        if isinstance(result, list) and len(result) > 0:
            init_status = result[0]
            status = {}
            for axis in range(6):
                axis_status = (init_status >> (axis * 2)) & 0b11
                if axis_status == 0b01:
                    status[f"axis_F{axis + 1}"] = "initialized"
                elif axis_status == 0b10:
                    status[f"axis_F{axis + 1}"] = "initializing"
                else:
                    status[f"axis_F{axis + 1}"] = "not initialized"
            return status
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    # Individual Axis Status Methods
    def get_axis_position(self, axis: int) -> Union[List[int], int]:
        """Get current position of specific axis

        Args:
            axis: Axis number (1-6)

        Returns:
            List containing position value, or error code
        """
        if axis < 1 or axis > 6:
            raise ValueError("Axis must be between 1 and 6")

        register_address = DH5Registers.AXIS_CURRENT_POSITION_BASE + (axis - 1)
        return self.send_modbus_command(
            function_code=ModbusFunction.READ_HOLDING_REGISTERS,
            register_address=register_address,
            data_length=1,
        )

    def get_axis_speed(self, axis: int) -> Union[List[int], int]:
        """Get current speed of specific axis

        Args:
            axis: Axis number (1-6)

        Returns:
            List containing speed value, or error code
        """
        if axis < 1 or axis > 6:
            raise ValueError("Axis must be between 1 and 6")

        register_address = DH5Registers.AXIS_SPEED_STATUS_BASE + (axis - 1)
        return self.send_modbus_command(
            function_code=ModbusFunction.READ_HOLDING_REGISTERS,
            register_address=register_address,
            data_length=1,
        )

    def get_axis_current(self, axis: int) -> Union[List[int], int]:
        """Get current of specific axis

        Args:
            axis: Axis number (1-6)

        Returns:
            List containing current value, or error code
        """
        if axis < 1 or axis > 6:
            raise ValueError("Axis must be between 1 and 6")

        register_address = DH5Registers.AXIS_CURRENT_BASE + (axis - 1)
        return self.send_modbus_command(
            function_code=ModbusFunction.READ_HOLDING_REGISTERS,
            register_address=register_address,
            data_length=1,
        )

    # Robot Control Methods
    def reset_faults(self) -> int:
        """Reset current faults

        Returns:
            SUCCESS if command sent successfully, error code otherwise
        """
        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_SINGLE_REGISTER,
            register_address=DH5Registers.RESET_FAULTS,
            data=1,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    def restart_system(self) -> int:
        """Restart the robot system

        Returns:
            SUCCESS if command sent successfully, error code otherwise
        """
        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_SINGLE_REGISTER,
            register_address=DH5Registers.RESTART_SYSTEM,
            data=1,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    def aging_test(self, flag: int) -> int:
        """
        Enable or disable aging test mode on the DH5 device.
        Args:
            flag (int): Test mode flag. Must be 0 to disable or 1 to enable aging test.
        Returns:
            int: Result of the Modbus command execution, or ERROR_INVALID_RESPONSE if the response is invalid.
        Raises:
            ValueError: If flag is not 0 or 1.
        """
        if flag not in [0, 1]:
            raise ValueError("Flag must be 0 (disable) or 1 (enable)")

        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_SINGLE_REGISTER,
            register_address=DH5Registers.AGING_TEST,
            data=flag,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    def set_all_positions(self, positions: List[int]) -> int:
        """Set target positions for all axes

        Args:
            positions: List of 6 position values for each axis

        Returns:
            SUCCESS if command sent successfully, error code otherwise

        Raises:
            ValueError: If positions list doesn't contain exactly 6 values
        """
        if len(positions) != DH5Registers.AXIS_COUNT:
            raise ValueError(
                f"Must provide exactly {DH5Registers.AXIS_COUNT} position values"
            )

        safe_positions = self._validate_and_clamp_positions(positions)

        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_MULTIPLE_REGISTERS,
            register_address=DH5Registers.AXIS_POSITION_BASE,
            data=safe_positions,
            data_length=DH5Registers.AXIS_COUNT,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    def set_all_positions_by_ratio(self, scalings: List[float]) -> int:
        """Set target positions for all axes using ratio scaling (0.0 to 1.0)

        Args:
            ratio_positions: List of 6 ratio values (0.0-1.0) for each axis

        Returns:
            SUCCESS if command sent successfully, error code otherwise

        Raises:
            ValueError: If positions list doesn't contain exactly 6 values
        """
        if len(scalings) != DH5Registers.AXIS_COUNT:
            raise ValueError(
                f"Must provide exactly {DH5Registers.AXIS_COUNT} ratio values"
            )

        # Check ratio values in ratio_positions
        if not all(0.0 <= ratio <= 1.0 for ratio in scalings):
            raise ValueError("All ratio values must be float and between 0 and 1")

        # Scale positions by ratio
        scaled_positions = [
            int(max_pos * ratio) for max_pos, ratio in zip(self.max_positions, scalings)
        ]
        logger.debug(f"Scaled positions: {scaled_positions} from ratios: {scalings}")

        safe_positions = self._validate_and_clamp_positions(scaled_positions)
        logger.debug(
            f"Safe positions after clamping: {safe_positions} (original: {scaled_positions})"
        )

        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_MULTIPLE_REGISTERS,
            register_address=DH5Registers.AXIS_POSITION_BASE,
            data=safe_positions,
            data_length=DH5Registers.AXIS_COUNT,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    def set_all_forces(self, forces: list[float]) -> int:
        if len(forces) != DH5Registers.AXIS_COUNT:
            raise ValueError(
                f"Must provide exactly {DH5Registers.AXIS_COUNT} force values"
            )

        if not all(0.2 <= force <= 1.0 for force in forces):
            raise ValueError("All force values must be float and between 0.2 and 1.0")

        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_MULTIPLE_REGISTERS,
            register_address=DH5Registers.AXIS_FORCE_BASE,
            data=[int(force * 100) for force in forces],
            data_length=DH5Registers.AXIS_COUNT,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    def set_all_speeds(self, speeds: list[float]) -> int:
        """Set target speeds for all axes

        Args:
            speeds: List of 6 speed values for each axis

        Returns:
            SUCCESS if command sent successfully, error code otherwise

        Raises:
            ValueError: If speeds list doesn't contain exactly 6 values
        """
        if len(speeds) != DH5Registers.AXIS_COUNT:
            raise ValueError(
                f"Must provide exactly {DH5Registers.AXIS_COUNT} speed values"
            )

        if not all(0.1 <= speed <= 1.0 for speed in speeds):
            raise ValueError("All speed values must be float and between 0.1 and 1.0")

        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_MULTIPLE_REGISTERS,
            register_address=DH5Registers.AXIS_SPEED_BASE,
            data=[int(speed * 100) for speed in speeds],
            data_length=DH5Registers.AXIS_COUNT,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    # Configuration Methods
    def set_uart_config(
        self,
        modbus_id: Optional[int] = None,
        baud_rate: Optional[int] = None,
        stop_bits: Optional[int] = None,
        parity: Optional[int] = None,
    ) -> int:
        """Set UART configuration parameters

        Args:
            modbus_id: Modbus device ID
            baud_rate: Serial baud rate
            stop_bits: Number of stop bits
            parity: Parity setting

        Returns:
            SUCCESS if command sent successfully, error code otherwise
        """
        uart_registers = [
            modbus_id or self.modbus_id,
            baud_rate or self.baud_rate,
            stop_bits or self.stop_bits,
            parity or (0 if self.parity == "N" else 1 if self.parity == "E" else 2),
        ]

        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_MULTIPLE_REGISTERS,
            register_address=DH5Registers.UART_CONFIG,
            data=uart_registers,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    def set_save_param(self, flag: int = 1) -> int:
        """Save parameters to non-volatile memory

        Args:
            flag: Save flag (default: 1)

        Returns:
            SUCCESS if command sent successfully, error code otherwise
        """
        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_SINGLE_REGISTER,
            register_address=DH5Registers.SAVE_PARAM,
            data=flag,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    # Initialization Methods
    def initialize(self, mode: int) -> int:
        """Initialize all 6 axes with a specific mode

        Args:
            mode: Initialization mode
                - 0b01 (1): Close
                - 0b10 (2): Open
                - 0b11 (3): Find total stroke

        Returns:
            SUCCESS if command sent successfully, error code otherwise
        """
        if mode not in [
            DH5Registers.INIT_MODE_CLOSE,
            DH5Registers.INIT_MODE_OPEN,
            DH5Registers.INIT_MODE_FIND_STROKE,
        ]:
            raise ValueError(
                "Mode must be 0b01 (close), 0b10 (open), or 0b11 (find stroke)"
            )

        data = 0
        for axis in range(6):
            data |= mode << (axis * 2)  # Set all axes to the given mode

        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_SINGLE_REGISTER,
            register_address=DH5Registers.RETURN_TO_ZERO,
            data=data,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    def initialize_axis(self, axis: int, mode: int) -> int:
        """Initialize a specific axis with a specific mode

        Args:
            axis: Axis number (1-6)
            mode: Initialization mode
                - 0b01 (1): Close
                - 0b10 (2): Open
                - 0b11 (3): Find total stroke

        Returns:
            SUCCESS if command sent successfully, error code otherwise
        """
        if axis < 1 or axis > 6:
            raise ValueError("Axis must be between 1 and 6")
        if mode not in [
            DH5Registers.INIT_MODE_CLOSE,
            DH5Registers.INIT_MODE_OPEN,
            DH5Registers.INIT_MODE_FIND_STROKE,
        ]:
            raise ValueError(
                "Mode must be 0b01 (close), 0b10 (open), or 0b11 (find stroke)"
            )

        init_status = 0
        # Set 2 bits for the specified axis
        init_status &= ~(0b11 << ((axis - 1) * 2))  # Clear the bits for the axis
        init_status |= mode << ((axis - 1) * 2)  # Set the mode bits for the axis

        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_SINGLE_REGISTER,
            register_address=DH5Registers.RETURN_TO_ZERO,
            data=init_status,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    # Individual Axis Control Methods
    def set_axis_position(self, axis: int, position: int) -> int:
        """Set target position for specific axis

        Args:
            axis: Axis number (1-6)
            position: Target position value

        Returns:
            SUCCESS if command sent successfully, error code otherwise
        """
        if axis < 1 or axis > 6:
            raise ValueError("Axis must be between 1 and 6")

        register_address = DH5Registers.AXIS_POSITION_BASE + (axis - 1)
        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_SINGLE_REGISTER,
            register_address=register_address,
            data=position,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    def set_axis_speed(self, axis: int, speed: int) -> int:
        """Set speed for specific axis

        Args:
            axis: Axis number (1-6)
            speed: Speed value

        Returns:
            SUCCESS if command sent successfully, error code otherwise
        """
        if axis < 1 or axis > 6:
            raise ValueError("Axis must be between 1 and 6")

        register_address = DH5Registers.AXIS_SPEED_BASE + (axis - 1)
        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_SINGLE_REGISTER,
            register_address=register_address,
            data=speed,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    def set_axis_force(self, axis: int, force: int) -> int:
        """Set force for specific axis

        Args:
            axis: Axis number (1-6)
            force: Force value

        Returns:
            SUCCESS if command sent successfully, error code otherwise
        """
        if axis < 1 or axis > 6:
            raise ValueError("Axis must be between 1 and 6")

        register_address = DH5Registers.AXIS_FORCE_BASE + (axis - 1) * 0x10
        result = self.send_modbus_command(
            function_code=ModbusFunction.WRITE_SINGLE_REGISTER,
            register_address=register_address,
            data=force,
        )
        return result if isinstance(result, int) else self.ERROR_INVALID_RESPONSE

    # Context Manager Support
    def __enter__(self):
        """Context manager entry"""
        result = self.open_connection()
        if result != self.SUCCESS:
            raise ConnectionError(f"Failed to open connection: {result}")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close_connection()
