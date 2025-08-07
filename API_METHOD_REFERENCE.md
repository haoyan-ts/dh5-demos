# DH5ModbusAPI Method Reference

This document provides a comprehensive reference for all methods available in the refactored DH5ModbusAPI.

## Connection Management

### `__init__(port="COM6", modbus_id=1, baud_rate=115200, stop_bits=1, parity="N", timeout=1.0)`
Initialize the DH5ModbusAPI instance.

**Parameters:**
- `port`: Serial port name (e.g., 'COM6', '/dev/ttyUSB0')
- `modbus_id`: Modbus device ID
- `baud_rate`: Serial communication baud rate
- `stop_bits`: Number of stop bits
- `parity`: Parity setting ('N', 'E', 'O')
- `timeout`: Communication timeout in seconds

### `open_connection() -> int`
Open Modbus RTU connection.

**Returns:** SUCCESS (0) or error code

### `close_connection() -> int`
Close Modbus RTU connection.

**Returns:** SUCCESS (0) or error code

### `is_connected` (property) `-> bool`
Check if Modbus connection is active.

**Returns:** True if connected, False otherwise

## Context Manager Support

### `__enter__()` and `__exit__()`
Context manager support for automatic connection management.

**Usage:**
```python
with DH5ModbusAPI(port="COM6") as robot:
    # Use robot methods here
    pass
```

## Status Methods

### `get_current_faults() -> Union[List[int], int]`
Get current fault status.

**Returns:** List containing fault register value, or error code

### `get_history_faults() -> Union[List[int], int]`
Get fault history.

**Returns:** List containing fault history registers, or error code

### `is_busy` (property) `-> bool`
Check if robot is currently executing a command.

**Returns:** True if robot is busy, False otherwise

### `get_all_positions() -> Union[List[int], int]`
Get current positions of all axes.

**Returns:** List of 6 position values for each axis, or error code

### `check_initialization() -> Union[dict, int]`
Check the initialization status of all 6 axes.

**Returns:** Dictionary with axis status or error code
- Status values: "not initialized", "initialized", "initializing"
- Dictionary keys: "axis_F1" through "axis_F6"

## Individual Axis Status Methods

### `get_axis_position(axis: int) -> Union[List[int], int]`
Get current position of specific axis.

**Parameters:**
- `axis`: Axis number (1-6)

**Returns:** List containing position value, or error code

### `get_axis_speed(axis: int) -> Union[List[int], int]`
Get current speed of specific axis.

**Parameters:**
- `axis`: Axis number (1-6)

**Returns:** List containing speed value, or error code

### `get_axis_current(axis: int) -> Union[List[int], int]`
Get current of specific axis.

**Parameters:**
- `axis`: Axis number (1-6)

**Returns:** List containing current value, or error code

## Robot Control Methods

### `reset_faults() -> int`
Reset current faults.

**Returns:** SUCCESS or error code

### `restart_system() -> int`
Restart the robot system.

**Returns:** SUCCESS or error code

### `back_to_initial_position() -> int`
Move robot back to initial position.

**Returns:** SUCCESS or error code

### `back_to_zero(axis_mask: int = 0b111111) -> int`
Move specified axes back to zero position.

**Parameters:**
- `axis_mask`: Bitmask indicating which axes to move (default: all 6 axes)
  - Bit 0 = axis 1, bit 1 = axis 2, etc.

**Returns:** SUCCESS or error code

### `set_all_positions(positions: List[int]) -> int`
Set target positions for all axes.

**Parameters:**
- `positions`: List of 6 position values for each axis

**Returns:** SUCCESS or error code

**Raises:** `ValueError` if positions list doesn't contain exactly 6 values

## Configuration Methods

### `set_uart_config(modbus_id=None, baud_rate=None, stop_bits=None, parity=None) -> int`
Set UART configuration parameters.

**Parameters:**
- `modbus_id`: Modbus device ID (optional, uses current if None)
- `baud_rate`: Serial baud rate (optional, uses current if None)
- `stop_bits`: Number of stop bits (optional, uses current if None)
- `parity`: Parity setting (optional, uses current if None)

**Returns:** SUCCESS or error code

### `set_save_param(flag: int = 1) -> int`
Save parameters to non-volatile memory.

**Parameters:**
- `flag`: Save flag (default: 1)

**Returns:** SUCCESS or error code

## Initialization Methods

### `initialize(mode: int) -> int`
Initialize all 6 axes with a specific mode.

**Parameters:**
- `mode`: Initialization mode
  - `DH5Registers.INIT_MODE_CLOSE` (0b01): Close
  - `DH5Registers.INIT_MODE_OPEN` (0b10): Open
  - `DH5Registers.INIT_MODE_FIND_STROKE` (0b11): Find total stroke

**Returns:** SUCCESS or error code

### `initialize_axis(axis: int, mode: int) -> int`
Initialize a specific axis with a specific mode.

**Parameters:**
- `axis`: Axis number (1-6)
- `mode`: Initialization mode (same as above)

**Returns:** SUCCESS or error code

## Individual Axis Control Methods

### `set_axis_position(axis: int, position: int) -> int`
Set target position for specific axis.

**Parameters:**
- `axis`: Axis number (1-6)
- `position`: Target position value

**Returns:** SUCCESS or error code

### `set_axis_speed(axis: int, speed: int) -> int`
Set speed for specific axis.

**Parameters:**
- `axis`: Axis number (1-6)
- `speed`: Speed value

**Returns:** SUCCESS or error code

### `set_axis_force(axis: int, force: int) -> int`
Set force for specific axis.

**Parameters:**
- `axis`: Axis number (1-6)
- `force`: Force value

**Returns:** SUCCESS or error code

## Constants and Enums

### ModbusFunction Enum
- `READ_HOLDING_REGISTERS = 0x03`
- `WRITE_SINGLE_REGISTER = 0x06`
- `WRITE_MULTIPLE_REGISTERS = 0x10`

### DH5Registers Class Constants

#### Initialization
- `INITIALIZE_ALL = 0x0001`
- `INITIALIZE_AXIS_BASE = 0x0100`
- `INITIALIZATION_STATUS = 0x0200`

#### Configuration
- `SAVE_PARAM = 0x0300`
- `UART_CONFIG = 0x0302`

#### Individual Axis Registers
- `AXIS_POSITION_BASE = 0x0101` (positions: 0x0101-0x0106)
- `AXIS_SPEED_BASE = 0x010D` (speeds: 0x010D-0x0112)
- `AXIS_FORCE_BASE = 0x0107` (forces: 0x0107, 0x0117, 0x0127...)
- `AXIS_POSITION_STATUS_BASE = 0x0207` (current positions: 0x0207-0x020C)
- `AXIS_SPEED_STATUS_BASE = 0x020D` (current speeds: 0x020D-0x0212)
- `AXIS_CURRENT_BASE = 0x0213` (currents: 0x0213-0x0218)

#### System Status
- `CURRENT_FAULTS = 0x021F`
- `IS_BUSY = 0x0220`
- `CURRENT_POSITIONS = 0x0230`

#### Commands
- `RESET_FAULTS = 0x0501`
- `BACK_TO_INITIAL = 0x0502`
- `RESTART_SYSTEM = 0x0503`
- `BACK_TO_ZERO = 0x0504`

#### Data
- `TARGET_POSITIONS = 0x0300`
- `HISTORY_FAULTS = 0x0B00`

#### System Constants
- `HISTORY_FAULTS_LENGTH = 0x3F`
- `AXIS_COUNT = 6`

#### Initialization Modes
- `INIT_MODE_CLOSE = 0b01`
- `INIT_MODE_OPEN = 0b10`
- `INIT_MODE_FIND_STROKE = 0b11`

## Error Codes

- `SUCCESS = 0` - Operation completed successfully
- `ERROR_CONNECTION_FAILED = 1` - Connection failed
- `ERROR_INVALID_RESPONSE = 2` - Invalid response received
- `ERROR_CRC_CHECK_FAILED = 3` - CRC check failed
- `ERROR_INVALID_COMMAND = 4` - Invalid command

## Usage Examples

### Basic Connection and Status Check
```python
from dh5_api import DH5ModbusAPI

with DH5ModbusAPI(port="COM6") as robot:
    if robot.is_busy:
        print("Robot is busy")
    
    faults = robot.get_current_faults()
    if isinstance(faults, list) and faults[0] == 0:
        print("No faults")
```

### Complete Initialization Workflow
```python
from dh5_api import DH5ModbusAPI, DH5Registers

with DH5ModbusAPI(port="COM6") as robot:
    # Reset any faults
    robot.reset_faults()
    
    # Initialize all axes
    robot.initialize(DH5Registers.INIT_MODE_FIND_STROKE)
    
    # Wait for completion
    while True:
        status = robot.check_initialization()
        if all(s == "initialized" for s in status.values()):
            break
        time.sleep(1)
```

### Individual Axis Control
```python
with DH5ModbusAPI(port="COM6") as robot:
    # Configure axis 1
    robot.set_axis_position(1, 2000)
    robot.set_axis_speed(1, 1000)
    robot.set_axis_force(1, 150)
    
    # Read status
    position = robot.get_axis_position(1)
    current = robot.get_axis_current(1)
```
