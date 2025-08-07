# DH5 Demos

A Python package for DH5 Modbus API demos and utilities.

## Features

- DH5 Modbus communication API
- Serial communication utilities
- Position, force, and speed control
- System management functions

## Installation

### Development Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/dh5-demos.git
cd dh5-demos

# Install in development mode
pip install -e .

# Install with development dependencies
pip install -e ".[dev]"
```

### Regular Installation

```bash
pip install dh5-demos
```

## Dependencies

- **loguru**: Modern logging library for Python
- **pyserial**: Serial communication library
- **numpy**: Numerical computing library

## Usage

```python
from dh5_api import DH5ModbusAPI
import time

# Initialize the API
dh5 = DH5ModbusAPI(port="COM3", baud_rate=115200)

# Open connection
dh5.open_connection()

# Restart system and reset faults
dh5.restart_system()
time.sleep(2)
dh5.reset_faults()
time.sleep(2)

# Back to initial position
dh5.back_to_initial_position()
time.sleep(8)

# Set all positions
dh5.set_all_positions([1700, 1700, 1700, 1700, 1700, 900])

# Close connection
dh5.close_connection()
```

## API Reference

### DH5ModbusAPI Class

#### Initialization
- `__init__(port="COM6", modbus_id=1, baud_rate=115200, stop_bits=1, parity="N")`

#### Connection Management
- `open_connection()`: Open serial connection
- `close_connection()`: Close serial connection

#### System Control
- `restart_system()`: Restart the DH5 system
- `reset_faults()`: Reset system faults
- `back_to_zero(mode)`: Back to zero with specified mode
- `back_to_initial_position()`: Return to initial position

#### Position Control
- `set_position(axis, position)`: Set position for single axis
- `set_all_positions(positions)`: Set positions for all axes
- `get_all_positions()`: Get current positions of all axes

#### Force Control
- `set_force(axis, force)`: Set force for single axis
- `set_all_forces(forces)`: Set forces for all axes

#### Speed and Acceleration Control
- `set_speed(axis, speed)`: Set speed for single axis
- `set_all_speeds(speeds)`: Set speeds for all axes
- `set_acc(axis, acc)`: Set acceleration for single axis
- `set_all_acc(acc)`: Set accelerations for all axes

## Development

### Running Tests

```bash
pytest
```

### Code Formatting

```bash
black .
isort .
```

### Type Checking

```bash
mypy dh5_api/
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Run the test suite
6. Submit a pull request

## Support

For issues and questions, please open an issue on the GitHub repository.
