# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2025-08-04

### Added
- Initial release of DH5 Modbus API package
- Core `DH5ModbusAPI` class with full functionality
- Support for position, force, speed, and acceleration control
- System management functions (restart, reset faults, back to zero)
- Individual and bulk axis control
- Comprehensive error handling and status codes
- CRC validation for Modbus communication
- Loguru integration for improved logging
- Complete test suite with pytest
- Example scripts for basic and advanced usage
- Full package structure with pyproject.toml
- Documentation and README

### Dependencies
- loguru >= 0.7.0 (for modern logging)
- pyserial >= 3.5 (for serial communication)
- numpy >= 1.21.0 (for numerical operations)

### Features
- **Connection Management**: Open/close serial connections with configurable parameters
- **System Control**: Restart system, reset faults, various back-to-zero modes
- **Position Control**: Set individual or all axis positions, get current positions
- **Force Control**: Set individual or all axis forces (open/closed loop modes)
- **Motion Control**: Configure speed and acceleration for smooth motion
- **Error Handling**: Comprehensive error codes and logging
- **Modbus Protocol**: Full implementation of custom Modbus commands (0x69, 0x6A, 0x06)

### Development Tools
- Black code formatting
- isort import sorting  
- flake8 linting
- mypy type checking
- pytest testing framework
- Development dependencies management
