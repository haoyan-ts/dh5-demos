from dh5_api import DH5ModbusAPI, DH5Registers
import time


def reset_param(api: DH5ModbusAPI, param: int):
    """Reset a specific parameter on the DH5 device."""
    print(f"Resetting parameter {param}...")
    result = api.set_save_param(param)
    if result == api.SUCCESS:
        print(f"Parameter {param} reset successfully.")
    else:
        print(f"Failed to reset parameter {param}: {result}")


def action(api: DH5ModbusAPI):
    """Perform a specific action on the DH5 device."""
    print("Performing action...")

    interval = 0.02
    count = 0
    while count < 1000:
        print(f"Setting positions for iteration {count + 1}")
        api.set_all_positions([700, 1600, 1600, 1600, 1600, 700])
        time.sleep(interval)  # Wait for position setting to complete
        api.set_all_positions([600, 1500, 1500, 1500, 1500, 600])
        time.sleep(interval)  # Wait for position setting to complete
        count += 1


def main_action():
    # api = DH5ModbusAPI(port="COM4", parity="N", timeout=2.0)
    api = DH5ModbusAPI(
        port="COM4", baud_rate=115200, parity="N", stop_bits=1, timeout=2.0
    )
    print(f"open connection")
    api.open_connection()
    # Perform your Modbus operations here

    faults = api.get_history_faults()

    print("Faults:", faults)

    api.restart_system()
    time.sleep(1)  # Wait for system restart
    api.reset_faults()
    time.sleep(1)  # Wait for faults to reset
    # api.set_uart_config(baud_rate=1, stop_bits=0, parity=0)
    # print("Setting save parameter 2...")
    api.initialize(2)
    time.sleep(2)  # Wait for initialization to complete
    print("Checking initialization status...")
    print("Initialization status:", api.check_initialization())
    time.sleep(2)  # Wait for initialization to complete

    # reset_param(api, 2)
    action(api)

    api.close_connection()


def main_fix():
    # api = DH5ModbusAPI(port="COM4", parity="N", timeout=2.0)
    api = DH5ModbusAPI(
        # port="COM4", baud_rate=115200, parity="N", stop_bits=1, timeout=2.0
        port="COM4",
        modbus_id=1,
        baud_rate=115200,
        parity="N",
        stop_bits=1,
        timeout=1.0,
    )
    print(f"open connection")
    api.open_connection()
    # Perform your Modbus operations here

    faults = api.get_history_faults()

    print("Faults:", faults)

    api.reset_faults()
    time.sleep(1)  # Wait for faults to reset
    # api.set_uart_config(baud_rate=1, stop_bits=0, parity=0)
    # print("Setting save parameter 2...")
    api.initialize(2)
    time.sleep(2)  # Wait for initialization to complete
    print("Checking initialization status...")
    print("Initialization status:", api.check_initialization())
    time.sleep(2)  # Wait for initialization to complete

    reset_param(api, 2)
    time.sleep(1)  # Wait for reset to complete

    api.initialize(3)
    time.sleep(2)  # Wait for initialization to complete
    print("Checking initialization status...")
    print("Initialization status:", api.check_initialization())
    time.sleep(2)  # Wait for initialization to complete

    api.close_connection()


def scan_devices():
    """Scan for available serial ports."""
    import serial

    ser = serial.Serial()

    baud_rates = [9600, 19200, 38400, 57600, 115200, 921600]
    parity_options = ["N", "E", "O"]
    stop_bits_options = [1, 2]

    for baud in baud_rates[4:]:
        for parity in parity_options:
            for stop_bits in stop_bits_options:
                try:
                    ser.baudrate = baud
                    ser.parity = parity
                    ser.stopbits = stop_bits
                    ser.port = "COM4"  # Change to your port
                    ser.timeout = 1.0
                    ser.open()
                    print("=" * 50)
                    print(f"Opened {ser.portstr} at {baud}, {parity}, {stop_bits}")

                    data = bytearray([0x01, 0x03, 0x03, 0x01, 0x00, 0x01, 0xD5, 0x8E])
                    ser.write(data)
                    recv = ser.read(7)
                    print(f"Received: {recv.hex(' ').upper()}")

                except Exception as e:
                    print(
                        f"Failed to open {ser.portstr} at {baud}, {parity}, {stop_bits}: {e}"
                    )
                finally:
                    ser.close()

    return ser


if __name__ == "__main__":
    # scan_devices()
    # main_fix()
    main_action()
