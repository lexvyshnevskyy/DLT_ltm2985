import json
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from msgs.msg import Measurement

try:
    import serial  # type: ignore
    from serial import SerialException  # type: ignore
except ImportError:  # pragma: no cover
    serial = None
    SerialException = Exception


class Ltm2985UartNode(Node):
    def __init__(self) -> None:
        super().__init__('ltm2985_uart_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('source_name', 'ltm2985')
        self.declare_parameter('raw_topic', 'ltm2985/raw_json')
        self.declare_parameter('measurement_topic', 'ltm2985/measurement')
        self.declare_parameter('frame_separator', '\n')
        self.declare_parameter('poll_period_sec', 0.02)
        self.declare_parameter('reconnect_period_sec', 2.0)

        self._port = str(self.get_parameter('port').value)
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._source_name = str(self.get_parameter('source_name').value)
        self._separator = str(self.get_parameter('frame_separator').value).encode('utf-8')
        self._poll_period = float(self.get_parameter('poll_period_sec').value)
        self._reconnect_period = float(self.get_parameter('reconnect_period_sec').value)

        self._raw_pub = self.create_publisher(String, str(self.get_parameter('raw_topic').value), 50)
        self._measurement_pub = self.create_publisher(
            Measurement,
            str(self.get_parameter('measurement_topic').value),
            50,
        )

        self._serial = None
        self._buffer = bytearray()
        self._last_connect_attempt_ns = 0

        self.create_timer(self._poll_period, self._poll_serial)
        self.get_logger().info(
            f'Starting UART reader on {self._port} at {self._baudrate} baud.'
        )

    def _try_connect(self) -> None:
        if serial is None:
            self.get_logger().error('pyserial is not installed. Install python3-serial.')
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_connect_attempt_ns < int(self._reconnect_period * 1e9):
            return
        self._last_connect_attempt_ns = now_ns

        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0,
            )
            self._buffer.clear()
            self.get_logger().info(f'Connected to {self._port}.')
        except SerialException as exc:
            self._serial = None
            self.get_logger().warning(f'Unable to open {self._port}: {exc}')

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def _poll_serial(self) -> None:
        if self._serial is None:
            self._try_connect()
            return

        try:
            waiting = int(self._serial.in_waiting)
            if waiting <= 0:
                return

            data = self._serial.read(waiting)
            if not data:
                return

            self._buffer.extend(data)
            self._drain_frames()
        except SerialException as exc:
            self.get_logger().warning(f'Serial read failed, reconnecting: {exc}')
            self._close_serial()
        except Exception as exc:
            self.get_logger().error(f'Unexpected UART error: {exc}')
            self._close_serial()

    def _drain_frames(self) -> None:
        while True:
            idx = self._buffer.find(self._separator)
            if idx < 0:
                break

            frame = bytes(self._buffer[:idx])
            del self._buffer[:idx + len(self._separator)]

            line = frame.decode('utf-8', errors='replace').strip()
            if not line:
                continue
            self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        raw_msg = String()
        raw_msg.data = line
        self._raw_pub.publish(raw_msg)

        parsed = self._parse_measurement(line)
        if parsed is None:
            return

        self._measurement_pub.publish(parsed)

    def _parse_measurement(self, line: str) -> Optional[Measurement]:
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Ignoring non-JSON UART line: {line}')
            return None

        required = ['channel', 'type', 'value', 'raw_code', 'sensor_value', 'fault', 'valid']
        missing = [key for key in required if key not in payload]
        if missing:
            self.get_logger().warning(
                f'Ignoring incomplete frame. Missing fields: {missing}. Frame: {line}'
            )
            return None

        msg = Measurement()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = self._source_name
        msg.channel = int(payload['channel'])
        msg.type = str(payload['type'])
        msg.value = float(payload['value'])
        msg.raw_code = int(payload['raw_code'])
        msg.sensor_value = float(payload['sensor_value'])
        msg.fault = int(payload['fault'])
        msg.valid = bool(payload['valid'])
        msg.raw_json = line
        return msg

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ltm2985UartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
