import json
import queue
import sys
import threading
import time
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
    """UART reader for LTM2985 JSON frames.

    Serial I/O runs on a dedicated thread so a stuck FTDI handle or blocking
    DDS publish cannot freeze the whole process (previously required SIGKILL
    and left core with stale temperatures for minutes).
    """

    def __init__(self) -> None:
        super().__init__('ltm2985_uart_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('source_name', 'ltm2985')
        self.declare_parameter('raw_topic', 'ltm2985/raw_json')
        self.declare_parameter('measurement_topic', 'ltm2985/measurement')
        self.declare_parameter('frame_separator', '\n')
        self.declare_parameter('publish_period_sec', 0.02)
        self.declare_parameter('reconnect_period_sec', 2.0)
        # No valid JSON measurement for this long → reopen / restart.
        self.declare_parameter('stale_timeout_sec', 15.0)
        self.declare_parameter('max_stale_recoveries', 3)

        self._port = str(self.get_parameter('port').value)
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._source_name = str(self.get_parameter('source_name').value)
        self._separator = str(self.get_parameter('frame_separator').value).encode('utf-8')
        self._publish_period = float(self.get_parameter('publish_period_sec').value)
        self._reconnect_period = float(self.get_parameter('reconnect_period_sec').value)
        self._stale_timeout = max(1.0, float(self.get_parameter('stale_timeout_sec').value))
        self._max_stale_recoveries = max(1, int(self.get_parameter('max_stale_recoveries').value))

        self._raw_pub = self.create_publisher(String, str(self.get_parameter('raw_topic').value), 50)
        self._measurement_pub = self.create_publisher(
            Measurement,
            str(self.get_parameter('measurement_topic').value),
            50,
        )

        self._line_queue: queue.Queue[str] = queue.Queue(maxsize=500)
        self._stop_event = threading.Event()
        self._last_good_measurement_monotonic = time.monotonic()
        self._consecutive_stale_recoveries = 0
        self._exit_requested = False
        self._reopen_event = threading.Event()

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name='ltm2985-uart-reader',
            daemon=True,
        )
        self._reader_thread.start()

        self.create_timer(self._publish_period, self._publish_tick)
        self.create_timer(1.0, self._watchdog_tick)
        self.get_logger().info(
            f'Starting UART reader thread on {self._port} at {self._baudrate} baud '
            f'(stale_timeout={self._stale_timeout:.1f}s, '
            f'max_stale_recoveries={self._max_stale_recoveries}).'
        )

    def _reader_loop(self) -> None:
        """Exclusive owner of the serial port — never touches ROS publishers."""
        if serial is None:
            self.get_logger().error('pyserial is not installed. Install python3-serial.')
            return

        ser = None
        buffer = bytearray()
        last_connect = 0.0

        while not self._stop_event.is_set():
            if self._reopen_event.is_set():
                self._reopen_event.clear()
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
                    buffer.clear()

            if ser is None:
                now = time.monotonic()
                if now - last_connect < self._reconnect_period:
                    self._stop_event.wait(0.1)
                    continue
                last_connect = now
                try:
                    # Critical: do NOT pulse DTR/RTS on open — that resets the
                    # LTM/Arduino into "Demo board DC2508 not found" and kills
                    # the JSON stream until a manual board reset.
                    ser = serial.Serial()
                    ser.port = self._port
                    ser.baudrate = self._baudrate
                    ser.timeout = 0.2
                    ser.write_timeout = 0.2
                    ser.dsrdtr = False
                    ser.rtscts = False
                    try:
                        ser.dtr = False
                        ser.rts = False
                    except Exception:
                        pass
                    ser.open()
                    try:
                        ser.setDTR(False)
                        ser.setRTS(False)
                    except Exception:
                        pass
                    buffer.clear()
                    self.get_logger().info(
                        f'Connected to {self._port} (DTR/RTS held low, no MCU reset).'
                    )
                except SerialException as exc:
                    ser = None
                    self.get_logger().warning(f'Unable to open {self._port}: {exc}')
                    self._stop_event.wait(self._reconnect_period)
                continue

            try:
                waiting = int(ser.in_waiting or 0)
                data = ser.read(waiting if waiting > 0 else 1)
                if not data:
                    continue
                buffer.extend(data)
                while True:
                    idx = buffer.find(self._separator)
                    if idx < 0:
                        break
                    frame = bytes(buffer[:idx])
                    del buffer[: idx + len(self._separator)]
                    line = frame.decode('utf-8', errors='replace').strip()
                    if not line:
                        continue
                    try:
                        self._line_queue.put_nowait(line)
                    except queue.Full:
                        try:
                            self._line_queue.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            self._line_queue.put_nowait(line)
                        except queue.Full:
                            pass
            except SerialException as exc:
                self.get_logger().warning(f'Serial read failed, reconnecting: {exc}')
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                buffer.clear()
            except Exception as exc:
                self.get_logger().error(f'Unexpected UART reader error: {exc}')
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                buffer.clear()

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _publish_tick(self) -> None:
        if self._exit_requested:
            return
        # Drain a bounded batch so a burst cannot starve the watchdog timer.
        for _ in range(64):
            try:
                line = self._line_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_line(line)

    def _watchdog_tick(self) -> None:
        if self._exit_requested:
            return
        age = time.monotonic() - self._last_good_measurement_monotonic
        if age < self._stale_timeout:
            return
        self._consecutive_stale_recoveries += 1
        self.get_logger().warning(
            f'No valid LTM measurement for {age:.1f}s on {self._port} '
            f'(recovery {self._consecutive_stale_recoveries}/{self._max_stale_recoveries}); '
            f'forcing serial reopen.'
        )
        self._reopen_event.set()
        # Reset grace so we don't immediately re-fire while reopening.
        self._last_good_measurement_monotonic = time.monotonic()
        if self._consecutive_stale_recoveries >= self._max_stale_recoveries:
            self._request_process_restart(
                'UART link still stalled after repeated reopen attempts.'
            )

    def _request_process_restart(self, reason: str) -> None:
        if self._exit_requested:
            return
        self._exit_requested = True
        self.get_logger().error(
            f'{reason} Exiting for systemd restart '
            f'(stale_timeout={self._stale_timeout:.1f}s, '
            f'recoveries={self._consecutive_stale_recoveries}/{self._max_stale_recoveries}).'
        )
        self._stop_event.set()
        self._reopen_event.set()
        raise SystemExit(1)

    def _handle_line(self, line: str) -> None:
        raw_msg = String()
        raw_msg.data = line
        try:
            self._raw_pub.publish(raw_msg)
        except Exception as exc:
            self.get_logger().warning(f'raw_json publish failed: {exc}')

        parsed = self._parse_measurement(line)
        if parsed is None:
            return

        try:
            self._measurement_pub.publish(parsed)
        except Exception as exc:
            self.get_logger().warning(f'measurement publish failed: {exc}')
            return

        self._last_good_measurement_monotonic = time.monotonic()
        self._consecutive_stale_recoveries = 0

    def _parse_measurement(self, line: str) -> Optional[Measurement]:
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            # Boot banners are normal; don't spam every line at warning forever.
            # Boot banners are common after board reset; keep noise low.
            if 'LTM2985' in line or line.startswith('*') or 'baud rate' in line.lower():
                return None
            self.get_logger().warning(f'Ignoring non-JSON UART line: {line[:120]}')
            return None

        required = ['channel', 'type', 'value', 'raw_code', 'sensor_value', 'fault', 'valid']
        missing = [key for key in required if key not in payload]
        if missing:
            self.get_logger().warning(
                f'Ignoring incomplete frame. Missing fields: {missing}. Frame: {line[:160]}'
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
        self._stop_event.set()
        self._reopen_event.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.5)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ltm2985UartNode()
    exit_code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        exit_code = int(exc.code) if isinstance(exc.code, int) else 1
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    if exit_code:
        sys.exit(exit_code)
