import os
import threading
import time

import serial


class RoombaController:
    def __init__(
        self,
        port: str = "/dev/serial0",
        baud: int = 115200,
        speed: int = 300,
    ) -> None:
        self.serial = serial.Serial(port, baud, timeout=2)
        self.speed = speed
        self.lock = threading.Lock()
        self.vacuum_enabled = False
        self.wake_gpio = os.environ.get("ROOMBA_WAKE_GPIO", "").strip()
        self.wake_pulse_seconds = float(
            os.environ.get("ROOMBA_WAKE_PULSE_SECONDS", "0.65")
        )
        self.wake_settle_seconds = float(
            os.environ.get("ROOMBA_WAKE_SETTLE_SECONDS", "1.2")
        )
        self.start_attempts = int(
            os.environ.get("ROOMBA_START_ATTEMPTS", "4")
        )

    def _send(self, command: list[int]) -> None:
        with self.lock:
            self.serial.write(bytes(command))
            self.serial.flush()

    def _query_list(
        self,
        packet_ids: list[int],
        byte_count: int,
        timeout: float = 0.12,
    ) -> bytes:
        with self.lock:
            previous_timeout = self.serial.timeout

            try:
                self.serial.timeout = timeout
                self.serial.reset_input_buffer()
                self.serial.write(bytes([149, len(packet_ids)] + packet_ids))
                self.serial.flush()
                data = self.serial.read(byte_count)
            finally:
                self.serial.timeout = previous_timeout

        if len(data) != byte_count:
            raise TimeoutError("Roomba sensor query timed out")

        return data

    def read_distance_angle(self) -> tuple[int, int]:
        data = self._query_list([19, 20], 4)

        distance = int.from_bytes(
            data[0:2],
            byteorder="big",
            signed=True,
        )

        angle = int.from_bytes(
            data[2:4],
            byteorder="big",
            signed=True,
        )

        return distance, angle

    def read_bumps_wheel_drops(self) -> dict:
        data = self._query_list([7], 1, timeout=0.08)
        value = data[0]

        bump_right = bool(value & 0b00001)
        bump_left = bool(value & 0b00010)
        wheel_drop_right = bool(value & 0b00100)
        wheel_drop_left = bool(value & 0b01000)
        wheel_drop_caster = bool(value & 0b10000)

        return {
            "raw": value,
            "bump_left": bump_left,
            "bump_right": bump_right,
            "bump": bump_left or bump_right,
            "wheel_drop_left": wheel_drop_left,
            "wheel_drop_right": wheel_drop_right,
            "wheel_drop_caster": wheel_drop_caster,
            "wheel_drop": (
                wheel_drop_left
                or wheel_drop_right
                or wheel_drop_caster
            ),
        }

    def _pulse_serial_wake_lines(self) -> None:
        with self.lock:
            try:
                self.serial.dtr = False
                self.serial.rts = False
                self.serial.break_condition = True
                time.sleep(self.wake_pulse_seconds)
                self.serial.break_condition = False
                self.serial.dtr = True
                self.serial.rts = True
                self.serial.flush()
            except Exception:
                try:
                    self.serial.break_condition = False
                except Exception:
                    pass

    def _pulse_gpio_wake_line(self) -> None:
        if not self.wake_gpio:
            return

        try:
            pin = int(self.wake_gpio)
        except ValueError:
            return

        try:
            import RPi.GPIO as GPIO
        except Exception:
            return

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(self.wake_pulse_seconds)
        GPIO.output(pin, GPIO.HIGH)

    def wake(self) -> None:
        self._pulse_serial_wake_lines()
        self._pulse_gpio_wake_line()
        time.sleep(self.wake_settle_seconds)

    def _verify_open_interface(self) -> None:
        self.read_bumps_wheel_drops()

    def start(self) -> None:
        last_error = None

        for _ in range(max(1, self.start_attempts)):
            try:
                self.wake()
                self._send([128])
                time.sleep(0.2)
                self._send([131])
                time.sleep(0.2)
                self._verify_open_interface()
                self.stop()
                return
            except Exception as error:
                last_error = error
                time.sleep(0.4)

        raise ConnectionError(
            "Roomba did not answer after wake/start attempts"
        ) from last_error

    @staticmethod
    def _signed_16(value: int) -> list[int]:
        value = max(-500, min(500, int(value)))
        return list(value.to_bytes(2, byteorder="big", signed=True))

    def drive_wheels(self, right_speed: int, left_speed: int) -> None:
        command = (
            [145]
            + self._signed_16(right_speed)
            + self._signed_16(left_speed)
        )
        self._send(command)

    def stop(self) -> None:
        self.drive_wheels(0, 0)

    def set_speed(self, speed: int) -> None:
        self.speed = max(50, min(500, int(speed)))

    def vacuum_on(self) -> None:
        # Side brush + suction fan + main brush.
        self._send([138, 7])
        self.vacuum_enabled = True

    def vacuum_off(self) -> None:
        self._send([138, 0])
        self.vacuum_enabled = False

    def seek_dock(self) -> None:
        self._send([143])

    def close(self) -> None:
        try:
            self.stop()
            self.vacuum_off()
        finally:
            self.serial.close()
