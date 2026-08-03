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

    def start(self) -> None:
        self._send([128])
        time.sleep(0.12)

        self._send([131])
        time.sleep(0.12)

        self.stop()

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
