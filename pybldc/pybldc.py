#!/usr/bin/env python
#
# Copyright (C) 2024  Kristian Sloth Lauszus.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# Contact information
# -------------------
# Kristian Sloth Lauszus
# Web      :  https://www.lauszus.com
# e-mail   :  lauszus@gmail.com

import abc
import argparse
import logging
import queue
import threading
import time
from enum import IntEnum
from timeit import default_timer as timer
from types import TracebackType
from typing import Any, Generator, List, Optional, Type, Union, cast

import can
import heatshrink2
import serial
from tqdm import tqdm

from . import __version__


class CanPacketId(IntEnum):
    CAN_PACKET_FILL_RX_BUFFER = 5  # Fill 7 bytes at a time
    CAN_PACKET_FILL_RX_BUFFER_LONG = 6  # Fill 6 bytes
    CAN_PACKET_PROCESS_RX_BUFFER = 7  # Process the received byte
    CAN_PACKET_PROCESS_SHORT_BUFFER = 8  # Used when sending data <= 6
    CAN_PACKET_PING = 17
    CAN_PACKET_PONG = 18


class CommPacketId(IntEnum):
    COMM_JUMP_TO_BOOTLOADER = 1
    COMM_ERASE_NEW_APP = 2
    COMM_WRITE_NEW_APP_DATA = 3
    COMM_REBOOT = 29
    COMM_ERASE_BOOTLOADER = 73


class HwType(IntEnum):
    HW_TYPE_VESC = 0
    HW_TYPE_VESC_BMS = 1
    HW_TYPE_CUSTOM_MODULE = 2


def crc16_ccitt(buf: Union[bytes, List[int]]) -> int:
    # fmt:off
    crc16_ccitt_table = [0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
                         0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
                         0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
                         0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
                         0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
                         0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
                         0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
                         0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
                         0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
                         0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
                         0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
                         0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
                         0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
                         0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
                         0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
                         0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
                         0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
                         0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
                         0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
                         0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
                         0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
                         0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
                         0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
                         0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
                         0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
                         0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
                         0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
                         0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
                         0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0]
    # fmt: on

    cksum = 0
    for b in buf:
        cksum = (crc16_ccitt_table[((cksum >> 8) ^ b) & 0xFF] ^ (cksum << 8)) & 0xFFFF
    return cksum


class PyBldcBase:
    def __init__(
        self,
        logger: logging.Logger,
    ) -> None:
        self._logger = logger

    def __enter__(self) -> "PyBldcBase":
        return self

    def __exit__(self, exc_type: Type[BaseException], exc_value: BaseException, traceback: TracebackType) -> None:
        self.shutdown()

    @abc.abstractmethod
    def shutdown(self, timeout: float = 1.0) -> None:
        pass

    @staticmethod
    def chunks(lst: List[int], n: int) -> Generator[List[int], None, None]:
        for i in range(0, len(lst), n):
            yield lst[i : i + n]

    def upload(
        self,
        binary_filename: str,
        timeout: float = 5.0,
        ping_repeat: int = 3,
        attempts: int = 1,
        is_bootloader: bool = False,
    ) -> Generator[Union[float, bool], None, None]:
        if attempts < 1:
            raise ValueError('PyBldcBase: "attempts" has to be greater than 0')

        # Read the binary data from the file
        self._logger.info(
            f'PyBldcBase: Uploading "{binary_filename}" via {"CAN" if isinstance(self, PyBldcCan) else "serial"}'
        )

        upload_result = False
        for _ in range(attempts):
            # Yield a progress while uploading and store the return value
            upload_result = yield from self._upload(binary_filename, timeout, ping_repeat, is_bootloader=is_bootloader)

            if upload_result:
                # Uploading was successful
                break

        # Finally yield the result
        yield upload_result

    @staticmethod
    def _pack_uint16(data: int) -> List[int]:
        return [
            (data >> 8) & 0xFF,
            data & 0xFF,
        ]

    @staticmethod
    def _pack_uint24(data: int) -> List[int]:
        return [
            (data >> 16) & 0xFF,
            (data >> 8) & 0xFF,
            data & 0xFF,
        ]

    @staticmethod
    def _pack_uint32(data: int) -> List[int]:
        return [
            (data >> 24) & 0xFF,
            (data >> 16) & 0xFF,
            (data >> 8) & 0xFF,
            data & 0xFF,
        ]

    def _upload(
        self,
        binary_filename: str,
        timeout: float,
        ping_repeat: int,
        is_bootloader: bool = False,
    ) -> Generator[float, None, bool]:
        # 1. Try to ping the target x times to make sure we can communicate with it
        for i in range(ping_repeat):
            if self.ping(timeout=timeout):
                self._logger.info(f"PyBldcBase: Found VESC in {i + 1} attempt(s)")
                break
        else:
            self._logger.warning("Timed out waiting for ping response for VESC")
            return False

        # 2. Load the binary
        with open(binary_filename, "rb") as f:
            binary_data = f.read()

        # 3. Erase the region of memory where application will be located
        binary_data_len = len(binary_data)
        if is_bootloader:
            self._logger.info("PyBldcBase: Erasing bootloader")
            if not self._send_implementation(
                [CommPacketId.COMM_ERASE_BOOTLOADER],
                [],
                timeout=timeout,
            ):
                self._logger.error('PyBldcBase: "COMM_ERASE_BOOTLOADER" response failed for VESC')
                return False
        else:
            self._logger.info(f"PyBldcBase: Erasing {binary_data_len} bytes")
            if not self._send_implementation(
                [
                    CommPacketId.COMM_ERASE_NEW_APP,
                ]
                + PyBldcBase._pack_uint32(binary_data_len),
                [],
                timeout=timeout,
            ):
                self._logger.error('PyBldcBase: "COMM_ERASE_NEW_APP" response failed for VESC')
                return False

            # 4. Compress the binary if needed
            # Note: This is not supported for bootloaders
            if binary_data_len > 393208:
                self._logger.info(f"PyBldcBase: Compression is required, as binary data is large: {binary_data_len}")
                binary_data = heatshrink2.encode(
                    binary_data,
                    window_sz2=13,
                    lookahead_sz2=5,
                )
                self._logger.info(f"PyBldcBase: Size after compression: {len(binary_data)}")

                if len(binary_data) > 393208:
                    self._logger.info("PyBldcBase: The firmware is too big even after compression")
                    return False

                # "0xCC" is used to indicate to the bootloader that the data is compressed
                # See: https://github.com/vedderb/bldc-bootloader/blob/master/main.c
                binary_data_len = (0xCC << 24) | len(binary_data)

        # 5. Upload a new program chunk by chunk
        #    For bootloaders:
        #       Simply upload the bootloader binary
        #    For applications:
        #       The first 4 bytes must be set to the size of the new main program
        #       The next 2 bytes are a CRC-checksum of the new main program
        #       The next bytes is the program binary
        binary_data_crc16 = crc16_ccitt(binary_data)
        self._logger.info(f"PyBldcBase: Uploading binary with checksum: {binary_data_crc16}")

        if is_bootloader:
            app_data = list(binary_data)
        else:
            app_data = (
                PyBldcBase._pack_uint32(binary_data_len)
                + PyBldcBase._pack_uint16(binary_data_crc16)
                + list(binary_data)
            )

        chunk_size = 384  # This is the same size as the VESC Tool uses

        if is_bootloader:
            # The location of the bootloader depends on the type of hardware
            if isinstance(self, PyBldcCan):
                if self._can_listener.hw_type == HwType.HW_TYPE_CUSTOM_MODULE:
                    raise ValueError("HW_TYPE_CUSTOM_MODULE is not supported")
                elif self._can_listener.hw_type == HwType.HW_TYPE_VESC_BMS:
                    start_addr = 0x0803E000 - 0x08020000
                else:
                    start_addr = 1024 * 128 * 3
            else:
                # TODO: Determine HW type when connected via serial
                start_addr = 1024 * 128 * 3
        else:
            # Regular applications are always uploaded to address 0
            start_addr = 0

        data_sent = 0
        yield 0.0  # The progress starts at 0 %
        for i, d in enumerate(PyBldcBase.chunks(app_data, chunk_size)):
            offset = start_addr + (chunk_size * i)
            offset_list = PyBldcBase._pack_uint32(offset)
            if not self._send_implementation(
                [CommPacketId.COMM_WRITE_NEW_APP_DATA] + offset_list + d,
                expected_response=offset_list,
                timeout=timeout,
            ):
                self._logger.error('PyBldcBase: "COMM_WRITE_NEW_APP_DATA" response failed for VESC')
                return False

            # Yield the progress in percent
            data_sent += len(d)
            yield data_sent / len(app_data) * 100.0

        if not is_bootloader:
            # 6. Call the bootloader
            # Note: This does not have a response
            _ = self._send_implementation([CommPacketId.COMM_JUMP_TO_BOOTLOADER], [], timeout=0.0)

        return True

    @abc.abstractmethod
    def ping(self, timeout: float = 1.0) -> bool:
        pass

    def reset(self) -> None:
        self._logger.info("PyBldcBase: Sending reset command")

        # Note: This does not have a response
        _ = self._send_implementation([CommPacketId.COMM_REBOOT], [], timeout=0.0)

    def _wait_for_packet_response(
        self,
        packet_queue: queue.Queue[List[int]],
        comm_packet_id: CommPacketId,
        expected_response: List[int],
        timeout: float,
    ) -> Optional[bool]:
        if timeout == 0.0:
            # We do not actually care about the answer, so return immediately
            return None

        start = timer()
        while True:
            dt = timer() - start
            if dt >= timeout:
                return False

            try:
                response = packet_queue.get(timeout=timeout - dt)
                self._logger.debug(f"PyBldcBase: Received packet response: {response}, expected: {expected_response}")

                # Make sure it replies with the command as the first byte and "OK" as the second byte
                # Some commands repeat the data
                if response[0] == comm_packet_id:
                    return response[1] == 1 and response[2:] == expected_response
            except queue.Empty:
                return False

    @abc.abstractmethod
    def _send_implementation(self, data: List[int], expected_response: List[int], timeout: float) -> Optional[bool]:
        pass


class PyBldcCanListener(can.Listener):
    def __init__(self, can_id: int, controller_id: int, logger: logging.Logger) -> None:
        super().__init__()
        self._id = can_id
        self._controller_id = controller_id
        self._logger = logger
        self._is_stopped = False
        self.packet_queue: queue.Queue[List[int]] = queue.Queue()
        self.pong_event = threading.Event()
        self._hw_type: Optional[HwType] = None

    @property
    def hw_type(self) -> Optional[HwType]:
        return self._hw_type

    def on_message_received(self, msg: can.Message) -> None:
        # Make sure we are not stopped and it is a valid frame
        if self._is_stopped or msg.is_error_frame or msg.is_remote_frame or not msg.is_extended_id:
            return

        # Decode the controller ID and packet ID
        target_id = msg.arbitration_id & 0xFF
        packet_id = (msg.arbitration_id >> 8) & 0xFF

        # We are only interested in frames to us
        if target_id != self._id:
            return

        self._logger.debug(
            f"PyBldcCanListener: Received ID: 0x{msg.arbitration_id:08X}, len: {msg.dlc}, data: {list(msg.data)}"
        )

        # TODO: Add support for receiving longer packets
        if packet_id == CanPacketId.CAN_PACKET_PROCESS_SHORT_BUFFER:
            controller_id = msg.data[0]
            send_flag = msg.data[1]  # Make sure it is a response to a packet we sent
            if controller_id == self._controller_id and send_flag == 1:
                data = list(msg.data[2:])  # Extract the data
                self.packet_queue.put_nowait(data)
        elif packet_id == CanPacketId.CAN_PACKET_PONG:
            controller_id = msg.data[0]
            self._hw_type = cast(HwType, msg.data[1])
            if controller_id == self._controller_id:
                self.pong_event.set()

    def on_error(self, exc: Exception) -> None:
        # Workaround issue with errors being printed when the interface is shutting down
        if not self._is_stopped:
            self._logger.exception("PyBldcCanListener: on_error")

    def stop(self) -> None:
        self._is_stopped = True


class PyBldcCan(PyBldcBase):
    def __init__(
        self,
        logger: logging.Logger,
        controller_id: int,
        interface: str = "socketcan",
        channel: str = "can0",
        bitrate: int = 500000,
    ) -> None:
        super().__init__(logger=logger)

        if controller_id < 0 or controller_id >= 253:
            raise ValueError('PyBldcBase: "controller_id" has to be >=0 and <253')
        self._controller_id = controller_id
        self._id = 253  # VESC Tool uses 254, lets use 253
        # Only receive the messages from the specified VESC
        can_filters = [
            {
                "can_id": self._id,
                "can_mask": 0xFF,  # The controller id is the first byte in the CAN ID
                "extended": True,
            }
        ]
        self._can_bus = can.Bus(interface=interface, channel=channel, can_filters=can_filters, bitrate=bitrate)
        self._can_listener = PyBldcCanListener(self._id, self._controller_id, self._logger)
        self._can_notifier = can.Notifier(self._can_bus, [self._can_listener])

    def __enter__(self) -> "PyBldcCan":
        return self

    def shutdown(self, timeout: float = 1.0) -> None:
        self._can_notifier.stop(timeout=timeout)
        self._can_bus.shutdown()

    def ping(self, timeout: float = 1.0) -> bool:
        self._logger.info("PyBldcCan: Sending ping command")
        self._can_listener.pong_event.clear()
        self._can_send_packet(CanPacketId.CAN_PACKET_PING, [self._id])
        return self._can_listener.pong_event.wait(timeout)

    def _send_implementation(self, data: List[int], expected_response: List[int], timeout: float) -> Optional[bool]:
        """
        See: "comm_can_send_buffer" in "bldc/comm/comm_can.c" and
             "packetDataToSend" in "vesc_tool/vescinterface.cpp"
        """
        if len(data) <= 6:
            send_buffer: List[int] = [
                self._id,
                0,  # Process packet at receiver and send a response
            ] + list(data)

            self._can_send_packet(CanPacketId.CAN_PACKET_PROCESS_SHORT_BUFFER, send_buffer)
        else:
            end_a = 0
            for i in range(0, len(data), 7):
                if i > 255:
                    break
                end_a = i + 7

                send_buffer = [i]
                if i + 7 <= len(data):
                    send_len = 7
                else:
                    send_len = len(data) - i
                send_buffer += data[i : i + send_len]

                self._can_send_packet(CanPacketId.CAN_PACKET_FILL_RX_BUFFER, send_buffer)

                # Just sleep a minimal time, so the CAN buffer does not get full
                # TODO: Figure out how to block until the message is sent or determine if the buffer is full
                time.sleep(0.0001)

            for i in range(end_a, len(data), 6):
                send_buffer = PyBldcBase._pack_uint16(i)
                if i + 6 <= len(data):
                    send_len = 6
                else:
                    send_len = len(data) - i
                send_buffer += data[i : i + send_len]

                self._can_send_packet(CanPacketId.CAN_PACKET_FILL_RX_BUFFER_LONG, send_buffer)

                # Just sleep a minimal time, so the CAN buffer does not get full
                # TODO: Figure out how to block until the message is sent or determine if the buffer is full
                time.sleep(0.0001)

            send_buffer = (
                [
                    self._id,
                    0,  # Process packet at receiver and send a response
                ]
                + PyBldcBase._pack_uint16(len(data))
                + PyBldcBase._pack_uint16(crc16_ccitt(data))
            )

            self._can_send_packet(CanPacketId.CAN_PACKET_PROCESS_RX_BUFFER, send_buffer)

        # Wait for the response
        return self._wait_for_packet_response(
            packet_queue=self._can_listener.packet_queue,
            comm_packet_id=cast(CommPacketId, data[0]),
            expected_response=expected_response,
            timeout=timeout,
        )

    def _can_send_packet(self, can_packet_id: CanPacketId, data: List[int]) -> None:
        msg = can.Message(arbitration_id=self._controller_id | (can_packet_id << 8), data=data, is_extended_id=True)
        self._logger.debug(
            f"PyBldcBase: Sending ID: 0x{msg.arbitration_id:08X}, len: {msg.dlc}, data: {list(msg.data)}"
        )
        self._can_bus.send(msg)


class PyBldcSerial(PyBldcBase):
    def __init__(
        self,
        logger: logging.Logger,
        port: str,
        baudrate: int = 115200,
    ) -> None:
        super().__init__(logger=logger)

        # Open the serial port, but read from it in a thread, so we are not blocking the main loop
        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        self._shutdown_thread = threading.Event()
        self._received_packet_queue: queue.Queue[List[int]] = queue.Queue()
        self._thread = threading.Thread(
            target=self._serial_read_thread,
            name="_serial_read_thread",
            args=(
                self._serial,
                self._shutdown_thread,
                self._logger,
                self._received_packet_queue,
            ),
        )
        self._thread.daemon = False  # Make sure the application joins this before closing down
        self._thread.start()

    def __enter__(self) -> "PyBldcSerial":
        return self

    def shutdown(self, timeout: float = 1.0) -> None:
        self._shutdown_thread.set()
        self._thread.join(timeout=timeout)
        self._serial.close()

    def ping(self, timeout: float = 1.0) -> bool:
        # We assume that we can ping it if the serial port is open
        return bool(self._serial.is_open)

    def _send_implementation(self, data: List[int], expected_response: List[int], timeout: float) -> Optional[bool]:
        if len(data) <= 255:
            send_buffer = [
                2,  # Start of data
                len(data) & 0xFF,  # Length of data as 8-bits
            ]
        elif len(data) <= 65535:
            send_buffer = [
                3,  # Start of data
                *PyBldcBase._pack_uint16(len(data)),  # Length of data as 16-bits
            ]
        else:
            send_buffer = [
                4,  # Start of data
                *PyBldcBase._pack_uint24(len(data)),  # Length of data as 24-bits
            ]

        send_buffer.extend(data)  # Data
        send_buffer.extend(PyBldcBase._pack_uint16(crc16_ccitt(data)))  # Checksum
        send_buffer.append(3)  # Stop byte

        # Send the buffer on the serial interface
        self._serial.write(send_buffer)

        # Wait for the response
        return self._wait_for_packet_response(
            packet_queue=self._received_packet_queue,
            comm_packet_id=cast(CommPacketId, data[0]),
            expected_response=expected_response,
            timeout=timeout,
        )

    @staticmethod
    def _serial_read_thread(
        ser: Any,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        received_packet_queue: queue.Queue[List[int]],
    ) -> None:
        try:
            data_buffer = bytearray()
            while not shutdown_event.is_set() and ser.is_open:
                data = ser.read()
                if data:
                    data_buffer += data

                    while True:
                        # Based on "try_decode_packet" in "vesc_tool/packet.cpp"
                        if len(data_buffer) == 0:
                            break

                        is_len_8b = data_buffer[0] == 2
                        is_len_16b = data_buffer[0] == 3
                        is_len_24b = data_buffer[0] == 4

                        if not is_len_8b and not is_len_16b and not is_len_24b:
                            logger.warning(f"_serial_read_thread: Discarding invalid data: {data_buffer[0]}")
                            data_buffer = data_buffer[1:]  # Discard the fist byte
                            continue

                        data_start = data_buffer[0]
                        if len(data_buffer) < data_start:
                            # We need more data before we can read the data
                            break

                        if is_len_8b:
                            data_len = data_buffer[1]
                            if data_len < 1:
                                logger.warning(f"_serial_read_thread: Data len is not valid: {data_len} < 1")
                                data_buffer = data_buffer[1:]  # Discard the fist byte
                                continue
                        elif is_len_16b:
                            data_len = data_buffer[1] << 8 | data_buffer[2]
                            if data_len < 255:
                                logger.warning(f"_serial_read_thread: Packet is too short: {data_len} < 255")
                                data_buffer = data_buffer[1:]  # Discard the fist byte
                                continue
                        else:
                            data_len = data_buffer[1] << 16 | data_buffer[2] << 8 | data_buffer[3]
                            if data_len < 65535:
                                logger.warning(f"_serial_read_thread: Packet is too short: {data_len} < 65535")
                                data_buffer = data_buffer[1:]  # Discard the fist byte
                                continue

                        if len(data_buffer) < data_len + data_start + 3:
                            # Need more data to determine rest of packet
                            break

                        parsed_data = data_buffer[data_start : data_start + data_len]
                        if (
                            crc16_ccitt(parsed_data)
                            != data_buffer[data_start + data_len] << 8 | data_buffer[data_start + data_len + 1]
                        ):
                            logger.warning("_serial_read_thread: CRC failed")
                            data_buffer = data_buffer[1:]  # Discard the fist byte
                            continue

                        stop_byte = data_buffer[data_start + data_len + 2]
                        if stop_byte != 3:
                            logger.warning(f"_serial_read_thread: Invalid stop byte: {stop_byte}")
                            data_buffer = data_buffer[1:]  # Discard the fist byte
                            continue

                        # The data is now parsed, so advance the buffer to the next message
                        data_buffer = data_buffer[data_start + data_len + 3 :]
                        received_packet_queue.put_nowait(list(parsed_data))

                        logger.debug(f"_serial_read_thread: Packet response: {list(parsed_data)})")
        except serial.serialutil.SerialException:
            # This is triggered when the VESC jumps to the bootloader, so just ignore it
            logger.debug('BlhostSerial: Caught SerialException exception in "_serial_read_thread"', exc_info=True)
        except Exception:
            logger.exception('BlhostSerial: Caught exception in "_serial_read_thread"')


def cli() -> None:
    parser = argparse.ArgumentParser(prog="pybldc", add_help=False, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        "hw_interface", help="Communicate with the target via either CAN or serial", choices=["can", "serial"]
    )
    parser.add_argument(
        "command",
        help="upload: write BINARY\n"
        "ping: send a ping command to the target and check for a response\n"
        "reset: send a reset command to the target and check for a response",
        choices=["upload", "ping", "reset"],
    )

    # Options for "can"
    required_can = parser.add_argument_group("required CAN arguments")
    required_can.add_argument("-id", "--controller-id", help="The VESC ID used for communication")

    optional_can = parser.add_argument_group("optional CAN arguments")
    optional_can.add_argument(
        "-i", "--interface", help='The CAN-Bus interface to use (default "socketcan")', default="socketcan"
    )
    optional_can.add_argument("-l", "--channel", help='The CAN-Bus channel to use (default "can0")', default="can0")

    # Options for "serial"
    required_serial = parser.add_argument_group("required serial arguments")
    required_serial.add_argument("-p", "--port", help="The port to use for serial")

    # Common optional arguments
    optional = parser.add_argument_group("optional arguments")
    optional.add_argument("-h", "--help", action="help", help="Show this help message and exit")
    optional.add_argument(
        "--version",
        action="version",
        help="Show program's version number and exit",
        version=f"%(prog)s {__version__}",
    )
    optional.add_argument("-B", "--binary", help="The binary to upload")
    optional.add_argument(
        "--bootloader",
        help="The binary to upload is a bootloader",
        action="store_true",
    )
    optional.add_argument(
        "-t",
        "--timeout",
        help="The time to wait in seconds for a response (default 5.0)",
        default=5.0,
        type=float,
    )
    optional.add_argument(
        "-r",
        "--ping-repeat",
        help="The number of times to try to establish a connection by pinging the VESC (default 3)",
        default=3,
        type=int,
    )
    optional.add_argument(
        "-b",
        "--baudrate",
        "--bitrate",
        help="The baudrate/bitrate to use for serial/can (defaults to 115200 for serial and 500000 for CAN)",
    )
    optional.add_argument(
        "--debug",
        help="Turn on debug logs",
        action="store_true",
    )

    parsed_args = parser.parse_args()
    if parsed_args.hw_interface == "can":
        if parsed_args.controller_id is None:
            parser.print_help()
            exit(1)
        PyBldcImpl: Type[PyBldcBase] = PyBldcCan
        kwargs = {
            "controller_id": int(parsed_args.controller_id, base=0),
            "interface": parsed_args.interface,
            "channel": parsed_args.channel,
        }
        if parsed_args.baudrate is not None:
            kwargs["bitrate"] = parsed_args.baudrate
    else:
        if parsed_args.port is None:
            parser.print_help()
            exit(1)
        PyBldcImpl = PyBldcSerial
        kwargs = {
            "port": parsed_args.port,
        }
        if parsed_args.baudrate is not None:
            kwargs["baudrate"] = parsed_args.baudrate

    # Print all log output directly in the terminal
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG if parsed_args.debug else logging.INFO)
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))
    logger.addHandler(stream_handler)
    kwargs["logger"] = logger

    with PyBldcImpl(**kwargs) as bldc:
        if parsed_args.command == "upload":
            if parsed_args.binary is None:
                parser.print_help()
                exit(1)

            pbar = None
            result = False
            for upload_progress in bldc.upload(
                parsed_args.binary,
                timeout=parsed_args.timeout,
                ping_repeat=parsed_args.ping_repeat,
                is_bootloader=parsed_args.bootloader,
            ):
                if not isinstance(upload_progress, bool):
                    if pbar is None:
                        # Create it here, so the progress is not printed before we actually start uploading
                        pbar = tqdm(
                            desc="[INFO] Upload progress",
                            total=100,
                            bar_format="{l_bar}{bar}| [{elapsed}]",
                            dynamic_ncols=True,
                        )
                    pbar.update(upload_progress - pbar.n)
                    if upload_progress >= 100.0:
                        # Ensure nothing is printed after the update finishes
                        pbar.close()
                else:
                    result = upload_progress
            if pbar is not None:
                pbar.close()  # Make sure it is closed
            if result is True:
                logger.info("Uploading succeeded")
                exit(0)
            else:
                logger.error("Uploading failed")
                exit(1)
        elif parsed_args.command == "ping":
            for i in range(parsed_args.ping_repeat):
                if bldc.ping(timeout=parsed_args.timeout):
                    logger.info(f"Found VESC in {i + 1} attempt(s)")
                    exit(0)

            logger.error("Timed out waiting for ping response for VESC")
            exit(1)
        else:
            bldc.reset()
            logger.info("VESC was reset")
            exit(0)


if __name__ == "__main__":
    cli()
