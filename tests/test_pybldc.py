#!/usr/bin/env python
#
# Copyright (C) 2024-2025  Kristian Sloth Lauszus.
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

from __future__ import annotations

import logging
import pathlib
import queue
import sys
import tempfile
import threading
import time
from unittest.mock import Mock, patch

import can
import pytest
import serial

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from pybldc.pybldc import (
    CanPacketId,
    CommPacketId,
    HwType,
    PyBldcBase,
    PyBldcCan,
    PyBldcCanListener,
    PyBldcSerial,
    cli,
    crc16_ccitt,
)


def test_version() -> None:
    """Test that the version string is the same as the one in pyproject.toml."""
    from pybldc import __version__

    with open(pathlib.Path(__file__).parent / "../pyproject.toml", "rb") as f:
        toml_dict = tomllib.load(f)

    assert isinstance(__version__, str)
    assert __version__ == toml_dict["project"]["version"]


class TestEnums:
    """Test the enum classes."""

    def test_can_packet_id_values(self) -> None:
        """Test CanPacketId enum values."""
        assert int(CanPacketId.CAN_PACKET_FILL_RX_BUFFER) == 5
        assert int(CanPacketId.CAN_PACKET_FILL_RX_BUFFER_LONG) == 6
        assert int(CanPacketId.CAN_PACKET_PROCESS_RX_BUFFER) == 7
        assert int(CanPacketId.CAN_PACKET_PROCESS_SHORT_BUFFER) == 8
        assert int(CanPacketId.CAN_PACKET_PING) == 17
        assert int(CanPacketId.CAN_PACKET_PONG) == 18

    def test_comm_packet_id_values(self) -> None:
        """Test CommPacketId enum values."""
        assert int(CommPacketId.COMM_JUMP_TO_BOOTLOADER) == 1
        assert int(CommPacketId.COMM_ERASE_NEW_APP) == 2
        assert int(CommPacketId.COMM_WRITE_NEW_APP_DATA) == 3
        assert int(CommPacketId.COMM_REBOOT) == 29
        assert int(CommPacketId.COMM_ERASE_BOOTLOADER) == 73

    def test_hw_type_values(self) -> None:
        """Test HwType enum values."""
        assert int(HwType.HW_TYPE_VESC) == 0
        assert int(HwType.HW_TYPE_VESC_BMS) == 1
        assert int(HwType.HW_TYPE_CUSTOM_MODULE) == 2


class TestCrc16Ccitt:
    """Test the CRC16 CCITT calculation function."""

    def test_empty_buffer(self) -> None:
        """Test CRC calculation with empty buffer."""
        assert crc16_ccitt([]) == 0
        assert crc16_ccitt(b"") == 0
        assert crc16_ccitt(bytearray()) == 0

    def test_single_byte(self) -> None:
        """Test CRC calculation with single byte."""
        assert crc16_ccitt([0x01]) == 0x1021
        assert crc16_ccitt(b"\x01") == 0x1021
        assert crc16_ccitt(bytearray([0x01])) == 0x1021

    def test_multiple_bytes(self) -> None:
        """Test CRC calculation with multiple bytes."""
        expected_crc = 0xB42C  # Correct CRC value

        test_data = [0x12, 0x34, 0x56, 0x78]
        assert crc16_ccitt(test_data) == expected_crc

        test_string = b"\x12\x34\x56\x78"
        assert crc16_ccitt(test_string) == expected_crc

    def test_different_input_types(self) -> None:
        """Test that different input types produce the same result."""
        test_data = [0xAB, 0xCD, 0xEF]
        crc_list = crc16_ccitt(test_data)
        crc_bytes = crc16_ccitt(bytes(test_data))
        crc_bytearray = crc16_ccitt(bytearray(test_data))

        assert crc_list == crc_bytes == crc_bytearray


class TestPyBldcBase:
    """Test the abstract PyBldcBase class."""

    class ConcretePyBldcBase(PyBldcBase):
        """Concrete implementation for testing."""

        def __init__(self, logger: logging.Logger) -> None:
            super().__init__(logger)
            self.shutdown_called = False
            self.ping_called = False
            self.send_implementation_calls: list[tuple[list[int], list[int], float]] = []

        def shutdown(self, timeout: float | None = 1.0) -> None:
            self.shutdown_called = True

        def ping(self, timeout: float = 1.0) -> bool:
            self.ping_called = True
            return True

        def _send_implementation(self, data: list[int], expected_response: list[int], timeout: float) -> bool | None:
            self.send_implementation_calls.append((data, expected_response, timeout))
            return True

    @pytest.fixture
    def logger(self) -> logging.Logger:
        """Create a logger for testing."""
        return logging.getLogger(self.__class__.__name__)

    @pytest.fixture
    def bldc_base(self, logger: logging.Logger) -> ConcretePyBldcBase:
        """Create a concrete PyBldcBase instance for testing."""
        return self.ConcretePyBldcBase(logger)

    def test_context_manager(self, bldc_base: ConcretePyBldcBase) -> None:
        """Test that the context manager works correctly."""
        with bldc_base as bldc:
            assert bldc is bldc_base
        assert bldc_base.shutdown_called

    def test_chunks_method(self) -> None:
        """Test the chunks static method."""
        data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        chunks = list(PyBldcBase.chunks(data, 3))
        expected = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10]]
        assert chunks == expected

    def test_pack_uint16(self) -> None:
        """Test the _pack_uint16 static method."""
        assert PyBldcBase._pack_uint16(0x1234) == [0x12, 0x34]
        assert PyBldcBase._pack_uint16(0x0000) == [0x00, 0x00]
        assert PyBldcBase._pack_uint16(0xFFFF) == [0xFF, 0xFF]

    def test_pack_uint24(self) -> None:
        """Test the _pack_uint24 static method."""
        assert PyBldcBase._pack_uint24(0x123456) == [0x12, 0x34, 0x56]
        assert PyBldcBase._pack_uint24(0x000000) == [0x00, 0x00, 0x00]
        assert PyBldcBase._pack_uint24(0xFFFFFF) == [0xFF, 0xFF, 0xFF]

    def test_pack_uint32(self) -> None:
        """Test the _pack_uint32 static method."""
        assert PyBldcBase._pack_uint32(0x12345678) == [0x12, 0x34, 0x56, 0x78]
        assert PyBldcBase._pack_uint32(0x00000000) == [0x00, 0x00, 0x00, 0x00]
        assert PyBldcBase._pack_uint32(0xFFFFFFFF) == [0xFF, 0xFF, 0xFF, 0xFF]

    def test_reset(self, bldc_base: ConcretePyBldcBase) -> None:
        """Test the reset method."""
        bldc_base.reset()
        assert len(bldc_base.send_implementation_calls) == 1
        call = bldc_base.send_implementation_calls[0]
        assert len(call) == 3
        assert call[0] == [CommPacketId.COMM_REBOOT]
        assert call[1] == []
        assert call[2] == 0.0

    def test_wait_for_packet_response_timeout_zero(self, bldc_base: ConcretePyBldcBase, logger: logging.Logger) -> None:
        """Test _wait_for_packet_response with zero timeout."""
        packet_queue: queue.Queue[list[int]] = queue.Queue()
        result = bldc_base._wait_for_packet_response(packet_queue, CommPacketId.COMM_REBOOT, [], 0.0)
        assert result is None

    def test_wait_for_packet_response_timeout(self, bldc_base: ConcretePyBldcBase, logger: logging.Logger) -> None:
        """Test _wait_for_packet_response with timeout."""
        packet_queue: queue.Queue[list[int]] = queue.Queue()
        result = bldc_base._wait_for_packet_response(packet_queue, CommPacketId.COMM_REBOOT, [], 0.1)
        assert result is False

    def test_wait_for_packet_response_success(self, bldc_base: ConcretePyBldcBase, logger: logging.Logger) -> None:
        """Test _wait_for_packet_response with successful response."""
        packet_queue: queue.Queue[list[int]] = queue.Queue()
        # Put a successful response in the queue
        packet_queue.put([CommPacketId.COMM_REBOOT, 1])  # Command ID and OK flag

        result = bldc_base._wait_for_packet_response(packet_queue, CommPacketId.COMM_REBOOT, [], 1.0)
        assert result is True

    def test_wait_for_packet_response_wrong_command(
        self, bldc_base: ConcretePyBldcBase, logger: logging.Logger
    ) -> None:
        """Test _wait_for_packet_response with wrong command in response."""
        packet_queue: queue.Queue[list[int]] = queue.Queue()
        # Put a response with wrong command ID
        packet_queue.put([CommPacketId.COMM_ERASE_NEW_APP, 1])

        result = bldc_base._wait_for_packet_response(packet_queue, CommPacketId.COMM_REBOOT, [], 0.5)
        assert result is False

    def test_wait_for_packet_response_not_ok(self, bldc_base: ConcretePyBldcBase, logger: logging.Logger) -> None:
        """Test _wait_for_packet_response with not OK response."""
        packet_queue: queue.Queue[list[int]] = queue.Queue()
        # Put a response with OK flag = 0
        packet_queue.put([CommPacketId.COMM_REBOOT, 0])

        result = bldc_base._wait_for_packet_response(packet_queue, CommPacketId.COMM_REBOOT, [], 1.0)
        assert result is False


class TestPyBldcCanListener:
    """Test the PyBldcCanListener class."""

    @pytest.fixture
    def logger(self) -> logging.Logger:
        """Create a logger for testing."""
        return logging.getLogger(self.__class__.__name__)

    @pytest.fixture
    def listener(self, logger: logging.Logger) -> PyBldcCanListener:
        """Create a PyBldcCanListener for testing."""
        return PyBldcCanListener(can_id=253, controller_id=42, logger=logger)

    def test_initialization(self, listener: PyBldcCanListener) -> None:
        """Test listener initialization."""
        assert listener._id == 253
        assert listener._controller_id == 42
        assert not listener._is_stopped
        assert listener.hw_type is None

    def test_on_message_received_stopped(self, listener: PyBldcCanListener) -> None:
        """Test that stopped listener ignores messages."""
        msg = can.Message(
            arbitration_id=253 | (CanPacketId.CAN_PACKET_PONG << 8),
            data=[42],
            is_extended_id=True,
        )

        listener.on_message_received(msg)
        assert listener.pong_event.is_set()

        listener.pong_event.clear()
        listener.stop()

        listener.on_message_received(msg)
        assert listener.pong_event.is_set() is False

    def test_on_message_received_invalid_frame(self, listener: PyBldcCanListener) -> None:
        """Test that invalid frames are ignored."""
        # Error frame
        msg = can.Message(arbitration_id=253 | (CanPacketId.CAN_PACKET_PONG << 8), data=[42], is_error_frame=True)
        listener.on_message_received(msg)
        assert listener.pong_event.is_set() is False

        # Remote frame
        msg = can.Message(arbitration_id=253 | (CanPacketId.CAN_PACKET_PONG << 8), data=[42], is_remote_frame=True)
        listener.on_message_received(msg)
        assert listener.pong_event.is_set() is False

        # Not extended ID
        msg = can.Message(arbitration_id=253 | (CanPacketId.CAN_PACKET_PONG << 8), data=[42], is_extended_id=False)
        listener.on_message_received(msg)
        assert listener.pong_event.is_set() is False

        # Corrected ID but too short data
        msg = can.Message(arbitration_id=253 | (CanPacketId.CAN_PACKET_PONG << 8), data=[], is_extended_id=True)
        with pytest.raises(IndexError):
            listener.on_message_received(msg)

        # Wrong target ID
        msg = can.Message(arbitration_id=254 | (CanPacketId.CAN_PACKET_PONG << 8), data=[42], is_extended_id=True)
        listener.on_message_received(msg)
        assert listener.pong_event.is_set() is False

        # Corrected message
        msg = can.Message(arbitration_id=253 | (CanPacketId.CAN_PACKET_PONG << 8), data=[42], is_extended_id=True)
        listener.on_message_received(msg)
        assert listener.pong_event.is_set() is True

    def test_on_message_received_pong(self, listener: PyBldcCanListener) -> None:
        """Test handling of PONG messages."""
        msg = can.Message(
            arbitration_id=253 | (CanPacketId.CAN_PACKET_PONG << 8),
            data=[42, HwType.HW_TYPE_VESC],
            is_extended_id=True,
        )

        listener.on_message_received(msg)
        assert listener.pong_event.is_set()
        assert listener.hw_type == HwType.HW_TYPE_VESC

    def test_on_message_received_pong_no_hw_type(self, listener: PyBldcCanListener) -> None:
        """Test handling of PONG messages without HW type (older firmware)."""
        msg = can.Message(
            arbitration_id=253 | (CanPacketId.CAN_PACKET_PONG << 8),
            data=[42],  # Only controller ID, no HW type
            is_extended_id=True,
        )

        listener.on_message_received(msg)
        assert listener.pong_event.is_set()
        assert listener.hw_type == HwType.HW_TYPE_VESC

    def test_on_message_received_short_buffer(self, listener: PyBldcCanListener) -> None:
        """Test handling of short buffer messages."""
        msg = can.Message(
            arbitration_id=253 | (CanPacketId.CAN_PACKET_PROCESS_SHORT_BUFFER << 8),
            data=[
                42,  # controller_id
                1,  # send_flag
                # data
                0x12,
                0x34,
                0x56,
            ],
            is_extended_id=True,
        )

        listener.on_message_received(msg)

        # Check that data was added to packet queue
        assert not listener.packet_queue.empty()
        received_data = listener.packet_queue.get_nowait()
        assert received_data == [0x12, 0x34, 0x56]

    def test_on_message_received_short_buffer_wrong_controller(self, listener: PyBldcCanListener) -> None:
        """Test that short buffer messages for wrong controller are ignored."""
        msg = can.Message(
            arbitration_id=253 | (CanPacketId.CAN_PACKET_PROCESS_SHORT_BUFFER << 8),
            data=[43, 1, 0x12, 0x34, 0x56],  # Wrong controller_id
            is_extended_id=True,
        )

        listener.on_message_received(msg)
        assert listener.packet_queue.empty()

    def test_on_message_received_short_buffer_wrong_flag(self, listener: PyBldcCanListener) -> None:
        """Test that short buffer messages with wrong flag are ignored."""
        msg = can.Message(
            arbitration_id=253 | (CanPacketId.CAN_PACKET_PROCESS_SHORT_BUFFER << 8),
            data=[42, 0, 0x12, 0x34, 0x56],  # Wrong send_flag
            is_extended_id=True,
        )

        listener.on_message_received(msg)
        assert listener.packet_queue.empty()

    def test_stop(self, listener: PyBldcCanListener) -> None:
        """Test stopping the listener."""
        listener.stop()
        assert listener._is_stopped

    def test_on_error_when_stopped(self, listener: PyBldcCanListener, caplog: pytest.LogCaptureFixture) -> None:
        """Test that errors are not logged when stopped."""
        listener.stop()
        with caplog.at_level(logging.ERROR):
            listener.on_error(Exception("test error"))
        assert "test error" not in caplog.text

    def test_on_error_when_running(self, listener: PyBldcCanListener, caplog: pytest.LogCaptureFixture) -> None:
        """Test that errors are logged when running."""
        with caplog.at_level(logging.ERROR):
            listener.on_error(Exception("test error"))
        # Check that the exception was logged (the message format includes "PyBldcCanListener: on_error")
        assert "PyBldcCanListener: on_error" in caplog.text


class TestPyBldcCan:
    """Test the PyBldcCan class."""

    @pytest.fixture
    def logger(self) -> logging.Logger:
        """Create a logger for testing."""
        return logging.getLogger(self.__class__.__name__)

    def test_invalid_controller_id(self, logger: logging.Logger) -> None:
        """Test that invalid controller IDs raise ValueError."""
        with pytest.raises(ValueError, match=r"controller_id.*>=0 and <253"):
            PyBldcCan(logger=logger, controller_id=-1)

        with pytest.raises(ValueError, match=r"controller_id.*>=0 and <253"):
            PyBldcCan(logger=logger, controller_id=253)

    @patch("can.Bus")
    @patch("can.Notifier")
    def test_initialization(self, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger) -> None:
        """Test PyBldcCan initialization."""
        bldc_can = PyBldcCan(
            logger=logger,
            controller_id=42,
            interface="test",
            channel="test0",
            bitrate=1000000,
        )

        assert bldc_can._controller_id == 42
        assert bldc_can._id == 253

        # Check that CAN bus was created with correct parameters
        mock_bus.assert_called_once_with(
            interface="test",
            channel="test0",
            can_filters=[{"can_id": 253, "can_mask": 0xFF, "extended": True}],
            bitrate=1000000,
        )

    @patch("can.Bus")
    @patch("can.Notifier")
    def test_shutdown(self, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger) -> None:
        """Test PyBldcCan shutdown."""
        with PyBldcCan(logger=logger, controller_id=42):
            pass  # Shutdown is called in __exit__

        mock_notifier.return_value.stop.assert_called_once_with(timeout=1.0)
        mock_bus.return_value.shutdown.assert_called_once()

    @patch("can.Bus")
    @patch("can.Notifier")
    def test_shutdown_none_timeout(self, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger) -> None:
        """Test that None timeout raises ValueError."""
        bldc_can = PyBldcCan(logger=logger, controller_id=42)

        with pytest.raises(ValueError, match=r"timeout.*None.*not supported"):
            bldc_can.shutdown(timeout=None)

    @patch("can.Bus")
    @patch("can.Notifier")
    def test_ping(self, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger) -> None:
        """Test ping functionality."""
        bldc_can = PyBldcCan(logger=logger, controller_id=42)

        # Mock the pong event
        bldc_can._can_listener.pong_event = Mock()
        bldc_can._can_listener.pong_event.wait.return_value = True

        result = bldc_can.ping(timeout=1.0)

        assert result is True
        bldc_can._can_listener.pong_event.clear.assert_called_once()
        bldc_can._can_listener.pong_event.wait.assert_called_once_with(1.0)

    @patch("can.Bus")
    @patch("can.Notifier")
    def test_can_send_packet(self, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger) -> None:
        """Test _can_send_packet method."""
        bldc_can = PyBldcCan(logger=logger, controller_id=42)

        bldc_can._can_send_packet(CanPacketId.CAN_PACKET_PING, [253])

        # Check that the message was sent with correct parameters
        mock_bus.return_value.send.assert_called_once()
        sent_message = mock_bus.return_value.send.call_args[0][0]

        assert sent_message.arbitration_id == 42 | (CanPacketId.CAN_PACKET_PING << 8)
        assert list(sent_message.data) == [253]
        assert sent_message.is_extended_id is True


class TestPyBldcSerial:
    """Test the PyBldcSerial class."""

    @pytest.fixture
    def logger(self) -> logging.Logger:
        """Create a logger for testing."""
        return logging.getLogger(self.__class__.__name__)

    @patch("serial.Serial")
    def test_initialization(self, mock_serial: Mock, logger: logging.Logger) -> None:
        """Test PyBldcSerial initialization."""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = True
        mock_serial.return_value = mock_serial_instance

        with patch("threading.Thread") as mock_thread:
            PyBldcSerial(
                logger=logger,
                port="/dev/ttyUSB0",
                baudrate=9600,
            )

            # Check that serial was opened with correct parameters
            mock_serial.assert_called_once_with(
                port="/dev/ttyUSB0",
                baudrate=9600,
                timeout=0.5,
                exclusive=True,
            )

            # Check that thread was started
            mock_thread.assert_called_once()
            mock_thread.return_value.start.assert_called_once()

    @patch("serial.Serial")
    def test_shutdown(self, mock_serial: Mock, logger: logging.Logger) -> None:
        """Test PyBldcSerial shutdown."""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = True
        mock_serial.return_value = mock_serial_instance

        with patch("threading.Thread") as mock_thread:
            mock_thread_instance = Mock()
            mock_thread.return_value = mock_thread_instance

            with PyBldcSerial(logger=logger, port="/dev/ttyUSB0") as bldc_serial:
                pass  # Shutdown is called in __exit__

            assert bldc_serial._shutdown_thread.is_set()
            mock_thread_instance.join.assert_called_once_with(timeout=1.0)
            mock_serial_instance.close.assert_called_once()

    @patch("serial.Serial")
    def test_ping_open_port(self, mock_serial: Mock, logger: logging.Logger) -> None:
        """Test ping with open serial port."""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = True
        mock_serial.return_value = mock_serial_instance

        with patch("threading.Thread"):
            bldc_serial = PyBldcSerial(logger=logger, port="/dev/ttyUSB0")
            result = bldc_serial.ping()

            assert result is True

    @patch("serial.Serial")
    @patch("time.sleep")
    def test_ping_closed_port(self, mock_sleep: Mock, mock_serial: Mock, logger: logging.Logger) -> None:
        """Test ping with closed serial port."""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = False
        mock_serial.return_value = mock_serial_instance

        with patch("threading.Thread"):
            bldc_serial = PyBldcSerial(logger=logger, port="/dev/ttyUSB0")
            result = bldc_serial.ping()

            assert result is False
            mock_sleep.assert_called_once()


class TestCli:
    """Test the CLI functionality."""

    def test_cli_help(self) -> None:
        """Test that CLI shows help when no arguments provided."""
        with patch("sys.argv", ["pybldc"]), pytest.raises(SystemExit) as exc_info:
            cli()

        assert exc_info.value.code == 2  # argparse error exit code

    def test_cli_can_missing_controller_id(self) -> None:
        """Test CLI with CAN but missing controller ID."""
        with (
            patch("sys.argv", ["pybldc", "can", "ping"]),
            pytest.raises(SystemExit) as exc_info,
        ):
            cli()

        assert exc_info.value.code == 2  # argparse error exit code

    def test_cli_serial_missing_port(self) -> None:
        """Test CLI with serial but missing port."""
        with (
            patch("sys.argv", ["pybldc", "serial", "ping"]),
            pytest.raises(SystemExit) as exc_info,
        ):
            cli()

        assert exc_info.value.code == 2  # argparse error exit code

    @patch("pybldc.pybldc.PyBldcCan")
    def test_cli_can_ping_success(self, mock_pybldc_can: Mock) -> None:
        """Test CLI CAN ping success."""
        mock_instance = Mock()
        mock_instance.ping.return_value = True
        mock_pybldc_can.return_value.__enter__.return_value = mock_instance

        with (
            patch("sys.argv", ["pybldc", "can", "ping", "-id", "42"]),
            pytest.raises(SystemExit) as exc_info,
            patch("logging.getLogger"),
        ):
            cli()
            assert mock_instance.ping.call_count == 3

        assert exc_info.value.code == 0

    @patch("pybldc.pybldc.PyBldcCan")
    def test_cli_can_ping_failure(self, mock_pybldc_can: Mock) -> None:
        """Test CLI CAN ping failure."""
        mock_instance = Mock()
        mock_instance.ping.return_value = False
        mock_pybldc_can.return_value.__enter__.return_value = mock_instance

        with (
            patch("sys.argv", ["pybldc", "can", "ping", "-id", "42"]),
            pytest.raises(SystemExit) as exc_info,
            patch("logging.getLogger"),
        ):
            cli()
            assert mock_instance.ping.call_count == 3

        assert exc_info.value.code == 1

    @patch("pybldc.pybldc.PyBldcSerial")
    def test_cli_serial_reset(self, mock_pybldc_serial: Mock) -> None:
        """Test CLI serial reset."""
        mock_instance = Mock()
        mock_pybldc_serial.return_value.__enter__.return_value = mock_instance

        with (
            patch("sys.argv", ["pybldc", "serial", "reset", "-p", "/dev/ttyUSB0"]),
            pytest.raises(SystemExit) as exc_info,
            patch("logging.getLogger"),
        ):
            cli()
            mock_instance.reset.assert_called_once()

        assert exc_info.value.code == 0

    @patch("pybldc.pybldc.PyBldcCan")
    def test_cli_upload_missing_binary(self, mock_pybldc_can: Mock) -> None:
        """Test CLI upload command without binary."""
        with (
            patch("sys.argv", ["pybldc", "can", "upload", "-id", "42"]),
            pytest.raises(SystemExit) as exc_info,
        ):
            cli()

        assert exc_info.value.code == 2  # argparse error exit code

    @patch("pybldc.pybldc.PyBldcCan")
    @patch("tqdm.tqdm")
    def test_cli_upload_success(self, mock_tqdm: Mock, mock_pybldc_can: Mock) -> None:
        """Test CLI upload success."""
        mock_instance = Mock()
        # Mock upload generator yielding progress then success
        mock_instance.upload.return_value = iter([0.0, 50.0, 100.0, True])
        mock_pybldc_can.return_value.__enter__.return_value = mock_instance

        # Create a temporary binary file
        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(b"test binary data")
            temp_file.flush()

            with (
                patch(
                    "sys.argv",
                    ["pybldc", "can", "upload", "-id", "42", "-B", temp_file.name],
                ),
                pytest.raises(SystemExit) as exc_info,
                patch("logging.getLogger"),
            ):
                cli()
                mock_instance.upload.assert_called_once()

            assert exc_info.value.code == 0

    @patch("pybldc.pybldc.PyBldcCan")
    @patch("tqdm.tqdm")
    def test_cli_upload_failure(self, mock_tqdm: Mock, mock_pybldc_can: Mock) -> None:
        """Test CLI upload failure."""
        mock_instance = Mock()
        # Mock upload generator yielding progress then failure
        mock_instance.upload.return_value = iter([0.0, 50.0, 100.0, False])
        mock_pybldc_can.return_value.__enter__.return_value = mock_instance

        # Create a temporary binary file
        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(b"test binary data")
            temp_file.flush()

            with (
                patch(
                    "sys.argv",
                    ["pybldc", "can", "upload", "-id", "42", "-B", temp_file.name],
                ),
                pytest.raises(SystemExit) as exc_info,
                patch("logging.getLogger"),
            ):
                cli()

            assert exc_info.value.code == 1

            mock_instance.upload.assert_called_once()

    @patch("pybldc.pybldc.PyBldcCan")
    @patch("logging.getLogger")
    def test_cli_with_debug_flag(self, mock_get_logger: Mock, mock_pybldc_can: Mock) -> None:
        """Test CLI with debug flag sets logging level."""

        mock_logger = Mock()
        mock_get_logger.return_value = mock_logger

        with (
            patch("sys.argv", ["pybldc", "can", "ping", "-id", "42", "--debug"]),
            pytest.raises(SystemExit) as exc_info,
        ):
            cli()

        assert exc_info.value.code == 0
        mock_logger.setLevel.assert_called_with(logging.DEBUG)

    def test_cli_version(self) -> None:
        """Test CLI version flag."""
        with patch("sys.argv", ["pybldc", "--version"]), pytest.raises(SystemExit) as exc_info:
            cli()

        assert exc_info.value.code == 0


class MockPyBldcBase(PyBldcBase):
    """Mock implementation for testing base class methods"""

    def __init__(self, logger: logging.Logger) -> None:
        super().__init__(logger)
        self.ping_responses: list[bool] = []
        self.send_responses: list[bool | None] = []
        self.send_calls: list[tuple[list[int], list[int], float]] = []

    def shutdown(self, timeout: float | None = 1.0) -> None:
        pass

    def ping(self, timeout: float = 1.0) -> bool:
        if self.ping_responses:
            return self.ping_responses.pop(0)
        return True

    def _send_implementation(self, data: list[int], expected_response: list[int], timeout: float) -> bool | None:
        self.send_calls.append((data, expected_response, timeout))
        if self.send_responses:
            return self.send_responses.pop(0)
        return True


class TestUploadMethods:
    """Test upload functionality with comprehensive coverage."""

    @pytest.fixture
    def logger(self) -> logging.Logger:
        return logging.getLogger(self.__class__.__name__)

    @pytest.fixture
    def mock_bldc(self, logger: logging.Logger) -> MockPyBldcBase:
        return MockPyBldcBase(logger)

    def test_upload_invalid_attempts(self, mock_bldc: MockPyBldcBase) -> None:
        """Test upload with invalid attempts parameter"""
        with pytest.raises(ValueError, match='PyBldcBase: "attempts" has to be greater than 0'):
            list(mock_bldc.upload("test.bin", attempts=0))

    def test_upload_ping_failure_all_attempts(self, mock_bldc: MockPyBldcBase) -> None:
        """Test upload when ping fails repeatedly across all attempts"""
        mock_bldc.ping_responses = [False] * 9  # 3 ping_repeat * 3 attempts

        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(b"test_binary_data")
            temp_file.flush()

            results = list(mock_bldc.upload(temp_file.name, ping_repeat=3, attempts=3))
            assert results[-1] is False

    def test_upload_ping_success_after_retries(self, mock_bldc: MockPyBldcBase) -> None:
        """Test upload when ping succeeds after some retries"""
        mock_bldc.ping_responses = [False, False, True]  # Fail twice, then succeed
        mock_bldc.send_responses = [True, True]  # ERASE and WRITE succeed

        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(b"test_data")
            temp_file.flush()

            results = list(mock_bldc.upload(temp_file.name, ping_repeat=3))
            assert results[-1] is True

    def test_upload_small_binary_success(self, mock_bldc: MockPyBldcBase) -> None:
        """Test successful upload of small binary"""
        mock_bldc.ping_responses = [True]
        mock_bldc.send_responses = [True, True, None]  # ERASE, WRITE, JUMP_TO_BOOTLOADER

        binary_data = b"small_test_binary"
        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(binary_data)
            temp_file.flush()

            results = list(mock_bldc.upload(temp_file.name))

            # Check progress and final result
            progress_values = [r for r in results if isinstance(r, float)]
            assert progress_values[0] == 0.0
            assert progress_values[-1] == 100.0
            assert results[-1] is True

            # Verify the sequence of calls
            assert len(mock_bldc.send_calls) == 1 + 1 + 1  # ERASE + WRITE chunks + JUMP

            # Check ERASE_NEW_APP call
            erase_call = mock_bldc.send_calls[0]
            assert erase_call[0][0] == CommPacketId.COMM_ERASE_NEW_APP
            assert erase_call[0][1:5] == PyBldcBase._pack_uint32(len(binary_data))

            # Check WRITE_NEW_APP_DATA call
            write_call = mock_bldc.send_calls[1]
            assert write_call[0][0] == CommPacketId.COMM_WRITE_NEW_APP_DATA
            # Verify the data structure: offset + size + crc + binary
            expected_app_data = (
                PyBldcBase._pack_uint32(len(binary_data))
                + PyBldcBase._pack_uint16(crc16_ccitt(binary_data))
                + list(binary_data)
            )
            assert write_call[0][5:] == expected_app_data

            # Check JUMP_TO_BOOTLOADER call
            jump_call = mock_bldc.send_calls[2]
            assert jump_call[0][0] == CommPacketId.COMM_JUMP_TO_BOOTLOADER
            assert jump_call[2] == 0.0  # No timeout expected

    def test_upload_bootloader_binary(self, mock_bldc: MockPyBldcBase) -> None:
        """Test upload of bootloader binary"""
        mock_bldc.ping_responses = [True]
        mock_bldc.send_responses = [True, True]  # ERASE_BOOTLOADER, WRITE

        binary_data = b"bootloader_binary_data"
        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(binary_data)
            temp_file.flush()

            results = list(mock_bldc.upload(temp_file.name, is_bootloader=True))
            assert results[-1] is True

            # Verify ERASE_BOOTLOADER was called instead of ERASE_NEW_APP
            erase_call = mock_bldc.send_calls[0]
            assert erase_call[0][0] == CommPacketId.COMM_ERASE_BOOTLOADER

            # Verify no JUMP_TO_BOOTLOADER call for bootloader uploads
            jump_calls = [call for call in mock_bldc.send_calls if call[0][0] == CommPacketId.COMM_JUMP_TO_BOOTLOADER]
            assert len(jump_calls) == 0

    def test_upload_erase_failure(self, mock_bldc: MockPyBldcBase) -> None:
        """Test upload when erase command fails"""
        mock_bldc.ping_responses = [True]
        mock_bldc.send_responses = [False]  # ERASE fails

        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(b"test")
            temp_file.flush()

            results = list(mock_bldc.upload(temp_file.name))
            assert results[-1] is False

    def test_upload_write_failure(self, mock_bldc: MockPyBldcBase) -> None:
        """Test upload when write command fails"""
        mock_bldc.ping_responses = [True]
        mock_bldc.send_responses = [True, False]  # ERASE succeeds, WRITE fails

        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(b"test_data")
            temp_file.flush()

            results = list(mock_bldc.upload(temp_file.name))
            assert results[-1] is False

    def test_upload_multiple_chunks(self, mock_bldc: MockPyBldcBase) -> None:
        """Test upload with data requiring multiple chunks"""
        mock_bldc.ping_responses = [True]
        # Need success for ERASE + multiple WRITE calls + JUMP
        mock_bldc.send_responses = [True] * 10

        # Create binary requiring multiple 384-byte chunks
        large_data = b"A" * 1000
        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(large_data)
            temp_file.flush()

            results = list(mock_bldc.upload(temp_file.name))
            assert results[-1] is True

            # Verify the sequence of calls
            assert len(mock_bldc.send_calls) == 1 + (1000 // 384 + 1) + 1  # ERASE + WRITE chunks + JUMP

            # Check ERASE_NEW_APP call
            erase_call = mock_bldc.send_calls[0]
            assert erase_call[0][0] == CommPacketId.COMM_ERASE_NEW_APP
            assert erase_call[0][1:5] == PyBldcBase._pack_uint32(len(large_data))

            # Verify multiple WRITE_NEW_APP_DATA calls
            write_calls = [call for call in mock_bldc.send_calls if call[0][0] == CommPacketId.COMM_WRITE_NEW_APP_DATA]
            assert len(write_calls) == 3  # Should require 3 chunks
            assert write_calls[0][0][0] == CommPacketId.COMM_WRITE_NEW_APP_DATA
            # Verify the data structure: offset + size + crc + binary
            expected_app_data = (
                PyBldcBase._pack_uint32(len(large_data))
                + PyBldcBase._pack_uint16(crc16_ccitt(large_data))
                + list(large_data)
            )
            assert write_calls[0][0][5:] + write_calls[1][0][5:] + write_calls[2][0][5:] == expected_app_data

            # Check JUMP_TO_BOOTLOADER call
            jump_call = mock_bldc.send_calls[-1]
            assert jump_call[0][0] == CommPacketId.COMM_JUMP_TO_BOOTLOADER
            assert jump_call[2] == 0.0  # No timeout expected

    def test_upload_large_binary_compression(self, mock_bldc: MockPyBldcBase) -> None:
        """Test upload of large binary that requires compression"""
        mock_bldc.ping_responses = [True]
        mock_bldc.send_responses = [True] * 10

        # Create a binary larger than 393208 bytes to trigger compression
        large_data = b"A" * 400_000
        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(large_data)
            temp_file.flush()

            results = list(mock_bldc.upload(temp_file.name))
            assert results[-1] is True

            # Check ERASE_NEW_APP call
            erase_call = mock_bldc.send_calls[0]
            assert erase_call[0][0] == CommPacketId.COMM_ERASE_NEW_APP
            assert erase_call[0][1:5] == PyBldcBase._pack_uint32(len(large_data))

            # Verify multiple WRITE_NEW_APP_DATA calls
            write_calls = [call for call in mock_bldc.send_calls if call[0][0] == CommPacketId.COMM_WRITE_NEW_APP_DATA]
            n_write_calls = len(write_calls)  # Should require multiple chunks due to compression
            assert n_write_calls >= 1
            assert n_write_calls < 40_000 // 383  # Should be less than uncompressed chunks
            assert write_calls[0][0][0] == CommPacketId.COMM_WRITE_NEW_APP_DATA
            assert write_calls[0][0][1:5] == [0] * 4  # Offset is 0 for the first chunk

            # The size field is at bytes 5-8 in the packet
            # The actual compressed size is not known here, but we can check the marker
            # The size field should be len(compressed_data) + 0xCC
            assert write_calls[0][0][5] == 0xCC  # Compression marker present
            assert write_calls[0][0][6:9] != [0, 0, 0]  # Size should not be zero

            # Check JUMP_TO_BOOTLOADER call
            jump_call = mock_bldc.send_calls[-1]
            assert jump_call[0][0] == CommPacketId.COMM_JUMP_TO_BOOTLOADER
            assert jump_call[2] == 0.0  # No timeout expected

    def test_upload_too_large_after_compression(self) -> None:
        """Test upload failure when binary is too large even after compression"""
        logger = Mock(spec=logging.Logger)
        mock_bldc = MockPyBldcBase(logger)
        mock_bldc.ping_responses = [True]
        mock_bldc.send_responses = [True]  # ERASE succeeds

        # Create data that won't compress well and will still be too large
        # Use random-like data that doesn't compress well
        large_data = bytes(range(256)) * 2000  # ~500KB of data that won't compress much

        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(large_data)
            temp_file.flush()

            # Mock heatshrink2.encode to return data that's still too large
            with patch("heatshrink2.encode", return_value=b"X" * 400000):
                results = list(mock_bldc.upload(temp_file.name))
                assert results[-1] is False
                logger.info.assert_called_with("PyBldcBase: The firmware is too big even after compression")

    def test_upload_retry_attempts(self, mock_bldc: MockPyBldcBase) -> None:
        """Test upload with multiple attempts - first fails, second succeeds"""
        # First attempt: ping fails repeatedly
        # Second attempt: ping succeeds
        mock_bldc.ping_responses = [False, False, False, True]
        mock_bldc.send_responses = [True, True, None]

        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(b"test")
            temp_file.flush()

            results = list(mock_bldc.upload(temp_file.name, ping_repeat=3, attempts=2))
            assert results[-1] is True

    def test_upload_all_attempts_fail(self, mock_bldc: MockPyBldcBase) -> None:
        """Test upload when all attempts fail"""
        # All ping attempts fail
        mock_bldc.ping_responses = [False] * 6  # 3 ping_repeat * 2 attempts

        with tempfile.NamedTemporaryFile(suffix=".bin") as temp_file:
            temp_file.write(b"test")
            temp_file.flush()

            results = list(mock_bldc.upload(temp_file.name, ping_repeat=3, attempts=2))
            assert results[-1] is False


class TestWaitForPacketResponse:
    """Test _wait_for_packet_response method thoroughly."""

    @pytest.fixture
    def mock_bldc(self) -> MockPyBldcBase:
        logger = logging.getLogger("test")
        return MockPyBldcBase(logger)

    def test_wait_zero_timeout(self, mock_bldc: MockPyBldcBase) -> None:
        """Test with zero timeout returns None immediately"""
        packet_queue: queue.Queue[list[int]] = queue.Queue()
        result = mock_bldc._wait_for_packet_response(packet_queue, CommPacketId.COMM_REBOOT, [], 0.0)
        assert result is None

    def test_wait_success_immediate(self, mock_bldc: MockPyBldcBase) -> None:
        """Test successful response received immediately"""
        packet_queue: queue.Queue[list[int]] = queue.Queue()
        packet_queue.put([CommPacketId.COMM_ERASE_NEW_APP, 1, 0x12, 0x34])

        result = mock_bldc._wait_for_packet_response(packet_queue, CommPacketId.COMM_ERASE_NEW_APP, [0x12, 0x34], 1.0)
        assert result is True

    def test_wait_wrong_command_id(self, mock_bldc: MockPyBldcBase) -> None:
        """Test response with wrong command ID"""
        packet_queue: queue.Queue[list[int]] = queue.Queue()
        packet_queue.put([CommPacketId.COMM_REBOOT, 1, 0x12, 0x34])

        result = mock_bldc._wait_for_packet_response(packet_queue, CommPacketId.COMM_ERASE_NEW_APP, [0x12, 0x34], 0.1)
        assert result is False

    def test_wait_error_status(self, mock_bldc: MockPyBldcBase) -> None:
        """Test response with error status (not OK)"""
        packet_queue: queue.Queue[list[int]] = queue.Queue()
        packet_queue.put([CommPacketId.COMM_ERASE_NEW_APP, 0, 0x12, 0x34])  # Status = 0 (error)

        result = mock_bldc._wait_for_packet_response(packet_queue, CommPacketId.COMM_ERASE_NEW_APP, [0x12, 0x34], 1.0)
        assert result is False

    def test_wait_wrong_response_data(self, mock_bldc: MockPyBldcBase) -> None:
        """Test response with wrong expected data"""
        packet_queue: queue.Queue[list[int]] = queue.Queue()
        packet_queue.put([CommPacketId.COMM_ERASE_NEW_APP, 1, 0x12, 0x99])  # Wrong data

        result = mock_bldc._wait_for_packet_response(packet_queue, CommPacketId.COMM_ERASE_NEW_APP, [0x12, 0x34], 1.0)
        assert result is False

    def test_wait_timeout_empty_queue(self, mock_bldc: MockPyBldcBase) -> None:
        """Test timeout when no response is received"""
        packet_queue: queue.Queue[list[int]] = queue.Queue()

        result = mock_bldc._wait_for_packet_response(packet_queue, CommPacketId.COMM_ERASE_NEW_APP, [], 0.1)
        assert result is False

    def test_wait_success_with_threading(self, mock_bldc: MockPyBldcBase) -> None:
        """Test response received via background thread"""
        packet_queue: queue.Queue[list[int]] = queue.Queue()

        def delayed_response() -> None:
            time.sleep(0.1)
            packet_queue.put([CommPacketId.COMM_WRITE_NEW_APP_DATA, 1, 0xAB, 0xCD])

        # Start background thread to simulate delayed response
        thread = threading.Thread(target=delayed_response)
        thread.start()

        result = mock_bldc._wait_for_packet_response(
            packet_queue, CommPacketId.COMM_WRITE_NEW_APP_DATA, [0xAB, 0xCD], 1.0
        )

        thread.join()
        assert result is True


class TestCanSendImplementation:
    """Test CAN _send_implementation method comprehensively."""

    @pytest.fixture
    def logger(self) -> logging.Logger:
        return logging.getLogger(self.__class__.__name__)

    @patch("can.Bus")
    @patch("can.Notifier")
    def test_short_packet_format(self, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger) -> None:
        """Test CAN _send_implementation with short packet (<=6 bytes)"""
        mock_bus_instance = Mock()
        mock_bus.return_value = mock_bus_instance

        can_bldc = PyBldcCan(logger, controller_id=42)

        with patch.object(can_bldc, "_wait_for_packet_response", return_value=True):
            result = can_bldc._send_implementation([1, 2, 3, 4], [5, 6], 1.0)

        assert result is True
        mock_bus_instance.send.assert_called_once()

        # Verify the message structure for short packets
        sent_msg = mock_bus_instance.send.call_args[0][0]
        assert sent_msg.arbitration_id == 42 | (CanPacketId.CAN_PACKET_PROCESS_SHORT_BUFFER << 8)
        expected_data = [253, 0, 1, 2, 3, 4]  # id, send_flag=0, data
        assert list(sent_msg.data) == expected_data

    @patch("can.Bus")
    @patch("can.Notifier")
    @patch("time.sleep")  # Mock sleep to speed up tests
    def test_rx_buffer_format(
        self, mock_sleep: Mock, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger
    ) -> None:
        """Test CAN _send_implementation with long packet (>6 bytes)"""
        mock_bus_instance = Mock()
        mock_bus.return_value = mock_bus_instance

        can_bldc = PyBldcCan(logger, controller_id=42)

        # Create a packet longer than 6 bytes
        long_data = list(range(20))  # 20 bytes of data

        with patch.object(can_bldc, "_wait_for_packet_response", return_value=True):
            result = can_bldc._send_implementation(long_data, [], 1.0)

        assert result is True

        # Should send multiple CAN frames:
        # - Multiple FILL_RX_BUFFER (7 bytes at a time)
        # - Final PROCESS_RX_BUFFER
        assert mock_bus_instance.send.call_count == 3 + 1  # 3 * FILL + PROCESS

        first_call = mock_bus_instance.send.call_args_list[0][0][0]
        assert first_call.arbitration_id == 42 | (CanPacketId.CAN_PACKET_FILL_RX_BUFFER << 8)
        expected_first_data = [0, *long_data[:7]]  # id, first 7 bytes
        assert list(first_call.data) == expected_first_data

        second_call = mock_bus_instance.send.call_args_list[1][0][0]
        assert second_call.arbitration_id == 42 | (CanPacketId.CAN_PACKET_FILL_RX_BUFFER << 8)
        expected_second_data = [7, *long_data[7:14]]  # id, next 7 bytes
        assert list(second_call.data) == expected_second_data

        third_call = mock_bus_instance.send.call_args_list[2][0][0]
        assert third_call.arbitration_id == 42 | (CanPacketId.CAN_PACKET_FILL_RX_BUFFER << 8)
        expected_third_data = [14, *long_data[14:]]  # id, next 7 bytes
        assert list(third_call.data) == expected_third_data

        # Verify the final PROCESS_RX_BUFFER frame
        final_call = mock_bus_instance.send.call_args_list[-1][0][0]
        assert final_call == mock_bus_instance.send.call_args_list[3][0][0]
        assert final_call.arbitration_id == 42 | (CanPacketId.CAN_PACKET_PROCESS_RX_BUFFER << 8)

        # Should contain: id, send_flag, length (2 bytes), crc (2 bytes)
        expected_final_data = [
            253,
            0,  # id, send_flag
            *PyBldcBase._pack_uint16(len(long_data)),  # length
            *PyBldcBase._pack_uint16(crc16_ccitt(long_data)),  # crc
        ]
        assert list(final_call.data) == expected_final_data

    @patch("can.Bus")
    @patch("can.Notifier")
    @patch("time.sleep")  # Mock sleep to speed up tests
    def test_rx_buffer_long_format(
        self, mock_sleep: Mock, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger
    ) -> None:
        """Test CAN _send_implementation with long packet (>6 bytes)"""
        mock_bus_instance = Mock()
        mock_bus.return_value = mock_bus_instance

        can_bldc = PyBldcCan(logger, controller_id=42)

        # Create a packet longer than 1785 bytes
        long_data = [i % 256 for i in range(2_000)]

        with patch.object(can_bldc, "_wait_for_packet_response", return_value=True):
            result = can_bldc._send_implementation(long_data, [], 1.0)

        assert result is True

        # Should send multiple CAN frames:
        # - Multiple FILL_RX_BUFFER (7 bytes at a time)
        # - Multiple FILL_RX_BUFFER_LONG (6 at a time)
        # - Final PROCESS_RX_BUFFER
        nof_fill = 255 // 7 + 1
        nof_fill_long = (len(long_data) - 255) // 6 + 1
        assert mock_bus_instance.send.call_count == nof_fill + nof_fill_long + 1  # FILL + LONG + PROCESS

        # The first 255 bytes use FILL_RX_BUFFER
        for i in range(nof_fill):
            call = mock_bus_instance.send.call_args_list[i][0][0]
            assert call.arbitration_id == 42 | (CanPacketId.CAN_PACKET_FILL_RX_BUFFER << 8)
            index = i * 7
            expected_data = [index, *(long_data[index : index + 7])]
            assert list(call.data) == expected_data

        # Just to be sure, check the last FILL_RX_BUFFER call
        call = mock_bus_instance.send.call_args_list[nof_fill - 1][0][0]
        assert call.arbitration_id == 42 | (CanPacketId.CAN_PACKET_FILL_RX_BUFFER << 8)
        index = (nof_fill - 1) * 7
        expected_data = [index, *(long_data[index : index + 7])]
        assert list(call.data) == expected_data

        # The next bytes use FILL_RX_BUFFER_LONG
        for i in range(nof_fill_long):
            call = mock_bus_instance.send.call_args_list[nof_fill + i][0][0]
            assert call.arbitration_id == 42 | (CanPacketId.CAN_PACKET_FILL_RX_BUFFER_LONG << 8)
            index = (nof_fill * 7) + i * 6
            expected_data = [*PyBldcBase._pack_uint16(index), *(long_data[index : index + 6])]
            assert list(call.data) == expected_data

        # Just to be sure, check the last FILL_RX_BUFFER_LONG call
        call = mock_bus_instance.send.call_args_list[nof_fill + nof_fill_long - 1][0][0]
        assert call.arbitration_id == 42 | (CanPacketId.CAN_PACKET_FILL_RX_BUFFER_LONG << 8)
        index = (nof_fill * 7) + (nof_fill_long - 1) * 6
        expected_data = [*PyBldcBase._pack_uint16(index), *(long_data[index : index + 6])]
        assert list(call.data) == expected_data

        # Verify the final PROCESS_RX_BUFFER frame
        final_call = mock_bus_instance.send.call_args_list[-1][0][0]
        assert final_call == mock_bus_instance.send.call_args_list[nof_fill + nof_fill_long][0][0]
        assert final_call.arbitration_id == 42 | (CanPacketId.CAN_PACKET_PROCESS_RX_BUFFER << 8)

        # Should contain: id, send_flag, length (2 bytes), crc (2 bytes)
        expected_final_data = [
            253,
            0,  # id, send_flag
            *PyBldcBase._pack_uint16(len(long_data)),  # length
            *PyBldcBase._pack_uint16(crc16_ccitt(long_data)),  # crc
        ]
        assert list(final_call.data) == expected_final_data

    @patch("can.Bus")
    @patch("can.Notifier")
    def test_ping_functionality(self, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger) -> None:
        """Test CAN ping sends correct packet"""
        mock_bus_instance = Mock()
        mock_bus.return_value = mock_bus_instance

        can_bldc = PyBldcCan(logger, controller_id=42)
        can_bldc._can_listener.pong_event = Mock()
        can_bldc._can_listener.pong_event.wait.return_value = True

        result = can_bldc.ping(timeout=2.0)

        assert result is True
        can_bldc._can_listener.pong_event.clear.assert_called_once()
        can_bldc._can_listener.pong_event.wait.assert_called_once_with(2.0)

        # Verify ping packet was sent
        mock_bus_instance.send.assert_called_once()
        ping_msg = mock_bus_instance.send.call_args[0][0]
        assert ping_msg.arbitration_id == 42 | (CanPacketId.CAN_PACKET_PING << 8)
        assert list(ping_msg.data) == [253]  # CAN ID


class TestSerialSendImplementation:
    """Test Serial _send_implementation method comprehensively."""

    @pytest.fixture
    def logger(self) -> logging.Logger:
        return logging.getLogger(self.__class__.__name__)

    @patch("serial.Serial")
    def test_send_8bit_length_packet(self, mock_serial: Mock, logger: logging.Logger) -> None:
        """Test Serial _send_implementation with 8-bit length packet (<=255 bytes)"""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = True
        mock_serial.return_value = mock_serial_instance

        with patch("threading.Thread"):
            serial_bldc = PyBldcSerial(logger, port="/dev/ttyUSB0")

        test_data = [1, 2, 3, 4, 5]
        with patch.object(serial_bldc, "_wait_for_packet_response", return_value=True):
            result = serial_bldc._send_implementation(test_data, [], 1.0)

        assert result is True
        mock_serial_instance.write.assert_called_once()

        # Verify packet format: start(1) + length(1) + data + crc(2) + stop(1)
        sent_bytes = mock_serial_instance.write.call_args[0][0]
        assert sent_bytes[0] == 2  # Start byte for 8-bit length
        assert sent_bytes[1] == len(test_data)  # Length
        assert sent_bytes[2:7] == bytes(test_data)  # Data
        assert sent_bytes[7:9] == bytes(PyBldcBase._pack_uint16(crc16_ccitt(test_data)))  # CRC
        assert sent_bytes[9] == 3  # Stop byte

    @patch("serial.Serial")
    def test_send_16bit_length_packet(self, mock_serial: Mock, logger: logging.Logger) -> None:
        """Test Serial _send_implementation with 16-bit length packet (>255, <=65535 bytes)"""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = True
        mock_serial.return_value = mock_serial_instance

        with patch("threading.Thread"):
            serial_bldc = PyBldcSerial(logger, port="/dev/ttyUSB0")

        # Create data longer than 255 bytes
        test_data = [i % 256 for i in range(300)]

        with patch.object(serial_bldc, "_wait_for_packet_response", return_value=True):
            result = serial_bldc._send_implementation(test_data, [], 1.0)

        assert result is True

        # Verify 16-bit length format
        sent_bytes = mock_serial_instance.write.call_args[0][0]
        assert sent_bytes[0] == 3  # Start byte for 16-bit length
        length_bytes = PyBldcBase._pack_uint16(len(test_data))
        assert sent_bytes[1:3] == bytes(length_bytes)  # Length
        assert sent_bytes[3 : 3 + len(test_data)] == bytes(test_data)  # Data
        assert sent_bytes[3 + len(test_data) : -1] == bytes(PyBldcBase._pack_uint16(crc16_ccitt(test_data)))  # CRC
        assert sent_bytes[-1] == 3  # Stop byte

    @patch("serial.Serial")
    def test_send_24bit_length_packet(self, mock_serial: Mock, logger: logging.Logger) -> None:
        """Test Serial _send_implementation with 24-bit length packet (>65535 bytes)"""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = True
        mock_serial.return_value = mock_serial_instance

        with patch("threading.Thread"):
            serial_bldc = PyBldcSerial(logger, port="/dev/ttyUSB0")

        # Create data longer than 65535 bytes
        test_data = [0x42] * 66_000

        with patch.object(serial_bldc, "_wait_for_packet_response", return_value=True):
            result = serial_bldc._send_implementation(test_data, [], 1.0)

        assert result is True

        # Verify 24-bit length format
        sent_bytes = mock_serial_instance.write.call_args[0][0]
        assert sent_bytes[0] == 4  # Start byte for 24-bit length
        length_bytes = PyBldcBase._pack_uint24(len(test_data))
        assert sent_bytes[1:4] == bytes(length_bytes)  # Length
        assert sent_bytes[4 : 4 + len(test_data)] == bytes(test_data)  # Data
        assert sent_bytes[4 + len(test_data) : -1] == bytes(PyBldcBase._pack_uint16(crc16_ccitt(test_data)))  # CRC
        assert sent_bytes[-1] == 3  # Stop byte

    @patch("serial.Serial")
    def test_send_when_port_closed(self, mock_serial: Mock, logger: logging.Logger) -> None:
        """Test Serial _send_implementation when port is closed"""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = False
        mock_serial.return_value = mock_serial_instance

        with patch("threading.Thread"):
            serial_bldc = PyBldcSerial(logger, port="/dev/ttyUSB0")

        result = serial_bldc._send_implementation([1, 2, 3], [], 1.0)
        assert result is False
        mock_serial_instance.write.assert_not_called()

    @patch("serial.Serial")
    def test_ping_open_port(self, mock_serial: Mock, logger: logging.Logger) -> None:
        """Test Serial ping with open port"""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = True
        mock_serial.return_value = mock_serial_instance

        with patch("threading.Thread"):
            serial_bldc = PyBldcSerial(logger, port="/dev/ttyUSB0")

        result = serial_bldc.ping(timeout=1.0)
        assert result is True

    @patch("serial.Serial")
    @patch("time.sleep")
    def test_ping_closed_port(self, mock_sleep: Mock, mock_serial: Mock, logger: logging.Logger) -> None:
        """Test Serial ping with closed port includes sleep"""
        mock_serial_instance = Mock()
        mock_serial_instance.is_open = False
        mock_serial.return_value = mock_serial_instance

        with patch("threading.Thread"):
            serial_bldc = PyBldcSerial(logger, port="/dev/ttyUSB0")

        result = serial_bldc.ping(timeout=0.1)
        assert result is False
        mock_sleep.assert_called_once_with(0.1)


class TestCanListener:
    """Test PyBldcCanListener functionality."""

    @pytest.fixture
    def logger(self) -> logging.Logger:
        return logging.getLogger(self.__class__.__name__)

    @pytest.fixture
    def listener(self, logger: logging.Logger) -> PyBldcCanListener:
        return PyBldcCanListener(can_id=253, controller_id=42, logger=logger)

    def test_listener_initialization(self, listener: PyBldcCanListener) -> None:
        """Test listener initialization"""
        assert listener._id == 253
        assert listener._controller_id == 42
        assert not listener._is_stopped
        assert listener.hw_type is None
        assert listener.packet_queue.empty()
        assert not listener.pong_event.is_set()

    def test_process_short_buffer_valid(self, listener: PyBldcCanListener) -> None:
        """Test processing valid short buffer message"""
        msg = Mock()
        msg.is_error_frame = False
        msg.is_remote_frame = False
        msg.is_extended_id = True
        msg.arbitration_id = 253 | (CanPacketId.CAN_PACKET_PROCESS_SHORT_BUFFER << 8)
        msg.data = bytes([42, 1, 0x11, 0x22, 0x33])  # controller_id=42, send_flag=1, data
        msg.dlc = 5

        listener.on_message_received(msg)

        assert not listener.packet_queue.empty()
        received_data = listener.packet_queue.get_nowait()
        assert received_data == [0x11, 0x22, 0x33]

    def test_process_short_buffer_wrong_controller(self, listener: PyBldcCanListener) -> None:
        """Test ignoring short buffer with wrong controller ID"""
        msg = Mock()
        msg.is_error_frame = False
        msg.is_remote_frame = False
        msg.is_extended_id = True
        msg.arbitration_id = 253 | (CanPacketId.CAN_PACKET_PROCESS_SHORT_BUFFER << 8)
        msg.data = bytes([99, 1, 0x11, 0x22])  # Wrong controller_id
        msg.dlc = 4

        listener.on_message_received(msg)
        assert listener.packet_queue.empty()

    def test_process_short_buffer_wrong_send_flag(self, listener: PyBldcCanListener) -> None:
        """Test ignoring short buffer with wrong send flag"""
        msg = Mock()
        msg.is_error_frame = False
        msg.is_remote_frame = False
        msg.is_extended_id = True
        msg.arbitration_id = 253 | (CanPacketId.CAN_PACKET_PROCESS_SHORT_BUFFER << 8)
        msg.data = bytes([42, 0, 0x11, 0x22])  # send_flag=0 (not a response)
        msg.dlc = 4

        listener.on_message_received(msg)
        assert listener.packet_queue.empty()

    def test_pong_with_hw_type(self, listener: PyBldcCanListener) -> None:
        """Test pong message with hardware type"""
        msg = Mock()
        msg.is_error_frame = False
        msg.is_remote_frame = False
        msg.is_extended_id = True
        msg.arbitration_id = 253 | (CanPacketId.CAN_PACKET_PONG << 8)
        msg.data = bytes([42, HwType.HW_TYPE_VESC_BMS])
        msg.dlc = 2
        msg.__len__ = Mock(return_value=2)

        listener.on_message_received(msg)

        assert listener.pong_event.is_set()
        assert listener.hw_type == HwType.HW_TYPE_VESC_BMS

    def test_pong_without_hw_type(self, listener: PyBldcCanListener) -> None:
        """Test pong message without hardware type (older firmware)"""
        msg = Mock()
        msg.is_error_frame = False
        msg.is_remote_frame = False
        msg.is_extended_id = True
        msg.arbitration_id = 253 | (CanPacketId.CAN_PACKET_PONG << 8)
        msg.data = bytes([42])  # Only controller ID
        msg.dlc = 1
        msg.__len__ = Mock(return_value=1)

        listener.on_message_received(msg)

        assert listener.pong_event.is_set()
        assert listener.hw_type == HwType.HW_TYPE_VESC  # Default for older firmware

    def test_pong_wrong_controller(self, listener: PyBldcCanListener) -> None:
        """Test ignoring pong from wrong controller"""
        msg = Mock()
        msg.is_error_frame = False
        msg.is_remote_frame = False
        msg.is_extended_id = True
        msg.arbitration_id = 253 | (CanPacketId.CAN_PACKET_PONG << 8)
        msg.data = bytes([99])  # Wrong controller ID
        msg.dlc = 1
        msg.__len__ = Mock(return_value=1)

        listener.on_message_received(msg)

        assert not listener.pong_event.is_set()

    def test_ignore_invalid_frames(self, listener: PyBldcCanListener) -> None:
        """Test that invalid frames are properly ignored"""
        # Test error frame
        msg = Mock()
        msg.is_error_frame = True
        msg.is_remote_frame = False
        msg.is_extended_id = True
        listener.on_message_received(msg)
        assert listener.packet_queue.empty()

        # Test remote frame
        msg.is_error_frame = False
        msg.is_remote_frame = True
        listener.on_message_received(msg)
        assert listener.packet_queue.empty()

        # Test non-extended ID
        msg.is_error_frame = False
        msg.is_remote_frame = False
        msg.is_extended_id = False
        listener.on_message_received(msg)
        assert listener.packet_queue.empty()

    def test_ignore_wrong_target_id(self, listener: PyBldcCanListener) -> None:
        """Test ignoring messages for different target ID"""
        msg = Mock()
        msg.is_error_frame = False
        msg.is_remote_frame = False
        msg.is_extended_id = True
        msg.arbitration_id = 200 | (CanPacketId.CAN_PACKET_PONG << 8)  # Wrong target ID
        msg.data = bytes([42])
        msg.dlc = 1

        listener.on_message_received(msg)
        assert not listener.pong_event.is_set()
        assert listener.packet_queue.empty()

    def test_listener_stop_ignores_messages(self, listener: PyBldcCanListener) -> None:
        """Test that stopped listener ignores all messages"""
        listener.stop()
        assert listener._is_stopped

        msg = Mock()
        msg.is_error_frame = False
        msg.is_remote_frame = False
        msg.is_extended_id = True
        msg.arbitration_id = 253 | (CanPacketId.CAN_PACKET_PONG << 8)
        msg.data = bytes([42])
        msg.dlc = 1

        listener.on_message_received(msg)
        assert not listener.pong_event.is_set()

    def test_on_error_when_stopped(self, listener: PyBldcCanListener) -> None:
        """Test that errors are not logged when listener is stopped"""
        with patch.object(listener._logger, "exception") as mock_log:
            listener.stop()
            listener.on_error(Exception("test error"))
            mock_log.assert_not_called()

    def test_on_error_when_running(self, listener: PyBldcCanListener) -> None:
        """Test that errors are logged when listener is running"""
        with patch.object(listener._logger, "exception") as mock_log:
            listener.on_error(Exception("test error"))
            mock_log.assert_called_once()


class TestCanInitialization:
    """Test CAN interface initialization and validation."""

    @pytest.fixture
    def logger(self) -> logging.Logger:
        return logging.getLogger(self.__class__.__name__)

    def test_invalid_controller_id_negative(self, logger: logging.Logger) -> None:
        """Test that negative controller ID raises ValueError"""
        with pytest.raises(ValueError, match='PyBldcBase: "controller_id" has to be >=0 and <253'):
            PyBldcCan(logger, controller_id=-1)

    def test_invalid_controller_id_too_large(self, logger: logging.Logger) -> None:
        """Test that controller ID >= 253 raises ValueError"""
        with pytest.raises(ValueError, match='PyBldcBase: "controller_id" has to be >=0 and <253'):
            PyBldcCan(logger, controller_id=253)

        with pytest.raises(ValueError, match='PyBldcBase: "controller_id" has to be >=0 and <253'):
            PyBldcCan(logger, controller_id=255)

    @patch("can.Bus")
    @patch("can.Notifier")
    def test_valid_controller_id_boundary(self, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger) -> None:
        """Test valid controller IDs at boundaries"""
        # Test minimum valid ID
        can_bldc = PyBldcCan(logger, controller_id=0)
        assert can_bldc._controller_id == 0

        # Test maximum valid ID
        can_bldc = PyBldcCan(logger, controller_id=252)
        assert can_bldc._controller_id == 252

    @patch("can.Bus")
    @patch("can.Notifier")
    def test_can_initialization_parameters(self, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger) -> None:
        """Test CAN initialization with custom parameters"""
        PyBldcCan(
            logger=logger,
            controller_id=42,
            interface="test_interface",
            channel="test_channel",
            bitrate=1000000,
        )

        # Verify CAN bus was initialized with correct parameters
        mock_bus.assert_called_once_with(
            interface="test_interface",
            channel="test_channel",
            can_filters=[{"can_id": 253, "can_mask": 0xFF, "extended": True}],
            bitrate=1000000,
        )

    @patch("can.Bus")
    @patch("can.Notifier")
    def test_can_shutdown_none_timeout_error(self, mock_notifier: Mock, mock_bus: Mock, logger: logging.Logger) -> None:
        """Test that None timeout in shutdown raises ValueError"""
        can_bldc = PyBldcCan(logger, controller_id=42)

        with pytest.raises(ValueError, match='A timeout of "None" is not supported'):
            can_bldc.shutdown(timeout=None)


class TestPyBldcSerialReadThread:
    """Test the _serial_read_thread static method."""

    @pytest.fixture
    def logger(self) -> logging.Logger:
        """Create a logger for testing."""
        return logging.getLogger(self.__class__.__name__)

    @pytest.fixture
    def mock_serial(self) -> Mock:
        """Create a mock serial object."""
        mock = Mock()
        mock.is_open = True
        mock.read.return_value = b""
        return mock

    @pytest.fixture
    def shutdown_event(self) -> threading.Event:
        """Create a shutdown event."""
        return threading.Event()

    @pytest.fixture
    def packet_queue(self) -> queue.Queue[list[int]]:
        """Create a packet queue."""
        return queue.Queue()

    @staticmethod
    def _create_valid_packet(data: list[int]) -> bytearray:
        """Helper to create a valid packet with proper format."""
        if len(data) <= 255:
            packet = bytearray(
                [
                    2,  # Start of data
                    # 8-bit length
                    len(data),
                ]
            )
        elif len(data) <= 65535:
            packet = bytearray(
                [
                    3,  # Start of data
                    # 16-bit length
                    (len(data) >> 8) & 0xFF,
                    len(data) & 0xFF,
                ]
            )
        else:
            packet = bytearray(
                [
                    4,  # Start of data
                    # 24-bit length
                    (len(data) >> 16) & 0xFF,
                    (len(data) >> 8) & 0xFF,
                    len(data) & 0xFF,
                ]
            )

        packet.extend(data)
        crc = crc16_ccitt(data)
        packet.extend([(crc >> 8) & 0xFF, crc & 0xFF])  # CRC
        packet.append(3)  # Stop byte
        return packet

    def test_valid_8bit_packet(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test parsing a valid 8-bit length packet."""
        test_data = [0x01, 0x02, 0x03, 0x04]
        valid_packet = self._create_valid_packet(test_data)

        # Setup mock to return the packet and then shutdown
        mock_serial.read.side_effect = [bytes([b]) for b in valid_packet] + [b""] * 10

        # Run the thread in a separate thread and shut it down after a short time
        thread = threading.Thread(
            target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
        )
        thread.start()

        # Wait a bit for the packet to be processed
        parsed_packet = packet_queue.get(timeout=1.0)
        shutdown_event.set()
        thread.join(timeout=1.0)

        # Check that the packet was parsed correctly
        assert parsed_packet == test_data

    def test_valid_16bit_packet(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test parsing a valid 16-bit length packet."""
        test_data = [0x01] * 300  # Data larger than 255 bytes
        valid_packet = self._create_valid_packet(test_data)

        # Setup mock to return the packet and then shutdown
        mock_serial.read.side_effect = [bytes([b]) for b in valid_packet] + [b""] * 10

        # Run the thread in a separate thread and shut it down after a short time
        thread = threading.Thread(
            target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
        )
        thread.start()

        # Wait a bit for the packet to be processed
        parsed_packet = packet_queue.get(timeout=1.0)
        shutdown_event.set()
        thread.join(timeout=1.0)

        # Check that the packet was parsed correctly
        assert parsed_packet == test_data

    def test_valid_24bit_packet(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test parsing a valid 24-bit length packet."""
        test_data = [0xAA] * 66_000  # Data larger than 65535 bytes
        valid_packet = self._create_valid_packet(test_data)

        # Setup mock to return the packet and then shutdown
        mock_serial.read.side_effect = [bytes([b]) for b in valid_packet] + [b""] * 10

        # Run the thread in a separate thread and shut it down after a short time
        thread = threading.Thread(
            target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
        )
        thread.start()

        # Wait a bit for the packet to be processed
        parsed_packet = packet_queue.get(timeout=5.0)
        shutdown_event.set()
        thread.join(timeout=1.0)

        # Check that the packet was parsed correctly
        assert parsed_packet == test_data

    def test_invalid_start_byte(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test handling of invalid start bytes."""
        # Send invalid start byte followed by valid packet
        test_data = [0x01, 0x02]
        valid_packet = self._create_valid_packet(test_data)
        invalid_data = bytearray([0x01, 0x05, 0xFF]) + valid_packet  # Invalid start bytes

        mock_serial.read.side_effect = [bytes([b]) for b in invalid_data] + [b""] * 10

        with patch.object(logger, "warning") as mock_warning:
            thread = threading.Thread(
                target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
            )
            thread.start()

            # Wait a bit for the packet to be processed
            parsed_packet = packet_queue.get(timeout=1.0)
            shutdown_event.set()
            thread.join(timeout=1.0)

            # Should have logged warnings for invalid start bytes
            warning_calls = [call for call in mock_warning.call_args_list if "Discarding invalid data" in str(call)]
            assert len(warning_calls) >= 2

            # Should still parse the valid packet
            assert parsed_packet == test_data

    def test_invalid_data_length_8bit(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test handling of invalid data length in 8-bit packets."""
        invalid_packet = bytearray([2, 0])  # 8-bit packet with 0 length

        mock_serial.read.side_effect = [bytes([b]) for b in invalid_packet] + [b""] * 10

        with patch.object(logger, "warning") as mock_warning:
            thread = threading.Thread(
                target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
            )
            thread.start()

            # Wait a bit for the packet to be processed
            with pytest.raises(queue.Empty):
                packet_queue.get(timeout=1.0)
            shutdown_event.set()
            thread.join(timeout=1.0)

            # Should have logged warning for invalid data length
            warning_calls = [call for call in mock_warning.call_args_list if "Data len is not valid" in str(call)]
            assert len(warning_calls) >= 1

    def test_invalid_data_length_16bit(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test handling of invalid data length in 16-bit packets."""
        invalid_packet = bytearray([3, 0, 254])  # 16-bit packet with length < 255

        mock_serial.read.side_effect = [bytes([b]) for b in invalid_packet] + [b""] * 10

        with patch.object(logger, "warning") as mock_warning:
            thread = threading.Thread(
                target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
            )
            thread.start()

            # Wait a bit for the packet to be processed
            with pytest.raises(queue.Empty):
                packet_queue.get(timeout=1.0)
            shutdown_event.set()
            thread.join(timeout=1.0)

            # Should have logged warning for packet too short
            warning_calls = [call for call in mock_warning.call_args_list if "Packet is too short" in str(call)]
            assert len(warning_calls) >= 1

    def test_invalid_data_length_24bit(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test handling of invalid data length in 24-bit packets."""
        invalid_packet = bytearray([4, 0, 255, 254])  # 24-bit packet with length < 65535

        mock_serial.read.side_effect = [bytes([b]) for b in invalid_packet] + [b""] + [b""] * 10

        with patch.object(logger, "warning") as mock_warning:
            thread = threading.Thread(
                target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
            )
            thread.start()

            # Wait a bit for the packet to be processed
            with pytest.raises(queue.Empty):
                packet_queue.get(timeout=1.0)
            shutdown_event.set()
            thread.join(timeout=1.0)

            # Should have logged warning for packet too short
            warning_calls = [call for call in mock_warning.call_args_list if "Packet is too short" in str(call)]
            assert len(warning_calls) >= 1

    def test_invalid_crc(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test handling of packets with invalid CRC."""
        test_data = [0x01, 0x02, 0x03]
        packet = bytearray([2, len(test_data)])  # 8-bit length
        packet.extend(test_data)
        packet.extend([0xFF, 0xFF])  # Invalid CRC
        packet.append(3)  # Stop byte

        mock_serial.read.side_effect = [bytes([b]) for b in packet] + [b""] * 10

        with patch.object(logger, "warning") as mock_warning:
            thread = threading.Thread(
                target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
            )
            thread.start()

            # Wait a bit for the packet to be processed
            with pytest.raises(queue.Empty):
                packet_queue.get(timeout=1.0)
            shutdown_event.set()
            thread.join(timeout=1.0)

            # Should have logged warning for CRC failure
            warning_calls = [call for call in mock_warning.call_args_list if "CRC failed" in str(call)]
            assert len(warning_calls) >= 1

    def test_invalid_stop_byte(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test handling of packets with invalid stop byte."""
        test_data = [0x01, 0x02, 0x03]
        packet = bytearray([2, len(test_data)])  # 8-bit length
        packet.extend(test_data)
        crc = crc16_ccitt(test_data)
        packet.extend([(crc >> 8) & 0xFF, crc & 0xFF])  # Valid CRC
        packet.append(0xFF)  # Invalid stop byte (should be 3)

        mock_serial.read.side_effect = [bytes([b]) for b in packet] + [b""] * 10

        with patch.object(logger, "warning") as mock_warning:
            thread = threading.Thread(
                target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
            )
            thread.start()

            # Wait a bit for the packet to be processed
            with pytest.raises(queue.Empty):
                packet_queue.get(timeout=1.0)
            shutdown_event.set()
            thread.join(timeout=1.0)

            # Should have logged warning for invalid stop byte
            warning_calls = [call for call in mock_warning.call_args_list if "Invalid stop byte" in str(call)]
            assert len(warning_calls) >= 1

    def test_incomplete_packet_needs_more_data(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test handling of incomplete packets that need more data."""
        test_data = [0x01, 0x02, 0x03, 0x04]
        valid_packet = self._create_valid_packet(test_data)

        # Send packet in chunks, with pauses
        partial_reads = [bytes([b]) for b in valid_packet[:3]]  # Send only first 3 bytes initially
        remaining_reads = [bytes([b]) for b in valid_packet[3:]]  # Send rest later

        mock_serial.read.side_effect = partial_reads + [b""] * 5 + remaining_reads + [b""] * 10

        thread = threading.Thread(
            target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
        )
        thread.start()

        parsed_packet = packet_queue.get(timeout=1.0)
        shutdown_event.set()
        thread.join(timeout=1.0)

        # Should eventually parse the complete packet
        assert parsed_packet == test_data

    def test_multiple_packets_in_buffer(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test parsing multiple packets in the same buffer."""
        test_data1 = [0x01, 0x02]
        test_data2 = [0x03, 0x04, 0x05]

        packet1 = self._create_valid_packet(test_data1)
        packet2 = self._create_valid_packet(test_data2)
        combined_packet = packet1 + packet2

        mock_serial.read.side_effect = [bytes([b]) for b in combined_packet] + [b""] * 10

        thread = threading.Thread(
            target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
        )
        thread.start()

        parsed_packet1 = packet_queue.get(timeout=1.0)
        parsed_packet2 = packet_queue.get(timeout=1.0)
        shutdown_event.set()
        thread.join(timeout=1.0)

        # Should have parsed both packets
        assert parsed_packet1 == test_data1
        assert parsed_packet2 == test_data2

    def test_serial_port_closed_and_reopened(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test handling when serial port is closed and needs to be reopened."""
        mock_serial.is_open = False
        open_call_count = 0

        def mock_open() -> None:
            nonlocal open_call_count
            open_call_count += 1
            if open_call_count == 1:
                raise serial.serialutil.SerialException("Port not available")
            else:
                mock_serial.is_open = True

        mock_serial.open.side_effect = mock_open

        with patch.object(logger, "debug") as mock_debug, patch("time.sleep"):
            thread = threading.Thread(
                target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
            )
            thread.start()

            time.sleep(0.2)  # Allow more time for multiple open attempts
            shutdown_event.set()
            thread.join(timeout=1.0)

            # Should have attempted to open the port twice
            assert mock_serial.open.call_count == 2

            # Should have logged debug messages about failed open attempt
            debug_calls = [call for call in mock_debug.call_args_list if "Failed to open serial port" in str(call)]
            assert len(debug_calls) == 1

    def test_serial_exception_during_read(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test handling of SerialException during read operation."""
        mock_serial.read.side_effect = [
            serial.serialutil.SerialException("Device disconnected"),
            b"",
            b"",
            b"",  # Empty reads after exception
        ]

        with patch.object(logger, "debug") as mock_debug, patch("time.sleep"):
            thread = threading.Thread(
                target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
            )
            thread.start()

            time.sleep(0.1)
            shutdown_event.set()
            thread.join(timeout=1.0)

            # Should have closed the serial port and logged the exception
            mock_serial.close.assert_called()
            debug_calls = [
                call for call in mock_debug.call_args_list if "Caught SerialException exception" in str(call)
            ]
            assert len(debug_calls) == 1

    def test_partial_packet_interrupted_by_shutdown(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test graceful shutdown even with partial packet in buffer."""
        # Send only the start of a packet
        partial_packet = bytearray([2, 10])  # 8-bit packet expecting 10 bytes but we won't send them

        mock_serial.read.side_effect = [bytes([b]) for b in partial_packet] + [b""] * 10

        thread = threading.Thread(
            target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
        )
        thread.start()

        # Should not have any complete packets
        with pytest.raises(queue.Empty):
            packet_queue.get(timeout=0.05)
        shutdown_event.set()
        thread.join(timeout=1.0)

        # Should have exited cleanly without processing any packets
        assert packet_queue.empty()

    def test_general_exception_handling(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test that general exceptions are caught and logged."""
        mock_serial.read.side_effect = Exception("Unexpected error")

        with patch.object(logger, "exception") as mock_exception:
            thread = threading.Thread(
                target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
            )
            thread.start()

            with pytest.raises(queue.Empty):
                packet_queue.get(timeout=0.1)
            shutdown_event.set()
            thread.join(timeout=1.0)

            # Should have logged the exception
            exception_calls = [call for call in mock_exception.call_args_list if "Caught exception" in str(call)]
            assert len(exception_calls) >= 1

    def test_empty_read_data(
        self,
        mock_serial: Mock,
        shutdown_event: threading.Event,
        logger: logging.Logger,
        packet_queue: queue.Queue[list[int]],
    ) -> None:
        """Test handling of empty read data (no data available)."""
        mock_serial.read.return_value = b""

        thread = threading.Thread(
            target=PyBldcSerial._serial_read_thread, args=(mock_serial, shutdown_event, logger, packet_queue)
        )
        thread.start()

        with pytest.raises(queue.Empty):
            packet_queue.get(timeout=0.1)
        shutdown_event.set()
        thread.join(timeout=1.0)

        # Should handle empty reads gracefully
        assert packet_queue.empty()
        assert mock_serial.read.call_count > 0
