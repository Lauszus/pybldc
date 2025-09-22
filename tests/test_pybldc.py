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
from unittest.mock import Mock, patch

import can
import pytest

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

    def test_upload_invalid_attempts(self, bldc_base: ConcretePyBldcBase) -> None:
        """Test upload with invalid attempts parameter."""
        with pytest.raises(ValueError, match=r"attempts.*greater than 0"):
            list(bldc_base.upload("test.bin", attempts=0))

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
