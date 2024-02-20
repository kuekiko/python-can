"""
Interface for canable2 on slcan compatible interfaces (win32/linux).
"""

import io
import logging
import time
import warnings
from typing import Any, Optional, Tuple

from can import BusABC, CanProtocol, Message, typechecking

from ..exceptions import (
    CanInitializationError,
    CanInterfaceNotImplementedError,
    CanOperationError,
    error_check,
)

logger = logging.getLogger(__name__)

try:
    import serial
except ImportError:
    logger.warning(
        "You won't be able to use the canable2 can backend without "
        "the serial module installed!"
    )
    serial = None


class Canable2Bus(BusABC):
    """
    canable2 interface
    """

    # the supported bitrates and their commands
    _BITRATES = {
        10000: "S0",
        20000: "S1",
        50000: "S2",
        100000: "S3",
        125000: "S4",
        250000: "S5",
        500000: "S6",
        750000: "S7",
        1000000: "S8",
        83300: "S9",
    }
    # 支持的CAN FD数据比特率及其命令
    _FD_BITRATES = {
        1000000: "Y1",  # 1M
        2000000: "Y2",  # 2M (默认)
        4000000: "Y4",  # 4M
        5000000: "Y5",  # 5M
    }

    _SLEEP_AFTER_SERIAL_OPEN = 2  # in seconds

    _OK = b"\r"
    _ERROR = b"\a"

    LINE_TERMINATOR = b"\r"

    def __init__(
        self,
        channel: typechecking.ChannelStr,
        ttyBaudrate: int = 1000000, ##115200
        bitrate: Optional[int] = None,
        fd: bool = False,
        fd_bitrate: Optional[int] = None,
        btr: Optional[str] = None,
        sleep_after_open: float = _SLEEP_AFTER_SERIAL_OPEN,
        rtscts: bool = False,
        timeout: float = 0.001,
        **kwargs: Any,
    ) -> None:
        """
        :param str channel:
            port of underlying serial or usb device (e.g. ``/dev/ttyUSB0``, ``COM8``, ...)
            Must not be empty. Can also end with ``@115200`` (or similarly) to specify the baudrate.
        :param int ttyBaudrate:
            baudrate of underlying serial or usb device (Ignored if set via the ``channel`` parameter)
        :param bitrate:
            Bitrate in bit/s
        :param btr:
            BTR register value to set custom can speed
        :param poll_interval:
            Poll interval in seconds when reading messages
        :param sleep_after_open:
            Time to wait in seconds after opening serial connection
        :param rtscts:
            turn hardware handshake (RTS/CTS) on and off
        :param timeout:
            Timeout for the serial or usb device in seconds (default 0.001)
        :raise ValueError: if both ``bitrate`` and ``btr`` are set or the channel is invalid
        :raise CanInterfaceNotImplementedError: if the serial module is missing
        :raise CanInitializationError: if the underlying serial connection could not be established
        """
        if serial is None:
            raise CanInterfaceNotImplementedError("The serial module is not installed")

        if not channel:  # if None or empty
            raise ValueError("Must specify a serial port.")
        if "@" in channel:
            (channel, baudrate) = channel.split("@")
            ttyBaudrate = int(baudrate)

        with error_check(exception_type=CanInitializationError):
            self.serialPortOrig = serial.serial_for_url(
                channel,
                baudrate=ttyBaudrate,
                rtscts=rtscts,
                timeout=timeout,
            )

        self._buffer = bytearray()
        
        self._can_protocol = CanProtocol.CAN_20
        if fd:
            self._can_protocol = CanProtocol.CAN_FD
            ## 考虑是不是统一写在下面
            if fd_bitrate is not None:
                self.set_fd_bitrate(fd_bitrate)
            else:
                self.set_fd_bitrate(2000000) #暂时默认设置为2M 也可以直接报错
        
        time.sleep(sleep_after_open)

        with error_check(exception_type=CanInitializationError):
            if bitrate is not None and btr is not None:
                raise ValueError("Bitrate and btr mutually exclusive.")
            if bitrate is not None:
                self.set_bitrate(bitrate)
            if btr is not None:
                self.set_bitrate_reg(btr)
            self.open()

        super().__init__(
            channel,
            ttyBaudrate=1000000,
            bitrate=None,
            rtscts=False,
            **kwargs,
        )
    
    @property
    def fd(self) -> bool:
        class_name = self.__class__.__name__
        warnings.warn(
            f"The {class_name}.fd property is deprecated and superseded by "
            f"{class_name}.protocol. It is scheduled for removal in python-can version 5.0.",
            DeprecationWarning,
            stacklevel=2,
        )
        return self._can_protocol is CanProtocol.CAN_FD

    def set_bitrate(self, bitrate: int) -> None:
        """
        :param bitrate:
            Bitrate in bit/s

        :raise ValueError: if ``bitrate`` is not among the possible values
        """
        if bitrate in self._BITRATES:
            bitrate_code = self._BITRATES[bitrate]
        else:
            bitrates = ", ".join(str(k) for k in self._BITRATES.keys())
            raise ValueError(f"Invalid bitrate, choose one of {bitrates}.")

        self.close()
        self._write(bitrate_code)
        self.open()

    def set_fd_bitrate(self, fd_bitrate: int) -> None:
        """
        设置CAN FD数据比特率。
        """
        if fd_bitrate in self._FD_BITRATES:
            fd_bitrate_code = self._FD_BITRATES[fd_bitrate]
            self.close()
            self._write(fd_bitrate_code)
            self.open()
        else:
            raise ValueError("Unsupported CAN FD bitrate.")

    def set_bitrate_reg(self, btr: str) -> None:
        """
        :param btr:
            BTR register value to set custom can speed
        """
        self.close()
        self._write("s" + btr)
        self.open()

    def _write(self, string: str) -> None:
        with error_check("Could not write to serial device"):
            self.serialPortOrig.write(string.encode() + self.LINE_TERMINATOR)
            self.serialPortOrig.flush()

    def _read(self, timeout: Optional[float]) -> Optional[str]:
        _timeout = serial.Timeout(timeout)

        with error_check("Could not read from serial device"):
            while True:
                # Due to accessing `serialPortOrig.in_waiting` too often will reduce the performance.
                # We read the `serialPortOrig.in_waiting` only once here.
                in_waiting = self.serialPortOrig.in_waiting
                for _ in range(max(1, in_waiting)):
                    new_byte = self.serialPortOrig.read(size=1)
                    if new_byte:
                        self._buffer.extend(new_byte)
                    else:
                        break

                    if new_byte in (self._ERROR, self._OK):
                        string = self._buffer.decode()
                        self._buffer.clear()
                        return string

                if _timeout.expired():
                    break

            return None

    def flush(self) -> None:
        self._buffer.clear()
        with error_check("Could not flush"):
            self.serialPortOrig.reset_input_buffer()

    def open(self) -> None:
        self._write("O")

    def close(self) -> None:
        self._write("C")

    def _recv_internal(
        self, timeout: Optional[float]
    ) -> Tuple[Optional[Message], bool]:
        canId = None
        remote = False
        extended = False
        fd_frame = False
        brs = False
        data = None

        string = self._read(timeout)
        dlc = 0
        if not string:
            pass

        elif string[0] == "T":
            # extended frame 扩展数据帧
            canId = int(string[1:9], 16)
            hex_dlc = int(string[9],16) ##BUG?
            dlc = self.get_dlc(hex_dlc)
            extended = True
            data = bytearray.fromhex(string[10 : 10 + dlc * 2])
        elif string[0] == "t":
            # normal frame 标准数据帧
            canId = int(string[1:4], 16)
            hex_dlc = int(string[4],16)
            dlc = self.get_dlc(hex_dlc)
            data = bytearray.fromhex(string[5 : 5 + dlc * 2])
        elif string[0] == "r":
            # remote frame 标准远程帧
            canId = int(string[1:4], 16)
            hex_dlc = int(string[4],16)
            dlc = self.get_dlc(hex_dlc)
            remote = True
        elif string[0] == "R":
            # remote extended frame 扩展远程帧
            canId = int(string[1:9], 16)
            hex_dlc = int(string[9],16)
            dlc = self.get_dlc(hex_dlc)
            extended = True
            remote = True
        elif string[0] in ('d', 'D', 'b', 'B'):
            fd_frame = True
            brs = string[0] in ('b', 'B')
            if string[0] in ('d', 'b'):
                # CAN FD 标准ID
                canId = int(string[1:4], 16)
                hex_dlc = int(string[4], 16)
                dlc = self.get_dlc(hex_dlc)
                data = bytearray.fromhex(string[5:])
            else:
                # CAN FD 扩展ID
                canId = int(string[1:9], 16)
                hex_dlc = int(string[9], 16)
                dlc = self.get_dlc(hex_dlc)
                extended = True
                data = bytearray.fromhex(string[10:])

        if canId is not None:
            msg = Message(
                arbitration_id=canId,
                is_extended_id=extended,
                timestamp=time.time(),  # Better than nothing...
                is_remote_frame=remote,
                dlc=dlc,
                data=data,
                is_fd=fd_frame,
                bitrate_switch=brs,
            )
            return msg, False
        return None, False
    
    def get_dlc(self,hex_dlc):
        if hex_dlc <= 0x8:
            dlc = hex_dlc
        elif hex_dlc == 0x9:
            dlc = 12
        elif hex_dlc == 0xA:
            dlc = 16
        elif hex_dlc == 0xB:
            dlc = 20
        elif hex_dlc == 0xC:
            dlc = 24
        elif hex_dlc == 0xD:
            dlc = 32
        elif hex_dlc == 0xE:
            dlc = 48
        elif hex_dlc == 0xF:
            dlc = 64
        else:
            # 处理无效的 DLC 值
            dlc = 0
            raise ValueError(f"Invalid dlc, dlc =  {hex_dlc}.")
        return dlc

    def send(self, msg: Message, timeout: Optional[float] = None) -> None:
        if timeout != self.serialPortOrig.write_timeout:
            self.serialPortOrig.write_timeout = timeout
        
        # 构造发送字符串
        if msg.is_fd:  # 对于CAN FD帧
            if msg.is_extended_id:  # 扩展ID
                frame_format = 'B' if msg.bitrate_switch else 'D'
                arbitration_id_str = f"{msg.arbitration_id:08X}"
            else:  # 标准ID
                frame_format = 'b' if msg.bitrate_switch else 'd'
                arbitration_id_str = f"{msg.arbitration_id:03X}"
            sendStr = f"{frame_format}{arbitration_id_str}{msg.dlc:X}"
            if not msg.is_remote_frame:  # 远程帧不包含数据
                sendStr += msg.data.hex().upper()
        else:  # 对于标准CAN帧
            if msg.is_remote_frame:
                frame_format = 'R' if msg.is_extended_id else 'r'
            else:
                frame_format = 'T' if msg.is_extended_id else 't'
            arbitration_id_str = f"{msg.arbitration_id:08X}" if msg.is_extended_id else f"{msg.arbitration_id:03X}"
            sendStr = f"{frame_format}{arbitration_id_str}{msg.dlc:X}"
            if not msg.is_remote_frame:  # 远程帧不包含数据
                sendStr += msg.data.hex().upper()

        # if msg.is_remote_frame:
        #     if msg.is_extended_id:
        #         sendStr = f"R{msg.arbitration_id:08X}{msg.dlc:d}"
        #     else:
        #         sendStr = f"r{msg.arbitration_id:03X}{msg.dlc:d}"
        # else:
        #     if msg.is_extended_id:
        #         sendStr = f"T{msg.arbitration_id:08X}{msg.dlc:d}"
        #     else:
        #         sendStr = f"t{msg.arbitration_id:03X}{msg.dlc:d}"
        #     sendStr += msg.data.hex().upper()
        self._write(sendStr)

    def shutdown(self) -> None:
        super().shutdown()
        self.close()
        with error_check("Could not close serial socket"):
            self.serialPortOrig.close()

    def fileno(self) -> int:
        try:
            return self.serialPortOrig.fileno()
        except io.UnsupportedOperation:
            raise NotImplementedError(
                "fileno is not implemented using current CAN bus on this platform"
            ) from None
        except Exception as exception:
            raise CanOperationError("Cannot fetch fileno") from exception

    def get_version(
        self, timeout: Optional[float]
    ) -> Tuple[Optional[int], Optional[int]]:
        """Get HW and SW version of the canable2 interface.

        :param timeout:
            seconds to wait for version or None to wait indefinitely

        :returns: tuple (hw_version, sw_version)
            WHERE
            int hw_version is the hardware version or None on timeout
            int sw_version is the software version or None on timeout
        """
        cmd = "V"
        self._write(cmd)

        string = self._read(timeout)

        if not string:
            pass
        elif string[0] == cmd and len(string) == 6:
            # convert ASCII coded version
            hw_version = int(string[1:3])
            sw_version = int(string[3:5])
            return hw_version, sw_version

        return None, None

    def get_serial_number(self, timeout: Optional[float]) -> Optional[str]:
        """Get serial number of the canable2 interface.

        :param timeout:
            seconds to wait for serial number or :obj:`None` to wait indefinitely

        :return:
            :obj:`None` on timeout or a :class:`str` object.
        """
        cmd = "N"
        self._write(cmd)

        string = self._read(timeout)

        if not string:
            pass
        elif string[0] == cmd and len(string) == 6:
            serial_number = string[1:-1]
            return serial_number

        return None