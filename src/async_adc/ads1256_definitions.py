"""CONSTANT DEFINITIONS for ADS1256."""

from __future__ import annotations

import enum
from typing import Literal, Self, override


class Commands(enum.IntEnum):
    """Chip-level-command Definitions."""

    WAKEUP = 0x00
    """Completes SYNC and exits standby mode"""

    RDATA = 0x01
    """Read data
    
    Issue this command after DRDY goes low to read a single conversion result.
    After all 24 bits have been shifted out on DOUT, DRDY goes high.
    It is not necessary to read back all 24 bits, but DRDY will then not return high
    until new data is being updated.
    See the Timing Characteristics for the required delay between the end of the RDATA command and
    the beginning of shifting data on DOUT: t6.
    """

    RDATAC = 0x03
    """Start read data continuously"""

    SDATAC = 0x0F
    """Stop read data continuously"""

    RREG = 0x10
    """Read from register"""

    WREG = 0x50
    """Write to register"""

    SELFCAL = 0xF0
    """Offset and gain self-calibration"""

    SELFOCAL = 0xF1
    """Offset self-calibration"""

    SELFGCAL = 0xF2
    """Gain self-calibration"""

    SYSOCAL = 0xF3
    """System offset calibration"""

    SYSGCAL = 0xF4
    """System gain calibration"""

    SYNC = 0xFC
    """Synchronize the A/D conversion"""

    STANDBY = 0xFD
    """Begin standby mode"""

    RESET = 0xFE
    """Reset to power-on values"""


class InputChannelSelect(enum.IntEnum):
    """Input Channel Select."""

    AIN0 = 0x00
    AIN1 = 0x01
    AIN2 = 0x02
    AIN3 = 0x03
    AIN4 = 0x04
    AIN5 = 0x05
    AIN6 = 0x06
    AIN7 = 0x07
    AINCOM = 0x08

    @classmethod
    @override
    def _missing_(cls, value: object) -> Self:
        """Get an Input Channel Select from and Integer.

        This is necessary, because as stated in the Datasheet, if the bit at position 0x08 is high,
        The state of AINCOM is correct no matter the other bits.
        """
        if not isinstance(value, int):
            msg = "IntEnum can only be initialized with an Int."
            raise TypeError(msg)
        if value & cls.AINCOM:
            return cls(cls.AINCOM)
        # mask of all higher bits and initialize normally
        return cls(value & 0x07)


class ClockOutRateSetting(enum.IntEnum):
    """Clock Out Rate Setting."""

    CLKOUT_OFF = 0b00
    CLKOUT_EQUAL = 0b01
    CLKOUT_HALF = 0b10
    CLKOUT_FOURTH = 0b11


class SensorDetectCurrentSources(enum.IntEnum):
    """Sensor Detect Current Source settings."""

    SDCS_OFF = 0b00
    SDCS_500pA = 0b01
    SDCS_2uA = 0b10
    SDCS_10uA = 0b11


ProgrammableGainAmplifierValues = Literal[1, 2, 4, 8, 16, 32, 64]


class ProgrammableGainAmplifierSetting(enum.IntEnum):
    """Programmable Gain Amplifier settings."""

    GAIN_1 = 0b000
    GAIN_2 = 0b001
    GAIN_4 = 0b010
    GAIN_8 = 0b011
    GAIN_16 = 0b100
    GAIN_32 = 0b101
    GAIN_64 = 0b110

    @classmethod
    @override
    def _missing_(cls, value: object) -> Self:
        alternative_gain_64 = 0b111
        if value == alternative_gain_64:
            return cls(cls.GAIN_64)
        msg = "No valid value."
        raise ValueError(msg)

    def as_gain(self) -> ProgrammableGainAmplifierValues:
        """Get the integer ."""
        return 2**self.value

    @classmethod
    def from_gain(cls, data: ProgrammableGainAmplifierValues) -> Self:
        """Get an Programmable Gain Amplifier Setting from an int value.

        Necessary, because valid integer values are only the 2**x up until x=6.
        """
        log2val = int.bit_length(data) - 1
        return cls(log2val)


class DataRateSetting(enum.IntEnum):
    """Available Data Rates."""

    # REG_DRATE: Sample rate definitions:
    DRATE_30000 = 0b11110000  # 30,000SPS (default)
    DRATE_15000 = 0b11100000  # 15,000SPS
    DRATE_7500 = 0b11010000  # 7,500SPS
    DRATE_3750 = 0b11000000  # 3,750SPS
    DRATE_2000 = 0b10110000  # 2,000SPS
    DRATE_1000 = 0b10100001  # 1,000SPS
    DRATE_500 = 0b10010010  # 500SPS
    DRATE_100 = 0b10000010  # 100SPS
    DRATE_60 = 0b01110010  # 60SPS
    DRATE_50 = 0b01100011  # 50SPS
    DRATE_30 = 0b01010011  # 30SPS
    DRATE_25 = 0b01000011  # 25SPS
    DRATE_15 = 0b00110011  # 15SPS
    DRATE_10 = 0b00100011  # 10SPS
    DRATE_5 = 0b00010011  # 5SPS
    DRATE_2_5 = 0b00000011  # 2.5SPS
