"""Contains all available Registers with there commands."""

from __future__ import annotations

from abc import ABC
from typing import ClassVar, NamedTuple, final

# This module only implements ADS1256 so this is imported as module-wide setting
from async_adc.ads1256_definitions import (
    ClockOutRateSetting,
    DataRateSetting,
    InputChannelSelect,
    ProgrammableGainAmplifierSetting,
    SensorDetectCurrentSources,
)


class Register(ABC):
    """Represent one register on the ADS1256."""

    address: ClassVar[int]
    number_of_bytes: ClassVar[int] = 1

    value: int

    def __init__(self) -> None:
        self.value = 0

    _wait_for_auto_calibration: bool = False
    """If this is set to true, there should be a wait done to wait for the auto calibration."""

    def set_value(self, raw_data: bytearray) -> None:
        """Update local properties to be in sync with remote register."""
        self.value = int.from_bytes(raw_data)

    def wait_for_auto_calibration(self) -> bool:
        """Determine if wait for auto calibration is set and reset value."""
        value = self._wait_for_auto_calibration
        self._wait_for_auto_calibration = False
        return value


@final
class StatusRegister(Register):
    """Status Register."""

    address = 0x00

    @property
    def data_ready(self) -> bool:
        """Data Ready Indikator from register."""
        return bool(self.value & 0x1)

    @property
    def analog_buffer(self) -> bool:
        """Analog Buffer Enable."""
        return bool(self.value & 0x2)

    @analog_buffer.setter
    def analog_buffer(self, value: bool) -> None:
        self.value &= 0b11111101
        self.value |= int(value) << 1
        self._wait_for_auto_calibration = True

    @property
    def auto_calibration(self) -> bool:
        """Auto Calibration."""
        return bool(self.value & 0x4)

    @auto_calibration.setter
    def auto_calibration(self, value: bool) -> None:
        self.value &= 0b11111011
        self.value |= int(value) << 2
        self._wait_for_auto_calibration = True

    @property
    def order_msb_first(self) -> bool:
        """Order MSB first."""
        return bool(self.value & 0x8)

    @order_msb_first.setter
    def order_msb_first(self, value: bool) -> None:
        self.value &= 0b11110111
        self.value |= int(value) << 3

    @property
    def chip_id(self) -> int:
        """Chip id set by supplier."""
        return self.value >> 4


class PositiveNegativChannelPair(NamedTuple):
    """Positive and Negative Input Channel Pair."""

    positive: InputChannelSelect
    negative: InputChannelSelect


@final
class InputMultiplexerControlRegister(Register):
    """Mux Register."""

    address = 0x01

    @property
    def negative_input_channel(self) -> InputChannelSelect:
        """Negative Input Channel (AIN N )Select."""
        return InputChannelSelect(self.value)

    @negative_input_channel.setter
    def negative_input_channel(self, value: InputChannelSelect) -> None:
        self.value &= 0b11110000
        self.value |= value

    @property
    def positive_input_channel(self) -> InputChannelSelect:
        """Positive Input Channel (AIN P) Select."""
        return InputChannelSelect(self.value >> 4)

    @positive_input_channel.setter
    def positive_input_channel(self, value: InputChannelSelect) -> None:
        self.value &= 0b00001111
        self.value |= value << 4

    def set_channels(self, pair: PositiveNegativChannelPair) -> None:
        """Set positive and negative in one go."""
        self.value = pair.positive << 4 | pair.negative


@final
class AdControlRegister(Register):
    """A/D Control Register."""

    address = 0x02

    @property
    def programmable_gain_amplifier(self) -> ProgrammableGainAmplifierSetting:
        """Programmable Gain Amplifier."""
        return ProgrammableGainAmplifierSetting(self.value & 0b111)

    @programmable_gain_amplifier.setter
    def programmable_gain_amplifier(self, value: ProgrammableGainAmplifierSetting) -> None:
        self.value &= 0b00000111
        self.value |= value
        self._wait_for_auto_calibration = True

    @property
    def sensor_detect_current(self) -> SensorDetectCurrentSources:
        """Sensor Detect Current Sources.

        The Sensor Detect Current Sources can be activated to verify the integrity of an external
        sensor supplying a signal to the ADS1255/6.
        A shorted sensor produces a very small signal while an open-circuit sensor
        produces a very large signal.
        """
        return SensorDetectCurrentSources((self.value >> 3) & 0b11)

    @sensor_detect_current.setter
    def sensor_detect_current(self, value: SensorDetectCurrentSources) -> None:
        self.value &= 0b00011000
        self.value |= value << 3

    @property
    def clock_out_rate(self) -> ClockOutRateSetting:
        """D0/CLKOUT Clock Out Rate Setting."""
        return ClockOutRateSetting((self.value >> 5) & 0b11)

    @clock_out_rate.setter
    def clock_out_rate(self, value: ClockOutRateSetting) -> None:
        self.value &= 0b01100000
        self.value |= value << 5


@final
class AdDataRateRegister(Register):
    """A/D Data Rate Register."""

    address = 0x03

    @property
    def data_rate(self) -> DataRateSetting:
        """Data Rate Setting."""
        return DataRateSetting(self.value)

    @data_rate.setter
    def data_rate(self, value: DataRateSetting) -> None:
        self.value = value
        self._wait_for_auto_calibration = True


@final
class GpioControlRegister(Register):
    """GPIO Control Register."""

    address = 0x04


@final
class OffsetCalibrationRegister(Register):
    """Offset Calibration Register."""

    address = 0x05
    number_of_bytes = 3


@final
class FullScaleCalibrationRegister(Register):
    """Full-scale Calibration."""

    address = 0x09
    number_of_bytes = 3
