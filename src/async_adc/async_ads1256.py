"""Async implementation of ADS1256 and ADS1255 ADC driver for Raspberry Pi."""

from __future__ import annotations

import asyncio
import logging
import time
from contextlib import contextmanager
from typing import TYPE_CHECKING, ClassVar, Literal, NoReturn, Self

import pigpio  # pyright:ignore[reportMissingTypeStubs]

from async_adc.ads1256_config import ADS1256Config
from async_adc.ads1256_definitions import (
    Commands,
    DataRateSetting,
    InputChannelSelect,
    ProgrammableGainAmplifierSetting,
    ProgrammableGainAmplifierValues,
)
from async_adc.registers import (
    AdControlRegister,
    AdDataRateRegister,
    FullScaleCalibrationRegister,
    GpioControlRegister,
    InputMultiplexerControlRegister,
    OffsetCalibrationRegister,
    PositiveNegativChannelPair,
    Register,
    StatusRegister,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Generator, Iterable

logger = logging.getLogger(__name__)

INT24_MIN = -0x800000
INT24_MAX = 0x7FFFFF
INT24_BYTES = 3


class ADS1256:
    """Async implementation of ADS1256 and ADS1255 ADC driver for Raspberry Pi.

    This is part of module PiPyADC
    Download: https://github.com/ul-gh/PiPyADC.

    Default pin and settings configuration is for the Open Hardware
    "Waveshare High-Precision AD/DA Board".

    See file ADS1256_default_config.py for
    configuration settings and description.

    Register read/write access is implemented via Python class/instance
    properties. Commands are implemented as functions.

    See help(ADS1256) for usage of the properties and functions herein.

    See ADS1256_definitions.py for chip registers, flags and commands.

    Documentation source: Texas Instruments ADS1255/ADS1256
    datasheet SBAS288: http://www.ti.com/lit/ds/sbas288j/sbas288j.pdf
    """

    # Pins and handles are unique on a system and thus stored as class variables
    open_spi_handles: ClassVar[list[int]] = []
    pins_initialized: ClassVar[set[int]] = set()
    exclusive_pins_used: ClassVar[set[int]] = set()

    status_register: StatusRegister
    input_multiplexer_control_register: InputMultiplexerControlRegister
    ad_control_register: AdControlRegister
    ad_data_rate_register: AdDataRateRegister
    gpio_control_register: GpioControlRegister
    offset_calibration_register: OffsetCalibrationRegister
    full_scale_calibration_register: FullScaleCalibrationRegister

    spi_handle: int
    pi: pigpio.pi

    conf: ADS1256Config

    data_ready_is_low: asyncio.Event

    lock: asyncio.Lock

    chip_selected: bool

    data_ready_callback: pigpio._callback  # pyright: ignore[reportPrivateUsage]

    ################### Initialize #############################################

    def _configure_spi(self) -> None:
        """Configure SPI registers."""
        logger.debug("Activating SPI, SW chip select on GPIO: %s", self.conf.CS_PIN)
        # The ADS1256 uses SPI MODE=1 <=> CPOL=0, CPHA=1.
        #             bbbbbbRTnnnnWAuuupppmm
        spi_flags = 0b0000000000000011100001
        if self.conf.SPI_BUS == 1:
            #              bbbbbbRTnnnnWAuuupppmm
            spi_flags |= 0b0000000000000100000000
        # PIGPIO library returns a numeric handle for each chip on this bus.
        try:
            spi_handle = self.pi.spi_open(self.conf.SPI_CHANNEL, self.conf.SPI_FREQUENCY, spi_flags)  # pyright: ignore[reportUnknownVariableType]

            self._check_chip_id()
        except Exception as e:
            logger.exception("SPI open error or wrong hardware setup. Abort.")
            self.stop_close_all()
            raise e from None

        if not isinstance(spi_handle, int):
            raise TypeError

        self.spi_handle = spi_handle
        # Add to class attribute
        self.open_spi_handles.append(self.spi_handle)
        logger.debug("Obtained SPI device handle: %s", self.spi_handle)

    def _configure_gpio_s(self) -> None:
        """Configure GPIOs.

        For configuration of multiple SPI devices on this bus:
        In order for the SPI bus to work at all, the chip select lines of
        all slave devices on the bus must be initialized and set to inactive
        level from the beginning. CS for all chips are given in config:
        Initializing all other chip select lines as input every time.
        """
        for pin in self.conf.CHIP_SELECT_GPIO_S_INITIALIZE:
            self._init_input(pin, pigpio.PUD_UP, "chip select")

        # In addition to the CS pins of other devices of the bus, init CS for this chip
        if self.conf.CS_PIN is not None:
            if self.conf.CS_PIN in self.exclusive_pins_used:
                self.stop_close_all()
                msg = "CS pin already used. Must be exclusive!"
                raise ValueError(msg)
            if self.conf.CS_PIN not in self.conf.CHIP_SELECT_GPIO_S_INITIALIZE:
                msg = (
                    "Chip select pins for all devices on the bus must be"
                    f"listed in config: {self.conf.CHIP_SELECT_GPIO_S_INITIALIZE=}"
                )
                raise ValueError(msg)
            self.exclusive_pins_used.add(self.conf.CS_PIN)

        # DRDY_PIN is the only GPIO input used by this ADC except from SPI pins
        if self.conf.DRDY_PIN is not None:
            if self.conf.DRDY_PIN in self.exclusive_pins_used:
                self.stop_close_all()
                msg = "Config error: DRDY pin already used. Must be exclusive!"
                raise ValueError(msg)
            self._init_input(self.conf.DRDY_PIN, pigpio.PUD_DOWN, "data ready")
            self.data_ready_callback = self.pi.callback(
                self.conf.DRDY_PIN,
                pigpio.EITHER_EDGE,
                self._data_ready_pin_callback(),
            )

        # GPIO Outputs. If chip select pin is set to None, the
        # respective ADC input pin is assumed to be hardwired to GND.
        if self.conf.RESET_PIN is not None:
            self._init_output(self.conf.RESET_PIN, pigpio.HIGH, "reset")

        if self.conf.PDWN_PIN is not None:
            self._init_output(self.conf.PDWN_PIN, pigpio.HIGH, "power down")

    def _configure_adc_registers(self) -> None:
        """Configure ADC registers."""
        self.status_register = StatusRegister()
        self.input_multiplexer_control_register = InputMultiplexerControlRegister()
        self.ad_control_register = AdControlRegister()
        self.ad_data_rate_register = AdDataRateRegister()
        self.gpio_control_register = GpioControlRegister()
        self.offset_calibration_register = OffsetCalibrationRegister()
        self.full_scale_calibration_register = FullScaleCalibrationRegister()

        self.input_multiplexer_control_register.negative_input_channel = (
            self.conf.negative_input_channel
        )
        self.input_multiplexer_control_register.positive_input_channel = (
            self.conf.positive_input_channel
        )
        asyncio.run(self._write_register(self.input_multiplexer_control_register))

        self.ad_control_register.clock_out_rate = self.conf.clock_output
        self.ad_control_register.sensor_detect_current = self.conf.sensor_detect
        self.ad_control_register.programmable_gain_amplifier = self.conf.pga_gain
        asyncio.run(self._write_register(self.ad_control_register))

        self.ad_data_rate_register.data_rate = self.conf.data_rate
        asyncio.run(self._write_register(self.ad_data_rate_register))

        # Status register written last as this can re-trigger ADC conversion
        self.status_register.analog_buffer = self.conf.buffer_enable
        asyncio.run(self._write_register(self.status_register))

    def __init__(self, pi: pigpio.pi, conf: ADS1256Config | None = None) -> None:
        """ADS1256 initialization.

        Hardware pin configuration must be set at initialization phase.
        Register/Configuration Flag settings are initialized, but these
        can be changed during runtime.

        Config settings are read from ads1256_config.py.
        """
        self.conf = conf or ADS1256Config()
        # Set up the pigpio object if not provided as an argument
        self.pi = pi
        if not self.pi.connected:
            msg = "Could not connect to hardware via pigpio library"
            raise OSError(msg)

        self.data_ready_is_low = asyncio.Event()
        self.lock = asyncio.Lock()
        self.chip_selected = False

        # Configure interfaces
        self._configure_gpio_s()
        self._configure_spi()
        # Device reset for defined initial state
        if self.conf.CHIP_HARD_RESET_ON_START:
            asyncio.run(self.hard_reset())
        else:
            asyncio.run(self.reset())

        # Configure ADC hardware registers
        self._configure_adc_registers()

    ################### Internal Methods #######################################

    def _data_ready_pin_callback(self) -> Callable[[int, int, int], None]:
        def callback(_gpio: int, _level: int, _tick: int) -> None:
            falling_edge_level = 0
            rising_edge_level = 1
            if _level == falling_edge_level:
                self.data_ready_is_low.set()
            elif _level == rising_edge_level:
                self.data_ready_is_low.clear()
            else:
                msg = (
                    "Callback was called with neither a rising nor a falling edge."
                    " A watchdog timeout was triggered."
                )
                raise ValueError(msg)

        return callback

    async def _wait_data_ready(self) -> None:
        """Wait for Data Ready.

        Delays until the configured DRDY input pin is pulled to
        active logic low level by the ADS1256 hardware or until the
        _DRDY_TIMEOUT has passed.

        The minimum necessary _DRDY_TIMEOUT when not using the hardware
        pin, can be up to approx. one and a half second, see datasheet..

        This delay is also necessary when using the automatic calibration feature
        (ACAL flag), after every access that changes the PGA gain bits in
        ADCON register, the DRATE register or the BUFFEN flag in status register.
        """
        # Waits for DRDY pin to go to active low or _DRDY_TIMEOUT seconds to pass
        if self.conf.DRDY_PIN is not None:
            try:
                _ = await asyncio.wait_for(self.data_ready_is_low.wait(), self.conf.DRDY_TIMEOUT)
            except TimeoutError:
                logger.exception("Timeout while waiting for DRDY pin to go low!")
        else:
            _ = await asyncio.sleep(self.conf.DRDY_TIMEOUT)

    @contextmanager
    def _select_chip(self) -> Generator[None, None, None]:
        # Enable Chip Select
        self.chip_selected = True
        # If no chip select pin is provided we can just continue
        if self.conf.CS_PIN is not None:
            self.pi.write(self.conf.CS_PIN, pigpio.LOW)
        # Go to inner context
        yield
        # Release Chip Select
        if self.conf.CS_PIN is not None:
            time.sleep(self.conf.CS_TIMEOUT)
            self.pi.write(self.conf.CS_PIN, pigpio.HIGH)
        self.chip_selected = False

    def _init_output(self, pin: int, init_state: int, name: str = "output") -> None:
        if pin not in self.pins_initialized:
            msg = "Setting as output: %s (%s)"
            logger.debug(msg, pin, name)
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, init_state)
            self.pins_initialized.add(pin)

    def _init_input(self, pin: int, pull_up_mode: int = pigpio.PUD_UP, name: str = "input") -> None:
        if pin not in self.pins_initialized:
            msg = "Setting as output: %s (%s)"
            logger.debug(msg, pin, name)
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pull_up_mode)
            self.pins_initialized.add(pin)

    def _write_to_spi(self, bytes_out: bytes) -> None:
        # check if at least t11 is over before sending next write
        if self.chip_selected:
            self.pi.spi_write(handle=self.spi_handle, data=bytes_out)
        else:
            with self._select_chip():
                self.pi.spi_write(handle=self.spi_handle, data=bytes_out)

    def _send_cmd(self, cmd: Commands) -> None:
        self._write_to_spi(cmd.to_bytes())

    async def _send_cmd_wait_data_ready(self, cmd: Commands) -> None:
        self._send_cmd(cmd)
        await self._wait_data_ready()

    def _read_from_spi(self, number_of_read_bytes: int) -> bytearray:
        n_input_bytes: int
        # pigpio library has wrong return type (str instead of bytearray) in case no bytes are read
        input_bytes: bytearray | Literal[""]
        if self.chip_selected:
            n_input_bytes, input_bytes = self.pi.spi_read(  # pyright:ignore[reportAny]
                handle=self.spi_handle,
                count=number_of_read_bytes,
            )
        else:
            with self._select_chip():
                n_input_bytes, input_bytes = self.pi.spi_read(  # pyright:ignore[reportAny]
                    handle=self.spi_handle,
                    count=number_of_read_bytes,
                )
        if n_input_bytes < number_of_read_bytes or not isinstance(input_bytes, bytearray):
            msg = "Read invalid data via SPI."
            raise OSError(msg)
        return input_bytes

    async def _read_data(self) -> int:
        # Send the read command
        self._send_cmd(Commands.RDATA)
        await asyncio.sleep(self.conf.DATA_TIMEOUT)
        # The result is 24 bits little endian two's complement
        return int.from_bytes(self._read_from_spi(INT24_BYTES), "little", signed=True)

    ################### Context Manager ########################################

    def __enter__(self) -> Self:
        """Async context manager entry method."""
        return self

    def __exit__(self, exc_type: type[BaseException] | None, *_: object) -> None:
        """Async context manager exit method."""
        if exc_type is not None:
            self.stop_close_all()

    def _check_chip_id(self) -> None:
        """Check if chip ID read via SPI matches the value defined in config.

        This raises a RuntimeError when the check fails.
        """
        chip_id = self.get_chip_id()
        logger.debug("Chip ID: %s", chip_id)
        if chip_id != self.conf.CHIP_ID:
            self.stop_close_all()
            msg = "Received wrong chip ID value for ADS1256. Correct hardware connected?"
            raise RuntimeError(msg)

    def __del__(self) -> None:
        """Close own SPI handle."""
        logger.debug("Closing SPI handle: %s", self.spi_handle)
        self.pi.spi_close(self.spi_handle)
        self.open_spi_handles.remove(self.spi_handle)
        if self.conf.CS_PIN is not None:
            self.exclusive_pins_used.remove(self.conf.CS_PIN)
        if self.conf.DRDY_PIN is not None:
            self.exclusive_pins_used.remove(self.conf.DRDY_PIN)

    def stop_close_all(self) -> None:
        """Close all open pigpio SPI handles and stop pigpio connection."""
        for handle in reversed(self.open_spi_handles):
            logger.debug("Closing SPI handle: %s", handle)
            self.pi.spi_close(handle)
        self.open_spi_handles.clear()
        self.exclusive_pins_used.clear()
        logger.debug("Closing PIGPIO instance")
        self.pi.stop()

    ################### IMPLEMENTED COMMANDS ###################################

    async def read_data(self) -> int:
        """Read previously started ADC conversion result.

        Returns:    Signed integer ADC conversion result

        Issue this command to read a single conversion result for a
        previously set /and stable/ input channel configuration.

        For the default, free-running mode of the ADC, this means
        invalid data is returned when not synchronizing acquisition
        and input channel configuration changes.

        To avoid this, after changing input channel configuration or
        with an external hardware multiplexer, use the hardware SYNC
        input pin or use the sync() method to restart the
        conversion cycle before calling this function.

        Because this function does not implicitly restart a running
        acquisition, it is faster that the read_one_shot() method.
        """
        async with self.lock:
            # Wait for data to be ready
            await self._wait_data_ready()
            return await self._read_data()

    @contextmanager
    def read_data_continuous(self) -> NoReturn:
        """Read Data Continuous Mode.

        ``Not yet implemented!``
        """
        # There are multiple different ways this could be implemented. Either through the use of an
        # Iterator, for example each new value will result in an yield. Problem is, this couldn't
        # be stopped. Another way this could be used is through the use of an context manager,
        # where inside of the context manager the ADC gets put into Continuous mode
        # and on exit it gets stopped. That way this would give an pythonic so to speak way of
        # interacting with the adc.
        # TODO(Alex): Discuss with Uli on how to implement this.
        raise NotImplementedError

    async def _write_register(self, register: Register) -> None:
        """Write data bytes to the specified registers."""
        data = register.value.to_bytes()
        assert len(data) == register.number_of_bytes  # noqa: S101
        self._write_to_spi(bytes((Commands.WREG | register.address, len(data) - 1)) + data)
        await asyncio.sleep(self.conf.T_11_TIMEOUT)
        if register.wait_for_auto_calibration() and self.status_register.auto_calibration:
            await self._wait_data_ready()

    async def _read_register(self, register: Register) -> None:
        """Get data from remote register."""
        self._write_to_spi(bytes((Commands.RREG | register.address, register.number_of_bytes - 1)))
        await asyncio.sleep(self.conf.DATA_TIMEOUT)
        register.set_value(
            self._read_from_spi(register.number_of_bytes),
        )

    async def self_calibration(self) -> None:
        """Perform an input zero and full-scale two-point-calibration.

        This uses the chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 OFC and FSC registers.
        """
        async with self.lock:
            await self._send_cmd_wait_data_ready(Commands.SELFCAL)

    async def self_offset_calibration(self) -> None:
        """Perform an input zero calibration using chip-internal reference switches.

        Sets the ADS1255/ADS1256 OFC register.
        """
        async with self.lock:
            await self._send_cmd_wait_data_ready(Commands.SELFOCAL)

    async def self_gain_calibration(self) -> None:
        """Perform an input full-scale calibration.

        This uses the chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 FSC register.
        """
        async with self.lock:
            await self._send_cmd_wait_data_ready(Commands.SELFGCAL)

    async def system_offset_calibration(self) -> None:
        """Perform in-system offset calibration.

        Set the ADS1255/ADS1256 OFC register such that the
        current input voltage corresponds to a zero output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        async with self.lock:
            await self._send_cmd_wait_data_ready(Commands.SYSOCAL)

    async def system_gain_calibration(self) -> None:
        """Perform in-system gain calibration.

        Set the ADS1255/ADS1256 FSC register such that the current
        input voltage corresponds to a full-scale output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        async with self.lock:
            await self._send_cmd_wait_data_ready(Commands.SYSGCAL)

    async def sync(self) -> None:
        """Restart the ADC conversion cycle with a SYNC + WAKEUP.

        The sequence is described in the ADS1256 datasheet.

        This is useful to restart the acquisition cycle after rapid
        changes of the input signals, for example when using an
        external input multiplexer or after changing ADC configuration
        flags.
        """
        async with self.lock:
            self._send_cmd(Commands.SYNC)
            await asyncio.sleep(self.conf.SYNC_TIMEOUT)
            self._send_cmd(Commands.WAKEUP)

    async def standby(self) -> None:
        """Put chip in low-power standby mode."""
        async with self.lock:
            self._send_cmd(Commands.STANDBY)

    async def wakeup(self) -> None:
        """Wake up the chip from standby mode.

        See datasheet for settling time specifications after wake-up.
        Data is ready when the DRDY pin becomes active low.

        You can then use the read_one_shot() function to read a new
        sample of input data.

        Call standby() to enter standby mode again.
        """
        async with self.lock:
            self._send_cmd(Commands.WAKEUP)

    async def reset(self) -> None:
        """Reset all registers except CLK0 and CLK1 bits to default values."""
        async with self.lock:
            self._send_cmd(Commands.RESET)

    ################### EXTRA METHODS ##########################################

    async def hard_reset(self) -> None:
        """Reset by toggling the hardware pin as configured as "RESET_PIN"."""
        async with self.lock:
            if self.conf.RESET_PIN is None:
                self.stop_close_all()
                msg = "Reset pin is not configured!"
                raise RuntimeError(msg)
            logger.debug("Performing hard RESET...")
            self.pi.write(self.conf.RESET_PIN, pigpio.LOW)
            await asyncio.sleep(100e-6)
            self.pi.write(self.conf.RESET_PIN, pigpio.HIGH)
            # At hardware initialization, a settling time for the oscillator
            # is necessary before doing any register access.
            # This is approx. 30ms, according to the datasheet.
            await asyncio.sleep(0.03)
            await self._wait_data_ready()

    def get_pga_gain(self) -> ProgrammableGainAmplifierValues:
        """Get ADC programmable gain amplifier setting."""
        return self.ad_control_register.programmable_gain_amplifier.as_gain()

    async def set_pga_gain(self, value: ProgrammableGainAmplifierValues) -> None:
        """Set ADC programmable gain amplifier setting.

        The available options for the ADS1256 are:
        1, 2, 4, 8, 16, 32 and 64.

        This function sets the ADCON register with the code values
        defined in file ADS1256_definitions.py.

        Note: When changing the gain setting at runtime, with activated
        ACAL flag (AUTOCAL_ENABLE), this causes a wait for data ready
        for the calibration process to finish.
        """
        async with self.lock:
            self.ad_control_register.programmable_gain_amplifier = (
                ProgrammableGainAmplifierSetting.from_gain(value)
            )
            await self._write_register(self.ad_control_register)

    def get_v_per_digit(self) -> float:
        """Get ADC LSB weight in volts per numeric output digit.

        Readonly: This is a convenience value calculated from
        gain and v_ref setting.
        """
        return self.conf.v_ref * 2.0 / (self.get_pga_gain() * (2**23 - 1))

    def get_mux(self) -> int:
        """Get value of ADC analog input multiplexer register."""
        return self.input_multiplexer_control_register.value

    async def select_input_channels(
        self,
        positive_channel: InputChannelSelect,
        negative_channel: InputChannelSelect,
    ) -> None:
        """Select input channels.

        You can set any arbitrary pair of input pins
        as a differential input channel. For single-ended measurements,
        choose NEG_AINCOM as the second input pin.
        """
        async with self.lock:
            self.input_multiplexer_control_register.positive_input_channel = positive_channel
            self.input_multiplexer_control_register.negative_input_channel = negative_channel
            await self._write_register(self.input_multiplexer_control_register)

    async def select_input_channel_pair(self, pair: PositiveNegativChannelPair) -> None:
        """Select a pair of input channels."""
        async with self.lock:
            self.input_multiplexer_control_register.set_channels(pair)
            await self._write_register(self.input_multiplexer_control_register)

    def get_data_rate(self) -> DataRateSetting:
        """Get value of the ADC output sample data rate (reads DRATE register)."""
        return self.ad_data_rate_register.data_rate

    async def set_data_rate(self, value: DataRateSetting) -> None:
        """Set value of the ADC output sample data (writes to DRATE register).

        This configures the hardware integrated moving average filter.

        When changing the register during a running acquisition,
        invalid data is sampled. In this case, call the sync() method
        to restart the acquisition cycle.

        The available data rates are defined in ADS1256_definitions.py.
        """
        async with self.lock:
            self.ad_data_rate_register.data_rate = value
            await self._write_register(self.ad_data_rate_register)

    async def get_ofc(self) -> int:
        """Get the offset compensation registers value, reading OFC0..2."""
        # The result is 24 bits little endian two's complement value by default
        async with self.lock:
            await self._read_register(self.offset_calibration_register)
            return self.offset_calibration_register.value

    async def set_ofc(self, value: int) -> None:
        """Set the offset compensation registers value, setting OFC0..2."""
        async with self.lock:
            if value < INT24_MIN or value > INT24_MAX:
                self.stop_close_all()
                msg = "Error: Offset value out of signed int24 range"
                raise ValueError(msg)
            self.offset_calibration_register.value = value
            await self._write_register(self.offset_calibration_register)

    async def get_fsc(self) -> int:
        """Get the full-scale adjustment registers value, reading OFC0..2."""
        async with self.lock:
            await self._read_register(self.full_scale_calibration_register)
            return self.full_scale_calibration_register.value

    async def set_fsc(self, value: int) -> None:
        """Set the full-scale adjustment registers value, setting OFC0..2."""
        async with self.lock:
            if value < 0 or value > INT24_MAX:
                self.stop_close_all()
                msg = "Error: Offset value must be positive in signed int24 range."
                raise ValueError(msg)
            self.full_scale_calibration_register.value = value
            await self._write_register(self.full_scale_calibration_register)

    def get_chip_id(self) -> int:
        """Get the numeric ID from the ADS chip.

        Useful to check if hardware is connected.
        """
        return self.status_register.chip_id

    async def read_one_shot(self) -> int:
        """Read one value from the ADC and send it back to sleep.

        Returns:    Signed integer conversion result for currently selected channel.

        Use this function after putting the device into standby.
        Then one conversion is done, and afterwards the device goes back to standby.
        """
        async with self.lock:
            self._send_cmd(Commands.WAKEUP)
            value = await self.read_data()
            await self.standby()
            return value

    async def read_sequence(self, input_pairs: Iterable[PositiveNegativChannelPair]) -> list[int]:
        """Read a sequence of input channel pairs."""
        async with self.lock:
            out: list[int] = []
            for pair in input_pairs:
                await self.select_input_channel_pair(pair)
                out.append(await self.read_data())
            return out
