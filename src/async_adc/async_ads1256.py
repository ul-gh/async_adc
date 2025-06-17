"""Async implementation of ADS1256 and ADS1255 ADC driver for Raspberry Pi."""

import logging
import time
from collections.abc import MutableSequence, Sequence
from typing import ClassVar, Literal, Self

import pigpio  # pyright:ignore[reportMissingTypeStubs]

# This module only implements ADS1256 so this is imported as module-wide setting
from async_adc.ads1256_config import ADS1256Config, init_or_read_from_config_file
from async_adc.ads1256_definitions import Commands, Registers, StatusFlags

logger = logging.getLogger(__name__)

INT24_MIN = -0x800000
INT24_MAX = 0x7FFFFF
INT24_BYTES = 3

conf: ADS1256Config = init_or_read_from_config_file()


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
    spi_handle: int

    def __init__(self, pi: pigpio.pi | None = None) -> None:
        """ADS1256 initialization.

        Hardware pin configuration must be set at initialization phase.
        Register/Configuration Flag settings are initialized, but these
        can be changed during runtime.

        Config settings are read from ads1256_config.py.
        """
        # Set up the pigpio object if not provided as an argument
        if pi is None:
            self.pi = pigpio.pi()
            logger.info(self.pi)
            self.created_pigpio = True
        else:
            self.pi = pi
            self.created_pigpio = False
        if not self.pi.connected:
            msg = "Could not connect to hardware via pigpio library"
            raise OSError(msg)
        # Configure interfaces
        self._configure_gpios()
        self._configure_spi()
        # Device reset for defined initial state
        if conf.CHIP_HARD_RESET_ON_START:
            self.hard_reset()
        else:
            self.reset()
        # Configure ADC hardware registers
        self._configure_adc_registers()

    def __enter__(self) -> Self:
        """Async context manager entry method."""
        return self

    def __exit__(self, exc_type: type[BaseException] | None, *_: object) -> None:
        """Async context manager exit method."""
        if exc_type is not None:
            self.stop_close_all()
        else:
            self.stop()

    def check_chip_id(self) -> None:
        """Check if chip ID read via SPI matches the value defined in config.

        This raises a RuntimeError when the check fails.
        """
        chip_id = self.get_chip_id()
        logger.debug("Chip ID: %s", chip_id)
        if chip_id != conf.CHIP_ID:
            self.stop_close_all()
            msg = "Received wrong chip ID value for ADS1256. Hardware connected?"
            raise RuntimeError(msg)

    def stop(self) -> None:
        """Close own SPI handle, only stop pigpio connection if we created it."""
        logger.debug("Closing SPI handle: %s", self.spi_handle)
        self.pi.spi_close(self.spi_handle)
        self.open_spi_handles.remove(self.spi_handle)
        exclusive_pins = (conf.CS_PIN, conf.DRDY_PIN)
        for pin in exclusive_pins:
            if pin is not None:
                self.exclusive_pins_used.remove(pin)
        if self.created_pigpio:
            logger.debug("Closing PIGPIO instance")
            self.pi.stop()
        # Else leaving external PIGPIO instance active

    def stop_close_all(self) -> None:
        """Close all open pigpio SPI handles and stop pigpio connection."""
        for handle in reversed(self.open_spi_handles):
            logger.debug("Closing SPI handle: %s", handle)
            self.pi.spi_close(handle)
        self.open_spi_handles.clear()
        self.exclusive_pins_used.clear()
        logger.debug("Closing PIGPIO instance")
        self.pi.stop()

    def get_pga_gain(self) -> int:
        """Get ADC programmable gain amplifier setting."""
        gain_bits = 0b111 & self._read_reg_uint8(Registers.ADCON)
        return 2**gain_bits

    def set_pga_gain(self, value: int) -> None:
        """Set ADC programmable gain amplifier setting.

        The available options for the ADS1256 are:
        1, 2, 4, 8, 16, 32 and 64.

        This function sets the ADCON register with the code values
        defined in file ADS1256_definitions.py.

        Note: When changing the gain setting at runtime, with activated
        ACAL flag (AUTOCAL_ENABLE), this causes a _wait_drdy() timeout
        for the calibration process to finish.
        """
        self.pga_gain = value
        if value not in (1, 2, 4, 8, 16, 32, 64):
            self.stop_close_all()
            msg = "Argument must be one of: 1, 2, 4, 8, 16, 32, 64"
            raise ValueError(msg)
        log2val = int.bit_length(value) - 1
        adcon_old = self._read_reg_uint8(Registers.ADCON)
        self._write_reg_uint8(Registers.ADCON, adcon_old & 0b11111000 | log2val)
        status_flags = self._read_reg_uint8(Registers.STATUS)
        if status_flags & StatusFlags.AUTOCAL_ENABLE:
            self._wait_drdy()

    def get_v_per_digit(self) -> float:
        """Get ADC LSB weight in volts per numeric output digit.

        Readonly: This is a convenience value calculated from
        gain and v_ref setting.
        """
        return conf.v_ref * 2.0 / (self.get_pga_gain() * (2**23 - 1))

    def get_mux(self) -> int:
        """Get value of ADC analog input multiplexer register."""
        return self._read_reg_uint8(Registers.MUX)

    def set_mux(self, value: int) -> None:
        """Set value of ADC analog input multiplexer register.

        You can set any arbitrary pair of input pins
        as a differential input channel. For single-ended measurements,
        choose NEG_AINCOM as the second input pin.

        The most significant four bits select the positive input pin.
        The least significant four bits select the negative input pin.

        Example: set_mux(POS_AIN4 | NEG_AINCOM)

        IMPORTANT: When switching inputs during a running conversion cycle,
        invalid data is acquired.

        To prevent this, you must restart the conversion using the
        sync() function or the SYNC hardware pin before doing an
        async_read().

        The resulting delay can be avoided. See functions:

        read_continue()
            for cyclic reads of multiple channels at once - the ADC must not be
            reconfigured between invocations of this function, otherwise false
            data is read for the first value of the sequence

        read_sequence()
            for reading a succession of multiple channels at once, configuring
            all input channels including the first one for each cycle
        """
        self._write_reg_uint8(Registers.MUX, value)
        self.sync()

    def get_drate(self) -> int:
        """Get value of the ADC output sample data rate (reads DRATE register)."""
        return self._read_reg_uint8(Registers.DRATE)

    def set_drate(self, value: int) -> None:
        """Set value of the ADC output sample data (writes to DRATE register).

        This configures the hardware integrated moving average filter.

        When changing the register during a running acquisition,
        invalid data is sampled. In this case, call the sync() method
        to restart the acquisition cycle.

        The available data rates are defined in ADS1256_definitions.py.
        """
        self._write_reg_uint8(Registers.DRATE, value)
        status_flags = self._read_reg_uint8(Registers.STATUS)
        if status_flags & StatusFlags.AUTOCAL_ENABLE:
            self._wait_drdy()

    def get_ofc(self) -> int:
        """Get the offset compensation registers value, reading OFC0..2."""
        # Result is 24 bits int, transmitted with big endian bit and byte order
        return self._read_reg_int24(Registers.OFC0)

    def set_ofc(self, value: int) -> None:
        """Set the offset compensation registers value, setting OFC0..2."""
        if value < INT24_MIN or value > INT24_MAX:
            self.stop_close_all()
            msg = "Error: Offset value out of signed int24 range"
            raise ValueError(msg)
        self._write_reg_int24(Registers.OFC0, value)

    def get_fsc(self) -> int:
        """Get the full-scale adjustment registers value, reading OFC0..2."""
        return self._read_reg_int24(Registers.FSC0)

    def set_fsc(self, value: int) -> None:
        """Set the full-scale adjustment registers value, setting OFC0..2."""
        if value < 0 or value > INT24_MAX:
            self.stop_close_all()
            msg = "Error: Offset value must be positive in signed int24 range."
            raise ValueError(msg)
        self._write_reg_int24(Registers.FSC0, value)

    def get_chip_id(self) -> int:
        """Get the numeric ID from the ADS chip.

        Useful to check if hardware is connected.
        """
        self._wait_drdy()
        return self._read_reg_uint8(Registers.STATUS) >> 4

    def cal_self_offset(self) -> None:
        """Perform an input zero calibration using chip-internal reference switches.

        Sets the ADS1255/ADS1256 OFC register.
        """
        self._send_cmd(Commands.SELFOCAL)

    def cal_self_gain(self) -> None:
        """Perform an input full-scale calibration.

        This uses the chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 FSC register.
        """
        self._send_cmd(Commands.SELFGCAL)

    def cal_self(self) -> None:
        """Perform an input zero and full-scale two-point-calibration.

        This uses the chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 OFC and FSC registers.
        """
        self._send_cmd(Commands.SELFCAL)

    def cal_system_offset(self) -> None:
        """Perform in-system offset calibration.

        Set the ADS1255/ADS1256 OFC register such that the
        current input voltage corresponds to a zero output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        self._send_cmd(Commands.SYSOCAL)

    def cal_system_gain(self) -> None:
        """Perform in-system gain calibration.

        Set the ADS1255/ADS1256 FSC register such that the current
        input voltage corresponds to a full-scale output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        self._send_cmd(Commands.SYSGCAL)

    def standby(self) -> None:
        """Put chip in low-power standby mode."""
        self._chip_select()
        self.pi.spi_write(self.spi_handle, Commands.STANDBY.to_bytes())
        self._chip_release()

    def wakeup(self) -> None:
        """Wake up the chip from standby mode.

        See datasheet for settling time specifications after wake-up.
        Data is ready when the DRDY pin becomes active low.

        You can then use the read_oneshot() function to read a new
        sample of input data.

        Call standby() to enter standby mode again.
        """
        self._send_cmd(Commands.WAKEUP)

    def reset(self) -> None:
        """Reset all registers except CLK0 and CLK1 bits to default values."""
        self._send_cmd(Commands.RESET)

    def hard_reset(self) -> None:
        """Reset by toggling the hardware pin as configured as "RESET_PIN"."""
        if conf.RESET_PIN is None:
            self.stop_close_all()
            msg = "Reset pin is not configured!"
            raise RuntimeError(msg)
        logger.debug("Performing hard RESET...")
        self.pi.write(conf.RESET_PIN, pigpio.LOW)
        time.sleep(100e-6)
        self.pi.write(conf.RESET_PIN, pigpio.HIGH)
        # At hardware initialisation, a settling time for the oscillator
        # is necessary before doing any register access.
        # This is approx. 30ms, according to the datasheet.
        time.sleep(0.03)
        self._wait_drdy()

    def sync(self) -> None:
        """Restart the ADC conversion cycle with a SYNC + WAKEUP.

        The sequence is described in the ADS1256 datasheet.

        This is useful to restart the acquisition cycle after rapid
        changes of the input signals, for example when using an
        external input multiplexer or after changing ADC configuration
        flags.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_handle, Commands.SYNC.to_bytes())
        time.sleep(conf.SYNC_TIMEOUT)
        self.pi.spi_write(self.spi_handle, Commands.WAKEUP.to_bytes())
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def read_oneshot(self, diff_channel: int) -> int:
        """Restart/re-sync ADC and read the specified input pin pair.

        Arguments:  8-bit code value for differential input channel
                        (See definitions for the Registers.MUX register)
        Returns:    Signed integer conversion result

        Use this function after waking up from STANDBY mode.

        When switching inputs during a running conversion cycle,
        invalid data is acquired.

        To prevent this, this function automatically restarts the
        conversion cycle after configuring the input channels.

        The resulting delay can be avoided. See functions:

        read_continue()
            for cyclic reads of multiple channels at once - the ADC must not be
            reconfigured between invocations of this function, otherwise false
            data is read for the first value of the sequence

        read_sequence()
            for reading a succession of multiple channels at once, configuring
            all input channels including the first one for each cycle
        """
        self._chip_select()
        # Set input pin mux position for this cycle"
        self.pi.spi_write(
            self.spi_handle,
            (Commands.WREG | Registers.MUX, 0x00, diff_channel, Commands.SYNC),
        )
        time.sleep(conf.SYNC_TIMEOUT)
        self.pi.spi_write(self.spi_handle, (Commands.WAKEUP,))
        self._wait_drdy()
        # Read data from ADC, which still returns the /previous/ conversion
        # result from before changing inputs
        self.pi.spi_write(self.spi_handle, (Commands.RDATA,))
        time.sleep(conf.DATA_TIMEOUT)
        n_inbytes: int
        # pigpio library has wrong return type (str instead of bytearray) in case no bytes are read
        inbytes: bytearray | Literal[""]
        n_inbytes, inbytes = self.pi.spi_read(self.spi_handle, INT24_BYTES)
        if n_inbytes < INT24_BYTES or not isinstance(inbytes, bytearray):
            msg = "Read invalid data via SPI."
            raise OSError(msg)
        # Result is 24 bits int, transmitted with big endian bit and byte order
        return int.from_bytes(inbytes, byteorder="big", signed=True)

    def read_result(self) -> int | None:
        """Read previously started ADC conversion result.

        Arguments:  None
        Returns:    Signed integer ADC conversion result

        Issue this command to read a single conversion result for a
        previously set /and stable/ input channel configuration.

        For the default, free-running mode of the ADC, this means
        invalid data is returned when not synchronising acquisition
        and input channel configuration changes.

        To avoid this, after changing input channel configuration or
        with an external hardware multiplexer, use the hardware SYNC
        input pin or use the sync() method to restart the
        conversion cycle before calling read_async().

        Because this function does not implicitly restart a running
        acquisition, it is faster that the read_oneshot() method.
        """
        self._chip_select()
        # Wait for data to be ready
        self._wait_drdy()
        # Send the read command
        self.pi.spi_write(self.spi_handle, (Commands.RDATA,))
        time.sleep(conf.DATA_TIMEOUT)
        n_inbytes: int
        # pigpio library has wrong return type (str instead of bytearray) in case no bytes are read
        inbytes: bytearray | Literal[""]
        n_inbytes, inbytes = self.pi.spi_read(self.spi_handle, INT24_BYTES)
        self._chip_release()
        if n_inbytes < INT24_BYTES or not isinstance(inbytes, bytearray):
            msg = "Read invalid data via SPI."
            raise OSError(msg)
        # Result is 24 bits int, transmitted with big endian bit and byte order
        return int.from_bytes(inbytes, byteorder="big", signed=True)

    def read_result_set_next_inputs(self, diff_channel: int) -> int:
        """Read previously started ADC conversion result and set next pair of input channels.

        This reads data from finished or still running ADC conversion, and sets
        and synchronises new input channel config for next sequential read.

        Arguments:  8-bit code value for differential input channel
                        (See definitions for the Registers.MUX register)
        Returns:    Signed integer conversion result for present read

        This enables rapid dycling through different channels and
        implements the timing sequence outlined in the ADS1256
        datasheet (Sept.2013) on page 21, figure 19: "Cycling the
        ADS1256 Input Multiplexer".

        Note: In most cases, a fixed sequence of input channels is known
        beforehand. For that case, this module implements the function:

        read_sequence(ch_sequence)
            which automates the process for cyclic data acquisition.
        """
        self._chip_select()
        self._wait_drdy()
        # Setting mux position for next cycle"
        self.pi.spi_write(
            handle=self.spi_handle,
            data=(Commands.WREG | Registers.MUX, 0x00, diff_channel, Commands.SYNC),
        )
        time.sleep(conf.SYNC_TIMEOUT)
        self.pi.spi_write(handle=self.spi_handle, data=(Commands.WAKEUP,))
        # The datasheet is a bit unclear if a t_11 timeout is needed here.
        # Assuming the extra timeout is the safe choice:
        time.sleep(conf.T_11_TIMEOUT)
        # Read data from ADC, which still returns the /previous/ conversion
        # result from before changing inputs
        self.pi.spi_write(handle=self.spi_handle, data=(Commands.RDATA,))
        time.sleep(conf.DATA_TIMEOUT)
        n_inbytes: int
        # pigpio library has wrong return type (str instead of bytearray) in case no bytes are read
        inbytes: bytearray | Literal[""]
        n_inbytes, inbytes = self.pi.spi_read(self.spi_handle, INT24_BYTES)
        self._chip_release()
        if n_inbytes < INT24_BYTES or not isinstance(inbytes, bytearray):
            msg = "Read invalid data via SPI."
            raise OSError(msg)
        # Result is 24 bits int, transmitted with big endian bit and byte order
        return int.from_bytes(bytes=inbytes, byteorder="big", signed=True)

    def init_cycle(self, ch_sequence: Sequence[int]) -> None:
        """Set up a sequence of ADC input channel pin pairs and trigger conversion.

        This re-starts and re-syncs the ADC for the first sample.

        The results for a whole cycle can then be repeatedly read using the
        read_cycle() method.

        Argument1:  Sequence of 8-bit integer code values for differential
                    input channel pins to read sequentially in a cycle.
                    (See definitions for the Registers.MUX register)
        Example:    ch_sequence=(POS_AIN0|NEG_AIN1, POS_AIN2|NEG_AINCOM)
        """
        self.set_mux(ch_sequence[0])

    def read_cycle(self, ch_sequence: Sequence[int], ch_buffer: MutableSequence[int]) -> None:
        """Continues reading a cyclic sequence of ADC input channel pin pairs.

        The cycle must be first set up using the init_cycle() method.
        Otherwise, the first sample likely contains invalid data.

        Argument1:  Sequence of 8-bit integer code values for differential
                    input channel pins to read sequentially in a cycle.
                    (See definitions for the Registers.MUX register)
        Example:    ch_sequence=(POS_AIN0|NEG_AIN1, POS_AIN2|NEG_AINCOM)

        Argument2:  Buffer for the output samples, whih are signed 24-bit int.

        This implements the timing sequence outlined in the ADS1256
        datasheet (Sept.2013) on page 21, figure 19: "Cycling the
        ADS1256 Input Multiplexer" for cyclic data acquisition.
        """
        # Assuming we do not want a run-time check for performance reason..
        assert len(ch_buffer) >= len(ch_sequence)  # noqa: S101
        buf_len = len(ch_sequence)
        for i in range(buf_len):
            ch_buffer[i] = self.read_result_set_next_inputs(ch_sequence[(i + 1) % buf_len])

    # Delays until the configured DRDY input pin is pulled to
    # active logic low level by the ADS1256 hardware or until the
    # _DRDY_TIMEOUT has passed.
    #
    # The minimum necessary _DRDY_TIMEOUT when not using the hardware
    # pin, can be up to approx. one and a half second, see datasheet..
    #
    # This delay is also necessary when using the automatic calibration feature
    # (ACAL flag), after every access that changes the PGA gain bits in
    # ADCON register, the DRATE register or the BUFFEN flag in status register.
    def _wait_drdy(self) -> None:
        start = time.time()
        elapsed = time.time() - start
        # Waits for DRDY pin to go to active low or _DRDY_TIMEOUT seconds to pass
        if conf.DRDY_PIN is not None:
            drdy_level = self.pi.read(conf.DRDY_PIN)
            while (drdy_level == pigpio.HIGH) and (elapsed < conf.DRDY_TIMEOUT):
                elapsed = time.time() - start
                drdy_level = self.pi.read(conf.DRDY_PIN)
                # Sleep in order to avoid busy wait and reduce CPU load.
                time.sleep(conf.DRDY_DELAY)
            if elapsed >= conf.DRDY_TIMEOUT:
                logger.warning("Timeout while polling configured DRDY pin!")
        else:
            time.sleep(conf.DRDY_TIMEOUT)

    def _chip_select(self) -> None:
        # If chip select pin is hardwired to GND, do nothing.
        if conf.CS_PIN is not None:
            self.pi.write(conf.CS_PIN, pigpio.LOW)

    # Release chip select and implement t_11 timeout
    def _chip_release(self) -> None:
        if conf.CS_PIN is not None:
            time.sleep(conf.CS_TIMEOUT)
            self.pi.write(conf.CS_PIN, pigpio.HIGH)
        else:
            # The minimum t_11 timeout between commands, see datasheet Figure 1.
            time.sleep(conf.T_11_TIMEOUT)

    def _init_output(self, pin: int, init_state: int, name: str = "output") -> None:
        if pin is not None and pin not in self.pins_initialized:
            msg = "Setting as output: %s (%s)"
            logger.debug(msg, pin, name)
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, init_state)
            self.pins_initialized.add(pin)

    def _init_input(self, pin: int, pullup_mode: int = pigpio.PUD_UP, name: str = "input") -> None:
        if pin is not None and pin not in self.pins_initialized:
            msg = "Setting as output: %s (%s)"
            logger.debug(msg, pin, name)
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pullup_mode)
            self.pins_initialized.add(pin)

    def _configure_spi(self) -> None:
        # Configure SPI registers
        logger.debug("Activating SPI, SW chip select on GPIO: %s", conf.CS_PIN)
        # The ADS1256 uses SPI MODE=1 <=> CPOL=0, CPHA=1.
        #             bbbbbbRTnnnnWAuuupppmm
        spi_flags = 0b0000000000000011100001
        if conf.SPI_BUS == 1:
            #              bbbbbbRTnnnnWAuuupppmm
            spi_flags |= 0b0000000000000100000000
        # PIGPIO library returns a numeric handle for each chip on this bus.
        try:
            self.spi_handle = self.pi.spi_open(conf.SPI_CHANNEL, conf.SPI_FREQUENCY, spi_flags)
            self.check_chip_id()
        except Exception as e:
            logger.exception("SPI open error or wrong hardware setup. Abort.")
            self.stop_close_all()
            raise e from None
        # Add to class attribute
        self.open_spi_handles.append(self.spi_handle)
        logger.debug("Obtained SPI device handle: %s", self.spi_handle)

    def _configure_gpios(self) -> None:
        # Configure GPIOs
        #
        # For configuration of multiple SPI devices on this bus:
        # In order for the SPI bus to work at all, the chip select lines of
        # all slave devices on the bus must be initialized and set to inactive
        # level from the beginning. CS for all chips are given in config:
        # Initializing all other chip select lines as input every time.
        for pin in conf.CHIP_SELECT_GPIOS_INITIALIZE:
            self._init_input(pin, pigpio.PUD_UP, "chip select")
        # In addition to the CS pins of other devices of the bus, init CS for this chip
        if conf.CS_PIN is not None:
            if conf.CS_PIN in self.exclusive_pins_used:
                self.stop_close_all()
                msg = "CS pin already used. Must be exclusive!"
                raise ValueError(msg)
            if conf.CS_PIN not in conf.CHIP_SELECT_GPIOS_INITIALIZE:
                msg = (
                    "Chip select pins for all devices on the bus must be"
                    "listed in config: CHIP_SELECT_GPIOS_INITIALIZE"
                )
                raise ValueError(msg)
            self.exclusive_pins_used.add(conf.CS_PIN)
        # DRDY_PIN is the only GPIO input used by this ADC except from SPI pins
        if conf.DRDY_PIN is not None:
            if conf.DRDY_PIN in self.exclusive_pins_used:
                self.stop_close_all()
                msg = "Config error: DRDY pin already used. Must be exclusive!"
                raise ValueError(msg)
            self._init_input(conf.DRDY_PIN, pigpio.PUD_DOWN, "data ready")
        # GPIO Outputs. If chip select pin is set to None, the
        # respective ADC input pin is assumed to be hardwired to GND.
        if conf.RESET_PIN is not None:
            self._init_output(conf.RESET_PIN, pigpio.HIGH, "reset")
        if conf.PDWN_PIN is not None:
            self._init_output(conf.PDWN_PIN, pigpio.HIGH, "power down")

    def _configure_adc_registers(self) -> None:
        # Configure ADC registers:
        self._write_reg_uint8(Registers.MUX, conf.mux)
        adcon_flags: int = conf.clock_output | conf.sensor_detect | conf.pga_gain
        self._write_reg_uint8(Registers.ADCON, adcon_flags)
        self._write_reg_uint8(Registers.DRATE, conf.drate)
        # Status register written last as this can re-trigger ADC conversion
        self._write_reg_uint8(Registers.STATUS, conf.status)

    def _send_cmd(self, cmd: int) -> None:
        self._chip_select()
        self.pi.spi_write(self.spi_handle, cmd.to_bytes())
        self._wait_drdy()
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def _read_reg_bytes(self, register_start: int, count: int = 1) -> bytearray:
        """Return data bytes from the specified registers."""
        self._chip_select()
        self.pi.spi_write(handle=self.spi_handle, data=(Commands.RREG | register_start, count))
        time.sleep(conf.DATA_TIMEOUT)
        n_inbytes: int
        # pigpio library has wrong return type (str instead of bytearray) in case no bytes are read
        inbytes: bytearray | Literal[""]
        n_inbytes, inbytes = self.pi.spi_read(handle=self.spi_handle, count=count)
        # Release chip select and implement t_11 timeout
        self._chip_release()
        if n_inbytes < count or not isinstance(inbytes, bytearray):
            msg = "Read invalid data via SPI."
            raise OSError(msg)
        return inbytes

    def _write_reg_bytes(self, register_start: int, data: bytes) -> None:
        """Write data bytes to the specified registers."""
        self._chip_select()
        bytes_out = bytes((Commands.WREG | register_start, len(data))) + data
        self.pi.spi_write(handle=self.spi_handle, data=bytes_out)
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def _read_reg_uint8(self, register: int) -> int:
        """Read an unsigned 8-bit integer from register."""
        inbytes = self._read_reg_bytes(register)
        return int.from_bytes(inbytes)

    def _write_reg_uint8(self, register: int, value: int) -> None:
        """Write an unsigned 8-bit integer to register."""
        self._write_reg_bytes(register, value.to_bytes())

    def _read_reg_int24(self, register_start: int) -> int:
        """Read a signed 24-bit integer beginning at register_start."""
        inbytes = self._read_reg_bytes(register_start, INT24_BYTES)
        # Result is 24 bits int, transmitted with big endian bit and byte order
        return int.from_bytes(inbytes, byteorder="big", signed=True)

    def _write_reg_int24(self, register_start: int, value: int) -> None:
        """Write a signed 24-bit integer beginning at register_start."""
        # Input is 24 bits int, transmitted with big endian bit and byte order
        data = int.to_bytes(value, INT24_BYTES, byteorder="big", signed=True)
        self._write_reg_bytes(register_start, data)
