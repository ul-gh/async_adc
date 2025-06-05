import logging
import time
from typing import ClassVar, Self

import pigpio

from .ads1256_default_config import ADS1256Config
from .ads1256_definitions import Flags, Inputs, Registers, Commands

logger = logging.getLogger(__name__)


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
    open_spi_handles: ClassVar[dict[int]] = {}
    pins_initialized: ClassVar[dict[int]] = {}
    exclusive_pins_used: ClassVar[dict[int]] = {}

    def __init__(self, conf: ADS1256Config = ADS1256Config, pi: pigpio.pi = None) -> None:
        """ADS1256 initialization.

        Hardware pin configuration must be set at initialization phase.
        Register/Configuration Flag settings are initialized, but these
        can be changed during runtime via class properties.
        Default config is read from external file (module).
        """
        self.conf = conf
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

        # Configure GPIOs
        # For configuration of multiple SPI devices on this bus:
        if conf.CS_PIN in self.exclusive_pins_used:
            self.stop_close_all()
            msg = "CS pin already used. Must be exclusive!"
            raise ValueError(msg)
        self.exclusive_pins_used[conf.CS_PIN] = conf.CS_PIN
        # In order for the SPI bus to work at all, the chip select lines of
        # all slave devices on the bus must be initialized and set to inactive
        # level from the beginning. CS for all chips are given in config:
        if conf.CS_PIN not in conf.CHIP_SELECT_GPIOS_INITIALIZE:
            msg = "Chip select pins for all devices on the bus must be listed in config: CHIP_SELECT_GPIOS_INITIALIZE"
            raise ValueError(msg)
        # Initializing all chip select lines as input every time.
        for pin in conf.CHIP_SELECT_GPIOS_INITIALIZE:
            self._init_input(pin, pigpio.PUD_UP, "chip select")
        if conf.DRDY_PIN in self.exclusive_pins_used:
            self.stop_close_all()
            msg = "Config error: DRDY pin already used. Must be exclusive!"
            raise ValueError(msg)
        # DRDY_PIN is the only GPIO input used by this ADC except from SPI pins
        if conf.DRDY_PIN is not None:
            self._init_input(conf.DRDY_PIN, pigpio.PUD_DOWN, "data ready")
        # GPIO Outputs. If chip select pin is set to None, the
        # respective ADC input pin is assumed to be hardwired to GND.
        self._init_output(conf.RESET_PIN, pigpio.HIGH, "reset")
        self._init_output(conf.PDWN_PIN, pigpio.HIGH, "power down")

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
            self.spi_handle = self.pi.spi_open(0, conf.SPI_FREQUENCY, spi_flags)
            self.check_chip_id()
        except Exception as e:
            logger.exception("SPI open error")
            self.stop_close_all()
            raise e from None
        # Add to class attribute
        self.open_spi_handles[self.spi_handle] = self.spi_handle
        logger.debug("Obtained SPI device handle: %s", self.spi_handle)

        # Device reset for defined initial state
        if conf.CHIP_HARD_RESET_ON_START:
            self.hard_reset()
        else:
            self.reset()

        # Configure ADC registers:
        # Status register not yet set, only variable written to avoid multiple
        # triggering of the AUTOCAL procedure by changing other register flags
        self._status = conf.status
        self.mux = conf.mux
        self.write_reg(Registers.ADCON, conf.adcon)
        # FIXME: set PGA gain
        self.set_pga_gain(conf.pga_gain)
        self.drate = conf.drate
        self.gpio = conf.gpio
        self.status = conf.status

    def __enter__(self) -> Self:
        """Async context manager."""
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        """Async context manager."""
        if _exc_type is not None:
            self.stop_close_all()
        else:
            self.stop()

    def check_chip_id(self) -> None:
        chip_id = self.get_chip_id()
        logger.debug("Chip ID: %s", chip_id)
        if chip_id != self.conf.CHIP_ID:
            self.stop_close_all()
            msg = "Received wrong chip ID value for ADS1256. Hardware connected?"
            raise RuntimeError(msg)

    def stop(self) -> None:
        """Close own SPI handle, only stop pigpio connection if we created it."""
        logger.debug("Closing SPI handle: %s", self.spi_handle)
        self.pi.spi_close(self.spi_handle)
        self.open_spi_handles.pop(self.spi_handle)
        self.exclusive_pins_used.pop(self._CS_PIN)
        self.exclusive_pins_used.pop(self._DRDY_PIN)
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

    def set_pga_gain(self, value: int) -> None:
        """Set ADC programmable gain amplifier setting.

        The available options for the ADS1256 are:
        1, 2, 4, 8, 16, 32 and 64.

        This function sets the ADCON register with the code values
        defined in file ADS1256_definitions.py.

        Note: When changing the gain setting at runtime, with activated
        ACAL flag (AUTOCAL_ENABLE), this causes a _wait_DRDY() timeout
        for the calibration process to finish.
        """
        self.pga_gain = value
        if value not in (1, 2, 4, 8, 16, 32, 64):
            self.stop_close_all()
            msg = "Argument must be one of: 1, 2, 4, 8, 16, 32, 64"
            raise ValueError(msg)
        log2val = int.bit_length(value) - 1
        adcon = self.read_reg(Registers.ADCON) & 0b11111000 | log2val
        self.write_reg(Registers.ADCON, adcon)
        if self._status & Flags.AUTOCAL_ENABLE:
            self._wait_DRDY()

    def get_v_per_digit(self) -> float:
        """Get ADC LSB weight in volts per numeric output digit.

        Readonly: This is a convenience value calculated from
        gain and v_ref setting.
        """
        return self.conf.v_ref * 2.0 / (self.pga_gain * (2**23 - 1))

    def get_mux(self) -> int:
        """Get value of ADC analog input multiplexer register."""
        return self.read_reg(Registers.MUX)

    def set_mux(self, value: int) -> None:
        """Set value of ADC analog input multiplexer register.

        You can set any arbitrary pair of input pins
        as a differential input channel. For single-ended measurements,
        choose NEG_AINCOM as the second input pin.

        The most significant four bits select the positive input pin.
        The least significant four bits select the negative input pin.

        Example: set_mux(POS_AIN4 | NEG_AINCOM)

        IMPORTANT:
        When switching inputs during a running conversion cycle,
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
        self.write_reg(Registers.MUX, value)

    def get_drate(self) -> int:
        """Get value of the ADC output sample data rate (reads DRATE register)."""
        return self.read_reg(Registers.DRATE)

    def set_drate(self, value: int) -> None:
        """Set value of the ADC output sample data (writes to DRATE register).

        This configures the hardware integrated moving average filter.

        When changing the register during a running acquisition,
        invalid data is sampled. In this case, call the sync() method
        to restart the acquisition cycle.

        The available data rates are defined in ADS1256_definitions.py.
        """
        self.write_reg(Registers.DRATE, value)
        if self._status & Flags.AUTOCAL_ENABLE:
            self._wait_DRDY()

    @property
    def ofc(self):
        """Get/Set the three offset compensation registers, OFC0..2.
        This property is supposed to be a signed integer value.
        Gets/sets 24-bit two's complement value in three 8-bit-registers.
        """
        ofc0 = self.read_reg(Registers.OFC0)
        ofc1 = self.read_reg(Registers.OFC1)
        ofc2 = self.read_reg(Registers.OFC2)
        int24_result = ofc2 << 16 | ofc1 << 8 | ofc0
        # Take care of 24-Bit 2's complement.
        if int24_result < 0x800000:
            return int24_result
        else:
            return int24_result - 0x1000000

    @ofc.setter
    def ofc(self, value):
        value = int(value)
        if value < -0x800000 or value > 0x7FFFFF:
            self.stop_close_all()
            raise ValueError("Error: Offset value out of signed int24 range")
        else:
            # Generate 24-Bit 2's complement.
            if value < 0:
                value += 0x1000000
            # self._send_byte() automatically truncates to uint8
            self.write_reg(Registers.OFC0, value)
            value >>= 8
            self.write_reg(Registers.OFC1, value)
            value >>= 8
            self.write_reg(Registers.OFC2, value)

    @property
    def fsc(self):
        """Get/Set the three full-scale adjustment registers, OFC0..2.

        This property is supposed to be a positive integer value.
        Gets/sets 24-bit unsigned int value in three 8-bit-registers.
        """
        fsc0 = self.read_reg(Registers.FSC0)
        fsc1 = self.read_reg(Registers.FSC1)
        fsc2 = self.read_reg(Registers.FSC2)
        return fsc2 << 16 | fsc1 << 8 | fsc0

    @fsc.setter
    def fsc(self, value):
        value = int(value)
        if value < 0 or value > 0xFFFFFF:
            self.stop_close_all()
            raise ValueError("Error: This must be a positive int in 24-bit range")
        else:
            # self._send_byte() automatically truncates to uint8
            self.write_reg(Registers.FSC0, value)
            value >>= 8
            self.write_reg(Registers.FSC1, value)
            value >>= 8
            self.write_reg(Registers.FSC2, value)

    def get_chip_id(self) -> int:
        """Get the numeric ID from the ADS chip.

        Useful to check if hardware is connected.
        """
        self._wait_DRDY()
        return self.read_reg(Registers.STATUS) >> 4

    def read_regs(self, register_start: int, count: int = 1) -> bytes:
        """Return data byte from the specified register."""
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.RREG | register_start, count])
        time.sleep(self._DATA_TIMEOUT)
        count, inbytes = self.pi.spi_read(self.spi_handle, count)
        # Release chip select and implement t_11 timeout
        self._chip_release()
        return inbytes

    def write_regs(self, data: bytes, register_start: int, count: int = 1) -> None:
        """Write data byte to the specified registers."""
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.WREG | register_start, count, data])
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def cal_self_offset(self):
        """Perform an input zero calibration using chip-internal
        reference switches.

        Sets the ADS1255/ADS1256 OFC register.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.SELFOCAL])
        self._wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def cal_self_gain(self):
        """Perform an input full-scale calibration
        using chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 FSC register.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.SELFGCAL])
        self._wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def cal_self(self):
        """Perform an input zero and full-scale two-point-calibration
        using chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 OFC and FSC registers.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.SELFCAL])
        self._wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def cal_system_offset(self):
        """Set the ADS1255/ADS1256 OFC register such that the
        current input voltage corresponds to a zero output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.SYSOCAL])
        self._wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def cal_system_gain(self):
        """Set the ADS1255/ADS1256 FSC register such that the current
        input voltage corresponds to a full-scale output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.SYSGCAL])
        self._wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def standby(self):
        """Put chip in low-power standby mode."""
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.STANDBY])
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def wakeup(self) -> None:
        """Wake up the chip from standby mode.

        See datasheet for settling time specifications after wake-up.
        Data is ready when the DRDY pin becomes active low.

        You can then use the read_oneshot() function to read a new
        sample of input data.

        Call standby() to enter standby mode again.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.WAKEUP])
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def reset(self) -> None:
        """Reset all registers except CLK0 and CLK1 bits to default values."""
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.RESET])
        self._wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def hard_reset(self) -> None:
        """Reset by toggling the hardware pin as configured as "RESET_PIN"."""
        if self.conf.RESET_PIN is None:
            self.stop_close_all()
            msg = "Reset pin is not configured!"
            raise RuntimeError(msg)
        logger.debug("Performing hard RESET...")
        self.pi.write(self.conf.RESET_PIN, 0)
        time.sleep(100e-6)
        self.pi.write(self.conf.RESET_PIN, 1)
        # At hardware initialisation, a settling time for the oscillator
        # is necessary before doing any register access.
        # This is approx. 30ms, according to the datasheet.
        time.sleep(0.03)
        self._wait_DRDY()

    def sync(self) -> None:
        """Restart the ADC conversion cycle with a SYNC + WAKEUP.

        The sequence is described in the ADS1256 datasheet.

        This is useful to restart the acquisition cycle after rapid
        changes of the input signals, for example when using an
        external input multiplexer or after changing ADC configuration
        flags.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_handle, [Commands.SYNC])
        time.sleep(self._SYNC_TIMEOUT)
        self.pi.spi_write(self.spi_handle, [Commands.WAKEUP])
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def read_async(self) -> int:
        """Read ADC result as soon as possible.

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
        self._wait_DRDY()
        # Send the read command
        self.pi.spi_write(self.spi_handle, [Commands.RDATA])
        time.sleep(self.conf.DATA_TIMEOUT)
        # The result is 24 bits little endian two's complement value by default
        _count, inbytes = self.pi.spi_read(self.spi_handle, 3)
        self._chip_release()
        return int.from_bytes(inbytes, "big", signed=True)

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
        self.pi.spi_write(self.spi_handle, [Commands.WREG | Registers.MUX, 0x00, diff_channel, Commands.SYNC])
        time.sleep(self._SYNC_TIMEOUT)
        self.pi.spi_write(self.spi_handle, [Commands.WAKEUP])
        self._wait_DRDY()
        # Read data from ADC, which still returns the /previous/ conversion
        # result from before changing inputs
        self.pi.spi_write(self.spi_handle, [Commands.RDATA])
        time.sleep(self._DATA_TIMEOUT)
        # The result is 24 bits little endian two's complement value by default
        _count, inbytes = self.pi.spi_read(self.spi_handle, 3)
        self._chip_release()
        return int.from_bytes(inbytes, "big", signed=True)

    def read_and_next_is(self, diff_channel: int) -> int:
        """Read ADC data and set next pair of input channels.

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
        self._wait_DRDY()
        # Setting mux position for next cycle"
        self.pi.spi_write(self.spi_handle, [Commands.WREG | Registers.MUX, 0x00, diff_channel, Commands.SYNC])
        time.sleep(self.conf.SYNC_TIMEOUT)
        self.pi.spi_write(self.spi_handle, [Commands.WAKEUP])
        # The datasheet is a bit unclear if a t_11 timeout is needed here.
        # Assuming the extra timeout is the safe choice:
        time.sleep(self.conf.T_11_TIMEOUT)
        # Read data from ADC, which still returns the /previous/ conversion
        # result from before changing inputs
        self.pi.spi_write(self.spi_handle, [Commands.RDATA])
        time.sleep(self.conf.DATA_TIMEOUT)
        # The result is 24 bits little endian two's complement value by default
        _count, inbytes = self.pi.spi_read(self.spi_handle, 3)
        self._chip_release()
        return int.from_bytes(inbytes, "big", signed=True)

    def read_continue(self, ch_sequence, ch_buffer=None):
        """Continues reading a cyclic sequence of ADC input channel pin pairs.

        The first data sample is only valid if the ADC data register contains
        valid data from a previous conversion. I.e. the last element of the
        ch_sequence must be the first channel configuration to be read during
        the next following cycle.

        For short sequences, this is faster than the read_sequence() method
        because it does not interrupt an already running and pre-configured
        conversion cycle.

        Argument1:  Tuple (list) of 8-bit code values for differential
                    input channel pins to read sequentially in a cycle.
                    (See definitions for the Registers.MUX register)

                    Example:
                    ch_sequence=(POS_AIN0|NEG_AIN1, POS_AIN2|NEG_AINCOM)

        Argument2:  List (array, buffer) of signed integer conversion
                    results for the sequence of input channels.

        Returns:    List (array, buffer) of signed integer conversion
                    results for the sequence of input channels.

        This implements the timing sequence outlined in the ADS1256
        datasheet (Sept.2013) on page 21, figure 19: "Cycling the
        ADS1256 Input Multiplexer" for cyclic data acquisition.
        """
        buf_len = len(ch_sequence)
        if ch_buffer is None:
            ch_buffer = [0] * buf_len
        for i in range(0, buf_len):
            ch_buffer[i] = self.read_and_next_is(ch_sequence[(i + 1) % buf_len])
        return ch_buffer

    def read_sequence(self, ch_sequence: list, ch_buffer=None):
        """Reads a sequence of ADC input channel pin pairs.

        Restarts and re-syncs the ADC for the first sample.

        The time delay resulting from this can be avoided when reading
        the ADC in a cyclical pattern using the read_continue() method.

        Argument1:  Tuple (list) of 8-bit code values for differential
                    input channel pins to read sequentially in a cycle.
                    (See definitions for the Registers.MUX register)

                    Example:
                    ch_sequence=(POS_AIN0|NEG_AIN1, POS_AIN2|NEG_AINCOM)

        Argument2:  List (array, buffer) of signed integer conversion
                    results for the sequence of input channels.

        Returns:    List (array, buffer) of signed integer conversion
                    results for the sequence of input channels.

        This implements the timing sequence outlined in the ADS1256
        datasheet (Sept.2013) on page 21, figure 19: "Cycling the
        ADS1256 Input Multiplexer" for cyclic data acquisition.
        """
        self.mux = ch_sequence[0]
        self.sync()
        buf_len = len(ch_sequence)
        if ch_buffer is None:
            ch_buffer = [0] * buf_len
        for i in range(0, buf_len):
            ch_buffer[i] = self.read_and_next_is(ch_sequence[(i + 1) % buf_len])
        return ch_buffer

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
    def _wait_DRDY(self):
        start = time.time()
        elapsed = time.time() - start
        # Waits for DRDY pin to go to active low or _DRDY_TIMEOUT seconds to pass
        if self._DRDY_PIN is not None:
            drdy_level = self.pi.read(self._DRDY_PIN)
            while (drdy_level == 1) and (elapsed < self._DRDY_TIMEOUT):
                elapsed = time.time() - start
                drdy_level = self.pi.read(self._DRDY_PIN)
                # Sleep in order to avoid busy wait and reduce CPU load.
                time.sleep(self._DRDY_DELAY)
            if elapsed >= self._DRDY_TIMEOUT:
                logger.warning("Timeout while polling configured DRDY pin!")
        else:
            time.sleep(self._DRDY_TIMEOUT)

    def _chip_select(self):
        # If chip select pin is hardwired to GND, do nothing.
        if self._CS_PIN is not None:
            self.pi.write(self._CS_PIN, 0)

    # Release chip select and implement t_11 timeout
    def _chip_release(self):
        if self._CS_PIN is not None:
            time.sleep(self._CS_TIMEOUT)
            self.pi.write(self._CS_PIN, 1)
        else:
            # The minimum t_11 timeout between commands, see datasheet Figure 1.
            time.sleep(self._T_11_TIMEOUT)

    def _init_output(self, pin: int, init_state: int, name="output") -> None:
        if pin is not None and pin not in self.pins_initialized:
            msg = "Setting as output: %s (%s)"
            logger.debug(msg, pin, name)
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, init_state)
            self.pins_initialized[pin] = pin

    def _init_input(self, pin: int, pullup_mode: int = pigpio.PUD_UP, name="input") -> None:
        if pin is not None and pin not in self.pins_initialized:
            msg = "Setting as output: %s (%s)"
            logger.debug(msg, pin, name)
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pullup_mode)
            self.pins_initialized[pin] = pin
