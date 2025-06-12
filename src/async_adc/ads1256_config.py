"""Configuration file for ADS1256.

SPI bus configuration and GPIO pins used for the ADS1255/ADS1256.

These settings are compatible with the "Waveshare High-Precision AD/DA" board.

To create multiple class instances for more than one AD converter, a unique
configuration must be specified for each instance.
"""

from __future__ import annotations

import logging
import shutil
import sys
from dataclasses import dataclass
from importlib.resources import files
from pathlib import Path

from dataclass_binder import Binder

from .ads1256_definitions import (
    AdconFlags,
    DrateFlags,
    MuxFlags,
    StatusFlags,
)

logger = logging.getLogger(__name__)

CONFIG_FILE_NAME: str = "ads1256_config.toml"
DEFAULT_CONFIG_FILE_NAME: str = "ads1256_config_default.toml"


@dataclass
class ADS1256Config:
    """ADS1256 configuration settings."""

    # 0 for main SPI bus, 1 for auxiliary SPI bus.
    SPI_BUS: int[0, 1] = 0
    # Only releveant when using hardware chip select. (FIXME: Not implemented)
    # This determines which chip select pin is activated for this chip.
    # Main SPI has CS 0 and 1. Aux SPI has 0, 1, 2
    SPI_CHANNEL: int[0, 1, 2] = 0
    # SPI clock rate in Hz. The ADS1256 supports a minimum of 1/10th of the output
    # sample data rate in Hz to 1/4th of the oscillator CLKIN_FREQUENCY which
    # results in a value of 1920000 Hz for the Waveshare board. However, since
    # the Raspberry pi only supports power-of-two fractions of the 250MHz system
    # clock, the closest value would be 1953125 Hz, which is slightly out of spec
    # for the ADS1256. Choosing 250MHz/256 = 976563 Hz is a safe choice.
    SPI_FREQUENCY: int[976563, 1953125] = 976563
    # If set to True this will perform a chip reset using the hardware reset line
    # when initializing the device.
    CHIP_HARD_RESET_ON_START: bool = False
    # This is expected to be "3". Do not change unless this has changed.
    CHIP_ID: int = 3

    #### Raspberry Pi GPIO configuration #######################################
    # Raspberry Pi pinning uses Broadcom numbering scheme.

    # Tuple of all (chip select) GPIO numbers to be configured as an output and
    # initialised to (inactive) logic high state before bus communication starts.
    # Necessary for more than one SPI device if GPIOs are not otherwise handled.
    CHIP_SELECT_GPIOS_INITIALIZE: tuple[int] = (22, 23)
    # Chip select GPIO pin number.
    # Only relevant when using bit-banging chip-select mode.
    # Set to None to enable hardware chip select (must be supported by hardware)
    CS_PIN: int | None = 22
    # If DRDY is not connected to an input, a sufficient DRDY_TIMEOUT must be
    # specified further below and aquisition will be slower.
    DRDY_PIN: int = 17
    # Hardware reset pin is optional but strongly suggested in case multiple devices
    # are connected to the bus as the ADS125x will lock-up in case multiple chips
    # are selected simultaneously by accident.
    RESET_PIN: int | None = 18  # Set to None if not used.
    # Optional power down pin
    PDWN_PIN: int = 27  # Set to None if not used.
    ############################################################################

    ##############  ADS1256 Default Runtime Adjustable Properties  #############
    # Analog reference voltage between VREFH and VREFN pins
    v_ref: float = 2.5
    # Gain seting of the integrated programmable amplifier. This value must be
    # one of 1, 2, 4, 8, 16, 32, 64.
    # Gain = 1, V_ref = 2.5V ==> full-scale input voltage = 5.00V, corresponding
    # to a 24-bit two's complement output value of 2**23 - 1 = 8388607
    pga_gain: AdconFlags = AdconFlags.GAIN_1
    ############################################################################

    ####################  ADS1256 Default Register Settings  ###################
    # REG_STATUS:
    # When enabling the AUTOCAL flag: Any following operation that changes
    # PGA GAIN, DRATE or BUFFER flags triggers a self calibration:
    # THIS REQUIRES an additional timeout via WaitDRDY() after each such operation.
    # Note: BUFFER_ENABLE means the ADC input voltage range is limited
    # to (AVDD-2V),see datasheet
    status: StatusFlags = StatusFlags.BUFFER_ENABLE
    # REG_MUX:
    # Default: positive input = AIN0, negative input = AINCOM
    mux: MuxFlags = MuxFlags.POS_AIN0 | MuxFlags.NEG_AINCOM
    # REG_ADCON:
    # Disable clk out signal (if not needed, source of disturbance),
    # sensor detect current sources disabled:
    adcon: AdconFlags = AdconFlags.CLKOUT_OFF | AdconFlags.SDCS_OFF
    # REG_DRATE:
    # 10 SPS places a filter zero at 50 Hz and 60 Hz for line noise rejection
    drate: DrateFlags = DrateFlags.DRATE_10
    # REG_IO: No ADS1256 GPIOs needed
    gpio: int = 0x00
    ############################################################################

    # ADS1256 Constants.
    #
    # Master clock rate in Hz. Depends on the hardware. Default is 7680000:
    CLKIN_FREQUENCY: int = 7680000
    # Seconds to wait in case DRDY pin is not connected or the chip
    # does not respond. See table 21 of ADS1256 datasheet: When using a
    # sample rate of 2.5 SPS and issuing a self calibration command,
    # the timeout can be up to 1228 milliseconds:
    DRDY_TIMEOUT: float = 2.0
    # Minimum delays for command timing. These are busy-waited for.
    #
    # Optional delay in seconds to avoid busy wait and reduce CPU load when
    # polling the DRDY pin. Default is 0.000001 or 1 µs (timing not accurate)
    DRDY_DELAY: float = 0.000001
    # Delay between requesting data and reading the bus for
    # RDATA, RDATAC and RREG commands (datasheet: t_6 >= 50*CLKIN period).
    DATA_TIMEOUT: float = 1e-6 + 50.0 / CLKIN_FREQUENCY
    # Command-to-command timeout after SYNC and RDATAC
    # commands (datasheet: t11)
    SYNC_TIMEOUT: float = 1e-6 + 24.0 / CLKIN_FREQUENCY
    # See datasheet ADS1256: CS needs to remain low
    # for t_10 = 8*T_CLKIN after last SCLK falling edge of a command.
    # Because this delay is longer than timeout t_11 for the
    # RREG, WREG and RDATA commands of 4*T_CLKIN, we do not need
    # the extra t_11 timeout for these commands when using software
    # chip select selection and the _CS_TIMEOUT.
    CS_TIMEOUT: float = 1e-6 + 8.0 / CLKIN_FREQUENCY
    # When using hardware/hard-wired chip select, still a command-
    # to command timeout of t_11 is needed as a minimum for the
    # RREG, WREG and RDATA commands.
    T_11_TIMEOUT: float = 1e-6 + 4.0 / CLKIN_FREQUENCY
    ############################################################################


def init_or_read_from_config_file(*, init: bool = False) -> ADS1256Config:
    """Read or initialize configuration file and return config data object."""
    conf_file = Path.home().joinpath(f".{__package__}").joinpath(CONFIG_FILE_NAME)
    default_conf_file = files(__package__).joinpath(DEFAULT_CONFIG_FILE_NAME)
    if init or not conf_file.is_file():
        conf_file.parent.mkdir(exist_ok=True)
        shutil.copy(default_conf_file, conf_file)
        msg = "Configuration initialized using file: %s\n==> Please check or edit this file NOW!"
        logger.log(logging.INFO if init else logging.ERROR, msg, conf_file)
        sys.exit(0 if init else 1)
    try:
        conf = Binder(ADS1256Config).parse_toml(conf_file)
        if not conf.GATEWAY_ACTIVATED:
            msg = "BMS Gateway not configured!  Edit config file first: %s"
            logger.error(msg, conf_file)
            sys.exit(1)
    except Exception:
        msg = "Error reading configuration file %s"
        logger.exception(msg, conf_file)
        sys.exit(1)
    return conf
