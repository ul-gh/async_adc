from importlib.metadata import PackageNotFoundError, version  # noqa: D104

try:  # noqa: SIM105
    __version__ = version("async_adc")
except PackageNotFoundError:
    # package is not installed
    pass

from async_adc.async_ads1256 import ADS1256  # noqa: F401
