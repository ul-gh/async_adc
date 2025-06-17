from async_adc import ADS1256
from async_adc.ads1256_definitions import MuxFlags

adc = ADS1256()

adc.check_chip_id()

test_ch = MuxFlags.POS_AIN0 | MuxFlags.NEG_AINCOM
test_seq = [test_ch] * 10

adc.read_oneshot(test_ch)

buf = [0] * 10
adc.init_cycle(test_seq)
adc.read_cycle(test_seq, buf)
