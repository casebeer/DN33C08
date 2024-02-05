
'''
Minimal RP2040 DMA configuration implementation

See the [RP2040 datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf). 

The RP2040 has 12 DMA channels. Each channel has memory space assigned
allowing writes to configure the channel and reads to get the channel's
status. 
'''

from .channel import Channel
from .ctrl import Ctrl
from .pio import Pio

