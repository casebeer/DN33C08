
from machine import mem32
import time

import dma.addresses
from .addresses import ( DMA_READ_ADDR_OFFSET, DMA_WRITE_ADDR_OFFSET,
  DMA_TRANS_COUNT_OFFSET )
from .ctrl import Ctrl
from .reset import reset_dma_subsystem

class Channel():
  '''
  Wrapper for a single DMA channel
  '''
  DMA_ABORT_WAIT_LIMIT = 20
  def __init__(self, channel_id):
    self.channel_id = channel_id
    #self.address = DMA_CHANNELS[self.channel_id]
    self.address = dma.addresses.Dma().Channel(self.channel_id).address()

  def is_busy(self):
    '''Return True if the DMA channel is busy'''
    return bool((self.get_ctrl() & (1 << Ctrl.Offsets.busy)))

  def get_ctrl(self):
    '''Return the raw value of the DMA channel's CTRL_TRIG register'''
    return mem32[dma.addresses.Dma().Channel(self.channel_id).ctrl_trig()]

  def configure(self, read_addr, write_addr, transfer_count, ctrl):
    '''Configure and trigger the DMA channel'''
    if self.is_busy():
      self.abort() # blocks until not is_busy()
    self._configure(read_addr, write_addr, transfer_count, ctrl)

  def make_ctrl(
    self,
    treq,
    wrap_write_addrs,
    ring_size_bits,
    incr_write,
    incr_read,
    enable,
    chain_to = None,
    data_size=2,
    ):
    '''Factory method to create a CTRL register config for this channel'''
    return Ctrl(
      channel_id = self.channel_id,
      treq = treq,
      wrap_write_addrs = wrap_write_addrs,
      ring_size_bits = ring_size_bits,
      incr_write = incr_write,
      incr_read = incr_read,
      enable = enable,
      chain_to = chain_to,
      data_size = data_size,
    )

  def _configure(self, read_addr, write_addr, transfer_count, ctrl):
    mem32[self.address + DMA_READ_ADDR_OFFSET] = read_addr
    mem32[self.address + DMA_WRITE_ADDR_OFFSET] = write_addr
    mem32[self.address + DMA_TRANS_COUNT_OFFSET] = transfer_count
    #mem32[self.address + DMA_CTRL_TRIG_OFFSET] = ctrl
    mem32[dma.addresses.Dma().Channel(self.channel_id).ctrl_trig()] = ctrl

  def abort(self):
    '''
    Set the CHAN_ABORT flag on the DMA channel

    See datasheet 2.5.5.3. 

    n.b. the datasheet recommends clearing the DMA channel's IRQ before
    aborting to avoid having the abort trigger an IRQ. The IRQ should then
    be reset once the abort is done. 

    n.b you MUST wait for the CTRL.BUSY flag to clear before starting
    another DMA transfer. This can take time for data transfers in flight
    to complete.
    '''
    #chan_abort_reg = DMA_BASE + DMA_CHAN_ABORT_OFFSET
    chan_abort_reg = dma.addresses.Dma().Abort().address()
    if self.is_busy():
      print(f"Aborting DMA channel {self.channel_id}...")
      #pause_dma(channel)
      mem32[chan_abort_reg] |= (1 << self.channel_id)
      counter = 0
      while self.is_busy():
        if counter > self.DMA_ABORT_WAIT_LIMIT:
          print("WARNING: Resetting DMA subsytem due to failure to abort DMA channel")
          reset_dma_subsystem()
          break
        print(f"Waiting for DMA channel {self.channel_id} to not be busy {self.get_ctrl():08x}, chan_abort:{mem32[chan_abort_reg]:08x}...")
        time.sleep_ms(100)
        mem32[chan_abort_reg] |= (1 << self.channel_id)
        counter += 1
    return mem32[chan_abort_reg]
