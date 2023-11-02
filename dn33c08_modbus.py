'''
Modbus Server for DN33C08 Pico 2040 "PLC" expander

Install micropython-modbus depdency:

    mpremote mip install github:brainelectronics/micropython-modbus
'''

from machine import Pin
from umodbus.serial import ModbusRTU

MODBUS_INPUT_HR_BASE_REGISTER = 0
MODBUS_BUTTON_HR_BASE_REGISTER = 10

def make_set_coil_handler(pin):
  '''Handle modbus write to pin'''
  def setter(reg_type, address, val):
    print(f"[modbus write] Setting {pin} to {val}")
    if val[0]:
      pin.on()
    else:
      pin.off()
  return setter

def serve_modbus(
  device_address,
  uart_id, 
  tx_pin_no, 
  rx_pin_no, 
  ctrl_pin_no,
  inputs,
  buttons,
  outputs,
  ):
  # n.b. micropython-modbus docs confuse client/server role naming
  print("Setting up Modbus server...")

  server = ModbusRTU(
    addr=device_address,
    uart_id=uart_id,
    pins=(Pin(tx_pin_no), Pin(rx_pin_no)), # n.b. must be Pin() instances for rp2 platform
    ctrl_pin=ctrl_pin_no, # n.b. must be pin *number*, not Pin() instance
  )

  for output_no, pin in enumerate(outputs):
    address = output_no
    server.add_coil(
      address=address,
      value=bool(pin.value()),
      on_set_cb=make_set_coil_handler(pin), # set pin state per modbus write_coil commands
      #on_get_cb=before_get,
    )
    # update modbus registers when pin state changes to support read_coil commands
    pin.irq(lambda pin, address=address: server.set_coil(address, bool(pin.value())))

  for input_no, pin in enumerate(inputs):
    address = input_no + MODBUS_INPUT_HR_BASE_REGISTER
    server.add_hreg(
      address=address,
      value=input_no, #pin.value(),
      # writes not supported
    )
    # update modbus holding register value when pin state changes
    pin.irq(lambda pin, address=address: server.set_hreg(address, pin.value()))

  for button_no, pin in enumerate(buttons):
    address = button_no + MODBUS_BUTTON_HR_BASE_REGISTER
    server.add_hreg(
      address=address,
      value=button_no, #pin.value(),
      # writes not supported
    )
    # update modbus holding register value when pin state changes
    #pin.irq(make_holding_register_interrupt_handler(server, address))
    pin.irq(lambda pin, address=address: server.set_hreg(address, pin.value()))

  print("Listening for modbus messages...")
  while True:
    if server.process():
      print("Modbus message processed.")

