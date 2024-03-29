
# 7-segment numerals
# Pattern below, active HIGH
#  _     8
# |_|  6   7
# |_|.   1
#      5   2
#        4  3
#
symbols = {
  '0': 0b11111010 << 4,
  '1': 0b01000010 << 4,
  '2': 0b11011001 << 4,
  '3': 0b11001011 << 4,
  '4': 0b01100011 << 4,
  '5': 0b10101011 << 4,
  '6': 0b10111011 << 4,
  '7': 0b11000010 << 4,
  '8': 0b11111011 << 4,
  '9': 0b11101011 << 4,
  'a': 0b11110011 << 4,
  'b': 0b00111011 << 4,
  'c': 0b00011001 << 4,
  'd': 0b01011011 << 4,
  'e': 0b10111001 << 4,
  'f': 0b10110001 << 4,
  'g': 0b11101011 << 4, # nine
  'h': 0b00110011 << 4,
  'i': 0b00000010 << 4,
  'j': 0b01001010 << 4,
  'l': 0b00111000 << 4,
  'n': 0b00010011 << 4,
  'o': 0b00011011 << 4,
  'p': 0b11110001 << 4,
  'q': 0b11111110 << 4, # ambiguous with "0."
  'r': 0b00010001 << 4,
  's': 0b10101011 << 4, # five
  't': 0b00111001 << 4,
  'u': 0b00011010 << 4,
  'y': 0b01101011 << 4,
  'z': 0b11011001 << 4, # two
  '.': 0b00000100 << 4,
  '_': 0b00001000 << 4,
  '-': 0b00000001 << 4,
  '‾': 0b10000000 << 4, # overbar
  '≡': 0b10001001 << 4, # triple bar
  '=': 0b00001001 << 4,
  '═': 0b10000001 << 4, # double horizontal box drawing, used for "upper" equals
  '|': 0b00110000 << 4, # pipe, ambiguous with "1"
  '‖': 0b01110010 << 4, # double vertical bar
  ' ': 0b00000000 << 4,
  '[': 0b10111000 << 4,
  ']': 0b11001010 << 4,
}

