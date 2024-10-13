  mov cx, [0x420] ; 0xB8 first byte, 0x20 second byte, 0x42 third
  cmp cx, 0x421
  jl _LOL
  mov bx, 0x1
_LOL:
  mov bx, 0x420
  cmp bx, cs:[2]