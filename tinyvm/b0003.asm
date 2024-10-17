org 0xA000:0x1 
  mov si, 0
  mov cx, 2
  mov bx, 3
  call _MAGIC
  mov si, 1 ; Signals done to the test

_MAGIC:
  add cx, bx
  mov ax, 1
  add cx, ax
  mov ax, cx
  ret
