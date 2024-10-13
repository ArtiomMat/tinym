    mov bx, 3
    mov cx, cs:[1 + bx] ; CS=0xA000, final addr=0xA0004, 0x1234 is there.
    mov si, 4
    cmp cx, cs:[si]
    jc _LOL
    mov ax, 0x1 ; Should happen, cuz no carry
    jz _LOL
    mov ax, 0x69 ; Should not happen
  _LOL:
    mov ax, bx
    xor ax, bx ; AX=0
    add ax, cs:[1 + bx] ; Add 0x1234 to ax, to test add, AX=0x1234