  mov bl, 0x12
  mov ch, 0x34,
  mov dh, bl
  mov dl, ch ; ax=0x1234
  mov es, dx
  mov ds:[2], es
