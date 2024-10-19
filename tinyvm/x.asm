  mov ax, 3

  mul ax ; Square AX, result should be 9

  mov dx, 2

  div dx ; AX should be 4

  mov cx, -3

  imul cx ; AX should be -12
  idiv cx ; AX should be 4
  neg ax ; AX should be -4
  not ax ; AX should be 0x3
  ; Now for the byte versions, just some little tests
  mov 
