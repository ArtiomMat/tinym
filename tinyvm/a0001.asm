mov ax, 3

mul ax ; Square AX, result should be 9

mov ax, 2
mov dx, ax
mov ax, 9

div dx ; AX should be 4

mov ax, -3
mov cx, ax
mov ax, 4

imul cx ; AX should be -12
idiv cx ; AX should be 4
neg ax ; AX should be -4
not ax ; AX should be 0xFFFC