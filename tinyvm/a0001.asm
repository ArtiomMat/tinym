mov dx, 0x69
mov word ds:[0], 0b10000000000
mov es:[0x14], dx ; DX to there
mov si, 0x420
xchg si, es:[0x14] ; exchange the two
test si, ds:[0]
jnz _OK
mov ax, 1
_OK:
mov ax, 2
