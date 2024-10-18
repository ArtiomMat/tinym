mov ax, 0x69
mov dx, ax
mov ax, 0
xchg es:[2], dx ; es:[2] holds 0x420
xchg dx, es:[2] ; dx back to 0x69
xchg ax, dx ; ax now 0x69, dx 0
