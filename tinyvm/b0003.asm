mov ax, 0xB
call 0xC000:0x1234 ; Call again to check nested calls
jmp _LOL
mov ax, 0xB1 ; Should be skipped
_LOL:
retf ; RET FAR in nasm apparently?
