/* INCLUDED AT BOTTOM */

/* Tests the regseg functions. */
static void test_regseg(void) {
  cpu8086_t cpu;
  mem_t mem;
  
  init_mem8086(&mem, 0);
  reset_cpu8086(&cpu, &mem);

  cpu.regs[REG8086_SP].x = 0xFFFF;
  cpu.regs[REG8086_SS].x = 0x0010;
  regseg_add(cpu.regs + REG8086_SP, cpu.regs + REG8086_SS, 1);
  HOPE_THAT(
    cpu.regs[REG8086_SP].x == 0x0000 && cpu.regs[REG8086_SS].x == 0x1010,
    "regseg_add() was correct for edge case."
  );

  cpu.regs[REG8086_SP].x = 0xFFFF;
  cpu.regs[REG8086_SS].x = 0x0010;
  regseg_add(cpu.regs + REG8086_SP, cpu.regs + REG8086_SS, 3);
  HOPE_THAT(
    cpu.regs[REG8086_SP].x == 0x0002 && cpu.regs[REG8086_SS].x == 0x1010,
    "regseg_add() was correct for edge case."
  );

  cpu.regs[REG8086_SP].x = 0x0030;
  cpu.regs[REG8086_SS].x = 0x3010;
  regseg_sub(cpu.regs + REG8086_SP, cpu.regs + REG8086_SS, 0x0032);
  HOPE_THAT(
    cpu.regs[REG8086_SP].x == (0xFFFF - 2) && cpu.regs[REG8086_SS].x == 0x2010,
    "regseg_add() was correct for edge case."
  );

  cpu.regs[REG8086_SP].x = 0x0030;
  cpu.regs[REG8086_SS].x = 0x3010;
  regseg_sub(cpu.regs + REG8086_SP, cpu.regs + REG8086_SS, 0x0030);
  HOPE_THAT(
    cpu.regs[REG8086_SP].x == 0 && cpu.regs[REG8086_SS].x == 0x3010,
    "regseg_add() was correct for edge case."
  );

  /* Now for a final simple test of regseg_imm() + regseg() */
  HOPE_THAT(
    regseg_imm(&cpu, 0xDEAD, 0xBEEF) == (0xDEAD) + 0xBEEF0,
    "regseg_add() was correct for edge case."
  );

  cpu.regs[REG8086_AX].x = 0xDEAD;
  cpu.regs[REG8086_SS].x = 0xBEEF;
  HOPE_THAT(
    regseg(&cpu, REG8086_AX, REG8086_SS) == (0xDEAD) + 0xBEEF0,
    "regseg_add() was correct for edge case."
  );
}

/* Tests a simple piece of code */
static void test_cycling0(void) {
  cpu8086_t cpu;
  mem_t mem;
  const uint8_t code[] = {
    0xB8, 0x69, 0x42, /* MOV AX, 0x4269 */
    0x50, /* PUSH AX */
    0x5B, /* POP BX */
    0x43, /* INC BX */
    0xB4, 0x69 /* MOV AL, 0x69 */
  };

  HOPE_THAT(init_mem8086(&mem, 0), "Memory initialized.");
  HOPE_THAT(reset_cpu8086(&cpu, &mem), "CPU initialized.");

  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_CS].x = 0;
  cpu.regs[REG8086_SS].x = 0xF000;

  memcpy(mem.bytes, code, sizeof (code));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x4269,
    "MOV AX, 0x4269."
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    mem.bytes[0xFFFFC] == 0x69 && mem.bytes[0xFFFFD] == 0x42,
    "PUSH AX."
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 0x4269,
    "POP BX => BX=0x4269"
  );
  HOPE_THAT(
    cpu.regs[REG8086_SP].x == 0xFFFE,
    "SP is back."
  );
  
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 0x426A,
    "INC BX => BX=0x426A."
  );
  
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x6969,
    "MOV AL, 0x69 => AX=0x6969."
  );

  free_mem(&mem);
}

static void test_modrm_888b(void) {
  cpu8086_t cpu;
  mem_t mem;
  const uint8_t code[] = {
    0xB8, 0x20, 0x04, /* MOV AX, 0x0420 */
    0x89, 0xC3 /* MOV BX, AX */
  };
  

  HOPE_THAT(init_mem8086(&mem, 0), "Memory initialized.");
  HOPE_THAT(reset_cpu8086(&cpu, &mem), "CPU initialized.");

  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_CS].x = 0;
  cpu.regs[REG8086_IP].x = 0;
  cpu.regs[REG8086_BX].x = 0;
  cpu.regs[REG8086_AX].x = 0;

  memcpy(mem.bytes, code, sizeof (code));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x420,
    "MOV AX, 0x420 => AX=0x420"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 0x420,
    "MOV BX, AX => BX=0x420"
  );
}

static void test_mov_rm_imm(void) {
  cpu8086_t cpu;
  mem_t mem;
  /*
    mov byte ds:[3], 0x12
    mov byte ds:[4], 0x34
    mov word es:[6], 0x1234
  */
  const uint8_t code[] = {
    0x3e, 0xc6, 0x06, 0x03, 0x00, 0x12, 0x3e, 0xc6, 0x06, 0x04, 0x00, 0x34,
    0x26, 0xc7, 0x06, 0x06, 0x00, 0x34, 0x12
  };

  HOPE_THAT(init_mem8086(&mem, 0), "Memory initialized.");
  HOPE_THAT(reset_cpu8086(&cpu, &mem), "CPU initialized.");

  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_SS].x = 0xF000;
  cpu.regs[REG8086_DS].x = 0xA000;
  cpu.regs[REG8086_ES].x = 0xB000;
  cpu.regs[REG8086_CS].x = 0xD000;
  cpu.regs[REG8086_IP].x = 1; /* Just to test unaligned instructions */

  memcpy(mem.bytes + 0xD0001, code, sizeof (code));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    mem.bytes[0xA0003] == 0x12,
    "mov byte ds:[3], 0x12"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    mem.bytes[0xA0004] == 0x34,
    "mov byte ds:[4], 0x12"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    mem.bytes[0xB0006] == 0x34 && mem.bytes[0xB0007] == 0x12,
    "mov word es:[6], 0x1234"
  );
}

/* Tests the mov instructions at 0xA0-0xA3 */
static void test_a0a3_and_sreg_prefix(void) {
  cpu8086_t cpu;
  mem_t mem;
  const uint8_t code[] = {
    0x2E, 0xA1, 0x00, 0x10, /* MOV AX, CS:[0x1000] */
    0x2E, 0xA0, 0x01, 0x10, /* MOV AL, CS:[0x1001] */
    0x2E, 0xA3, 0x00, 0x10, /* MOV CS:[0x1000], AX */
    0x3E, 0x50, /* DS:PUSH AX */
    0x3E, 0x50, /* DS:PUSH AX */
    0x5B, /* POP BX, no prefix for tests. */
    0x3E, 0x5B /* DS:POP BX, correct one this time. */
  };

  HOPE_THAT(init_mem8086(&mem, 0), "Memory initialized.");
  HOPE_THAT(reset_cpu8086(&cpu, &mem), "CPU initialized.");

  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_SS].x = 0xF000;
  cpu.regs[REG8086_DS].x = 0xC000;
  cpu.regs[REG8086_SP].x = 4; /* To make life easier */
  cpu.regs[REG8086_CS].x = 0xA000;
  cpu.regs[REG8086_IP].x = 0;
  
  mem.bytes[0xA1000] = 0x33;
  mem.bytes[0xA1001] = 0x44;

  memcpy(mem.bytes + 0xA0000, code, sizeof (code));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x4433,
    "MOV AX, CS:[0x1000] => AX=0x4433"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x4444,
    "MOV AL, CS:[0x1001] => AX=0x4444"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    mem.bytes[0xA1000] == 0x44 && mem.bytes[0xA1001] == 0x44,
    "MOV CS:[0x1000], AX(0x4444)"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_SP].x == 0,
    "SP should be 0 now."
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].x != 0x4444,
    "Wrong segment register used so BX shouldn't be 0x4444."
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 0x4444,
    "Correct segment register used so BX should be 0x4444."
  );
}

/* Tests various aspects of the dstsrc_0030 + conditional jumps */
static void test_dstsrc_0030_and_condjmp(void) {
  cpu8086_t cpu;
  mem_t mem;
  /*
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
  */
  const uint8_t code[] = {
    0xbb, 0x03, 0x00, 0x2e, 0x8b, 0x4f, 0x01, 0xbe, 0x04, 0x00, 0x2e, 0x3b,
    0x0c, 0x72, 0x08, 0xb8, 0x01, 0x00, 0x74, 0x03, 0xb8, 0x69, 0x00, 0x89,
    0xd8, 0x31, 0xd8, 0x2e, 0x03, 0x47, 0x01
  };

  HOPE_THAT(init_mem8086(&mem, 0), "Memory initialized.");
  HOPE_THAT(reset_cpu8086(&cpu, &mem), "CPU initialized.");

  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_SS].x = 0xF000; /* No need but anyway. */
  cpu.regs[REG8086_DS].x = 0xD000; /* To make sure prefixes work too. */
  cpu.regs[REG8086_CS].x = 0xA000;
  cpu.regs[REG8086_IP].x = 0x3000;
  cpu.regs[REG8086_CX].x = 0x3000;
  
  mem.bytes[0xA0004] = 0x34;
  mem.bytes[0xA0005] = 0x12;

  /* Load code away from the 0x1234 */
  memcpy(mem.bytes + 0xA3000, code, sizeof (code));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 3,
    "MOV BX, 3"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_CX].x == 0x1234,
    "mov cx, cs:[1 + bx] => CX=0x1234 which is placed there."
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_SI].x == 4,
    "mov si, 4."
  );

  cpu.regs[REG8086_F].x = 0; /* Just to confirm if it's actually set after the cycle. */
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_F].x & F8086_Z,
    "cmp cx, cs:[si] => ZF=1"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 1,
    "jc _LOL didn't happen"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error."); /* mov ax, bx */
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 3 && cpu.regs[REG8086_AX].x == cpu.regs[REG8086_BX].x,
    "jz _LOL happened"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0,
    "xor ax, bx"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x1234,
    "add ax, cs:[1 + bx]"
  );
}

/* Test moving into segment registers and out, and also if byte modrm works */
static void test_sreg_mov_and_modrmb(void) {
  cpu8086_t cpu;
  mem_t mem;
  /*
    mov bl, 0x12
    mov ch, 0x34,
    mov dh, bl
    mov dl, ch ; ax=0x1234
    mov es, dx
    mov ds:[2], es
  */
  const uint8_t code[] = {
    0xb3, 0x12, 0xb5, 0x34, 0x88, 0xde, 0x88, 0xea, 0x8e, 0xc2, 0x3e, 0x8c,
    0x06, 0x02, 0x00
  };

  HOPE_THAT(init_mem8086(&mem, 0), "Memory initialized.");
  HOPE_THAT(reset_cpu8086(&cpu, &mem), "CPU initialized.");

  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_SS].x = 0xF000;
  cpu.regs[REG8086_DS].x = 0xD000;
  cpu.regs[REG8086_CS].x = 0xA000;
  cpu.regs[REG8086_IP].x = 0x3000;
  cpu.regs[REG8086_CX].x = 0x3000;
  
  mem.bytes[0xD0002] = 0x78;
  mem.bytes[0xD0003] = 0x56;

  memcpy(mem.bytes + 0xA3000, code, sizeof (code));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].p[0] == 0x12,
    "mov bl, 0x12"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_CX].p[1] == 0x34,
    "mov ch, 0x12"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_DX].x == 0x1234,
    "mov dh, bl + mov dl, ch"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_ES].x == 0x1234,
    "mov es, dx"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    mem.bytes[0xD0002] == 0x34 && mem.bytes[0xD0003] == 0x12,
    "mov ds:[2], es"
  );
}

static void test_grp1(void) {
  cpu8086_t cpu;
  mem_t mem;
  /*
    mov ax, 3

    mul ax ; Square AX, result should be 9

    mov dx, 2

    div dx ; AX should be 4

    mov cx, -3

    imul cx ; AX should be -12
    idiv cx ; AX should be 4
    neg ax ; AX should be -4
    not ax ; AX should be 0x3
  */
  uint8_t code[] = {
  0xb8, 0x03, 0x00, 0xf7, 0xe0, 0xba, 0x02, 0x00, 0xf7, 0xf2, 0xb9, 0xfd,
  0xff, 0xf7, 0xe9, 0xf7, 0xf9, 0xf7, 0xd8, 0xf7, 0xd0
  };

  HOPE_THAT(init_mem8086(&mem, 0), "Memory initialized.");
  HOPE_THAT(reset_cpu8086(&cpu, &mem), "CPU initialized.");

  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_SS].x = 0xF000;
  cpu.regs[REG8086_SP].x = 0xE000;
  cpu.regs[REG8086_CS].x = 0xA000;
  cpu.regs[REG8086_IP].x = 0x3;

  memcpy(mem.bytes + 0xA0003, code, sizeof (code));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 9,
    "squaring successful"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 4,
    "division successful"
  );
  
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    (int16_t)cpu.regs[REG8086_AX].x == -12 && (int16_t)cpu.regs[REG8086_DX].x == -1, /* IDK why, but for some reason need (int16_t) */
    "integer multiplication successful"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    (int16_t)cpu.regs[REG8086_AX].x == 4 && (int16_t)cpu.regs[REG8086_DX].x == 0, /* IDK why, but for some reason need (int16_t) */
    "integer division successful"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    (int16_t)cpu.regs[REG8086_AX].x == -4,
    "negation successful"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == ~(-4),
    "notting successful"
  );
}

static void test_xchg(void) {
  cpu8086_t cpu;
  mem_t mem;
  /*
    mov ax, 0x69
    mov dx, ax
    mov ax, 0
    xchg es:[2], dx ; es:[2] holds 0x420
    xchg dx, es:[2] ; dx back to 0x69
    xchg ax, dx ; ax now 0x69, dx 0
  */
  uint8_t code[] = {
    0xb8, 0x69, 0x00, 0x89, 0xc2, 0xb8, 0x00, 0x00, 0x26, 0x87, 0x16, 0x02,
    0x00, 0x26, 0x87, 0x16, 0x02, 0x00, 0x92
  };

  HOPE_THAT(init_mem8086(&mem, 0), "Memory initialized.");
  HOPE_THAT(reset_cpu8086(&cpu, &mem), "CPU initialized.");

  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_SS].x = 0xF000;
  cpu.regs[REG8086_SP].x = 0xE000;
  cpu.regs[REG8086_ES].x = 0xD000;
  cpu.regs[REG8086_CS].x = 0xA000;
  cpu.regs[REG8086_IP].x = 0x3;
  cpu.regs[REG8086_SI].x = 0;

  mem.bytes[0xD0002] = 0x20;
  mem.bytes[0xD0003] = 0x4;

  memcpy(mem.bytes + 0xA0003, code, sizeof (code));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_DX].x == 0x420 && mem.bytes[0xD0002] == 0x69 && mem.bytes[0xD0003] == 0,
    "xchg successful"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_DX].x == 0x69 && mem.bytes[0xD0002] == 0x20 && mem.bytes[0xD0003] == 0x4,
    "xchg successful"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x69 && cpu.regs[REG8086_DX].x == 0,
    "xchg successful"
  );
}

static void test_call_ret_near(void) {
  uint8_t i;
  cpu8086_t cpu;
  mem_t mem;
  /*
    org 0xA000:0x1
      mov si, 0
      mov cx, 2
      mov bx, 3
      call _MAGIC
      mov si, 1 ; Signals done to the test

    _MAGIC:
      add cx, bx
      mov ax, 1
      add cx, dx
      mov ax, cx
      ret
  */
  uint8_t code[] = {
    0xbe, 0x00, 0x00, 0xb9, 0x02, 0x00, 0xbb, 0x03, 0x00, 0xe8, 0x03, 0x00,
    0xbe, 0x01, 0x00, 0x01, 0xd9, 0xb8, 0x01, 0x00, 0x01, 0xc1, 0x89, 0xc8,
    0xc3
  };

  HOPE_THAT(init_mem8086(&mem, 0), "Memory initialized.");
  HOPE_THAT(reset_cpu8086(&cpu, &mem), "CPU initialized.");

  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_SS].x = 0xF000;
  cpu.regs[REG8086_SP].x = 0xE000;
  cpu.regs[REG8086_CS].x = 0xA000;
  cpu.regs[REG8086_IP].x = 0x1;
  cpu.regs[REG8086_SI].x = 0;

  memcpy(mem.bytes + 0xa0001, code, sizeof (code));

  /* New format for testing, where we only test the end result. */
  for (i = 0; !cpu.regs[REG8086_SI].x && i < 16; ++i) {
    HOPE_THAT(!cycle_cpu8086(&cpu), "No error");
  }

  HOPE_THAT(
    i < 16,
    "The right amount of instructions ran."
  );

  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 6,
    "AX should be 6 by the end."
  );
}

/* Tests intersegment calls and rets. */
static void test_call_ret_far(void) {
  cpu8086_t cpu;
  mem_t mem;
  /*
    mov ax, 0xA
    call 0xC000:0x1234
    mov ax, 0xA1
    call 0xB000:0x0003
    mov ax, 0xA2
  */
  const uint8_t a0001[] = {
    0xb8, 0x0a, 0x00, 0x9a, 0x34, 0x12, 0x00, 0xc0, 0xb8, 0xa1, 0x00, 0x9a,
    0x03, 0x00, 0x00, 0xb0, 0xb8, 0xa2, 0x00
  };
  /*
    mov ax, 0xB
    call 0xC000:0x1234 ; Call again to check nested calls
    jmp _LOL
    mov ax, 0xB1 ; Should be skipped
    _LOL:
    retf ; RET FAR in nasm apparently?
  */
  const uint8_t b0003[] = {
    0xb8, 0x0b, 0x00, 0x9a, 0x34, 0x12, 0x00, 0xc0, 0xeb, 0x03, 0xb8, 0xb1,
    0x00, 0xcb
  };
  /*
    mov ax, 0xC
    retf
  */
  const uint8_t c1234[] = {
    0xb8, 0x0c, 0x00, 0xcb
  };

  HOPE_THAT(init_mem8086(&mem, 0), "Memory initialized.");
  HOPE_THAT(reset_cpu8086(&cpu, &mem), "CPU initialized.");

  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_SS].x = 0xF000;
  cpu.regs[REG8086_SP].x = 0xE000;
  cpu.regs[REG8086_CS].x = 0xA000;
  cpu.regs[REG8086_IP].x = 0x1;

  memcpy(mem.bytes + 0xa0001, a0001, sizeof (a0001));
  memcpy(mem.bytes + 0xb0003, b0003, sizeof (b0003));
  memcpy(mem.bytes + 0xc1234, c1234, sizeof (c1234));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0xA,
    "mov ax, 0xA"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error."); /* call far */
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0xC,
    "mov ax, 0xC"
  );
  
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error."); /* ret far */
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0xA1,
    "mov ax, 0xA1"
  );
  HOPE_THAT(
    cpu.regs[REG8086_SP].x == 0xE000 && cpu.regs[REG8086_SS].x == 0xF000,
    "Stack is correct after calls and rets"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error."); /* call far */
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0xB,
    "mov ax, 0xB"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error."); /* call far from 0xb0003 */
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0xC,
    "mov ax, 0xC"
  );
  
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error."); /* retf */
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error."); /* jmp over mov ax, 0xB1 */
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0xC,
    "ax still 0xC"
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error."); /* retf */
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0xA2,
    "mov ax, 0xA2"
  );
  HOPE_THAT(
    cpu.regs[REG8086_SP].x == 0xE000 && cpu.regs[REG8086_SS].x == 0xF000,
    "Stack is correct after calls and rets"
  );
}

/* Tests update_bit_flags() and its related *_ins() functions. */
static void test_update_flags(void) {
  cpu8086_t cpu;
  uint16_t* fptr = &cpu.regs[REG8086_F].x;
  *fptr = 0;

  update_bit_flags(&cpu, 0, 0);

  add_ins(&cpu, 1, 2, 1);
  HOPE_THAT(
    *fptr & F8086_P,
    "1+2 mean nothing for flags, except parity for 3=0b11."
  );

  sub_ins(&cpu, 2, 4, 1);
  HOPE_THAT(
    *fptr & F8086_C,
    "W 2-4 means carry."
  );
  HOPE_THAT(
    !(*fptr & F8086_O),
    "W 2-4 doesn't mean overflow."
  );

  sub_ins(&cpu, 2, 131, 0);
  HOPE_THAT(
    *fptr & F8086_O,
    "!W 2-131 means overflow."
  );
  HOPE_THAT(
    *fptr & F8086_C,
    "!W 2-131 also means carry."
  );
  HOPE_THAT(
    !(*fptr & F8086_Z),
    "!W 2-131 doesn't mean zero."
  );
  HOPE_THAT(
    !(*fptr & F8086_S),
    "!W 2-131 also means positive."
  );

  sub_ins(&cpu, 2, 131, 1);
  HOPE_THAT(
    *fptr & F8086_C,
    "W 2-131 means carry."
  );
  HOPE_THAT(
    !(*fptr & F8086_O),
    "W 2-131 doesn't mean overflow."
  );

  add_ins(&cpu, 2, 131, 0);
  HOPE_THAT(
    !(*fptr & F8086_C),
    "!W 2+131 doesn't mean carry."
  );
  HOPE_THAT(
    !(*fptr & F8086_Z),
    "!W 2+131 doesn't mean zero."
  );
  HOPE_THAT(
    *fptr & F8086_O,
    "!W 2+131 means overflow."
  );

  add_ins(&cpu, -2, 2, 1);
  HOPE_THAT(
    *fptr & F8086_Z,
    "!W -2+2 means zero."
  );
  HOPE_THAT(
    !(*fptr & F8086_O),
    "!W -2+2 doesn't mean overflow."
  );
  HOPE_THAT(
    !(*fptr & F8086_O),
    "!W -2+2 doesn't mean carry."
  );
}

static void test_parity_table(void) {
  HOPE_THAT(
    get_arr_bit(parity_table, 0) == 1,
    "parity=1 for 0."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 1) == 0,
    "parity=0 for 1."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 2) == 0,
    "parity=0 for 2."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 3) == 1,
    "parity=1 for 3."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 104) == 0,
    "parity=0 for 104."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 96) == 1,
    "Carity =1for 96."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 108) == 1,
    "Cority f=1or 108."
  );
}


void add_cpu8086_tests(void) {
  add_mem_tests(); /* cpu8086_t depends on proper mem_t function. */
  ADD_TEST(test_regseg);
  ADD_TEST(test_parity_table);
  ADD_TEST(test_update_flags);

  ADD_TEST(test_cycling0);
  ADD_TEST(test_a0a3_and_sreg_prefix);
  ADD_TEST(test_dstsrc_0030_and_condjmp);
  ADD_TEST(test_sreg_mov_and_modrmb);
  ADD_TEST(test_modrm_888b);
  ADD_TEST(test_mov_rm_imm);
  ADD_TEST(test_xchg);
  ADD_TEST(test_call_ret_far);
  ADD_TEST(test_call_ret_near);
  ADD_TEST(test_grp1);
}
