/*
  NOTE TO DEV: Every function here must be safe yet performant, isolated anarchy in the CPU is
  better than anarchy in the VM due to out-of-bounds access.
*/

#include "cpu8086.h"
#include "ops.h"
#include "test.h"

#include <string.h>

#define REGSEG_SP_SS(CPU) regseg_imm(CPU, CPU->regs[REG8086_SP].x, CPU->regs[REG8086_SS].x)
#define REGSEG_IP_CS(CPU) regseg_imm(CPU, CPU->regs[REG8086_IP].x, CPU->regs[REG8086_CS].x)
#define REGSEG_IP_CS_ADD(CPU, N) regseg_add(&CPU->regs[REG8086_IP], &CPU->regs[REG8086_CS], N)
#define REGSEG_IP_CS_SUB(CPU, N) regseg_sub(&CPU->regs[REG8086_IP], &CPU->regs[REG8086_CS], N)

int reset_cpu8086(cpu8086_t* __restrict cpu, mem_t* __restrict mem) {
  /* Set up register starting points */
  memset(cpu->regs, 0, sizeof (cpu->regs));
  cpu->regs[REG8086_CS].x = 0xFFFF;
  cpu->regs[REG8086_SP].x = 0xFFFF; /* TODO: Not an actual standard config. */
  cpu->regs[REG8086_F].x |= F8086_I;

  /* A minimum size for addressable memory. */
  if (mem->size < MB1) {
    return 0;
  }
  cpu->mem = mem;

  return 1;
}

/*
  SAFE 1MB ACCESS. Joins 2 immidiete values into a segmented 20-bit address.
  The rest of the 12 bits returned will be 0, to avoid potential buffer overflow.
  May set CPU->e to E8086_ACCESS_VIOLATION if we go out of 1MB bounds.
*/
static uint32_t regseg_imm(cpu8086_t* cpu, const uint16_t reg, const uint16_t seg) {
  uint32_t ret = reg + (seg << 4);
  if (0xFFF00000 & ret) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return ret & 0xFFFFF; /* Still want to return something to let CPU finish */
  }
  return ret;
}
/*
  SAFE 1MB ACCESS.
  Equivalent of regseg_imm but for actual regs. REG and SEG are the REG8086_* enum.
*/
static uint32_t regseg(cpu8086_t* cpu, const int reg, const int seg) {
  return regseg_imm(cpu, cpu->regs[reg].x, cpu->regs[seg].x);
}
/*
  Allows reg/seg > 1MB, but it's ok. Adds ADDR to the REG+SEG in a way that prevents overflow.
  E.G if ADDR=1, REG=0xFFFF then we need to do *SEG+=0x10000 and REG=0
*/
static void regseg_add(reg8086_t* __restrict reg, reg8086_t* __restrict seg, const uint16_t addr) {
  if ((uint32_t)reg->x + addr > 0xFFFF) {
    reg->x = (0x0000 + addr) - 1;
    seg->x += 0x1000;
  }
  else {
    reg->x += addr;
  }
}
/*
  Allows reg/seg > 1MB, but it's ok. Subtract ADDR to the REG+SEG in a way that prevents undeflow.
  E.G if ADDR=1, *REG=0x0 then we need to do *SEG-=0x10000 and *REG=0xFFFF
*/
static void regseg_sub(reg8086_t* __restrict reg, reg8086_t* __restrict seg, const uint16_t addr) {
  if ((int32_t)reg->x - addr < 0) {
    reg->x = (0xFFFF - addr) + reg->x;
    seg->x -= 0x1000;
  }
  else {
    reg->x -= addr;
  }
}

/* SAFE 1MB ACCESS. 8 bit equivalent of get16. */
static uint8_t get8(cpu8086_t* cpu, const uint32_t addr) {
  if (addr >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return 0;
  }
  
  return cpu->mem->bytes[addr];
}

/*
  SAFE 1MB ACCESS. Get a value from CPU->mem[ADDR]. Preffered over raw access for: safety(will 
  CPU->e and return invalid but safely constructed value) & easier for unaligned access.
  As mentioned above, may set CPU->e to E8086_ACCESS_VIOLATION.
*/
static uint16_t get16(cpu8086_t* cpu, const uint32_t addr) {
  uint16_t ret;

  if (addr >= MB1 - 1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return 0;
  }
  
  memcpy(&ret, cpu->mem->bytes + addr, 2);
  return ret;
}

/*
  SAFE 1MB ACCESS. Set CPU->mem[ADDR] = what. Preffered over raw access for: safety(will CPU->e 
  and will not set) & easier for unaligned access.
  As mentioned above, may set CPU->e to E8086_ACCESS_VIOLATION.
*/
static void put16(cpu8086_t* cpu, const uint32_t addr, const uint16_t what) {
  if (addr >= MB1 - 1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return;
  }
  
  memcpy(cpu->mem->bytes + addr, &what, 2);
}

/*
  SAFE 1MB ACCESS. Pushes X into CPU->mem using CPU's stack registers.
  May set CPU->e to E8086_ACCESS_VIOLATION if the stack wrapped around and wont push. Uses regseg() so may set to its errors.
*/
static void push16(cpu8086_t* cpu, const uint16_t x) {
  uint32_t stack_ptr = REGSEG_SP_SS(cpu);

  stack_ptr -= 2; /* Since gotta first decrement */
  regseg_sub(&cpu->regs[REG8086_SP], &cpu->regs[REG8086_SS], 2);

  /* Now copy the register contents */
  put16(cpu, stack_ptr, x);
}

/*
  SAFE 1MB ACCESS. Pops the stack from CPU->mem using CPU's stack regisers, and returns the popped value.
  cycle_cpu8086().
  Uses regseg() so may set to its errors.
*/
static uint16_t pop16(cpu8086_t* cpu) {
  uint16_t ret;
  uint32_t stack_ptr = REGSEG_SP_SS(cpu);

  /* Copy the content before modifying stack_ptr */
  ret = get16(cpu, stack_ptr);

  /* Add 2 back */
  regseg_add(&cpu->regs[REG8086_SP], &cpu->regs[REG8086_SS], 2);

  return ret;
}

/*
  Note that REL_ADDR is signed.
  IP_CS must be REGSEG_IP_CS(CPU).
  May set CPU->e to E8086_ACCESS_VIOLATION in the case where REL_ADDR over/underflows.
*/
static void short_jump(cpu8086_t* cpu, int8_t rel_addr) {
  if (rel_addr + REGSEG_IP_CS(cpu) >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return;
  }

  if (rel_addr < 0) {
    REGSEG_IP_CS_ADD(cpu, rel_addr);
  }
  else {
    REGSEG_IP_CS_SUB(cpu, rel_addr);
  }
}

/*
  Returns whether according to the first instruction byte the CPU should do conditional jump.
  INS_BYTE is the instruction, it must be 0x70-0x7F inclusive.
*/
static int check_cond_jmp(cpu8086_t* cpu, uint8_t ins_byte) {
  int do_jump;
  uint16_t f = cpu->regs[REG8086_F].x;

  /*
    A pattern is that if (ins_byte&1) is 1 then it's the NOT equivalent.
    I exploit it to less boilerplate code for both the normal and NOT versions.
  */
  switch (ins_byte) {
    case 0x70: /* JO */
    case 0x71: /* JNO */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_O));
    break;

    case 0x72: /* JC/JB/JNAE */
    case 0x73: /* JNC/JNB/JAE */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_CY));
    break;

    case 0x74: /* JE/JZ */
    case 0x75: /* JNE/JNZ */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_Z));
    break;
    
    case 0x76: /* JBE/JNA */
    case 0x77: /* JA/JNBE */
    do_jump = (ins_byte&1) ^ ((!!(f & F8086_CY)) | (!!(f & F8086_Z))); 
    break;

    case 0x78: /* JS */
    case 0x79: /* JNS */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_S));
    break;

    case 0x7A: /* JP/JPE */
    case 0x7B: /* JNP/JPO */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_P));
    break;

    case 0x7C: /* JL/JNGE */
    case 0x7D: /* JGE/JNL */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_S)) ^ (!!(f & F8086_O));
    break;

    case 0x7E: /* JLE/JNG */
    case 0x7F: /* JGE/J */
    do_jump = (ins_byte&1) ^ (((!!(f & F8086_S)) ^ (!!(f & F8086_O))) | (!!(f & F8086_Z)));
    break;
  }

  return do_jump;
}

/*
  A specific type of instructions that have the reg field.
*/
static void parse_regrm(cpu8086_t* cpu) {
  
}

int cycle_cpu8086(cpu8086_t* cpu) {
  mem_t* mem = cpu->mem;

  uint32_t ip_cs = REGSEG_IP_CS(cpu); /* Address of current instruction */

  uint8_t* ins_bytes = mem->bytes + ip_cs; /* Where the instruction is */
  uint8_t opcode = (ins_bytes[0] & 0xFC) >> 2;
  uint8_t d_bit = (ins_bytes[0] & 0x2) >> 1;
  uint8_t w_bit = ins_bytes[0] & 0x1;

  uint8_t ip_add = 0; /* How much to add if reading of instruction succeeded */

  cpu->e = E8086_OK; /* TODO: But should it reset? An error is fatal anyway. */

  /* Instructions that simply act on the 16-bit registers in order. */
  /* INC R16 */
  if (ins_bytes[0] >= 0x40 && ins_bytes[0] <= 0x47) {
    ++ (cpu->regs[REG8086_AX + (ins_bytes[0] - 0x40)].x);
    cpu->cycles += 2;
    ip_add = 1;
  }
  /* DEC R16 */
  else if (ins_bytes[0] >= 0x48 && ins_bytes[0] <= 0x4F) {
    -- (cpu->regs[REG8086_AX + (ins_bytes[0] - 0x48)].x);
    cpu->cycles += 2;
    ip_add = 1;
  }
  /* PUSH R16 */
  else if (ins_bytes[0] >= 0x50 && ins_bytes[0] <= 0x57) {
    push16(cpu, cpu->regs[REG8086_AX + (ins_bytes[0] - 0x50)].x);
    cpu->cycles += 11;
    ip_add = 1;
  }
  /* POP R16 */
  else if (ins_bytes[0] >= 0x58 && ins_bytes[0] <= 0x5F) {
    cpu->regs[REG8086_AX + (ins_bytes[0] - 0x58)].x = pop16(cpu);
    cpu->cycles += 8;
    ip_add = 1;
  }
  /* MOV R16, IMM16 */
  else if (ins_bytes[0] >= 0xB8 && ins_bytes[0] <= 0xBF) {
    uint16_t v;

    if (ip_cs + 3 >= MB1) {
      return E8086_CUT_OFF;
    }
    
    /* Construct value. */
    v = mem->bytes[ip_cs + 1] + (mem->bytes[ip_cs + 2] << 8);
    
    cpu->regs[REG8086_AX + (ins_bytes[0] - 0xB8)].x = v;
    
    cpu->cycles += 4;
    ip_add = 3;
  }
  /* Various conditional jumps, relative to current instruction. */
  else if (ins_bytes[0] >= 0x70 && ins_bytes[0] <= 0x7F) {
    uint8_t rel_addr = get8(cpu, ip_cs);
    
    if (check_cond_jmp(cpu, ins_bytes[0])) {
      short_jump(cpu, rel_addr);
      cpu->cycles += 16;
    }
    else {
      cpu->cycles += 4;
    }
  }
  else {
    cpu->e = E8086_BAD_OPCODE;
  }

  regseg_add(&cpu->regs[REG8086_IP], &cpu->regs[REG8086_CS], ip_add);
  return cpu->e != E8086_OK;
}

/*
=============================================================================================
                                        TESTS
=============================================================================================
*/

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
    0x43 /* INC BX */
  };

  HOPE_THAT(
    init_mem8086(&mem, 0),
    "Memory initialized."
  );

  HOPE_THAT(
    reset_cpu8086(&cpu, &mem),
    "CPU initialized."
  );
  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_CS].x = 0;
  cpu.regs[REG8086_SS].x = 0xF000;
  cpu.regs[REG8086_SP].x = 0xFFF0;

  memcpy(mem.bytes, code, sizeof (code));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x4269,
    "MOV AX, 0x4269."
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    mem.bytes[0xFFFEE] == 0x69 && mem.bytes[0xFFFEF] == 0x42,
    "PUSH AX."
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 0x4269 && cpu.regs[REG8086_SP].x == 0xFFF0,
    "POP BX => BX=0x4269 && SP is back."
  );
  
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 0x426A,
    "INC BX => BX=0x426A."
  );

  free_mem(&mem);
}

void add_cpu8086_tests(void) {
  ADD_TEST(test_regseg);
  ADD_TEST(test_cycling0);
}
