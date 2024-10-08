#include "cpu8086.h"
#include "ops.h"
#include "test.h"

#include <string.h>

#define REGSEG_SP_SS(CPU) regseg_imm(CPU->regs[REG8086_SP].x, CPU->regs[REG8086_SS].x)
#define REGSEG_IP_CS(CPU) regseg_imm(CPU->regs[REG8086_IP].x, CPU->regs[REG8086_CS].x)

int reset_cpu8086(cpu8086_t* __restrict cpu, mem_t* __restrict mem) {
  /* Set up register starting points */
  memset(cpu->regs, 0, sizeof (cpu->regs));
  cpu->regs[REG8086_CS].x = 0xFFFF;

  cpu->regs[REG8086_SP].x = 0xFFFF; /* TODO: Not an actual standard config. */
  
  cpu->enable_ivt = 1;

  /* A minimum size for addressable memory. */
  if (mem->size < MB1) {
    return 0;
  }
  cpu->mem = mem;

  return 1;
}

/* Analyzes the opcode and determines information */
/* static unsigned analyze_opcode(uint8_t opcode) {

// }*/

/*
  Joins 2 immidiete values into a segmented 20-bit address.
  The rest of the 12 bits returned will be 0, to avoid potential buffer overflow.
*/
static uint32_t regseg_imm(uint16_t reg, uint16_t seg) {
  return (reg + (seg << 4)) & 0xFFFFF;
}
/*
  Joins a register and a segment register into a segmented 20-bit address.
  The rest of the 12 bits returned will be 0, to avoid potential buffer overflow.
*/
static uint32_t regseg(reg8086_t reg, reg8086_t seg) {
  return regseg_imm(reg.x, seg.x);
}
/*
  Adds ADDR to the REG+SEG in a way that prevents overflow.
  E.G if ADDR=1, REG=0xFFFF then we need to do *SEG+=0x10000 and REG=0
*/
static void regseg_add(reg8086_t* __restrict reg, reg8086_t* __restrict seg, uint16_t addr) {
  if ((uint32_t)reg->x + addr > 0xFFFF) {
    reg->x = (0x0000 + addr) - 1;
    seg->x += 0x1000;
  }
  else {
    reg->x += addr;
  }
}
/*
  Subtract ADDR to the REG+SEG in a way that prevents undeflow.
  E.G if ADDR=1, *REG=0x0 then we need to do *SEG-=0x10000 and *REG=0xFFFF
*/
static void regseg_sub(reg8086_t* __restrict reg, reg8086_t* __restrict seg, uint16_t addr) {
  if ((int32_t)reg->x - addr < 0) {
    reg->x = (0xFFFF - addr) + reg->x;
    seg->x -= 0x1000;
  }
  else {
    reg->x -= addr;
  }
}

int cycle_cpu8086(cpu8086_t* cpu) {
  mem_t* mem = cpu->mem;

  uint32_t ip_cs = REGSEG_IP_CS(cpu);

  uint8_t* bytes = mem->bytes + ip_cs;
  uint8_t opcode = (bytes[0] & 0xFC) >> 2;
  uint8_t d_bit = (bytes[0] & 0x2) >> 1;
  uint8_t w_bit = bytes[0] & 0x1;

  uint8_t ip_add = 0; /* How much to add if reading of instruction succeeded */

  /* Instructions that simply act on the 16-bit registers in order. */
  /* INC R16 */
  if (bytes[0] >= 0x40 && bytes[0] <= 0x47) {
    ++ (cpu->regs[REG8086_AX + (bytes[0] - 0x40)].x);
    ip_add = 1;
  }
  /* DEC R16 */
  else if (bytes[0] >= 0x48 && bytes[0] <= 0x4F) {
    -- (cpu->regs[REG8086_AX + (bytes[0] - 0x48)].x);
    ip_add = 1;
  }
  /* PUSH R16 */
  else if (bytes[0] >= 0x50 && bytes[0] <= 0x57) {
    /* Get stack pointer with segmentation - 2 */
    uint32_t stack_ptr = REGSEG_SP_SS(cpu);
    stack_ptr -= 2;

    /* It wrapped around */
    if (stack_ptr >= MB1) {
      return E8086_ACCESS_VIOLATION;
    }

    regseg_sub(&cpu->regs[REG8086_SP], &cpu->regs[REG8086_SS], 2);

    /* Now copy the register contents */
    memcpy(mem->bytes + stack_ptr, &cpu->regs[REG8086_AX + (bytes[0] - 0x50)], 2);

    ip_add = 1;
  }
  /* POP R16 */
  else if (bytes[0] >= 0x58 && bytes[0] <= 0x5F) {
    /* Get stack pointer with segmentation */
    uint32_t stack_ptr = REGSEG_SP_SS(cpu);

    /* Copy the content before modifying stack_ptr */
    memcpy(&cpu->regs[REG8086_AX + (bytes[0] - 0x58)], mem->bytes + stack_ptr, 2);

    regseg_add(&cpu->regs[REG8086_SP], &cpu->regs[REG8086_SS], 2);

    ip_add = 1;
  }
  /* MOV R16, IMM16 */
  else if (bytes[0] >= 0xB8 && bytes[0] <= 0xBF) {
    uint16_t v;

    if (ip_cs + 3 >= MB1) {
      return E8086_CUT_OFF;
    }
    
    /* Construct value. */
    v = mem->bytes[ip_cs + 1] + (mem->bytes[ip_cs + 2] << 8);
    
    cpu->regs[REG8086_AX + (bytes[0] - 0xB8)].x = v;
    
    ip_add = 3;
  }

  /* Check opcode. */
  switch (opcode) {
    case 0x0: /* ADD */
    
    break;
  }

  regseg_add(&cpu->regs[REG8086_IP], &cpu->regs[REG8086_CS], ip_add);
  return E8086_OK;
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

  /* Now for a final simple test of regseg() */
  HOPE_THAT(
    regseg_imm(0xDEAD, 0xBEEF) == (0xDEAD) + 0xBEEF0,
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
    "Memory initialized"
  );

  HOPE_THAT(
    reset_cpu8086(&cpu, &mem),
    "CPU initialized"
  );
  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_CS].x = 0;
  cpu.regs[REG8086_SS].x = 0xF000;
  cpu.regs[REG8086_SP].x = 0xFFF0;

  memcpy(mem.bytes, code, sizeof (code));
  

  cycle_cpu8086(&cpu);
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x4269,
    "MOV AX, 0x4269"
  );

  cycle_cpu8086(&cpu);
  HOPE_THAT(
    mem.bytes[0xFFFEE] == 0x69 && mem.bytes[0xFFFEF] == 0x42,
    "PUSH AX"
  );

  cycle_cpu8086(&cpu);
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 0x4269 && cpu.regs[REG8086_SP].x == 0xFFF0,
    "POP BX => BX=0x4269 && SP is back."
  );
  
  cycle_cpu8086(&cpu);
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 0x426A,
    "INC BX => BX=0x426A"
  );

  free_mem(&mem);
}

void add_cpu8086_tests(void) {
  ADD_TEST(test_regseg);
  ADD_TEST(test_cycling0);
}
