#include "cpu8086.h"
#include "ops.h"
#include "test.h"

#include <string.h>

int reset_cpu8086(cpu8086_t* cpu, mem_t* mem) {
  /* Set up register starting points */
  memset(cpu->regs, 0, sizeof (cpu->regs));
  cpu->regs[REG8086_CS].x = 0xFFFF;
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
static void regseg_add(reg8086_t* reg, reg8086_t* seg, uint16_t addr) {
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
static void regseg_sub(reg8086_t* reg, reg8086_t* seg, uint16_t addr) {
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

  uint32_t ip_cs = regseg(cpu->regs[REG8086_IP], cpu->regs[REG8086_CS]);

  uint8_t* bytes = mem->bytes + ip_cs;
  uint8_t opcode = (mem->bytes[ip_cs] & 0xFC) >> 2;
  uint8_t d_bit = (mem->bytes[ip_cs] & 0x2) >> 1;
  uint8_t w_bit = mem->bytes[ip_cs] & 0x1;

  /* Instructions that simply act on the 16-bit registers in order */
  /* INC R16 */
  if (opcode >= 0x40 && opcode <= 0x47) {
    ++ (cpu->regs[REG8086_AX + (opcode - 0x40)].x);
  }
  /* DEC R16 */
  else if (opcode >= 0x48 && opcode <= 0x4F) {
    -- (cpu->regs[REG8086_AX + (opcode - 0x48)].x);
  }
  /* PUSH R16 */
  else if (opcode >= 0x50 && opcode <= 0x57) {
    reg8086_t reg = cpu->regs[REG8086_AX + (opcode - 0x50)];
    /* Get stack pointer with segmentation - 2 */
    uint32_t stack_ptr = regseg_imm(cpu->regs[REG8086_SP].x, cpu->regs[REG8086_SS].x);
    
    stack_ptr -= 2;

    if (stack_ptr > MB1) {
      return E8086_ACCESS_VIOLATION;
    }

    regseg_sub(cpu->regs + REG8086_SP, cpu->regs + REG8086_SS, 2);

    mem->bytes[stack_ptr + 0] = reg.p[0];
    mem->bytes[stack_ptr + 1] = reg.p[1];
  }

  /* Check opcode. */
  switch (opcode) {
    case 0x0: /* ADD */
    
    break;
  }

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

void add_cpu8086_tests(void) {
  ADD_TEST(test_regseg);
}
