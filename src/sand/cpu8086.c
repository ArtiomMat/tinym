#include "cpu8086.h"

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

static uint32_t regseg_imm(uint16_t reg, uint16_t seg) {
  return reg + (seg << 4);
}
/* Joins a register and a segment register into an address. */
static uint32_t regseg(reg8086_t reg, reg8086_t seg) {
  return regseg_imm(reg.x, seg.x);
}

int cycle_cpu8086(cpu8086_t* cpu) {
  mem_t* mem = cpu->mem;


  uint32_t ip_cs = regseg(cpu->regs[REG8086_IP], cpu->regs[REG8086_CS]);



  return 1;
}
