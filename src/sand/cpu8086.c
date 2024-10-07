#include "cpu8086.h"

#include <string.h>

void reset_cpu8086(cpu8086_t* cpu) {
  /* Set up register starting points */
  memset(cpu->regs, 0, sizeof (cpu->regs));
  cpu->regs[REG8086_CS].x = 0xFFFF;
  cpu->enable_ivt = 1;
}

/* Analyzes the opcode and determines information */
/* static unsigned analyze_opcode(uint8_t opcode) {

// }*/

int tick_cpu8086(cpu8086_t* cpu, mem_t* mem) {
  /* A minimum size for ram. */
  if (mem->size < MEGABYTE) {
    return 0;
  }



  return 1;
}
