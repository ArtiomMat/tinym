/* cpu8086 : As the name suggests, 8086 architecture things. */

#ifndef CPU_H
#define CPU_H

#include "mem.h"

#include <stdint.h>

/* General purpose register. */
typedef union {
  uint8_t p[2]; /* Lower is [0] and upper is [1] */
  uint16_t x; /* 16 bit part. */
} reg8086_t;

enum {
  REG8086_GENERAL, /* The index of AX, compatible with REGW_* */
  REG8086_AX = REG8086_GENERAL,
  REG8086_CX,
  REG8086_DX,
  REG8086_BX,
  
  REG8086_SP,
  REG8086_BP,

  REG8086_SI,
  REG8086_DI,
  
  REG8086_SEGMENTS, /* The index of ES, compatible with SEG_* */
  REG8086_ES = REG8086_SEGMENTS,
  REG8086_CS,
  REG8086_SS,
  REG8086_DS,

  REG8086_PRIVATE, /* The index of IP, private registers. */
  REG8086_IP = REG8086_PRIVATE
};

typedef struct {
  mem_t* mem; /* Pointer to the used mem */
  reg8086_t regs[16];
  char enable_ivt; /* Whether or not interrupts are enabled. */
  uint64_t cycles; /* How many cycles passed. */
} cpu8086_t;

/*
  Returns 0 if MEM is invalid.
  You should use init_mem8086() for MEM, manual also ok but must adhere to standard 8086 mapping.
*/
int reset_cpu8086(cpu8086_t* cpu, mem_t* mem);
void interrupt_cpu8086(cpu8086_t* cpu, uint16_t sig);
/*
  The function is not necessarily 1 cycle, it simply running the current instruction.
*/
int cycle_cpu8086(cpu8086_t* cpu);

#endif
