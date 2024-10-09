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
  REG8086_IP = REG8086_PRIVATE,
  REG8086_F

};

/* Error codes */
enum {
  E8086_OK, /* No error. */
  E8086_UNDEFINED, /* Misc error for undefined operation/behavior. */
  E8086_ACCESS_VIOLATION, /* The CPU would've accessed outside 1MB. */
  E8086_BAD_OPCODE, /* A bad opcode. */
  E8086_CUT_OFF /* The instruction was cut off. */
};

enum {
  F8086_CY = 1 << 0, /* Carry */
  F8086_P = 1 << 2, /* Parity */
  F8086_AC = 1 << 4, /* Auxiliary carry */
  F8086_Z = 1 << 6, /* Zero */
  F8086_S = 1 << 7, /* Sign */
  F8086_T = 1 << 8, /* Trap/Debug mode? */
  F8086_I = 1 << 9, /* Interrupts enabled? */
  F8086_D = 1 << 10, /* Direction is higher memory to lower? for strings */
  F8086_O = 1 << 11 /* Overflow? */
};

typedef struct {
  mem_t* mem; /* Pointer to the used mem */
  reg8086_t regs[16];
  uint64_t cycles; /* How many cycles passed. */
  int e; /* Error code */
} cpu8086_t;

/*
  Returns 0 if MEM is invalid.
  You should use init_mem8086() for MEM, manual also ok but must adhere to standard 8086 mapping.
*/
int reset_cpu8086(cpu8086_t* __restrict cpu, mem_t* __restrict mem);
void interrupt_cpu8086(cpu8086_t* cpu, uint16_t sig);
/*
  The function is not necessarily 1 cycle, it simply running the current instruction.
  Returns whether or not an error occured, in which case the vm SHOULD handle it via CPU->e.
  Errors are irrecoverable, the internal logic tries to always recover from any weird stuff, but
  in some cases the error is so bad that it causes an incomplete operation, or the intended result
  will not happen.
*/
int cycle_cpu8086(cpu8086_t* cpu);

void add_cpu8086_tests(void);

#endif
