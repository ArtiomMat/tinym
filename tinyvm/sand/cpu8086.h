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
  REG8086_AX = REG8086_GENERAL, /* (REGW_* - REGW_AX) MUST EQUAL (REG8086_* - REGW_AX) */
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
  REG8086_F,

  REG8086_NULL /* Used internally to signal "no register, for stuff like prefixing of segment registers." */
};

/* Error codes */
enum {
  E8086_OK, /* No error. */
  E8086_UNDEFINED, /* Misc error for undefined operation/behavior. */
  E8086_ACCESS_VIOLATION, /* The CPU would've accessed outside 1MB. */
  E8086_UNALIGNED, /* Access to memory was unaligned, and not part of 8086 spec. */
  E8086_BAD_OPCODE, /* A bad opcode. */
  E8086_CUT_OFF /* The instruction was cut off. */
};

/* Interrupts */
enum {
  I8086_NULL = -1,
  I8086_DIVZ /* Zero division. */
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
  uint64_t cycles; /* How many cycles passed. */
  reg8086_t regs[16];
  int16_t i; /* Interrupt that was called. */
  
  /* cycle_cpu8086() stuff for helper functions to have easy access. Reset and recalculated each cycle. */
  uint32_t ip_cs; /* Address of current instruction. Reset to REGSEG_IP_CS(CPU). */
  uint8_t e; /* Error code. Reset to E8086_OK. */
  uint8_t ip_add; /* How much to be added to IP:CS if instruction succeeds. Reset to 1. */
  uint8_t seg; /* If REG8086_NULL use default sreg, else this is the sreg prefixed. Reset to REG8086_NULL. */
} cpu8086_t;

/*
  Returns 0 if MEM is invalid.
  You should use init_mem8086() for MEM, manual also ok but must adhere to standard 8086 mapping.
*/
int reset_cpu8086(cpu8086_t* cpu, mem_t* mem);
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
