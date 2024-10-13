/*
  NOTE TO DEV: Every function here must be safe yet performant, isolated anarchy in the CPU is
  better than anarchy in the VM due to out-of-bounds access.
*/

#include "cpu8086.h"
#include "modrm.h"
#include "test.h"
#include "util.h"
#include <stdio.h>

#include <string.h>

#define REGSEG_SP_SS(CPU) regseg_imm(CPU, CPU->regs[REG8086_SP].x, CPU->regs[REG8086_SS].x)
#define REGSEG_IP_CS(CPU) regseg_imm(CPU, CPU->regs[REG8086_IP].x, CPU->regs[REG8086_CS].x)
#define REGSEG_IP_CS_ADD(CPU, N) regseg_add(&CPU->regs[REG8086_IP], &CPU->regs[REG8086_CS], N)
#define REGSEG_IP_CS_SUB(CPU, N) regseg_sub(&CPU->regs[REG8086_IP], &CPU->regs[REG8086_CS], N)
/* ES is DEFS */
#define GET_SEG_ES(CPU) get_seg(CPU, REG8086_ES)
/* CS is DEFS */
#define GET_SEG_CS(CPU) get_seg(CPU, REG8086_CS)
/* SS is DEFS */
#define GET_SEG_SS(CPU) get_seg(CPU, REG8086_SS)
/* DS is DEFS */
#define GET_SEG_DS(CPU) get_seg(CPU, REG8086_DS)

static const uint8_t parity_table[] = {
  0x69, 0x96, 0x96, 0x69, 0x96, 0x69, 0x69, 0x96, 0x96, 0x69, 0x69, 0x96, 0x69, 0x96, 0x96, 0x69, 
  0x96, 0x69, 0x69, 0x96, 0x69, 0x96, 0x96, 0x69, 0x69, 0x96, 0x96, 0x69, 0x96, 0x69, 0x69, 0x96
};

int reset_cpu8086(cpu8086_t* cpu, mem_t* mem) {
  /* Set up register starting points */
  memset(cpu->regs, 0, sizeof (cpu->regs));
  cpu->regs[REG8086_CS].x = 0xFFFF;
  cpu->regs[REG8086_SP].x = 0xFFFE;
  cpu->regs[REG8086_F].x |= F8086_I;

  /* A minimum size for addressable memory. */
  if (mem == NULL || mem->size < MB1) {
    return 0;
  }
  cpu->mem = mem;

  return 1;
}


/*
  Returns the sreg, if CPU->seg has been specified(mainly via prefixes), that will be used as 
  override, otherwise the default, DEFS is the default REG8086_* that should be used.
*/
static reg8086_t* get_seg(cpu8086_t* cpu, const int defs) {
  if (cpu->seg != REG8086_NULL) {
    return cpu->regs + cpu->seg;
  }
  return cpu->regs + defs;
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
static void regseg_add(reg8086_t* reg, reg8086_t* seg, const uint16_t addr) {
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
static void regseg_sub(reg8086_t* reg, reg8086_t* seg, const uint16_t addr) {
  if ((int32_t)reg->x - addr < 0) {
    reg->x = (0xFFFF - addr) + reg->x;
    seg->x -= 0x1000;
  }
  else {
    reg->x -= addr;
  }
}

/* SAFE 1MB ACCESS. 8 bit equivalent of get16(). */
static uint8_t get8(cpu8086_t* cpu, const uint32_t addr) {
  if (addr >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return 0;
  }
  
  return cpu->mem->bytes[addr];
}
/* SAFE 1MB ACCESS. 8 bit equivalent of put16(). */
static void put8(cpu8086_t* cpu, const uint32_t addr, const uint16_t what) {
  if (addr >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return;
  }

  cpu->mem->bytes[addr] = what;
}
/*
  SAFE 1MB ACCESS. 8 bit equivalent of mem_get16(), but there is ofc no need for unaligned checks.
*/
static uint8_t* mem_get8(cpu8086_t* cpu, uint32_t addr, const int defs) {
  addr = regseg_imm(cpu, addr, get_seg(cpu, defs)->x); /* Combine with the sreg */

  if (addr >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return cpu->mem->bytes;
  }
  
  return cpu->mem->bytes + addr;
}

/*
  SAFE 1MB ACCESS. Get a value from CPU->mem[ADDR]. Preffered over raw access for: safety(will 
  CPU->e and return invalid but safely constructed value) & deals with unaligned address.
  As mentioned above, may set CPU->e to E8086_ACCESS_VIOLATION.
*/
static uint16_t get16(cpu8086_t* cpu, uint32_t addr) {
  uint16_t ret;
  uint8_t* p = cpu->mem->bytes + addr;

  if (addr >= MB1 - 1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return 0;
  }
  
  ret = (uint16_t)p[0] | ((uint16_t)(p[1]) << 8);

  return ret;
}
/*
  SAFE 1MB ACCESS. Get direct ptr of CPU->mem[ADDR]. Preffered over raw access for: safety(will 
  CPU->e and return undefined but non NULL/unsafe address) & easier for unaligned access.
  Emulates memory access.
  DEFS must be REG8086_*, it's the default sreg used, pop needs mem_get8/16() + regular addressing.
  As mentioned above, may set CPU->e to E8086_ACCESS_VIOLATION, or E8086_UNALIGNED.
*/
static uint16_t* mem_get16(cpu8086_t* cpu, uint32_t addr, const int defs) {
  addr = regseg_imm(cpu, addr, get_seg(cpu, defs)->x); /* Combine with the sreg */

  if (addr >= MB1 - 1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return (uint16_t*)(cpu->mem->bytes);
  }
  else if (addr & 1) {
    cpu->e = E8086_UNALIGNED;
    addr &= ~1;
  }
  
  return (uint16_t*)(cpu->mem->bytes + addr);
}
/*
  SAFE 1MB ACCESS. Set CPU->mem[ADDR] = what. Preffered over raw access for: safety(will CPU->e 
  and will not set) & easier for unaligned access.
  As mentioned above, may set CPU->e to E8086_ACCESS_VIOLATION.
*/
static void put16(cpu8086_t* cpu, const uint32_t addr, const uint16_t what) {
  uint8_t* p = cpu->mem->bytes + addr;

  if (addr >= MB1 - 1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return;
  }
  
  /* memcpy(cpu->mem->bytes + addr, &what, 2); */

  p[0] = what;
  p[1] = what >> 8;
}

/*
  SAFE 1MB ACCESS. Pushes X into CPU->mem using CPU's stack registers.
  May set CPU->e to E8086_ACCESS_VIOLATION if the stack wrapped around and wont push. Uses regseg() so may set to its errors.
*/
static void push16(cpu8086_t* cpu, const uint16_t x) {
  uint32_t stack_ptr = regseg_imm(cpu, cpu->regs[REG8086_SP].x, GET_SEG_SS(cpu)->x);

  stack_ptr -= 2; /* Since gotta first decrement */
  regseg_sub(&cpu->regs[REG8086_SP], GET_SEG_SS(cpu), 2);

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
  uint32_t stack_ptr = regseg_imm(cpu, cpu->regs[REG8086_SP].x, GET_SEG_SS(cpu)->x);

  /* Copy the content before modifying stack_ptr */
  ret = *mem_get16(cpu, stack_ptr, REG8086_SS);

  /* Add 2 back */
  regseg_add(&cpu->regs[REG8086_SP], GET_SEG_SS(cpu), 2);

  return ret;
}

/*
  Updates CPU's flags accroding to _RESULT, of any operation that updates flags.
  NOTE: _RESULT is uint32_t to allow for accurate range detection within W's context,
  make sure that you use+pass int32_t or uint32_t, and not 16/8 ints, because overflow
  would not be visible, hence nullifying the arithmetic analysis.
  W is a boolean value and determines if it was a 16 bit operation(1) or 8 bit(0).
*/
static void update_flags(cpu8086_t* cpu, const uint32_t result, const int w) {
  cpu->regs[REG8086_F].x &= F8086_I | F8086_D | F8086_T;

  if (!result) {
    cpu->regs[REG8086_F].x |= F8086_Z;
  }
  else if (
    (w && ((uint16_t)result) & 0x8000)
    || (!w && ((uint8_t)result) & 0x80)
  ) {
    cpu->regs[REG8086_F].x |= F8086_S;
  }

  if (
    (w && get_arr_bit(parity_table, ((uint16_t)result) & 0xFF))
    || (!w && get_arr_bit(parity_table, ((uint8_t)result) & 0xFF))
  ) {
    cpu->regs[REG8086_F].x |= F8086_P;
  }

  if (w) {
    if (((int32_t)result) > 32767 || ((int32_t)result) < -32768) {
      cpu->regs[REG8086_F].x |= F8086_O;
    }
    if (result > 65535) {
      cpu->regs[REG8086_F].x |= F8086_CY;
    }
  }
  else {
    if (((int32_t)result) > 127 || ((int32_t)result) < -128) {
      cpu->regs[REG8086_F].x |= F8086_O;
    }
    if (result > 255) {
      cpu->regs[REG8086_F].x |= F8086_CY;
    }
  }
}

/* Does not add cycles because not enough info. */
static uint32_t add_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a + b;
  update_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t adc_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t carry = !!(cpu->regs[REG8086_F].x & F8086_CY);
  uint32_t result = a + b + carry;
  update_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t sub_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a - b;
  update_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t sbb_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t carry = !!(cpu->regs[REG8086_F].x & F8086_CY);
  uint32_t result = a - b - carry;
  update_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t and_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a & b;
  update_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t or_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a | b;
  update_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t xor_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a ^ b;
  update_flags(cpu, result, w);
  return result;
}
/* Only affects flags. Does not add cycles because not enough info. */
static void cmp_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a - b;
  update_flags(cpu, result, w);
}
/* Only affects flags. Does not add cycles because not enough info. */
static void test_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a & b;
  update_flags(cpu, result, w);
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

  if (rel_addr > 0) {
    REGSEG_IP_CS_ADD(cpu, rel_addr);
  }
  else {
    REGSEG_IP_CS_SUB(cpu, -rel_addr);
  }
}

/*
  Returns whether according to the first instruction byte the CPU should do conditional jump.
  OPCODE is the opcode byte, it must be 0x70-0x7F inclusive.
*/
static int check_cond_jmp(cpu8086_t* cpu, uint8_t opcode) {
  int do_jump;
  uint16_t f = cpu->regs[REG8086_F].x;

  /*
    A pattern is that if (opcode&1) is 1 then it's the NOT equivalent.
    I exploit it to less boilerplate code for both the normal and NOT versions.
  */
  switch (opcode & 0xF) {
    case 0x00: /* JO */
    case 0x01: /* JNO */
    do_jump = (opcode&1) ^ (!!(f & F8086_O));
    break;

    case 0x02: /* JC/JB/JNAE */
    case 0x03: /* JNC/JNB/JAE */
    do_jump = (opcode&1) ^ (!!(f & F8086_CY));
    break;

    case 0x04: /* JE/JZ */
    case 0x05: /* JNE/JNZ */
    do_jump = (opcode&1) ^ (!!(f & F8086_Z));
    break;
    
    case 0x06: /* JBE/JNA */
    case 0x07: /* JA/JNBE */
    do_jump = (opcode&1) ^ ((!!(f & F8086_CY)) | (!!(f & F8086_Z))); 
    break;

    case 0x08: /* JS */
    case 0x09: /* JNS */
    do_jump = (opcode&1) ^ (!!(f & F8086_S));
    break;

    case 0x0A: /* JP/JPE */
    case 0x0B: /* JNP/JPO */
    do_jump = (opcode&1) ^ (!!(f & F8086_P));
    break;

    case 0x0C: /* JL/JNGE */
    case 0x0D: /* JGE/JNL */
    do_jump = (opcode&1) ^ (!!(f & F8086_S)) ^ (!!(f & F8086_O));
    break;

    case 0x0E: /* JLE/JNG */
    case 0x0F: /* JGE/J */
    do_jump = (opcode&1) ^ (((!!(f & F8086_S)) ^ (!!(f & F8086_O))) | (!!(f & F8086_Z)));
    break;
  }

  return do_jump;
}

/*
  * CPU->ip_cs is expected to be at the first byte, not modrm.
  * MODRM is the modr/m byte.
  * W_BIT indicates if it's a word operation, and hence the size of the data in the pointer.
  * Returns a pointer to the register referenced by the REG field.
*/
static void* get_modrm_reg(cpu8086_t* cpu, uint8_t modrm) {
  /* Since REG8086_* is compatible + Little-endain makes stuff easier. */
  return &cpu->regs[(modrm & MODRM_REG_MASK) >> 3].x;
}

/*
  * CPU->ip_cs is expected to be at the first byte, not modrm.
  * MODRM is the modr/m byte.
  * W_BIT indicates if it's a word operation, and hence the size of the data in the pointer.
  * Returns a pointer to the *SEGMENT REGISTER* referenced by the REG field.
*/
static void* get_modrm_seg(cpu8086_t* cpu, uint8_t modrm) {
  switch (modrm & MODRM_REG_MASK) {
    case SEG_ES:
    return &cpu->regs[REG8086_ES];
    case SEG_CS:
    return &cpu->regs[REG8086_CS];
    case SEG_SS:
    return &cpu->regs[REG8086_SS];
    default:
    return &cpu->regs[REG8086_DS];
  }
}

/*
  * CPU->ip_cs is expected to be at the first byte, not modrm.
  * MODRM is the modr/m byte.
  * W_BIT indicates if it's a word operation, and hence the size of the data in the pointer.
  * Returns a pointer to what the R/M points to, e.g address in CPU->mem, or a register.
*/
static void* get_modrm_rm(cpu8086_t* cpu, uint8_t modrm, uint8_t w_bit) {
  void* rm;
  uint32_t addr;

  /* R/M is REG, MOD=3 */
  if ((modrm & MODRM_MOD_MASK) == MOD_RM_IS_REG) {
    /* 3 bits right fit the REG mask */
    rm = &cpu->regs[(modrm << 3 & MODRM_REG_MASK) >> 3].x;
    return rm;
  }
  /* Direct addressing, MOD=0,R/M=6 */
  else if ((modrm & MODRM_MOD_MASK) == MOD_NDISP && (modrm & MODRM_RM_MASK) == RM_BP) {
    addr = regseg_imm(cpu, get16(cpu, cpu->ip_cs + 2), cpu->regs[REG8086_DS].x);
    cpu->ip_add = 4;
  }
  /* Using various register-displacement combinations depending on MOD+R/M */
  else {
    /* Add the register stuff */
    switch (modrm & MODRM_RM_MASK) {
      case RM_BX_SI:
      addr = regseg_imm(cpu, cpu->regs[REG8086_BX].x + cpu->regs[REG8086_SI].x, GET_SEG_DS(cpu)->x);
      break;
      case RM_BX_DI:
      addr = regseg_imm(cpu, cpu->regs[REG8086_BX].x + cpu->regs[REG8086_DI].x, GET_SEG_DS(cpu)->x);
      break;
      case RM_BP_SI:
      addr = regseg_imm(cpu, cpu->regs[REG8086_BP].x + cpu->regs[REG8086_SI].x, GET_SEG_SS(cpu)->x);
      break;
      case RM_BP_DI:
      addr = regseg_imm(cpu, cpu->regs[REG8086_BP].x + cpu->regs[REG8086_DI].x, GET_SEG_SS(cpu)->x);
      break;
      case RM_SI:
      addr = regseg_imm(cpu, cpu->regs[REG8086_SI].x, GET_SEG_DS(cpu)->x);
      break;
      case RM_DI:
      addr = regseg_imm(cpu, cpu->regs[REG8086_DI].x, GET_SEG_DS(cpu)->x);
      break;
      case RM_BP:
      addr = regseg_imm(cpu, cpu->regs[REG8086_BP].x, GET_SEG_SS(cpu)->x);
      break;
      case RM_BX:
      addr = regseg_imm(cpu, cpu->regs[REG8086_BX].x, GET_SEG_DS(cpu)->x);
      break;
    }
    /* Add displacement */
    switch (modrm & MODRM_MOD_MASK) {
      case MOD_DISP16:
      addr += get16(cpu, cpu->ip_cs + 2);
      break;

      case MOD_DISP8:
      addr += get8(cpu, cpu->ip_cs + 2);
      break;
      
      default:
      break;
    }
  }

  if (w_bit) {
    rm = mem_get16(cpu, addr, REG8086_DS);
  }
  else {
    rm = mem_get8(cpu, addr, REG8086_DS);
  }

  return rm;
}
/*
  Generalizes uint32_t *_ins() to take from *DST and *SRC uint16_t/uint8_t depending on W.
  And stores the return value into *DST. This is done to avoid too much boilerplate.
*/
#define UNSIGNED_INS_DSTSRC_0030(CPU, OPCODE, INS)\
  {\
    void* _dst_, *_src_;\
    int _w_bit_ = get_dstsrc_0030(CPU, OPCODE, &_dst_, &_src_);\
    if (_w_bit_) {\
      *(uint16_t*)_dst_ = INS(CPU, *(uint16_t*)_dst_, *(uint16_t*)_src_, _w_bit_);\
    }\
    else {\
      *(uint8_t*)_dst_ = INS(CPU, *(uint8_t*)_dst_, *(uint8_t*)_src_, _w_bit_);\
    }\
  }
/*
  Same as UNSIGNED_INS_DSTSRC_0030() but for the void functions that don't actually modify *DST.
*/
#define VOID_INS_DSTSRC_0030(CPU, OPCODE, INS)\
  {\
    void* _dst_, *_src_;\
    int _w_bit_ = get_dstsrc_0030(CPU, OPCODE, &_dst_, &_src_);\
    if (_w_bit_) {\
      INS(CPU, *(uint16_t*)_dst_, *(uint16_t*)_src_, _w_bit_);\
    }\
    else {\
      INS(CPU, *(uint8_t*)_dst_, *(uint8_t*)_src_, _w_bit_);\
    }\
  }
/*
  * A specific format of instructions that appear in 0x00-0x30, like ADD in the start, or CMP in the end, they are distinct and can be generalized to this source-destination retrieval.
  * It's also applicable to MOV in 0x88-0x8B, even through it doesn't have the AL-AX thingy, due 
  * to modulu-like design, and 8086 aligned stuff nicely.

  * *SRC is the source(copied/utilized/constant) operand.
  * *DST is (usually, but sometimes theoretically) modified operand.
  * OPCODE is the opcode byte.
  
  * Sets CPU->ip_add automaically.

  * Returns if W bit was on, so you know if *SRC and *DST are uint16_t or uint8_t.
*/
static int get_dstsrc_0030(cpu8086_t* cpu, uint8_t opcode, void** dst, void** src) {
  /*
    In case we need to return copies/constants, that aren't easily referencable, like immidietes 
    that may be unaligned.
  */
  static uint16_t s_src;

  uint8_t i_bits = (opcode & 0xFC) >> 2;
  uint8_t d_bit = (opcode & 0x2) >> 1;
  uint8_t w_bit = opcode & 0x1;
  uint8_t modrm;
  

  /* If 3rd bit on it's the INS AL/AX, IMM8/IMM16 thingy, works for both 0x04/5 and 0x0C/D. */
  if (i_bits & 0x1) {
    s_src = w_bit ? get16(cpu, cpu->ip_cs + 1) : get8(cpu, cpu->ip_cs + 1);
    *dst = &cpu->regs[REG8086_AX].x; /* &p[0] is &x so no checks. Little-endian rules. */
    *src = &s_src;
    cpu->ip_add = 2 + w_bit;
    return w_bit;
  }

  modrm = get8(cpu, cpu->ip_cs + 1);

  *dst = get_modrm_rm(cpu, modrm, w_bit);
  *src = get_modrm_reg(cpu, modrm);

  if (d_bit) {
    void* tmp = *src;
    *src = *dst;
    *dst = tmp;
  }

  /* CPU->ip_add */
  switch (modrm & MODRM_MOD_MASK) {
    case MOD_DISP16:
    cpu->ip_add = 4;
    break;
    case MOD_DISP8:
    cpu->ip_add = 3;
    break;

    case MOD_NDISP:
    /* Direct addressing mode */
    if ((modrm & MODRM_RM_MASK) == RM_BP) {
      cpu->ip_add = 4;
    }
    else {
      cpu->ip_add = 2;
    }
    break;

    default:
    cpu->ip_add = 2;
    break;
  }

  return w_bit;
}

int cycle_cpu8086(cpu8086_t* cpu) {
  uint8_t opcode;

  cpu->e = E8086_OK;
  cpu->ip_cs = REGSEG_IP_CS(cpu);
  cpu->ip_add = 1;
  cpu->seg = REG8086_NULL;

  opcode = get8(cpu, cpu->ip_cs); /* opcode byte */

  /* TODO: Make multi-prefix support, since it's possible I think? */
  /* Prefixes */
  switch (opcode) {
    case 0x26:
    cpu->seg = REG8086_ES;
    break;
    case 0x36:
    cpu->seg = REG8086_SS;
    break;
    case 0x2E:
    cpu->seg = REG8086_CS;
    break;
    case 0x3E:
    cpu->seg = REG8086_DS;
    break;
  }
  if (cpu->seg != REG8086_NULL) {
    REGSEG_IP_CS_ADD(cpu, 1);
    cpu->ip_cs = REGSEG_IP_CS(cpu);
    opcode = get8(cpu, cpu->ip_cs);
  }

  /* INC R16 */
  if (opcode >= 0x40 && opcode <= 0x47) {
    uint16_t* ptr = &cpu->regs[REG8086_AX + (opcode & 7)].x;
    *ptr = add_ins(cpu, *ptr, 1, 1);
    cpu->cycles += 2;
    cpu->ip_add = 1;
  }
  /* DEC R16 */
  else if (opcode >= 0x48 && opcode <= 0x4F) {
    uint16_t* ptr = &cpu->regs[REG8086_AX + (opcode & 7)].x;
    *ptr = sub_ins(cpu, *ptr, 1, 1);
    cpu->cycles += 2;
    cpu->ip_add = 1;
  }
  /* PUSH R16 */
  else if (opcode >= 0x50 && opcode <= 0x57) {
    push16(cpu, cpu->regs[REG8086_AX + (opcode & 7)].x);
    cpu->cycles += 11;
    cpu->ip_add = 1;
  }
  /* POP R16 */
  else if (opcode >= 0x58 && opcode <= 0x5F) {
    cpu->regs[REG8086_AX + (opcode & 7)].x = pop16(cpu);
    cpu->cycles += 8;
    cpu->ip_add = 1;
  }
  /* MOV R16, IMM16 */
  else if (opcode >= 0xB8 && opcode <= 0xBF) {
    cpu->regs[REG8086_AX + (opcode & 7)].x = get16(cpu, cpu->ip_cs + 1);
    
    cpu->cycles += 4;
    cpu->ip_add = 3;
  }
  /* MOV R8, IMM8 */
  else if (opcode >= 0xB0 && opcode <= 0xB7) {
    unsigned p_i = opcode >= 0xB4; /* It's the upper register parts if the second half */

    /* &3 for %4 because it's split to L and H */
    cpu->regs[REG8086_AX + (opcode & 3)].p[p_i] = get8(cpu, cpu->ip_cs + 1);
    
    cpu->cycles += 4;
    cpu->ip_add = 2;
  }
  /* MOV MODR/M */
  else if (opcode >= 0x88 && opcode <= 0x8B) {
    void* dst;
    void* src;
    /* W_BIT=1? */
    if (get_dstsrc_0030(cpu, opcode, &dst, &src)) {
      *(uint16_t*)dst = *(uint16_t*)src;
    }
    else {
      *(uint8_t*)dst = *(uint8_t*)src;
    }
  }
  /* ??? MODR/M or ??? AL/AX, IMM8/16 section */
  /* ADD */
  else if (/* opcode >= 0x00 && */ opcode <= 0x05) {
    UNSIGNED_INS_DSTSRC_0030(cpu, opcode, add_ins);
  }
  /* OR */
  else if (opcode >= 0x08 && opcode <= 0x0D) {
    UNSIGNED_INS_DSTSRC_0030(cpu, opcode, or_ins);
  }
  /* ADC */
  else if (opcode >= 0x10 && opcode <= 0x15) {
    UNSIGNED_INS_DSTSRC_0030(cpu, opcode, adc_ins);
  }
  /* SBB */
  else if (opcode >= 0x18 && opcode <= 0x1D) {
    UNSIGNED_INS_DSTSRC_0030(cpu, opcode, sbb_ins);
  }
  /* AND */
  else if (opcode >= 0x20 && opcode <= 0x25) {
    UNSIGNED_INS_DSTSRC_0030(cpu, opcode, and_ins);
  }
  /* SUB */
  else if (opcode >= 0x28 && opcode <= 0x2D) {
    UNSIGNED_INS_DSTSRC_0030(cpu, opcode, sub_ins);
  }
  /* XOR */
  else if (opcode >= 0x30 && opcode <= 0x35) {
    UNSIGNED_INS_DSTSRC_0030(cpu, opcode, xor_ins);
  }
  /* CMP */
  else if (opcode >= 0x38 && opcode <= 0x3D) {
    VOID_INS_DSTSRC_0030(cpu, opcode, cmp_ins);
  }
  /* Various conditional jumps, relative to current instruction. */
  else if (opcode >= 0x70 && opcode <= 0x7F) {
    uint8_t rel_addr = get8(cpu, cpu->ip_cs + 1);
    
    if (check_cond_jmp(cpu, opcode)) {
      short_jump(cpu, rel_addr);
      cpu->cycles += 16;
    }
    else {
      cpu->cycles += 4;
    }
    cpu->ip_add = 2;
  }
  /* MOV AL/AX, ADDR or MOV ADDR, AL/AX. So much hard-wired stuff man. */
  else if (opcode >= 0xA0 && opcode <= 0xA3) {
    uint8_t d_bit = (opcode & 0x2) >> 1;
    uint8_t w_bit = opcode & 0x1;
    uint16_t addr = get16(cpu, cpu->ip_cs + 1);
    void* ax_al = &cpu->regs[REG8086_AX]; /* Little-endian magic */
    /* XXX: I hate the way it looks. */
    if (d_bit) {
      if (w_bit) {
        *mem_get16(cpu, addr, REG8086_DS) = *(uint16_t*)ax_al;
      }
      else {
        *mem_get8(cpu, addr, REG8086_DS) = *(uint8_t*)ax_al;
      }
    }
    else {
      if (w_bit) {
        *(uint16_t*)ax_al = *mem_get16(cpu, addr, REG8086_DS);
      }
      else {
        *(uint8_t*)ax_al = *mem_get8(cpu, addr, REG8086_DS);
      }
    }

    cpu->ip_add = 3;
    cpu->cycles += 10;
  }
  /* UNKNOWN! */
  else {
    if (opcode == 0x66 || opcode == 0x67) {
      fputs("cycle_cpu8086(): you'll need i386 for that.", stderr);
    }
    cpu->cycles += 1;
  }

  REGSEG_IP_CS_ADD(cpu, cpu->ip_add);
  return cpu->e != E8086_OK;
}

#ifdef NDEBUG
  void add_cpu8086_tests(void) {}
#else
  #include "cpu8086_tests.h"
#endif