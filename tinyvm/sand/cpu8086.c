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
  cpu->regs[REG8086_F].x = F8086_I | F8086_D | F8086_T;

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
  uint32_t result = a ^ b;
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

  if (rel_addr < 0) {
    REGSEG_IP_CS_ADD(cpu, rel_addr);
  }
  else {
    REGSEG_IP_CS_SUB(cpu, rel_addr);
  }
}

/*
  Returns whether according to the first instruction byte the CPU should do conditional jump.
  INS_BYTE is the opcode byte, it must be 0x70-0x7F inclusive.
*/
static int check_cond_jmp(cpu8086_t* cpu, uint8_t ins_byte) {
  int do_jump;
  uint16_t f = cpu->regs[REG8086_F].x;

  /*
    A pattern is that if (ins_byte&1) is 1 then it's the NOT equivalent.
    I exploit it to less boilerplate code for both the normal and NOT versions.
  */
  switch (ins_byte & 0xF) {
    case 0x00: /* JO */
    case 0x01: /* JNO */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_O));
    break;

    case 0x02: /* JC/JB/JNAE */
    case 0x03: /* JNC/JNB/JAE */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_CY));
    break;

    case 0x04: /* JE/JZ */
    case 0x05: /* JNE/JNZ */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_Z));
    break;
    
    case 0x06: /* JBE/JNA */
    case 0x07: /* JA/JNBE */
    do_jump = (ins_byte&1) ^ ((!!(f & F8086_CY)) | (!!(f & F8086_Z))); 
    break;

    case 0x08: /* JS */
    case 0x09: /* JNS */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_S));
    break;

    case 0x0A: /* JP/JPE */
    case 0x0B: /* JNP/JPO */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_P));
    break;

    case 0x0C: /* JL/JNGE */
    case 0x0D: /* JGE/JNL */
    do_jump = (ins_byte&1) ^ (!!(f & F8086_S)) ^ (!!(f & F8086_O));
    break;

    case 0x0E: /* JLE/JNG */
    case 0x0F: /* JGE/J */
    do_jump = (ins_byte&1) ^ (((!!(f & F8086_S)) ^ (!!(f & F8086_O))) | (!!(f & F8086_Z)));
    break;
  }

  return do_jump;
}

/*
  * INS_BYTES must be a valid pointer within CPU->mem->bytes!!!
  * A specific format of instructions that appear in 0x00-0x30, like ADD in the start, or CMP in the end, they are distinct and can be generalized to this function.
  * It's also applicable to MOV in 0x88-0x8B, even through it doesn't have the AL-AX thingy.

  * *SRC is the source(copied/utilized/constant) operand.
  * *DST is (usually, but sometimes 
  theoretically) modified operand.
  * *IP_ADD(how much to add to the insturction-pointer) will be set according to the instruction.

  * Returns if W bit was on, so you know if *SRC and *DST are uint16_t or uint8_t.
*/
static int srcdst_0030(cpu8086_t* cpu, void** dst, void** src) {
  /*
    In case we need to return copies/constants, that aren't easily referencable, like immidietes 
    that may be unaligned.
  */
  static uint16_t s_src, s_dst;

  uint8_t opcode = (cpu->i_ptr[0] & 0xFC) >> 2;
  uint8_t d_bit = (cpu->i_ptr[0] & 0x2) >> 1;
  uint8_t w_bit = cpu->i_ptr[0] & 0x1;
  uint8_t modrm;
  
  /* If 3rd bit on it's the INS AL/AX, IMM8/IMM16 thingy, works for both 0x04/5 and 0x0C/D. */
  if (opcode & 0x4) {
    s_src = w_bit ? get16(cpu, cpu->ip_cs + 1) : get8(cpu, cpu->ip_cs + 1);
    *dst = &cpu->regs[REG8086_AX].x; /* &p[0] is &x so no checks. Little-endian rules. */
    *src = &s_src;
    cpu->ip_add = 2 + w_bit;
    return w_bit;
  }

  modrm = get8(cpu, cpu->ip_cs + 1);

  /* Simply index the register according to REG field, since REG8086_* is compatible. */
  *dst = &cpu->regs[(modrm & MODRM_REG_MASK) - REGW_AX].x;

  /* Now for the src which is a little harder... */
  if ((modrm & MODRM_MOD_MASK) == MOD_RM_IS_REG) {
    /* 3 bits right fit the REG mask */
    *src = &cpu->regs[((modrm << 3) & MODRM_REG_MASK) - REGW_AX].x;
  }
  /* Direct addressing */
  else if ((modrm & MODRM_MOD_MASK) == MOD_NDISP && (modrm & MODRM_RM_MASK) == RM_BP) {
    uint32_t addr = regseg_imm(cpu, get16(cpu, cpu->ip_cs + 2), cpu->regs[REG8086_DS].x);
    if (w_bit) {
      *src = mem_get16(cpu, addr, REG8086_DS);
    }
    else {
      *src = mem_get8(cpu, addr, REG8086_DS);
    }
    cpu->ip_add = 4;
  }
  else {
    
    
    switch (modrm & MODRM_RM_MASK) {
    }

  }

  if (d_bit) {
    void* tmp = *src;
    *src = *dst;
    *dst = tmp;
  }

  return w_bit;
}

int cycle_cpu8086(cpu8086_t* cpu) {
  mem_t* mem = cpu->mem;

  cpu->e = E8086_OK;
  cpu->ip_cs = REGSEG_IP_CS(cpu);
  cpu->ip_add = 1;
  cpu->i_ptr = mem->bytes + cpu->ip_cs;
  cpu->seg = REG8086_NULL;

  /* TODO: Make multi-prefix support, since it's possible I think? */
  /* Prefixes */
  switch (cpu->i_ptr[0]) {
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
    cpu->i_ptr = mem->bytes + cpu->ip_cs;
  }

  /* INC R16 */
  if (cpu->i_ptr[0] >= 0x40 && cpu->i_ptr[0] <= 0x47) {
    uint16_t* ptr = &cpu->regs[REG8086_AX + (cpu->i_ptr[0] & 7)].x;
    *ptr = add_ins(cpu, *ptr, 1, 1);
    cpu->cycles += 2;
    cpu->ip_add = 1;
  }
  /* DEC R16 */
  else if (cpu->i_ptr[0] >= 0x48 && cpu->i_ptr[0] <= 0x4F) {
    uint16_t* ptr = &cpu->regs[REG8086_AX + (cpu->i_ptr[0] & 7)].x;
    *ptr = sub_ins(cpu, *ptr, 1, 1);
    cpu->cycles += 2;
    cpu->ip_add = 1;
  }
  /* PUSH R16 */
  else if (cpu->i_ptr[0] >= 0x50 && cpu->i_ptr[0] <= 0x57) {
    push16(cpu, cpu->regs[REG8086_AX + (cpu->i_ptr[0] & 7)].x);
    cpu->cycles += 11;
    cpu->ip_add = 1;
  }
  /* POP R16 */
  else if (cpu->i_ptr[0] >= 0x58 && cpu->i_ptr[0] <= 0x5F) {
    cpu->regs[REG8086_AX + (cpu->i_ptr[0] & 7)].x = pop16(cpu);
    cpu->cycles += 8;
    cpu->ip_add = 1;
  }
  /* MOV R16, IMM16 */
  else if (cpu->i_ptr[0] >= 0xB8 && cpu->i_ptr[0] <= 0xBF) {
    cpu->regs[REG8086_AX + (cpu->i_ptr[0] & 7)].x = get16(cpu, cpu->ip_cs + 1);
    
    cpu->cycles += 4;
    cpu->ip_add = 3;
  }
  /* MOV R8, IMM8 */
  else if (cpu->i_ptr[0] >= 0xB0 && cpu->i_ptr[0] <= 0xB7) {
    unsigned p_i = cpu->i_ptr[0] >= 0xB4; /* It's the upper register parts if the second half */

    /* &3 for %4 because it's split to L and H */
    cpu->regs[REG8086_AX + (cpu->i_ptr[0] & 3)].p[p_i] = get8(cpu, cpu->ip_cs + 1);
    
    cpu->cycles += 4;
    cpu->ip_add = 2;
  }
  /* Various conditional jumps, relative to current instruction. */
  else if (cpu->i_ptr[0] >= 0x70 && cpu->i_ptr[0] <= 0x7F) {
    uint8_t rel_addr = get8(cpu, cpu->ip_cs);
    
    if (check_cond_jmp(cpu, cpu->i_ptr[0])) {
      short_jump(cpu, rel_addr);
      cpu->cycles += 16;
    }
    else {
      cpu->cycles += 4;
    }
  }
  /* MOV AL/AX, ADDR or MOV ADDR, AL/AX. So much hard-wired stuff man. */
  else if (cpu->i_ptr[0] >= 0xA0 && cpu->i_ptr[0] <= 0xA3) {
    uint8_t d_bit = (cpu->i_ptr[0] & 0x2) >> 1;
    uint8_t w_bit = cpu->i_ptr[0] & 0x1;
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
    if (cpu->i_ptr[0] == 0x66 || cpu->i_ptr[0] == 0x67) {
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