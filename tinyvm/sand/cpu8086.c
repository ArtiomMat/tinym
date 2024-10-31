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

#define ROL(X, N) ((X << N) | (X >> (sizeof (X) - N)))
#define ROR(X, N) ((X >> N) | (X << (sizeof (X) - N)))

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
  cpu->i = 256;

  /* A minimum size for addressable memory. */
  if (mem == NULL || mem->size < MB1) {
    return 0;
  }
  cpu->mem = mem;

  return 1;
}

void interrupt_cpu8086(cpu8086_t* cpu, uint8_t i) {
  cpu->i = i;
}

/**
 * @brief Get a default segment register unless prefixed otherwise.
 * @param cpu
 * @param defs must be `REG8086_*`, it's the default segment regregister returned, unless `cpu->seg` was set due to prefixing.
 * @return a segment register pointer from `cpu`, and takes segment prefixes into account via `cpu->seg`.
 */
static reg8086_t* get_seg(cpu8086_t* cpu, const int defs) {
  if (cpu->seg != REG8086_NULL) {
    return cpu->regs + cpu->seg;
  }
  return cpu->regs + defs;
}

/**
 * @brief Joins `seg:reg` into an address. Wraps around if address becomes too large.
 * @param cpu
 * @param reg
 * @param seg
 * @return `seg:reg`.
 */
static uint32_t regseg_imm(cpu8086_t* cpu, const uint16_t reg, const uint16_t seg) {
  uint32_t ret = reg + (seg << 4);
  ret &= 0xFFFFF;
  return ret;
}

/**
 * @brief `regseg_imm()` but for `cpu->regs`.
 * @param reg Must be `REG8086_*`.
 * @param seg Must be `REG8086_*`.
 */
static uint32_t regseg(cpu8086_t* cpu, const int reg, const int seg) {
  return regseg_imm(cpu, cpu->regs[reg].x, cpu->regs[seg].x);
}

/**
 * @attention `*seg` is allowed to exceed 0xF000 for perfomance reasons. Always use safe indexing functions.
 * @brief Adds `addr` to `*seg`:`*reg` combined to avoid overflow of adding to `*reg` alone.
 * @param reg
 * @param seg
 * @param addr At most a 16 bit address offseg can be used.
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

/**
 * @attention `*seg` is allowed to exceed 0xF000 for perfomance reasons. Always use safe indexing functions.
 * @brief Subtracts `addr` from `*seg`:`*reg` combined to avoid overflow of subtrating from `*reg` alone.
 * @param reg
 * @param seg
 * @param addr At most a 16 bit address offseg can be used.
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

/**
 * @param cpu
 * @param addr Full 32-bit address.
 * @return The byte at `cpu->mem.bytes + addr` if in-bounds, otherwise undefined but safe.
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static uint8_t get8(cpu8086_t* cpu, const uint32_t addr) {
  if (addr >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return 0;
  }

  return cpu->mem->bytes[addr];
}
/**
 * @brief Puts byte at `cpu->mem.bytes + addr` if in-bounds, otherwise doesn't.
 * @param cpu
 * @param addr Full 32-bit address.
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static void put8(cpu8086_t* cpu, const uint32_t addr, const uint16_t what) {
  if (addr >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return;
  }

  cpu->mem->bytes[addr] = what;
}
/**
 * @param cpu
 * @param addr Address within a given segment.
 * @param defs Must be `REG8086_*`. The segment register used, unless prefixed otherwise.
 * @return Direct pointer to `cpu->mem.bytes + addr` if in-bounds, otherwise undefined but safe pointer.
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static uint8_t* get8_ptr(cpu8086_t* cpu, const uint16_t addr, const int defs) {
  uint32_t addr32;
  addr32 = regseg_imm(cpu, addr, get_seg(cpu, defs)->x); /* Combine with the sreg */

  if (addr32 >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return cpu->mem->bytes;
  }

  return cpu->mem->bytes + addr32;
}

/**
 * @param cpu
 * @param addr Full 32-bit address.
 * @return The word at `cpu->mem.bytes + addr` if in-bounds, otherwise undefined but safe.
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
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
/**
 * @param cpu
 * @param addr Address within a given segment, should be aligned.
 * @param defs Must be `REG8086_*`. The segment register used, unless prefixed otherwise.
 * @return Direct pointer to `cpu->mem.bytes + addr` if in-bounds + aligned, otherwise undefined but safe pointer.
 * @warning Errors: `E8086_ACCESS_VIOLATION`, `E8086_UNALIGNED`.
 */
static uint16_t* get16_ptr(cpu8086_t* cpu, const uint16_t addr, const int defs) {
  uint32_t addr32;
  addr32 = regseg_imm(cpu, addr, get_seg(cpu, defs)->x); /* Combine with the sreg */

  if (addr32 >= MB1 - 1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return (uint16_t*)(cpu->mem->bytes);
  }
  else if (addr32 & 1) {
    cpu->e = E8086_UNALIGNED;
    addr32 &= ~1;
  }

  return (uint16_t*)(cpu->mem->bytes + addr32);
}
/**
 * @brief Puts the word at `cpu->mem.bytes + addr` if in-bounds, otherwise doesn't.
 * @param cpu
 * @param addr Full 32-bit address.
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static void put16(cpu8086_t* cpu, const uint32_t addr, const uint16_t what) {
  uint8_t* p = cpu->mem->bytes + addr;

  if (addr >= MB1 - 1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return;
  }

  p[0] = what;
  p[1] = what >> 8;
}

/**
 * @brief Pushes SEG, where SEG is the enum REG8086_*. increments cycles and ip_add accordingly.
 */
#define PUSH_SEG(CPU, SEG) push16(CPU, CPU->regs[SEG].x);\
    CPU->cycles += 10;\
    CPU->ip_add = 1;

/**
 * @brief Pops into SEG, where SEG is the enum REG8086_*. increments cycles and ip_add accordingly.
 */
#define POP_SEG(CPU, SEG) CPU->regs[SEG].x = pop16(CPU);\
    CPU->cycles += 8;\
    CPU->ip_add = 1;

/**
 * @brief Pushes `x` into the stack using+modifying `cpu`'s statck registers.
 * @param cpu
 * @param x
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static void push16(cpu8086_t* cpu, const uint16_t x) {
  uint32_t stack_ptr = regseg_imm(cpu, cpu->regs[REG8086_SP].x, GET_SEG_SS(cpu)->x);

  stack_ptr -= 2; /* Since gotta first decrement */
  regseg_sub(&cpu->regs[REG8086_SP], GET_SEG_SS(cpu), 2);

  /* Now copy the register contents */
  put16(cpu, stack_ptr, x);
}

/**
 * @param cpu
 * @return Popped value from `cpu` using+modifying its statck registers.
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static uint16_t pop16(cpu8086_t* cpu) {
  uint16_t ret;
  uint32_t stack_ptr = regseg_imm(cpu, cpu->regs[REG8086_SP].x, GET_SEG_SS(cpu)->x);

  /* Copy the content before modifying stack_ptr */
  ret = *get16_ptr(cpu, stack_ptr, REG8086_SS);

  /* Add 2 back */
  regseg_add(&cpu->regs[REG8086_SP], GET_SEG_SS(cpu), 2);

  return ret;
}

/**
 * @brief Updates the flags of `cpu` using only the result of an operation that modifies bits of the result.
 * @param cpu
 * @param result The result of the previous operation, notice it's 16 bit, to fit both 8 bit results and 16 bits.
 * @param w If result is in the context of a word, or 0 for byte operation.
 */
static void update_bit_flags(cpu8086_t* cpu, const uint32_t result, const int w) {
  cpu->regs[REG8086_F].x &= ~(F8086_Z | F8086_S | F8086_P);

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
}

/**
 * @brief Update flags after an arithmetic operation exclusively.
 * @param cpu
 * @param result The result of the previous operation, notice it's 32 bit, for arithmetic operations
 * to detect overflow, this is why you should always do the operations in 32 bit, and then pass
 * here, and only then truncate them back to 8/16 bit results.
 * @param w If result is in the context of a word, or 0 for byte operation.
 */
static void update_math_flags(cpu8086_t* cpu, const uint32_t result, const int w) {
  cpu->regs[REG8086_F].x &= ~(F8086_A | F8086_O | F8086_C);

  /* Auxilary carry flag */
  if (result & 0xFFFFFF00) {
    cpu->regs[REG8086_F].x |= F8086_A;
  }

  if (w) {
    if (((int32_t)result) > 32767 || ((int32_t)result) < -32768) {
      cpu->regs[REG8086_F].x |= F8086_O;
    }
    if (result > 65535) {
      cpu->regs[REG8086_F].x |= F8086_C;
    }
  }
  else {
    if (((int32_t)result) > 127 || ((int32_t)result) < -128) {
      cpu->regs[REG8086_F].x |= F8086_O;
    }
    if (result > 255) {
      cpu->regs[REG8086_F].x |= F8086_C;
    }
  }
}

/* Does not add cycles because not enough info. */
static uint32_t add_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a + b;
  update_bit_flags(cpu, result, w);
  update_math_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t adc_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t carry = cpu->regs[REG8086_F].x & F8086_C; /* CY=1 so no shifts */
  uint32_t result = a + b + carry;
  update_bit_flags(cpu, result, w);
  update_math_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t sub_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a - b;
  update_bit_flags(cpu, result, w);
  update_math_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t sbb_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t carry = cpu->regs[REG8086_F].x & F8086_C; /* CY=1 so no shifts */
  uint32_t result = a - b - carry;
  update_bit_flags(cpu, result, w);
  update_math_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t and_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a & b;
  update_bit_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t or_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a | b;
  update_bit_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t xor_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a ^ b;
  update_bit_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static uint32_t mul_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a * b;
  update_bit_flags(cpu, result, w);
  update_math_flags(cpu, result, w);
  return result;
}
/* Does not add cycles because not enough info. */
static int32_t imul_ins(cpu8086_t* cpu, const int32_t a, const int32_t b, const int w) {
  int32_t result = a * b;
  update_bit_flags(cpu, result, w);
  update_math_flags(cpu, result, w);
  return result;
}
typedef struct {
  uint16_t q, r;
} div_result_t;
/* NOTE: Does not interrupt if b=0, you must do it. Does not add cycles because not enough info. */
static div_result_t div_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result;
  div_result_t div_result;
  if (!b) {
    div_result.q = div_result.r = 0;
    return div_result;
  }
  result = a / b;
  update_bit_flags(cpu, result, w);
  update_math_flags(cpu, result, w);

  div_result.q = result;
  div_result.r = a % b;
  return div_result;
}
/* NOTE: Does not interrupt if b=0, you must do it. Does not add cycles because not enough info. */
static div_result_t idiv_ins(cpu8086_t* cpu, const int32_t a, const int32_t b, const int w) {
  int32_t result;
  div_result_t div_result;
  if (!b) {
    div_result.q = div_result.r = 0;
    return div_result;
  }
  result = a / b;
  update_bit_flags(cpu, result, w);
  update_math_flags(cpu, result, w);

  div_result.q = (int32_t)result;
  div_result.r = a % b;
  return div_result;
}
/* Only affects flags. Does not add cycles because not enough info. */
static void cmp_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a - b;
  update_bit_flags(cpu, result, w);
  update_math_flags(cpu, result, w);
}
/* Only affects flags. Does not add cycles because not enough info. */
static void test_ins(cpu8086_t* cpu, const uint32_t a, const uint32_t b, const int w) {
  uint32_t result = a & b;
  update_bit_flags(cpu, result, w);
}

/**
 * @brief Adds to `cpu`'s `cs:ip` `disp16`+`next_ins_off`.
 * If it succeeds `cpu->ip_add` is set to 0, otherwise `next_ins_off`.
 * @param cpu
 * @param next_ins_off The offset from current `cpu->ip_cs` of the first byte of the next instruction.
 * In other words the length of this short jump instruction(probably 2).
 * @param disp16 Notice that it's signed, since it can also jump backward.
 * @return 1 if jumped, 0 if failed.
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static int near_jump(cpu8086_t* cpu, uint8_t next_ins_off, int16_t disp16) {
  disp16 += next_ins_off;

  if (cpu->ip_cs + disp16 >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    cpu->ip_add = next_ins_off;
    return 0;
  }

  if (disp16 > 0) {
    REGSEG_IP_CS_ADD(cpu, disp16);
  }
  else {
    REGSEG_IP_CS_SUB(cpu, -disp16);
  }

  cpu->ip_add = 0;
  return 1;
}

/**
 * @brief Adds to `cpu`'s `cs:ip` `disp`+`next_ins_off`.
 * If it succeeds `cpu->ip_add` is set to 0, otherwise `next_ins_off`.
 * @param cpu
 * @param next_ins_off The offset from current `cpu->ip_cs` of the first byte of the next instruction.
 * In other words the length of this short jump instruction(probably 2).
 * @param disp Notice that it's signed, since it can also jump backward.
 * @return 1 if jumped, 0 if failed.
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static int short_jump(cpu8086_t* cpu, uint8_t next_ins_off, int8_t disp) {
  return near_jump(cpu, next_ins_off, disp);
}

/**
 * @brief Jumps to `cs:ip`.
 * @param cpu
 * @param ip
 * @param cs
 * @return 1 if jumped, 0 if failed.
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static int far_jump(cpu8086_t* cpu, uint16_t ip, uint16_t cs) {
  if ((uint32_t)ip + cs >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return 0;
  }

  cpu->regs[REG8086_IP].x = ip;
  cpu->regs[REG8086_CS].x = cs;
  return 1;
}

/**
 * @brief Pushes the current `ip` and `cs` registers (with `next_ins_off` added) in respective 
 * order.
 * @param cpu
 * @param ip
 * @param cs
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static void push_cs_ip_off(cpu8086_t* cpu, uint8_t off) {
  /* These are what is pushed */
  reg8086_t ip2, cs2;
  cs2 = cpu->regs[REG8086_CS];
  ip2 = cpu->regs[REG8086_IP];
  regseg_add(&ip2, &cs2, off);

  push16(cpu, cs2.x);
  push16(cpu, ip2.x);
}

/**
 * @param cpu
 * @param opcode The opcode byte, it must be 0x70-0x7F inclusive.
 * @return Whether according to the first instruction byte the CPU should do conditional jump.
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
    do_jump = (opcode&1) ^ (!!(f & F8086_C));
    break;

    case 0x04: /* JE/JZ */
    case 0x05: /* JNE/JNZ */
    do_jump = (opcode&1) ^ (!!(f & F8086_Z));
    break;

    case 0x06: /* JBE/JNA */
    case 0x07: /* JA/JNBE */
    do_jump = (opcode&1) ^ ((!!(f & F8086_C)) | (!!(f & F8086_Z)));
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

/**
 * @attention `cpu->ip_cs` must be at the first byte, not modrm.
 * @param cpu
 * @param modrm The modr/m byte.
 * @param w_bit Indicates if it's a word operation, and hence the size of the data in the pointer.
 * @return A pointer to the register referenced by the reg field.
 * `w_bit` indicates whether the underlying pointed value is 16 or 8 bit.
 */
static void* get_modrm_reg(cpu8086_t* cpu, uint8_t modrm, int w_bit) {
  unsigned reg = (modrm & MODRM_REG_MASK) >> 3;
  if (w_bit) {
    return &cpu->regs[reg].x;
  }
  else {
    unsigned p_i = reg >= 0x4;
    return &cpu->regs[REG8086_AX + (reg & 3)].p[p_i];
  }
}

/**
 * @attention `cpu->ip_cs` must be at the first byte, not modrm.
 * @param cpu
 * @param modrm The modr/m byte.
 * @return A pointer to the *SEGMENT REGISTER* referenced by the REG field.
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

/**
 * @brief Analyzes `modrm`+`w_bit` as if MOD=3, meaning it's literally a reg field again.
 * @param cpu
 * @param modrm the modr/m byte.
 * @param w_bit Whether it's a word operation, 1 or 0.
 * @return A pointer to the register.
 * `w_bit` indicates whether the underlying pointed value is 16 or 8 bit.
 */
static void* get_modrm_rm_reg(cpu8086_t* cpu, uint8_t modrm, uint8_t w_bit) {
  return get_modrm_reg(cpu, modrm << 3, w_bit);
}

/**
 * @attention Can return unsafe address(out of bounds), always use safe safe indexing functions.
 * @brief Analyzes MODRM(the modr/m byte) and returns a full 32 but address if R/M part represents an address.
 * @param cpu
 * @param modrm the modr/m byte.
 * @return `UINT32_MAX` if it's not an address(MOD=3, RM is not an address it's a register),
 * otherwise returns a full 32 bit address.
 * @warning Errors: `E8086_ACCESS_VIOLATION`.
 */
static uint32_t get_modrm_rm_addr(cpu8086_t* cpu, uint8_t modrm) {
  /* R/M is REG, MOD=3, so just shift by 3 to put RM in REG and return. */
  if ((modrm & MODRM_MOD_MASK) == MOD_RM_IS_REG) {
    return UINT32_MAX;
  }
  /* Direct addressing, MOD=0,R/M=6 */
  else if ((modrm & MODRM_MOD_MASK) == MOD_NDISP && (modrm & MODRM_RM_MASK) == RM_BP) {
    return regseg_imm(cpu, get16(cpu, cpu->ip_cs + 2), cpu->regs[REG8086_DS].x);
  }
  /* Using various register-displacement combinations depending on MOD+R/M */
  else {
    uint32_t addr;
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

    return addr;
  }
}

/**
 * @attention `cpu->ip_cs` must be at the first byte, not modrm.
 * @param cpu
 * @param modrm The modr/m byte.
 * @param w_bit Whether it's a word operation, 1 or 0.
 * @return A pointer to what the R/M points to, e.g in `cpu->mem-bytes`, or a register.
 * `w_bit` indicates whether the underlying pointed value is 16 or 8 bit.
 * @warning Errors: `E8086_ACCESS_VIOLATION`, `E8086_UNALIGNED`.
 */
static void* get_modrm_rm(cpu8086_t* cpu, uint8_t modrm, uint8_t w_bit) {
  uint32_t addr = get_modrm_rm_addr(cpu, modrm);

  /* It's a REG field. */
  if (addr == UINT32_MAX) {
    return get_modrm_rm_reg(cpu, modrm, w_bit);
  }

  if (w_bit) {
    return get16_ptr(cpu, addr, REG8086_DS);
  }
  else {
    return get8_ptr(cpu, addr, REG8086_DS);
  }
}

/**
 * @warning Includes the opcode byte in the return value.
 * @param modrm The MODR/M byte
 * @return What `cpu8086_t->ip_add` should be given a MODR/M byte.
 */
static uint8_t get_modrm_ip_add(uint8_t modrm) {
  switch (modrm & MODRM_MOD_MASK) {
    case MOD_DISP16:
    return 4;
    break;
    case MOD_DISP8:
    return 3;
    break;

    case MOD_NDISP:
    /* Direct addressing mode */
    if ((modrm & MODRM_RM_MASK) == RM_BP) {
      return 4;
    }
    else {
      return 2;
    }
    break;

    default: /* MOD_RM_IS_REG */
    return 2;
    break;
  }
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

/**
  * @brief A specific format of instructions that appear in 0x00-0x30, like ADD in the start, or
  * CMP in the end, they are distinct and can be generalized to this source-destination retrieval.
  *
  * It's also applicable to MOV in 0x88-0x8B, even through it doesn't have the AL-AX thingy, due
  * to modulus-like design, and 8086 aligned stuff nicely.
  *
  * Sets `cpu->ip_add` by itself.
 * @param cpu
 * @param opcode The opcode byte.
 * @param dst A pointer to a pointer of the modified(usually, but sometimes theoretically) operand.
 * @param src A pointer to a pointer of the source(copied/utilized/constant) operand.
 * @return If W bit was set in the modr/m byte, so you know if `*src` and `*dst` are `uint16_t` or `uint8_t`.
 * @warning Errors: `E8086_ACCESS_VIOLATION`, `E8086_UNALIGNED`.
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
  *src = get_modrm_reg(cpu, modrm, w_bit);

  if (d_bit) {
    void* tmp = *src;
    *src = *dst;
    *dst = tmp;
  }

  cpu->ip_add = get_modrm_ip_add(modrm);

  return w_bit;
}

/**
 * @brief Meant to be used if current instruction byte is a prefix, skip it.
 * @param cpu
 * @return The next opcode.
 */
uint8_t skip_prefix(cpu8086_t* cpu) {
  cpu->cycles += 1;
  REGSEG_IP_CS_ADD(cpu, 1);
  cpu->ip_cs = REGSEG_IP_CS(cpu);
  return get8(cpu, cpu->ip_cs);
}

int cycle_cpu8086(cpu8086_t* cpu) {
  uint8_t opcode;

  /* Interrupted jump before proceeding and reset cpu->i */
  if (cpu->i != 256) {
    uint32_t i_addr = cpu->i * 4;
    cpu->i = 256;

    push16(cpu, cpu->regs[REG8086_F].x);
    push16(cpu, cpu->regs[REG8086_CS].x);
    push16(cpu, cpu->regs[REG8086_IP].x);

    far_jump(cpu, get16(cpu, i_addr), get16(cpu, i_addr + 2));
  }

  cpu->e = E8086_OK;
  cpu->ip_cs = REGSEG_IP_CS(cpu);
  cpu->ip_add = 1;
  cpu->seg = REG8086_NULL;

  opcode = get8(cpu, cpu->ip_cs); /* opcode byte */

  /* Prefixes */
  while (1) {
    int done = 0;

    switch (opcode) {
      case 0x26:
      cpu->seg = REG8086_ES;
      opcode = skip_prefix(cpu);
      break;
      case 0x36:
      cpu->seg = REG8086_SS;
      opcode = skip_prefix(cpu);
      break;
      case 0x2E:
      cpu->seg = REG8086_CS;
      opcode = skip_prefix(cpu);
      break;
      case 0x3E:
      cpu->seg = REG8086_DS;
      opcode = skip_prefix(cpu);
      break;

      case 0xF0: /* LOCK, we don't do threads anyway so no need. */
      break;

      default:
      done = 1;
      break;
    }

    if (done) {
      break;
    }
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
  /* PUSH ES */
  else if (opcode == 0x06) {
    PUSH_SEG(cpu, REG8086_ES);
  }
  /* PUSH SS */
  else if (opcode == 0x16) {
    PUSH_SEG(cpu, REG8086_SS);
  }
  /* PUSH CS */
  else if (opcode == 0x0E) {
    PUSH_SEG(cpu, REG8086_CS);
  }
  /* PUSH DS */
  else if (opcode == 0x1E) {
    PUSH_SEG(cpu, REG8086_DS);
  }
  /* POP R16 */
  else if (opcode >= 0x58 && opcode <= 0x5F) {
    cpu->regs[REG8086_AX + (opcode & 7)].x = pop16(cpu);
    cpu->cycles += 8;
    cpu->ip_add = 1;
  }
  /* POP MODR/M with W */
  else if (opcode == 0x8F) {
    uint8_t modrm = get8(cpu, cpu->ip_cs + 1);
    uint16_t* rm = get_modrm_rm(cpu, modrm, 1);
    *rm = pop16(cpu);
    cpu->ip_add = get_modrm_ip_add(modrm);
  }
  /* POP ES */
  else if (opcode == 0x07) {
    POP_SEG(cpu, REG8086_ES);
  }
  /* POP SS */
  else if (opcode == 0x17) {
    POP_SEG(cpu, REG8086_SS);
  }
  /* POP DS */
  else if (opcode == 0x1F) {
    POP_SEG(cpu, REG8086_DS);
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
  /* MOV AL/AX, ADDR or MOV ADDR, AL/AX. So much hard-wired stuff man. */
  else if (opcode >= 0xA0 && opcode <= 0xA3) {
    uint8_t d_bit = (opcode & 0x2) >> 1;
    uint8_t w_bit = opcode & 0x1;
    uint16_t addr = get16(cpu, cpu->ip_cs + 1);
    void* ax_al = &cpu->regs[REG8086_AX]; /* Little-endian magic */
    /* XXX: I hate the way it looks. */
    if (d_bit) {
      if (w_bit) {
        *get16_ptr(cpu, addr, REG8086_DS) = *(uint16_t*)ax_al;
      }
      else {
        *get8_ptr(cpu, addr, REG8086_DS) = *(uint8_t*)ax_al;
      }
    }
    else {
      if (w_bit) {
        *(uint16_t*)ax_al = *get16_ptr(cpu, addr, REG8086_DS);
      }
      else {
        *(uint8_t*)ax_al = *get8_ptr(cpu, addr, REG8086_DS);
      }
    }

    cpu->ip_add = 3;
    cpu->cycles += 10;
  }
  /* LEA MODR/M */
  else if (opcode == 0x8D) {
    uint8_t modrm = get8(cpu, cpu->ip_cs + 1);
    uint16_t* reg = get_modrm_reg(cpu, modrm, 1);

    uint32_t addr = get_modrm_rm_addr(cpu, modrm);
    if (addr == UINT32_MAX) {
      addr = *(uint16_t*)get_modrm_rm_reg(cpu, modrm, 1);
    }

    *reg = addr;
  }
  /* XCHG R16, AX */
  else if (opcode >= 0x91 && opcode <= 0x97) {
    uint8_t reg = opcode & 7; /* Wont be AX */
    uint16_t tmp = cpu->regs[reg].x;
    cpu->regs[reg].x = cpu->regs[REG8086_AX].x;
    cpu->regs[REG8086_AX].x = tmp;

    cpu->ip_add = 1;
  }
  /* XCHG MODR/M */
  else if (opcode == 0x86 || opcode == 0x87) {
    int w_bit = (opcode & 0x1);
    uint8_t modrm = get8(cpu, cpu->ip_cs + 1);
    void* dst = get_modrm_reg(cpu, modrm, w_bit);
    void* src = get_modrm_rm(cpu, modrm, w_bit);

    if (w_bit) {
      uint16_t tmp = *(uint16_t*)src;
      *(uint16_t*)src = *(uint16_t*)dst;
      *(uint16_t*)dst = tmp;
    }
    else {
      uint8_t tmp = *(uint8_t*)src;
      *(uint8_t*)src = *(uint8_t*)dst;
      *(uint8_t*)dst = tmp;
    }
    cpu->ip_add = get_modrm_ip_add(modrm);
  }
  /* ??? MODR/M or ??? AL/AX, IMM8/16 section */
  /* ADD */
  else if (/* opcode >= 0x00 && */ opcode <= 0x05) {
    /* TODO: Best way to add cycles is to estimate them, maybe add cpu->cyclef, which is flags for what kind of stuff happened, and try to estimate them, better than 0 cycles added, or literally make 10000 branches for this type of OP lol. */
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
  /* TEST AL, IMM8 */
  else if (opcode == 0xA8) {
    test_ins(cpu, cpu->regs[REG8086_AX].p[0], get8(cpu, cpu->ip_cs + 1), 0);
    cpu->ip_add = 2;
  }
  /* TEST AX, IMM16 */
  else if (opcode == 0xA9) {
    test_ins(cpu, cpu->regs[REG8086_AX].x, get16(cpu, cpu->ip_cs + 1), 1);
    cpu->ip_add = 3;
  }
  /* TEST MODR/M */
  else if (opcode == 0x84 || opcode == 0x85) {
    int w_bit = (opcode & 0x1);
    uint8_t modrm = get8(cpu, cpu->ip_cs + 1);
    void* dst = get_modrm_reg(cpu, modrm, w_bit);
    void* src = get_modrm_rm(cpu, modrm, w_bit);
    if (w_bit) {
      test_ins(cpu, *(uint16_t*)dst, *(uint16_t*)src, w_bit);
    }
    else {
      test_ins(cpu, *(uint8_t*)dst, *(uint8_t*)src, w_bit);
    }
    cpu->ip_add = get_modrm_ip_add(modrm);
  }
  /* Various conditional jumps, relative to current instruction. */
  else if (opcode >= 0x70 && opcode <= 0x7F) {
    uint8_t rel_addr = get8(cpu, cpu->ip_cs + 1);

    if (check_cond_jmp(cpu, opcode)) {
      short_jump(cpu, 2, rel_addr);
      cpu->cycles += 16;
    }
    else {
      cpu->cycles += 4;
      cpu->ip_add = 2; /* Cuz didn't jump. */
    }
  }
  /* MOV MOD/RM for sregs */
  else if (opcode == 0x8C || opcode == 0x8E) {
    int d_bit = (opcode & 0x2) >> 1;
    uint8_t modrm = get8(cpu, cpu->ip_cs + 1);
    uint16_t* dst = get_modrm_rm(cpu, modrm, 1);
    uint16_t* src = get_modrm_seg(cpu, modrm);

    if (d_bit) {
      uint16_t* tmp = src;
      src = dst;
      dst = tmp;
    }

    *dst = *src;

    cpu->ip_add = get_modrm_ip_add(modrm);
  }
  /* MOV MOD/RM IMM8 or MOV MOD/RM IMM16 */
  else if (opcode == 0xC6 || opcode == 0xC7) {
    int w_bit = opcode & 1;
    uint8_t modrm = get8(cpu, cpu->ip_cs + 1);
    void* dst = get_modrm_rm(cpu, modrm, w_bit);
    uint32_t imm_addr = cpu->ip_cs + get_modrm_ip_add(modrm);

    if (w_bit) {
      *(uint16_t*)dst = get16(cpu, imm_addr);
      cpu->ip_add = get_modrm_ip_add(modrm) + 2;
    }
    else {
      *(uint8_t*)dst = get8(cpu, imm_addr);
      cpu->ip_add = get_modrm_ip_add(modrm) + 1;
    }
  }
  /* JMP IMM16 or CALL IMM16 */
  else if (opcode == 0xE9 || opcode == 0xE8) {
    /* Was it a call? */
    if (opcode == 0xE8) {
      push16(cpu, cpu->regs[REG8086_IP].x + 3);
    }
    near_jump(cpu, 3, get16(cpu, cpu->ip_cs + 1));
  }
  /* JMP IMM8 */
  else if (opcode == 0xEB) {
    short_jump(cpu, 2, get8(cpu, cpu->ip_cs + 1));
  }
  /* JMP FAR IMM16:IMM16 or CALL FAR IMM16:IMM16 */
  else if (opcode == 0xEA || opcode == 0x9A) {
    /* These are where we jump */
    uint16_t ip = get16(cpu, cpu->ip_cs + 1);
    uint16_t cs = get16(cpu, cpu->ip_cs + 3);

    /* Was it a call? */
    if (opcode == 0x9A) {
      push_cs_ip_off(cpu, 5);
    }
    /* 4 if didn't jump since we don't want to be stuck in a loop */
    cpu->ip_add = far_jump(cpu, ip, cs) ? 0 : 4;
  }
  /* RET IMM16 or RET */
  else if (opcode == 0xC2 || opcode == 0xC3) {
    cpu->regs[REG8086_IP].x = pop16(cpu);
    /* Extra popping */
    if (opcode == 0xC3) {
      uint16_t extra = get16(cpu, cpu->ip_cs + 1);
      regseg_add(&cpu->regs[REG8086_SP], GET_SEG_SS(cpu), extra);
    }
    cpu->ip_add = 0; /* ADD BAD! */
  }
  /* RET FAR IMM16 or RET FAR */
  else if (opcode == 0xCA || opcode == 0xCB) {
    cpu->regs[REG8086_IP].x = pop16(cpu);
    cpu->regs[REG8086_CS].x = pop16(cpu);
    /* Extra popping */
    if (opcode == 0xCB) {
      uint16_t extra = get16(cpu, cpu->ip_cs + 1);
      regseg_add(&cpu->regs[REG8086_SP], GET_SEG_SS(cpu), extra);
    }
    cpu->ip_add = 0; /* ADD BAD! */
  }
  /* INT 3 */
  else if (opcode == 0xCC) {
    interrupt_cpu8086(cpu, 3);
    cpu->ip_add = 1; /* Add for IRET to return after this INT */
  }
  /* INT IMM8 */
  else if (opcode == 0xCD) {
    interrupt_cpu8086(cpu, get8(cpu, cpu->ip_cs + 1));
    cpu->ip_add = 2; /* Add for IRET to return after this INT */
  }
  /* INTO */
  else if (opcode == 0xCE && (cpu->regs[REG8086_F].x & F8086_O)) {
    interrupt_cpu8086(cpu, 4);
    cpu->ip_add = 1; /* Add for IRET to return after this INT */
  }
  /* IRET */
  else if (opcode == 0xCF) {
    cpu->regs[REG8086_IP].x = pop16(cpu);
    cpu->regs[REG8086_CS].x = pop16(cpu);
    cpu->regs[REG8086_F].x = pop16(cpu);
    cpu->ip_add = 0; /* ADD BAD! */
  }
  /* IMM byte */
  else if (opcode == 0x80) {
    uint8_t modrm = get8(cpu, cpu->ip_cs + 1);
    uint8_t* rm = get_modrm_rm(cpu, modrm, 0);
    uint8_t op = modrm & MODRM_OP_MASK;
    uint8_t imm = get8(cpu, cpu->ip_cs + 2);
    switch (op) {
      
    }
    cpu->ip_add = 3;
  }
  /* IMM word */
  else if (opcode == 0x81) {

  }
  /* Grp1 byte */
  else if (opcode == 0xF6) {
    uint8_t modrm = get8(cpu, cpu->ip_cs + 1);
    uint8_t* rm = get_modrm_rm(cpu, modrm, 0);
    uint8_t op = modrm & MODRM_OP_MASK;
    switch (op) {
      case OP_G1_NOT:
      *rm = ~(*rm);
      break;
      case OP_G1_NEG:
      *rm = -(*rm);
      break;

      case OP_G1_MUL:
      case OP_G1_IMUL:
      {
        uint32_t res;
        if (op == OP_G1_MUL) {
          res = mul_ins(cpu, cpu->regs[REG8086_AX].p[0], *rm, 0);
        }
        else {
          res = imul_ins(cpu, (int32_t)cpu->regs[REG8086_AX].p[0], (int16_t)*rm, 0);
        }
        cpu->regs[REG8086_AX].x = res;
      }
      break;

      case OP_G1_DIV:
      case OP_G1_IDIV:
      {
        div_result_t res;
        if (*rm == 0) {
          break;
        }
        if (op == OP_G1_DIV) {
          res = div_ins(cpu, cpu->regs[REG8086_AX].x, *rm, 0);
        }
        else {
          res = idiv_ins(cpu, (int32_t)cpu->regs[REG8086_AX].x, (int16_t)*rm, 0);
        }
        cpu->regs[REG8086_AX].p[0] = res.q;
        cpu->regs[REG8086_AX].p[1] = res.r;
      }
      break;

      default:
      break;
    }

    cpu->ip_add = 2;
  }
  /* Grp1 word */
  else if (opcode == 0xF7) {
    uint8_t modrm = get8(cpu, cpu->ip_cs + 1);
    uint16_t* rm = get_modrm_rm(cpu, modrm, 1);
    uint8_t op = modrm & MODRM_OP_MASK;
    switch (op) {
      case OP_G1_NOT:
      *rm = ~(*rm);
      break;
      case OP_G1_NEG:
      *rm = -(*rm);
      break;

      case OP_G1_MUL:
      case OP_G1_IMUL:
      {
        uint32_t res;
        if (op == OP_G1_MUL) {
          res = mul_ins(cpu, cpu->regs[REG8086_AX].x, *rm, 1);
        }
        else {
          res = imul_ins(cpu, (int32_t)cpu->regs[REG8086_AX].x, (int16_t)*rm, 1);
        }
        cpu->regs[REG8086_AX].x = res;
        cpu->regs[REG8086_DX].x = res >> 16;
      }
      break;

      case OP_G1_DIV:
      case OP_G1_IDIV:
      {
        uint32_t dxax;
        div_result_t res;
        if (*rm == 0) {
          interrupt_cpu8086(cpu, 0);
          break;
        }
        dxax = cpu->regs[REG8086_AX].x | (((uint32_t)cpu->regs[REG8086_DX].x) << 16);
        if (op == OP_G1_DIV) {
          res = div_ins(cpu, dxax, *rm, 1);
        }
        else {
          res = idiv_ins(cpu, (int32_t)dxax, (int16_t)*rm, 1);
        }
        cpu->regs[REG8086_AX].x = res.q;
        cpu->regs[REG8086_DX].x = res.r;
      }
      break;

      default:
      break;
    }

    cpu->ip_add = 2;
  }
  /* UNKNOWN! or NOP */
  else {
    if (opcode == 0x66 || opcode == 0x67) {
      fputs("cycle_cpu8086(): you'll need i386 for that.\n", stderr);
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
