/*
  NOTE TO DEV: Every function here must be safe yet performant, isolated anarchy in the CPU is
  better than anarchy in the VM due to out-of-bounds access.
*/

#include "cpu8086.h"
#include "ops.h"
#include "test.h"
#include "util.h"

#include <string.h>

#define REGSEG_SP_SS(CPU) regseg_imm(CPU, CPU->regs[REG8086_SP].x, CPU->regs[REG8086_SS].x)
#define REGSEG_IP_CS(CPU) regseg_imm(CPU, CPU->regs[REG8086_IP].x, CPU->regs[REG8086_CS].x)
#define REGSEG_IP_CS_ADD(CPU, N) regseg_add(&CPU->regs[REG8086_IP], &CPU->regs[REG8086_CS], N)
#define REGSEG_IP_CS_SUB(CPU, N) regseg_sub(&CPU->regs[REG8086_IP], &CPU->regs[REG8086_CS], N)

/* These are used to identify an instruction when there are multiple opcodes. */
enum {
  INS_ADD,
  INS_OR,
  INS_ADC,
  INS_SBB,
  INS_AND,
  INS_SUB,
  INS_XOR,
  INS_CMP
};

static const uint8_t parity_table[] = {
  0x69, 0x96, 0x96, 0x69, 0x96, 0x69, 0x69, 0x96, 0x96, 0x69, 0x69, 0x96, 0x69, 0x96, 0x96, 0x69, 
  0x96, 0x69, 0x69, 0x96, 0x69, 0x96, 0x96, 0x69, 0x69, 0x96, 0x96, 0x69, 0x96, 0x69, 0x69, 0x96
};

int reset_cpu8086(cpu8086_t* __restrict cpu, mem_t* __restrict mem) {
  /* Set up register starting points */
  memset(cpu->regs, 0, sizeof (cpu->regs));
  cpu->regs[REG8086_CS].x = 0xFFFF;
  cpu->regs[REG8086_SP].x = 0xFFFF; /* TODO: Not an actual standard config. */
  cpu->regs[REG8086_F].x |= F8086_I;

  /* A minimum size for addressable memory. */
  if (mem->size < MB1) {
    return 0;
  }
  cpu->mem = mem;

  return 1;
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
static void regseg_add(reg8086_t* __restrict reg, reg8086_t* __restrict seg, const uint16_t addr) {
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
static void regseg_sub(reg8086_t* __restrict reg, reg8086_t* __restrict seg, const uint16_t addr) {
  if ((int32_t)reg->x - addr < 0) {
    reg->x = (0xFFFF - addr) + reg->x;
    seg->x -= 0x1000;
  }
  else {
    reg->x -= addr;
  }
}

/* SAFE 1MB ACCESS. 8 bit equivalent of get16. */
static uint8_t get8(cpu8086_t* cpu, const uint32_t addr) {
  if (addr >= MB1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return 0;
  }
  
  return cpu->mem->bytes[addr];
}

/*
  SAFE 1MB ACCESS. Get a value from CPU->mem[ADDR]. Preffered over raw access for: safety(will 
  CPU->e and return invalid but safely constructed value) & easier for unaligned access.
  As mentioned above, may set CPU->e to E8086_ACCESS_VIOLATION.
*/
static uint16_t get16(cpu8086_t* cpu, const uint32_t addr) {
  uint16_t ret;

  if (addr >= MB1 - 1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return 0;
  }
  
  memcpy(&ret, cpu->mem->bytes + addr, 2);
  return ret;
}

/*
  SAFE 1MB ACCESS. Set CPU->mem[ADDR] = what. Preffered over raw access for: safety(will CPU->e 
  and will not set) & easier for unaligned access.
  As mentioned above, may set CPU->e to E8086_ACCESS_VIOLATION.
*/
static void put16(cpu8086_t* cpu, const uint32_t addr, const uint16_t what) {
  if (addr >= MB1 - 1) {
    cpu->e = E8086_ACCESS_VIOLATION;
    return;
  }
  
  memcpy(cpu->mem->bytes + addr, &what, 2);
}

/*
  SAFE 1MB ACCESS. Pushes X into CPU->mem using CPU's stack registers.
  May set CPU->e to E8086_ACCESS_VIOLATION if the stack wrapped around and wont push. Uses regseg() so may set to its errors.
*/
static void push16(cpu8086_t* cpu, const uint16_t x) {
  uint32_t stack_ptr = REGSEG_SP_SS(cpu);

  stack_ptr -= 2; /* Since gotta first decrement */
  regseg_sub(&cpu->regs[REG8086_SP], &cpu->regs[REG8086_SS], 2);

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
  uint32_t stack_ptr = REGSEG_SP_SS(cpu);

  /* Copy the content before modifying stack_ptr */
  ret = get16(cpu, stack_ptr);

  /* Add 2 back */
  regseg_add(&cpu->regs[REG8086_SP], &cpu->regs[REG8086_SS], 2);

  return ret;
}

/*
  Updates CPU's flags accroding to _RESULT, of any operation that updates flags.
  NOTE: _RESULT is uint32_t to allow for accurate range detection within W's context,
  make sure that you use+pass int32_t or uint32_t, and not 16/8 ints, because overflow
  would not be visible, hence nullifying the arithmetic analysis.
  W is a boolean value and determines if it was a 16 bit operation(1) or 8 bit(0).
*/
static void update_flags(cpu8086_t* cpu, const uint32_t _result, const int w) {
  union {
    uint32_t u;
    int32_t i;
  } result = { .u = _result };

  cpu->regs[REG8086_F].x = F8086_I | F8086_D | F8086_T;

  if (!result.u) {
    cpu->regs[REG8086_F].x |= F8086_Z;
  }
  else if (result.u & 0x80000000) {
    cpu->regs[REG8086_F].x |= F8086_S;
  }

  if (get_arr_bit(parity_table, result.u & 0xFF)) {
    cpu->regs[REG8086_F].x |= F8086_P;
  }

  if (w) {
    if (result.i > 32767 || result.i < -32768) {
      cpu->regs[REG8086_F].x |= F8086_O;
    }
    if (result.u > 65535) {
      cpu->regs[REG8086_F].x |= F8086_CY;
    }
  }
  else {
    if (result.i > 127 || result.i < -128) {
      cpu->regs[REG8086_F].x |= F8086_O;
    }
    if (result.u > 255) {
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
  INS_BYTE is the instruction, it must be 0x70-0x7F inclusive.
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
  A specific subset of instructions like ADD, 0x?4, 0x?5, 0x?C, 0x?D.
*/
static void parse_alax(cpu8086_t* cpu, uint8_t* ins_bytes, int ins) {

}

/*
  A specific format of instructions that appear in 0x00-0x30, like ADD in the start, or CMP in the end, they are distinct and can be generalized to this function.
  Returns how much to add to ip_cs.
  It's also applicable to MOV in 0x88-0x8B, even through it doesn't have the AL-AX thingy.
  INS is the instruction and is from the enum INS_, INS_BYTES is the pointer to the instruction in CPU->mem.
  Since these instruction can access memory, CPU->e may be set to E8086_ACCESS_VIOLATION.
*/
static unsigned parse_0030(cpu8086_t* cpu, uint8_t* ins_bytes, int ins) {
  uint32_t ip_cs = REGSEG_IP_CS(cpu);
  uint8_t opcode = (ins_bytes[0] & 0xFC) >> 2;
  uint8_t d_bit = (ins_bytes[0] & 0x2) >> 1;
  uint8_t w_bit = ins_bytes[0] & 0x1;

  uint32_t ip_add = 1;

  /* If 3rd bit on it's the INS AL/AX, IMM8/IMM16 thingy, works for both 0x04/5 and 0x0C/D */
  if (opcode & 0x4) {
    if (w_bit) {
      uint16_t imm = get16(cpu, ip_cs + 1);
      switch (ins) {
        case INS_ADD:
        cpu->regs[REG8086_AX].x += imm;
        break;

        case INS_OR:
        cpu->regs[REG8086_AX].x |= imm;
        break;

        case INS_ADC:
        uint32_t carry = !!(cpu->regs[REG8086_F].x & F8086_CY);
        uint32_t add32 = cpu->regs[REG8086_AX].x + imm + carry;
        
        if (add32 & 0x10000) {
          cpu->regs[REG8086_F].x |= F8086_CY;
        }
        else {
          cpu->regs[REG8086_F].x &= ~F8086_CY;
        }

        cpu->regs[REG8086_AX].x = (uint16_t)add32;
        break;

        case INS_SBB:
        /*TODO*/
        break;

        case INS_AND:
        cpu->regs[REG8086_AX].x &= imm;
        break;

        case INS_SUB:
        cpu->regs[REG8086_AX].x -= imm;
        break;

        case INS_XOR:
        cpu->regs[REG8086_AX].x ^= imm;
        break;

        case INS_CMP:
        if (cpu->regs[REG8086_AX].x == imm) {
          cpu->regs[REG8086_F].x |= F8086_Z;
        }
        else {
          cpu->regs[REG8086_F].x &= ~F8086_Z;
        }

        cpu->regs[REG8086_AX].x ^= imm;
        break;
      }
      ip_add = 3;
    }
    else {
      ip_add = 2;
    }
    return ip_add;
  }
}

int cycle_cpu8086(cpu8086_t* cpu) {
  mem_t* mem = cpu->mem;
  uint32_t ip_cs = REGSEG_IP_CS(cpu); /* Address of current instruction */
  uint8_t* ins_bytes = mem->bytes + ip_cs; /* Where the instruction is */
  uint8_t ip_add = 1; /* How much to add if reading of instruction succeeded */

  cpu->e = E8086_OK; /* TODO: But should it reset? An error is fatal anyway. */

  /* Instructions that simply act on the 16-bit registers in order. */
  /* INC R16 */
  if (ins_bytes[0] >= 0x40 && ins_bytes[0] <= 0x47) {
    ++ (cpu->regs[REG8086_AX + (ins_bytes[0] & 7)].x);
    cpu->cycles += 2;
    ip_add = 1;
  }
  /* DEC R16 */
  else if (ins_bytes[0] >= 0x48 && ins_bytes[0] <= 0x4F) {
    -- (cpu->regs[REG8086_AX + (ins_bytes[0] & 7)].x);
    cpu->cycles += 2;
    ip_add = 1;
  }
  /* PUSH R16 */
  else if (ins_bytes[0] >= 0x50 && ins_bytes[0] <= 0x57) {
    push16(cpu, cpu->regs[REG8086_AX + (ins_bytes[0] & 7)].x);
    cpu->cycles += 11;
    ip_add = 1;
  }
  /* POP R16 */
  else if (ins_bytes[0] >= 0x58 && ins_bytes[0] <= 0x5F) {
    cpu->regs[REG8086_AX + (ins_bytes[0] & 7)].x = pop16(cpu);
    cpu->cycles += 8;
    ip_add = 1;
  }
  /* MOV R16, IMM16 */
  else if (ins_bytes[0] >= 0xB8 && ins_bytes[0] <= 0xBF) {
    cpu->regs[REG8086_AX + (ins_bytes[0] & 7)].x = get16(cpu, ip_cs + 1);
    
    cpu->cycles += 4;
    ip_add = 3;
  }
  /* MOV R8, IMM8 */
  else if (ins_bytes[0] >= 0xB0 && ins_bytes[0] <= 0xB7) {
    unsigned p_i = ins_bytes[0] >= 0xB4; /* It's the upper register parts if the second half */

    /* & 3 for % 4 */
    cpu->regs[REG8086_AX + (ins_bytes[0] & 3)].p[p_i] = get8(cpu, ip_cs + 1);
    
    cpu->cycles += 4;
    ip_add = 2;
  }
  /* Various conditional jumps, relative to current instruction. */
  else if (ins_bytes[0] >= 0x70 && ins_bytes[0] <= 0x7F) {
    uint8_t rel_addr = get8(cpu, ip_cs);
    
    if (check_cond_jmp(cpu, ins_bytes[0])) {
      short_jump(cpu, rel_addr);
      cpu->cycles += 16;
    }
    else {
      cpu->cycles += 4;
    }
  }
  else {
    cpu->e = E8086_BAD_OPCODE;
  }

  regseg_add(&cpu->regs[REG8086_IP], &cpu->regs[REG8086_CS], ip_add);
  return cpu->e != E8086_OK;
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

  /* Now for a final simple test of regseg_imm() + regseg() */
  HOPE_THAT(
    regseg_imm(&cpu, 0xDEAD, 0xBEEF) == (0xDEAD) + 0xBEEF0,
    "regseg_add() was correct for edge case."
  );

  cpu.regs[REG8086_AX].x = 0xDEAD;
  cpu.regs[REG8086_SS].x = 0xBEEF;
  HOPE_THAT(
    regseg(&cpu, REG8086_AX, REG8086_SS) == (0xDEAD) + 0xBEEF0,
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
    0x43, /* INC BX */
    0xB4, 0x69 /* MOV AL, 0x69 */
  };

  HOPE_THAT(
    init_mem8086(&mem, 0),
    "Memory initialized."
  );

  HOPE_THAT(
    reset_cpu8086(&cpu, &mem),
    "CPU initialized."
  );
  /* Force custom config on some registers for testing purposes */
  cpu.regs[REG8086_CS].x = 0;
  cpu.regs[REG8086_SS].x = 0xF000;
  cpu.regs[REG8086_SP].x = 0xFFF0;

  memcpy(mem.bytes, code, sizeof (code));

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x4269,
    "MOV AX, 0x4269."
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    mem.bytes[0xFFFEE] == 0x69 && mem.bytes[0xFFFEF] == 0x42,
    "PUSH AX."
  );

  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 0x4269 && cpu.regs[REG8086_SP].x == 0xFFF0,
    "POP BX => BX=0x4269 && SP is back."
  );
  
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_BX].x == 0x426A,
    "INC BX => BX=0x426A."
  );
  
  HOPE_THAT(!cycle_cpu8086(&cpu), "No error.");
  HOPE_THAT(
    cpu.regs[REG8086_AX].x == 0x6969,
    "MOV AL, 0x69 => AX=0x6969."
  );

  free_mem(&mem);
}

void test_parity_table(void) {
  HOPE_THAT(
    get_arr_bit(parity_table, 0) == 1,
    "Correct parity for 0."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 1) == 0,
    "Correct parity for 1."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 2) == 0,
    "Correct parity for 2."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 3) == 1,
    "Correct parity for 3."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 104) == 0,
    "Correct parity for 104."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 96) == 1,
    "Correct parity for 96."
  );
  HOPE_THAT(
    get_arr_bit(parity_table, 108) == 1,
    "Correct parity for 108."
  );
}

void add_cpu8086_tests(void) {
  ADD_TEST(test_regseg);
  ADD_TEST(test_cycling0);
  ADD_TEST(test_parity_table);
}
