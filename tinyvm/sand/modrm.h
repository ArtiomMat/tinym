/* ops.h : Enums and constants that help idetifying and parsing modrm for the 86 family. */

#ifndef OPS_H
#define OPS_H

enum {
  MODRM_MOD_MASK = 0x3 << 6,
  MODRM_REG_MASK = 0x7 << 3,
  MODRM_OP_MASK = MODRM_REG_MASK,
  MODRM_RM_MASK = 0x7
};

enum {
  MOD_NDISP = 0 << 6, /* No low/high order displacement added. */
  MOD_DISP8 = 1 << 6, /* 8 bit displacement added, so 1 byte follow ModRM byte.. */
  MOD_DISP16 = 2 << 6, /* 16 bit displacement added, so 2 bytes follow ModRM byte. */
  MOD_RM_IS_REG = 3 << 6 /* R/M is a REG field. */
};

/* REG part when Word bit is 1 */
enum {
  REGW_AX = 0 << 3, /* (REGW_* - REGW_AX) MUST EQUAL (REG8086_* - REGW_AX) */
  REGW_CX = 1 << 3,
  REGW_DX = 2 << 3,
  REGW_BX = 3 << 3,
  
  REGW_SP = 4 << 3,
  REGW_BP = 5 << 3,

  REGW_SI = 6 << 3,
  REGW_DI = 7 << 3
};

/* REG part when Word bit is 0 */
enum {
  REGB_AL = 0 << 3,
  REGB_CL = 1 << 3,
  REGB_DL = 2 << 3,
  REGB_BL = 3 << 3,

  REGB_AH = 4 << 3,
  REGB_CH = 5 << 3,
  REGB_DH = 6 << 3,
  REGB_BH = 7 << 3
};

/* Segment registers in special instructions where the destination operand is the  */
enum {
  SEG_ES = 0 << 3,
  SEG_CS = 1 << 3,
  SEG_SS = 2 << 3,
  SEG_DS = 3 << 3,

  /* 386+ segment registers */
  SEG_FS, SEG_GS
};

/*
  The various combinations for R/M.
  The possible MOD values can be viewed in MOD_*, where it's described what happens when RM is
  expanded.
*/
enum {
  RM_BX_SI = 0,
  RM_BX_DI = 1,
  RM_BP_SI = 2,
  RM_BP_DI = 3,
  RM_SI = 4,
  RM_DI = 5,
  RM_BP = 6, /* This one is special, for MOD=0 it's "direct address", otherwise BP + MOD's displacement config */
  RM_BX = 7
};

/* Grp1 OP part */
enum {
  OP_G1_TEST = 0 << 3,

  OP_G1_NOT = 2 << 3,
  OP_G1_NEG = 3 << 3,
  OP_G1_MUL = 4 << 3,
  OP_G1_IMUL = 5 << 3,
  OP_G1_DIV = 6 << 3,
  OP_G1_IDIV = 7 << 3
};
/* IMM OP part */
enum {
  OP_IMM_ADD = 0 << 3,
  OP_IMM_OR = 1 << 3,
  OP_IMM_ADC = 2 << 3,
  OP_IMM_SBB = 3 << 3,
  OP_IMM_AND = 4 << 3,
  OP_IMM_SUB = 5 << 3,
  OP_IMM_XOR = 6 << 3,
  OP_IMM_CMP = 7 << 3
};
/* Grp2B OP part */
enum {
  OP_G2B_INC = 0 << 3,
  OP_G2B_DEC = 1 << 3,
};

#endif
