/* ops.h : Enums and constants that help idetifying and parsing instructions for the 86 family. */

#ifndef OPS_H
#define OPS_H

enum {
  MOD_NO_DISP, /* No low/high order displacement added. */
  MOD_DISP8, /* 8 bit displacement added, so 1 byte follow ModRM byte.. */
  MOD_DISP16, /* 16 bit displacement added, so 2 bytes follow ModRM byte. */
  MOD_RM_IS_REG /* R/M is a REG field. */
};

/* REG part when Word bit is 1 */
enum {
  REGW_AX,
  REGW_CX,
  REGW_DX,
  REGW_BX,
  
  REGW_SP,
  REGW_BP,

  REGW_SI,
  REGW_DI
};

/* REG part when Word bit is 0 */
enum {
  REGB_AL,
  REGB_CL,
  REGB_DL,
  REGB_BL,

  REGB_AH,
  REGB_CH,
  REGB_DH,
  REGB_BH
};

/* Segment registers */
enum {
  SEG_ES,
  SEG_CS,
  SEG_SS,
  SEG_DS,

  /* 386+ segment registers */
  SEG_FS, SEG_GS
};

/*
  The various combinations for R/M.
  The possible MOD values can be viewed in MOD_*, where it's described what happens when RM is
  expanded.
*/
enum {
  RM_BX_SI,
  RM_BX_DI,
  RM_BP_SI,
  RM_BP_DI,
  RM_SI,
  RM_DI,
  RM_BP, /* This one is special, for MOD=0 it's "direct address", otherwise BP + MOD's displacement config */
  RM_BX
};

#endif
