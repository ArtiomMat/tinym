/* ops.h : Enums and constants that help idetifying and parsing instructions for the 86 family. */

#ifndef OPS_H
#define OPS_H

enum {
  MOD_NLH, /* No low/high order disp */
  MOD_L16, /* Low order disp, sign extended to 16-bits */
  MOD_LH, /* Both low/high order disp */
  MOD_RM_IS_REG /* R/M is a REG field  */
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

/* The various combinations for R/M */
enum {
  RM_BX_SI_D,
  RM_BX_DI_D,
  RM_BP_SI_D,
  RM_BP_DI_D,
  RM_SI_D,
  RM_DI_D,
  RM_BP_D,
  RM_BX_D
};

#endif
