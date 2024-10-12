#include <stdlib.h>
#include <string.h>

#include "mem.h"
#include "test.h"
#include "os.h"
#include "util.h"

int init_mem(mem_t* m, unsigned size) {
  m->size = size;
  m->bytes = balloc(size, BALLOC_R | BALLOC_W);
  m->rom_table = NULL;

  return m->bytes != NULL;
}

int init_mem8086(mem_t* m, unsigned extra_size) {
  if (!init_mem(m, MB1 + extra_size)) {
    return 0;
  }
  /* BIOS code for hardware, address 0xC0000 to 0xF0000 */
  mark_rom_segs(m, 0xC, 0xF);
  return 1;
}

int mark_rom_segs(mem_t* m, unsigned from, unsigned to) {
  unsigned byte; /* The byte index in question in the loop */
  unsigned bit; /* The absolute bit index, in the entire bit array, not in the byte. */
  unsigned byte_bit; /* The bit index inside of the byte, not absolute. */

  if (m->rom_table == NULL) {
    /* 16 for the 64KB and 3 for 8 bits per uint8_t, +1 for any rounding error */
    m->rom_table = malloc((m->size >> (16 + 3)) + 1);

    if (m->rom_table == NULL) {
      return 0;
    }
  }

  #if 0
  for (bit = from, byte = (from >> 3), byte_bit = (from & 7); bit <= to; ++bit, ++byte_bit) {
    /* Every 8 bits we gotta increment the bit counter */
    if (byte_bit >= 8) {
      ++byte;
      byte_bit = 0;
    }

    m->rom_table[byte] |= (1 << byte_bit);
  }
  #endif
  
  for (bit = from; bit <= to; ++bit) {
    set_arr_bit(m->rom_table, bit, 1);
  }

  return 1;
}

int is_seg_rom(mem_t* m, unsigned i) {
  if (m->rom_table == NULL) {
    return 0;
  }
  /*return m->rom_table[i >> 3] & (1 << (i & 7));*/
  return get_arr_bit(m->rom_table, i);
}

void clear_rom_segs(mem_t* m) {
  free(m->rom_table);
  m->rom_table = NULL;
}

void free_mem(mem_t* m) {
  bfree(m->bytes, m->size);
  clear_rom_segs(m);
}

/*
=============================================================================================
                                        TESTS
=============================================================================================
*/

/* Tests all the ROM functionality. */
static void test_rom(void) {
  mem_t mem;
  init_mem(&mem, MB1 + 3);

  HOPE_THAT(
    mem.rom_table == NULL,
    "mem.rom_table is NULL in start."
  );

  HOPE_THAT(
    !is_seg_rom(&mem, 0x10003 >> 16),
    "memory doesn't have ROM."
  );

  mark_rom_segs(&mem, 0x1, 0x2);

  HOPE_THAT(
    mem.rom_table != NULL,
    "mem.rom_table allocated table."
  );
  HOPE_THAT(
    is_seg_rom(&mem, 0x10003 / KB(64)),
    "0x10003 is ROM."
  );

  mark_rom_segs(&mem, 0x5, 0x8);
  mark_rom_segs(&mem, 0x10, 0x10);

  HOPE_THAT(
    !is_seg_rom(&mem, 0x9533 / KB(64)),
    "0x9533 isn't ROM."
  );
  HOPE_THAT(
    is_seg_rom(&mem, 0x100002 / KB(64)),
    "0x100002 is ROM."
  );
  HOPE_THAT(
    is_seg_rom(&mem, 0x60302 / KB(64)),
    "0x60302 is ROM."
  );
  HOPE_THAT(
    !is_seg_rom(&mem, 0x40302 / KB(64)),
    "0x40302 isn't ROM."
  );

  free_mem(&mem);
}

void add_mem_tests(void) {
  add_os_tests(); /* Depends on proper functionality of balloc(). */
  ADD_TEST(test_rom);
}
