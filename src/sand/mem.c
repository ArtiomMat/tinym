#include <stdlib.h>
#include <string.h>

#include "mem.h"
#include "../box/os.h"

int init_mem(mem_t* m, unsigned size) {
  m->size = size;
  m->bytes = balloc(size, BALLOC_R);

  return m->bytes != NULL;
}

int mark_mem_rom(mem_t* m, unsigned from, unsigned to) {
  if (m->rom_table == NULL) {
    /* 14 for the 16KB and 3 for 8 bits per uint8_t, +1 for any rounding error */
    m->rom_table = malloc(m->size >> (14 + 3) + 1);
    
    if (m->rom_table == NULL) {
      return 0;
    }
  }

  memset(m->rom_table + from, 1, to);

  return 1;
}

int is_mem_rom(mem_t* m, unsigned i) {
  if (m->rom_table == NULL) {
    return 0;
  }

  return m->rom_table[i >> 3] & (1 << (i & 7));
}

void clear_mem_rom(mem_t* m) {
  free(m->rom_table);
  m->rom_table = NULL;
}

void free_mem(mem_t* m) {
  bfree(m->bytes, m->size);
  clear_mem_rom(m);
}
