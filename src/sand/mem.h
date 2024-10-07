#ifndef MEM_H
#define MEM_H

#include <stdint.h>

#define KILOBYTE 1024U
#define MEGABYTE 1048576U
#define GIGABYTE 1073741824U

/*
  May act as a fake memory, like in real-mode, where it's not actually a contiguous 1MB but rather an illusion.
  The idea is to just make memory interfacing and virtualization straight forward, there is mem_t
  and the whole idea is that it stores a contiguous buffer, and the CPU virtualization decides how
  to interpret it. 
*/
typedef struct {
  uint8_t* bytes;
  /* Array of tightly packed 1bit-booleans, 1 means this 16KB segment is ROM, 0 it isn't */
  uint8_t* rom_table;
  unsigned size;
} mem_t;

int init_mem(mem_t* m, unsigned size);
/* Preconfigure M to fit the 8086 requirements, EXTRA_SIZE is the extra memory added. */
int init_mem8086(mem_t* m, unsigned extra_size);
/*
  FROM and TO must are 16KB memory segments, not single byte segments.
  If this is the first time marking then the rom table is automatically allocated.
*/
int mark_mem_rom(mem_t* m, unsigned from, unsigned to);
/*
  Returns 0 if M->bytes[I] is, 1 if it isn't.
  If mark_mem_rom was never called then it will always return 0.
*/
int is_mem_rom(mem_t* m, unsigned i);
/* Make all of M's 16KB segments non ROM marked. And also free M->rom_table since don't need it. */
void clear_mem_rom(mem_t* m);
int free_mem(mem_t* m);

#endif