#ifndef MEM_H
#define MEM_H

#include <stdint.h>

#define KB1 1024U
#define MB1 1048576U
#define GB1 1073741824U

#define KB(N) (N*1024U)
#define MB(N) (N*1048576U)
#define GB(N) (N*1073741824U)

/**
  * @brief May act as a fake memory, like in real-mode, where it's not actually a contiguous 1MB 
  * but rather an illusion. The idea is to just make memory interfacing and virtualization straight 
  * forward, there is mem_t and the whole idea is that it stores a contiguous buffer, and the CPU 
  * virtualization decides how to interpret it. 
*/
typedef struct {
  uint8_t* bytes;
  /* Array of tightly packed 1bit-booleans, 1 means this 64KB segment is ROM, 0 it isn't */
  uint8_t* rom_table;
  unsigned size;
} mem_t;

int init_mem(mem_t* m, unsigned size);
/* Preconfigure M to fit the 8086 requirements, EXTRA_SIZE is the extra memory added. */
int init_mem8086(mem_t* m, unsigned extra_size);
/*
  FROM and TO must are 64KB memory segments, not single byte segments.
  If this is the first time marking then the rom table is automatically allocated.
  Return value can be 0 if the M->rom_table failed to allocate.
*/
int mark_rom_segs(mem_t* m, unsigned from, unsigned to);
/*
  Returns 0 if the I-th 64KB segment is ROM marked.
  If mark_rom_segs() was never called then it will always return 0.
*/
int is_seg_rom(mem_t* m, unsigned i);
/* Make all of M's 64KB segments non ROM marked. And also free M->rom_table since don't need it. */
void clear_rom_segs(mem_t* m);
void free_mem(mem_t* m);

void add_mem_tests(void);

#endif