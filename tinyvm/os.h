/**
 * @brief An interface with various operating system stuff needed by the VM.
 */

#ifndef OS_H
#define OS_H

#include <stdio.h>
#include <stddef.h>

#if defined(__x86_64__)
#define GET_BYTE(X, B) (X >> (8 * B))
#else
#define GET_BYTE(X, B) (X >> (8 * B))
#error cant identify system endian.
#endif

enum {
  BALLOC_X = 1, /* Executable */
  BALLOC_R = 2, /* Readable */
  BALLOC_W = 4 /* Writable */
};

/* But first init_os(). 0 means OS module isn't initialized. */
extern unsigned os_page_size;

int init_os(void);
void free_os(void);

FILE* fopen_rel(const char* fp, const char* m);

/*
  Allocate a big chunk of memory with N bytes, FL determines various flags from BALLOC_*.
  Return NULL if fails to allocate properly.
  Aligned to os_page_size bytes.
  Must be freed with bfree.
*/
void* balloc(unsigned n, int fl);
/* B may be NULL in which case nothing happens. N must be the allocated number of bytes. */
void bfree(void* b, unsigned n);

/* Returns a handler to the process, SANDBOX determines if it's to be limited for safety. */
int open_process(int sandbox);
void close_process(int p);

void add_os_tests(void);

#endif