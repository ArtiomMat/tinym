/* os.h : An interface with various operating system stuff needed by the VM. */

#ifndef OS_H
#define OS_H

#include <stddef.h>

enum {
  BALLOC_X = 1, /* Executable */
  BALLOC_R = 2, /* Readable */
  BALLOC_W = 4 /* Writable */
};

/* But first init_os() */
extern unsigned os_page_size;

int init_os();
void free_os();

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

#endif