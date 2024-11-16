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

/**
 * @brief `fopen()` but relative to this executable's path.
 * @attention `init_os()` must be called first.
 * @param fp File path, relative to this file.
 * @param m Mode, just like in `fopen()`.
 * @return File pointer from `fopen()`.
 */
FILE* fopen_rel(const char* fp, const char* m);

/**
 * @brief Allocate a big chunk of memory.
 * @attention Free the returned buffer with `bfree()`
 * @param n Size of the chunk.
 * @param fl Flags, `BALLOC_*` enum.
 * @return Pointer to the buffer, can be NULL if fails, aligned to `os_page_size`.
 */
void* balloc(unsigned n, int fl);

/**
 * @brief Free a buffer alocated with `balloc()`
 * @param b Can be NULL.
 * @param n Size of the buffer allocated, must match what was in `balloc()`.
 */
void bfree(void* b, unsigned n);

/* Returns a handler to the process, SANDBOX determines if it's to be limited for safety. */

/**
 * @brief Opens a seperate process, with seperate registers, but its a child.
 * @param sandbox Whether or not to sandbox it, limit syscalls mainly.
 * @return Handle to the process.
 */
int open_process(int sandbox);
/**
 * @brief Forcefully close the process.
 * @param p 
 */
void close_process(int p);

void add_os_tests(void);

#endif