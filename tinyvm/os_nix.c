#ifdef __linux__

#include <unistd.h>
#include <sys/mman.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <stdint.h>

#include "os.h"

int init_os(void) {
  uint32_t one = 1;
  os_page_size = sysconf(_SC_PAGE_SIZE);

  if (*(char *)&one != 1) {
    puts("init_os(): Big-endian is not supported.");
    return 0;
  }

  return 1;
}

void free_os(void) {
  
}

void* balloc(unsigned n, int fl) {
  void* ret;
  
  int prot = PROT_NONE;
  
  if (fl & BALLOC_R) {
    prot |= PROT_READ;
  }
  if (fl & BALLOC_W) {
    prot |= PROT_WRITE;
  }
  if (fl & BALLOC_X) {
    prot |= PROT_EXEC;
  }

  ret = mmap(NULL, n, prot, MAP_SHARED | MAP_ANONYMOUS, 0, 0);

  return ret;
}

void bfree(void* b, unsigned n) {
  if (b == NULL) {
    return;
  }
  
  munmap(b, n);
}

#endif