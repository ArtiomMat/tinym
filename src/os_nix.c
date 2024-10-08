#ifdef __linux__

#include <unistd.h>
#include <sys/mman.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>

#include "os.h"

int init_os(void) {
  os_page_size = sysconf(_SC_PAGE_SIZE);

  return 1;
}

void free_os(void) {
  
}

void* balloc(unsigned n, int fl) {
  void* ret;
  
  int prot = PROT_NONE;
  
  if (fl & BALLOC_R) {
    prot |= PROT_READ;
  } else if (fl & BALLOC_W) {
    prot |= PROT_WRITE;
  } else if (fl & BALLOC_X) {
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