#ifdef __linux__
#include <unistd.h>
#include <sys/mman.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#endif

#include "os.h"

unsigned os_page_size = 0;

int init_os() {
  #ifdef __linux__
    os_page_size = sysconf(_SC_PAGE_SIZE);
  #else
    return 0;
  #endif

  return 1;
}

void free_os() {
  #ifdef __linux__
  #else
  #endif
}

void* balloc(unsigned n, int fl) {
  void* ret;
  
  #ifdef __linux__
    int prot = PROT_NONE;
    
    if (fl & BALLOC_R) {
      prot |= PROT_READ;
    } else if (fl & BALLOC_W) {
      prot |= PROT_WRITE;
    } else if (fl & BALLOC_X) {
      prot |= PROT_EXEC;
    }

    ret = mmap(NULL, n, prot, MAP_SHARED | MAP_ANONYMOUS, 0, 0);
  #else
    return NULL;
  #endif

  return ret;
}

void bfree(void* b, unsigned n) {
  if (b == NULL) {
    return;
  }
  
  #ifdef __linux__
    munmap(b, n);
  #else
  #endif
}