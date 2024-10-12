#ifdef __linux__

#include <unistd.h>
#include <sys/mman.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <stdint.h>
#include <string.h>
#include "util.h"
#include "os.h"

static char exe_path[256];
static unsigned last_slash;

int init_os(void) {
  unsigned i, j;
  uint32_t one = 1;

  if (os_page_size) {
    return 1;
  }

  /*
    Determining exe_path, not full implementation because the program might be 
    in $PATH and have no relative path to cwd.
  */
  if (getcwd(exe_path, sizeof (exe_path)) == NULL) {
    fputs("init_os(): getcwd() failed.", stderr);
    return 0;
  }
  if (!args_n) {
    fputs("init_os(): args and args_n not initialized.", stderr);
    return 0;
  }
  if (args[0][0] != '.') {
    fputs("init_os(): tinyvm is in $PATH probably, don't support that yet.", stderr);
    return 0;
  }
  /* Find last slash */
  for (i = 0; exe_path[i]; ++i) {
    if (exe_path[i] == '/') {
      last_slash = i;
    }
  }
  /* Copy args[0] to add where the exe is in that cwd */
  for (i = last_slash + 1, j = 0; i < sizeof (exe_path) && args[0][j]; ++i, ++j) {
    exe_path[i] = args[0][j];
    if (exe_path[i] == '/') {
      last_slash = i;
    }
  }

  os_page_size = sysconf(_SC_PAGE_SIZE);

  if (*(char *)&one != 1) {
    fputs("init_os(): Big-endian is not supported.", stderr);
    return 0;
  }

  return 1;
}

void free_os(void) {
  os_page_size = 0;
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

FILE* fopen_rel(const char* fp, const char* m) {
  unsigned i, j;
  for (i = last_slash + 1, j = 0; i < sizeof (exe_path)-1 && fp[j]; ++i, ++j) {
    exe_path[i] = fp[j];
  }
  exe_path[i] = 0;
  return fopen(exe_path, m);
}

#endif