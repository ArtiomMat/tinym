#include "../sand/cpu8086.h"
#include "os.h"

#include <stdio.h>

int main(int args_n, const char** args) {
  init_os();
  printf("%u\n", os_page_size);
  return 0;
}
