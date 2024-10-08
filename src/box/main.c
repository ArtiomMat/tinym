#include "sand/cpu8086.h"
#include "os.h"
#include "test.h"

#include <string.h>
#include <stdio.h>

int main(int args_n, const char** args) {
  mem_t mem;
  cpu8086_t cpu;

  /* TESTS */
  if (args_n > 1 && !strcmp(args[1], "--test")) {
    init_tests();

    add_mem_tests();

    run_tests();
    free_tests();
  }

  init_os();
  printf("OS PAGE SIZE: %u.\n", os_page_size);
  
  init_mem8086(&mem, 0);
  reset_cpu8086(&cpu, &mem);

  /* printf("0xC0020 is ROM: %i.\n", ) */
  
  return 0;
}
