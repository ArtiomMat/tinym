#include "sand/cpu8086.h"
#include "os.h"
#include "test.h"

#include <assert.h>
#include <string.h>
#include <stdio.h>

int args_n;
const char** args;

/*
  Find A in argv.
  Returns 0 if not found(the first argument is useless), otherwise index.
*/
unsigned find_arg(const char* a) {
  int i;
  for (i = 1; i < args_n; ++i) {
    if (!strcmp(args[i], a)) {
      return i;
    }
  }
  return 0;
}

int main(int _args_n, const char** _args) {
  unsigned i;
  mem_t mem;
  cpu8086_t cpu;

  args_n = _args_n;
  args = _args;

  if (find_arg("--test")) {
    init_tests();

    add_mem_tests();
    add_cpu8086_tests();

    run_tests();
    free_tests();
  }

  if (i = find_arg("--boot-sector")) {
    
  }

  init_os();
  
  init_mem8086(&mem, 0);
  reset_cpu8086(&cpu, &mem);
  cycle_cpu8086(&cpu);

  /* printf("0xC0020 is ROM: %i.\n", ) */
  
  return 0;
}
