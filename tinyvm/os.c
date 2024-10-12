#include "os.h"
#include "sand/mem.h"
#include "test.h"
#include "util.h"

unsigned os_page_size = 0;

#ifdef NDEBUG
  void add_os_tests(void) {}
#else
  #include "os_tests.h"
#endif
