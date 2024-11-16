/* INCLUDED AT BOTTOM */

static void test_balloc(void) {
  const unsigned size = KB(64);

  char* ptr = balloc(size, BALLOC_R | BALLOC_W);
  HOPE_THAT(
    ptr != NULL,
    "balloc() works."
  );

  /* Should lead to a segfault if fails. */
  ptr[0] = ptr[1];

  bfree(ptr, size);
}

static void test_fopen_rel(void) {
  FILE* f;
  const char* exe_arg = args[0];
  unsigned i, last_slash;

  HOPE_THAT(
    args_n, 
    "there are arguments in the global vars args and args_n"
  );
  
  HOPE_THAT(
    init_os(), 
    "os initialized"
  );
  
  for (i = 0; exe_arg[i]; ++i) {
    if (exe_arg[i] == '/' || exe_arg[i] == '\\') {
      last_slash = i;
    }
  }

  f = fopen_rel(exe_arg + last_slash + 1, "rb");

  HOPE_THAT(
    f != NULL,
    "Opening this binary worked."
  );

  fclose(f);
  free_os();
}

void add_os_tests(void) {
  ADD_TEST(test_balloc);
  ADD_TEST(test_fopen_rel);
}
