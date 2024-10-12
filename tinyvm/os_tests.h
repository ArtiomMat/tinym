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
  if (args_n) {
    HOPE_THAT(
      init_os(), 
      "os initialized"
    );
    f = fopen_rel(args[0], "rb");

    HOPE_THAT(
      f != NULL,
      "Opening this binary worked."
    );

    fclose(f);
    free_os();
  }
}

void add_os_tests(void) {
  ADD_TEST(test_balloc);
  ADD_TEST(test_fopen_rel);
}
