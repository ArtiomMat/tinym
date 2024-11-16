/* INCLUDED AT BOTTOM */

/* Tests all the ROM functionality. */
static void test_rom(void) {
  mem_t mem;
  init_mem(&mem, MB1 + 3);

  HOPE_THAT(
    mem.rom_table == NULL,
    "mem.rom_table is NULL in start."
  );

  HOPE_THAT(
    !is_seg_rom(&mem, 0x10003 >> 16),
    "memory doesn't have ROM."
  );

  mark_rom_segs(&mem, 0x1, 0x2);

  HOPE_THAT(
    mem.rom_table != NULL,
    "mem.rom_table allocated table."
  );
  HOPE_THAT(
    is_seg_rom(&mem, 0x10003 / KB(64)),
    "0x10003 is ROM."
  );

  mark_rom_segs(&mem, 0x5, 0x8);
  mark_rom_segs(&mem, 0x10, 0x10);

  HOPE_THAT(
    !is_seg_rom(&mem, 0x9533 / KB(64)),
    "0x9533 isn't ROM."
  );
  HOPE_THAT(
    is_seg_rom(&mem, 0x100002 / KB(64)),
    "0x100002 is ROM."
  );
  HOPE_THAT(
    is_seg_rom(&mem, 0x60302 / KB(64)),
    "0x60302 is ROM."
  );
  HOPE_THAT(
    !is_seg_rom(&mem, 0x40302 / KB(64)),
    "0x40302 isn't ROM."
  );

  free_mem(&mem);
}

void add_mem_tests(void) {
  add_os_tests(); /* Depends on proper functionality of balloc(). */
  ADD_TEST(test_rom);
}
