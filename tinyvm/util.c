#include "util.h"
#include <stdio.h>
#include <string.h>

unsigned args_n = 0;
const char** args = NULL;

unsigned find_arg(const char* a) {
  unsigned i;
  for (i = 1; i < args_n; ++i) {
    if (!strcmp(args[i], a)) {
      return i;
    }
  }
  return 0;
}

int get_arr_bit(const void* _arr, unsigned i) {
  unsigned byte = i >> 3; /* The byte index in question in the loop */
  unsigned byte_bit = i & 7; /* The bit index inside of the byte, not absolute. */
  const char* arr = _arr;
  
  return (arr[byte] >> byte_bit) & 1;
}

void set_arr_bit(void* _arr, unsigned i, int x) {
  unsigned byte = i >> 3; /* The byte index in question in the loop */
  unsigned byte_bit = i & 7; /* The bit index inside of the byte, not absolute. */
  char* arr = _arr;
  
  if (x) {
    arr[byte] |= (1 << byte_bit);
  }
  else {
    arr[byte] &= ~(1 << byte_bit);
  }
}

void print_parity_table(void) {
  unsigned byte, bit, value_bit, set_bits;
  char value = 0; /*  */
  unsigned final = 0; /* Final byte to be printed for each 8 values. */

  fputs("static const unsigned char parity_table[] = {", stdout);
  /* Loop over each byte that houses 8 parity cached booleans. */
  for (byte = 0; byte < 32; ++byte) {
    /* Loop over 8 values for 8 bits */
    for (bit = 0, final = 0; bit < 8; ++bit, ++value) {
      /* Loop over the bits of that value to determine its parity */
      for (value_bit = 0, set_bits = 0; value_bit < 8; ++value_bit) {
        if (value & (1<<value_bit)) {
          ++set_bits;
        }
      }
      /* So was it even? */
      final |= (!(set_bits % 2)) << bit;
      
    }

    printf("0x%x, ", final);
  }
  puts("};");
}
