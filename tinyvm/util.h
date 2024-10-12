#ifndef UTIL_H
#define UTIL_H

extern unsigned args_n;
extern const char** args;

/*
  Find A in args.
  Returns 0 if not found(the first argument is useless), otherwise index.
*/
unsigned find_arg(const char* a);

/* It's for source code and caching in constant memory. */
void print_parity_table(void);

/* Returns bit I in ARR, order is from LSB to MSB, and from lower to upper memory in ARR. */
int get_arr_bit(const void* arr, unsigned i);
/* Same rules as get_arr_bit(), but you set that bit. */
void set_arr_bit(void* arr, unsigned i, int x);

#endif
