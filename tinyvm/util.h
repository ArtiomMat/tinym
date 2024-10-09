#ifndef UTIL_H
#define UTIL_H

/* It's for source code and caching in constant memory. */
void print_parity_table(void);

/* Returns bit I in ARR, order is from LSB to MSB, and from lower to upper memory in ARR. */
int get_arr_bit(const void* arr, unsigned i);
/* Same rules as get_arr_bit(), but you set that bit. */
void set_arr_bit(void* arr, unsigned i, int x);

#endif
