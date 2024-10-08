# Tiny VM
Maizuz wants a VM to run viruses on. Time to emulate 8086.
Written entirely in ANSI C, 0 dependency, -pedantic.

# Trigger warning
The VM might be innacurate, simulating things incorrectly while appearing to give the same result,
due to my ignorance or just me not getting the opportunity to learn an aspect of the CPU.
I tried/trying my best to make the logical part of things as true to the OG, but also as optimized
as possible.

# Project structure

- `src` : source/header directory, the files found there are shared.
  - `box` : the parent process that initializes everything and keeps track of user-end.
  - `sand` : the actual code for the virtualized environment, that is safely isolated.
