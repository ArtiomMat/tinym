# Tiny VM
Maizuz wants a VM to run viruses on. Time to emulate 8086.
Note that this is an emulator, rather than a VM.
Written entirely in ANSI C, 0 dependency, -pedantic.

# Virtualization progress
"roleh" from [here](https://hackaday.io/project/193288-improved-8086-opcode-map) built a really nice map for 8086 instructions, helps a lot! So here's the current progress on the 8086 CPU:
![](8086-progress.png)
* `GREEN`: Implemented, adequately(for my standards) tested.\
* `YELLOW`: Implemented, in theory, not tested enough to be green.\
* `RED`: Unimplemented, on the radar to be implemented next.
* `BLUE`: Unimplemented, but identified as relatively easy to implement.

# Trigger warning
The VM might be innacurate, simulating things incorrectly while appearing to give the same result,
due to my ignorance or just me not getting the opportunity to learn an aspect of the CPU.
I tried/trying my best to make the logical part of things as true to the OG, but also as optimized
as possible.
## Current non standards(DEFINATELY FEATURES AND NOT LAZINESS)
* Segment prefixes work for everything, even `CALL` and `JMP`, so you can replace `CS` sreg with `DS` for example in some cases.
* Some errors like 1MB access violation are fatal and pretty much UB(UB that is safe to the host machine), but not in the OG 8086.
* Cycles, cycle counting needs to be worked on, and timing.

# Project structure

- `tinyvm` : source/header directory, the files found there are shared.
  - `box` : the parent process that initializes everything and keeps track of user-end.
  - `sand` : the actual code for the virtualized environment, that is safely isolated.

----

<small>5'nizza...</small>
