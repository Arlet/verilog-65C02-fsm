# verilog-65C02-fsm
A verilog model of the 65C02 CPU. The code is rewritten from scratch.

* Assumes synchronous memory
* Uses finite state machine rather than microcode for control
* Designed for simplicity, size and speed
* Reduced cycle count eliminates all unnecessary cycles

## Design goals
The main design goal is to provide an easy understand implementation that has good performance

## Code
Code is far from complete.  Right now it's in a 'proof of concept' stage where the address
generation and ALU are done in a quick and dirty fashion to test some new ideas. Once I'm happy
with the overall design, I can do some optimizations. 

* cpu.v module is the top level. 

Code has been tested with Verilator. 

## Status

* All CMOS/NMOS 6502 instructions added (except for NOPs as undefined, Rockwell/WDC extensions)
* Model passes Klaus Dormann's test suite for 6502 (with BCD *disabled*)
* BCD not yet supported
* SYNC, RST supported
* IRQ, RDY, NMI not yet supported

### Cycle counts
For purpose of minimizing design and performance improvement, I did not keep the original cycle
count. All of the so-called dead cycles have been removed.

| Instruction type | Cycles |
| :--------------: | :----: |
| Implied PHx/PLx  |   2    |
| RTS              |   4    |
| RTI              |   5    |
| BRK              |   7    |
| Other implied    |   1    |
| JMP Absolute     |   3    |
| JMP (Indirect)   |   5    |
| JSR Absolute     |   5    |
| branch           |   2    |
| Immediate        |   2    |
| Zero page        |   3    |
| Zero page, X     |   3    |
| Zero page, Y     |   3    |
| Absolute         |   4    |
| Absolute, X      |   4    |
| Absolute, Y      |   4    |
| (Zero page)      |   5    |
| (Zero page), Y   |   5    |
| (Zero page, X)   |   5    |

Add 1 cycle for any read-modify-write. There is no extra cycle for taken branches, page overflows, or for X/Y offset calculations.

Have fun. 
