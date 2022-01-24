Instruction sequences
=====================

These are all important instruction sequences used in the model. In these
examples, the code is located at F800 and further. Some examples use the
'Z' register. This is a register that contains the constant value 0.

* The value 'AB' is the combinatorial output of the address bus.

* The 'PC' value is 16 bit register that holds the current program
counter in most cycles, but can be temporarily wrong in some cases.

* The 'DB' value represents the data bus, and can be either input or
output. When reading from the data bus, the DB value corresponds to
the AB value from the line above, because the memory is synchronous and
takes 1 clock cycle to produce the data. When writing, the AB/DB values
on the same row are used together.

* The 'DR' is a data register that can hold a copy of a value seen on the
data bus. This value may be used subsequently in the ALU, for instance
in the LDA instruction, or sometimes it can hold part of the address,
for instance in absolute addressing modes.

* Whenever a cell in the table is empty, then its value is not relevant. 


LDA IMM
-------

example: LDA #$55, sequence A9 55

| state | AB   |  PC  | DB | DR | Comment       |
|:-----:|:----:|:----:|:--:|:--:|---------------|
|       | F800 |      |    |    |               |
| SYNC  | F801 | F801 | A9 |    |               |
| IMM0  | F802 | F802 | 55 |    |               |
| SYNC  | F803 | F803 |    | 55 | store DR in A |


LDA ZP
------
example: LDA #$12, sequence A5 12. The location $12
contains the value $55

| state | AB   |  PC  | DB | DR | Comment       |
|:-----:|:----:|:----:|:--:|:--:|---------------|
|       | F800 |      |    |    |               |
| SYNC  | F801 | F801 | A5 |    |               |
| ZPG0  | 0012 | F802 | 12 |    | AB = DB + Z   |
| DATA  | F802 | F802 | 55 |    |               |
| SYNC  | F803 | F803 |    | 55 | store DR in A |

The LDA ZP,X follows the same pattern, but uses +X instead 
of +Z in the ZPG0 state.

LDA ABS
-------

example: LDA $1234, sequence AD 34 12. The location $1234
contains the value $55

| state | AB   |  PC  | DB | DR | Comment       |
|:-----:|:----:|:----:|:--:|:--:|---------------|
|       | F800 |      |    |    |               |
| SYNC  | F801 | F801 | AD |    |               |
| ABS0  | F802 | F802 | 34 |    |               |
| ABS1  | 1234 | F803 | 12 | 34 | AB={DB,DR}+Z  |
| DATA  | F803 | F803 | 55 |    |               |
| SYNC  | F804 | F804 |    | 55 | store DR in A |

The LDA ABS,X/Y instructions follow the same pattern, but use +X or +Y instead of +Z in the ABS1 state.

LDA (ZP),Y
----------

example: LDA ($14),Y sequence B1 14. The location $14
contains the value $32, location $15 contains $12, and Y=2.
The location $1234 contains $55

| state | AB   |  PC  | DB | DR | Comment        |
|:-----:|:----:|:----:|:--:|:--:|----------------|
|       | F800 |      |    |    |                |
| SYNC  | F801 | F801 | B1 |    |                |
| IDX0  | 0014 | F802 | 14 |    | AB = DB+Z      |
| IDX1  | 0015 | F802 | 32 |    | AB = ABR+1     |
| IDX2  | 1234 | F802 | 12 | 32 | AB = {DR,DB}+Y |
| DATA  | F802 | F803 | 55 |    |                |
| SYNC  | F803 | F804 |    | 55 | store DR in A  | 

The LDA (ZP) follows the same sequence, but adds Z register instead of Y in IDX2 state.
The LDA (ZP,X) also follows same sequence, but adds X register instead of Z in IDX0 state

The ORA, EOR, AND, ADC, SBC, and CMP instructions follow the same patterns, but have a different ALU operation.

The STA instructions follow the same patterns, but perform a memory write when the effective
address is on the bus in the ZPG0, ABS1, or IDX2 states. 

INC ZP
------
example: INC $12, sequence E6 12. The location $12
contains the value $55

| state | AB   |  PC  | DB | DR | Comment          |
|:-----:|:----:|:----:|:--:|:--:|------------------|
|       | F800 |      |    |    |                  |
| SYNC  | F801 | F801 | E6 |    |                  |
| ZPG0  | 0012 | F802 | 12 |    | AB = DB + Z      |
| DATA  | F802 | F802 | 55 |    | load next opcode |
| RDWR  | 0012 | F803 | 56 | 55 | write DR+1       |
| SYNC  | F803 | F803 |    |    |                  |


INC ABS 
------
example: INC $1234, sequence EE 34 12. The location $1234
contains the value $55

| state | AB   |  PC  | DB | DR | Comment          |
|:-----:|:----:|:----:|:--:|:--:|------------------|
|       | F800 |      |    |    |                  |
| SYNC  | F801 | F801 | EE |    |                  |
| ABS0  | F802 | F802 | 34 |    |                  |
| ABS1  | 1234 | F803 | 12 | 34 | AB = {DB,DR}+Z   |
| DATA  | F803 | F803 | 55 |    | load next opcode |
| RDWR  | 0012 | F804 | 56 | 55 | write DR+1       |
| SYNC  | F804 | F804 |    |    |                  |


INX
---
example INX, sequence E8. This is a very simple instruction, executed in a single cycle. 
All other register implied instructions follow the same pattern, but use a different ALU
operation, and different register file controls.

| state | AB   |  PC  | DB | DR | Comment          |
|:-----:|:----:|:----:|:--:|:--:|------------------|
|       | F800 |      |    |    |                  |
| SYNC  | F801 | F801 | E8 |    | write ALU to X   |
| SYNC  | F802 | F802 |    |    |                  |




