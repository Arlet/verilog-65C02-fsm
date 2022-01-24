Instruction sequences
=====================


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
| ZPG0  | 0012 | F802 | 12 |    |               |
| DATA  | F802 | F802 | 55 |    |               |
| SYNC  | F803 | F803 |    | 55 | store DR in A |


LDA ABS
-------

example: LDA $1234, sequence AD 34 12. The location $1234
contains the value $55

| state | AB   |  PC  | DB | DR | Comment       |
|:-----:|:----:|:----:|:--:|:--:|---------------|
|       | F800 |      |    |    |               |
| SYNC  | F801 | F801 | AD |    |               |
| ABS0  | F802 | F802 | 34 |    |               |
| ABS1  | 1234 | F803 | 12 | 34 |               |
| DATA  | F803 | F803 | 55 |    |               |
| SYNC  | F804 | F804 |    | 55 | store DR in A |


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


