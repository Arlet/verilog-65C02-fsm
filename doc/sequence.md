Instruction sequences
=====================


LDA IMM
-------

example: LDA #$55, sequence A9 55

| state | AB   |  PC  | DB | ALU | Comment       |
|:-----:|:----:|:----:|:--:|:---:|---------------|
|       | F800 |      |    |     |               |
| SYNC  | F801 | F801 | A9 |     |               |
| IMM0  | F802 | F802 | 55 |     |               |
| SYNC  | F803 | F803 |    | 55  | -> store in A |


LDA ZP
------
example: LDA #$12, sequence A5 12. The location $12
contains the value $55

| state | AB   |  PC  | DB | ALU | Comment       |
|:-----:|:----:|:----:|:--:|:---:|---------------|
|       | F800 |      |    |     |               |
| SYNC  | F801 | F801 | A5 |     |               |
| ZPG0  | 0012 | F802 | 12 |     |               |
| DATA  | F802 | F802 | 55 |     |               |
| SYNC  | F803 | F803 |    | 55  | -> store in A |


LDA ABS
-------

example: LDA $1234, sequence AD 34 12. The location $1234
contains the value $55

| state | AB   |  PC  | DB | ALU | Comment       |
|:-----:|:----:|:----:|:--:|:---:|---------------|
|       | F800 |      |    |     |               |
| SYNC  | F801 | F801 | AD |     |               |
| ABS0  | F802 | F802 | 34 |     |               |
| ABS1  | 1234 | F803 | 12 |     |               |
| DATA  | F803 | F803 | 55 |     |               |
| SYNC  | F804 | F804 |    | 55  | -> store in A |

