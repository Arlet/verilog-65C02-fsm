module regfile(
    input clk,
    input reg_we,
    input [1:0] reg_src,
    input [1:0] reg_dst,
    input [1:0] reg_idx,
    output [7:0] src,
    output [7:0] idx,
    input [7:0] dst,
    output reg [7:0] S,
    input txs,
    input push,
    input pull );

`include "define.i"

/*
 * register file
 */
reg [7:0] regs[3:0];                    // register file

/* 
 * initial values for easy debugging, not required
 */
initial begin
    regs[SEL_Z] = 0;                    // Z register 
    regs[SEL_X] = 1;                    // X register 
    regs[SEL_Y] = 2;                    // Y register
    regs[SEL_A] = 8'h41;                // A register
    S = 8'hff;                          // S register
end

/*
 * 1st read port: source register
 *
 */
assign src = regs[reg_src];

/*
 * 2nd read port: index register
 */
assign idx = regs[reg_idx];

/*
 * write port: destination register. 
 */
always @(posedge clk)
    if( reg_we )
        regs[reg_dst] <= dst;

/*
 * update stack pointer
 */
always @(posedge clk)
    if( txs )       S <= src;
    else if( push ) S <= S - 1;
    else if( pull ) S <= S + 1;

endmodule
