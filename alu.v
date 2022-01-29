/*
 * alu.v 
 *
 * (C) Arlet Ottens, <arlet@c-scape.nl>
 */

module alu(
    input [8:0] alu_op,
    input [7:0] R,
    input [7:0] S,
    input [7:0] DI,
    input [7:0] DR,
    input C,
    output reg [7:0] alu_out,
    output reg alu_C,
    output alu_Z,
    output alu_N,
    output alu_V );

wire shift = alu_op[8];
wire right = alu_op[7];
reg [7:0] alu_ai;
reg [7:0] alu_bi;
reg alu_ci;
reg alu_si;
assign alu_Z = !alu_out;
assign alu_N = alu_out[7];
assign alu_V = alu_ai[7] ^  alu_bi[7] ^ alu_C ^ alu_N; 

always @* begin

    /* 
     * determine ALU A input.
     */
    casez( alu_op[6:4] )
        3'b0?0: alu_ai = R;         // input from register file
        3'b0?1: alu_ai = DR;        // input from data bus 
        3'b100: alu_ai = R | DR;    // ORA between register and memory
        3'b101: alu_ai = R & DR;    // AND between register and memory 
        3'b110: alu_ai = R ^ DR;    // EOR between register and memory 
        3'b111: alu_ai = S;         // stack pointer (for TSX)
    endcase
    
    /*
     * determine ALU B input
     */
    casez( alu_op[3:2] )
        2'b00: alu_bi = 0;          // for LDA, logic operations and INC
        2'b01: alu_bi = DR;         // for ADC
        2'b10: alu_bi = ~0;         // for DEC
        2'b11: alu_bi = ~DR;        // for SBC/CMP
    endcase

    /*
     * determine ALU carry input
     */
    casez( alu_op[1:0] )
        2'b00: alu_ci = 0;          // no carry
        2'b01: alu_ci = 1;          // carry=1 for INC
        2'b10: alu_ci = C;          // for ADC/SBC
        2'b11: alu_ci = 0;          // for rotate
    endcase

    /*
     * add it all up. If we don't need addition, then the B/C inputs
     * should be kept at 0.
     */

    {alu_C, alu_out} = alu_ai + alu_bi + alu_ci;

    /* 
     * determine shift input for rotate instructions
     */
    alu_si = C & alu_op[0];

    /* 
     * shift/rotate the result if necessary. Note that there's 
     * a trick to replace alu_out with DI input when shift=0, 
     * but right=1. This allows ALU bypass for PLA/PLX/PLY.
     */

    if( shift )
        if( right )
            {alu_out, alu_C} = {alu_si, alu_out};
        else
            {alu_C, alu_out} = {alu_out, alu_si};
    else if( right )
        alu_out = DI;
end

endmodule
