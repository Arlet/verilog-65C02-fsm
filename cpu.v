/*
 * verilog model of 65C02 CPU.
 *
 * (C) Arlet Ottens, <arlet@c-scape.nl>
 *
 */

module cpu( 
    input clk,                          // CPU clock
    input RST,                          // RST signal
    output [15:0] AB,                   // address bus (combinatorial) 
    output sync,                        // start of new instruction
    input [7:0] DI,                     // data bus input
    output reg [7:0] DO,                // data bus output 
    output reg WE,                      // write enable
    input IRQ,                          // interrupt request
    input NMI,                          // non-maskable interrupt request
    input RDY,                          // Ready signal. Pauses CPU when RDY=0
    input debug );                      // debug for simulation

wire [15:0] PC;                         // program counter 

reg [7:0] DR;                           // data register, registered DI 
wire [7:0] IR;                          // instruction register

wire B = 1;
reg N, V, D, I, Z, C;                   // processor status flags 
wire [7:0] P = { N, V, 1'b1, B, D, I, Z, C };

wire alu_C, alu_Z, alu_V, alu_N;
wire [7:0] alu_out;

/*
 * state
 */

`include "define.i"

/*
 * control bits
 */

reg [4:0] state;
wire [4:0] init_state;
assign sync = (state == SYNC);
reg [19:0] control;

wire adc_sbc = control[19];
wire cmp     = control[18];
wire bit_isn = control[17];

// special instructions 
reg clc, sec, cld, sed, cli, sei, clv, plp, php, rti, txs;

wire store = control[16];
wire load = control[15];
wire rmw = load & store;

wire [1:0] ctl_src = control[14:13];
wire [1:0] ctl_dst = control[12:11];

wire [8:0] alu_op = control[10:2];
wire shift = alu_op[8];
reg [3:0] cond_code;
reg cond;

/*
 * register file
 */
reg [7:0] regs[31:0];                   // register file

reg [5:0] reg_op;
reg [1:0] reg_idx;
wire [7:0] R = regs[ctl_src];
wire [7:0] XY = regs[reg_idx];

parameter
    SEL_Z = 2'b00,
    SEL_X = 2'b01,
    SEL_Y = 2'b10,
    SEL_A = 2'b11;

initial begin
    regs[SEL_Z] = 0;                    // Z register 
    regs[SEL_X] = 1;                    // X register 
    regs[SEL_Y] = 2;                    // Y register
    regs[SEL_A] = 8'h41;                // A register
end

/*
 * pick index register
 */
always @*
    case( state )
        IDX0: reg_idx = {1'b0, control[0]}; // idx = X or Z
        IDX2: reg_idx = {control[1], 1'b0}; // idx = Y or Z
     default: reg_idx = control[1:0];
    endcase

/*
 * write register file. 
 */
always @(posedge clk)
    if( load & ~rmw & sync )
        regs[ctl_dst] <= alu_out;

/*
 * Data register
 */


always @(posedge clk)
    case( state )
        IMM0: DR <= DI;
        PLA0: DR <= DI;
        DATA: DR <= DI;
        RMW0: DR <= DI;
        ABS0: DR <= DI;
        ABW0: DR <= DI;
        IDX0: DR <= DI;
        IDX1: DR <= DI;
        JMP0: DR <= DI;
        IND0: DR <= DI;
        JSR0: DR <= DI;
        RTS1: DR <= DI;
    endcase

/*
 * ALU
 */
alu alu(
    .C(C),
    .S(S),
    .R(R),
    .DR(DR),
    .DI(DI),
    .alu_out(alu_out),
    .alu_op(alu_op),
    .alu_C(alu_C), 
    .alu_Z(alu_Z), 
    .alu_N(alu_N), 
    .alu_V(alu_V) );

/*
 * stack pointer gets its own register
 */

reg [7:0] S = 8'hff;                   // stack pointer

always @(posedge clk)
    case( state )
        SYNC:   if( txs ) S <= R;
        BRK0:   S <= S - 1;
        BRK1:   S <= S - 1;
        BRK2:   S <= S - 1;
        JSR0:   S <= S - 1;
        JSR1:   S <= S - 1;
        PHA0:   S <= S - 1;
        PLA0:   S <= S + 1;
        RTI0:   S <= S + 1;
        RTS0:   S <= S + 1;
        RTS1:   S <= S + 1;
    endcase

/* 
 * address bus
 */

wire PCL_CO;

reg [9:0] ab_op;

wire [6:0] adh_op = {ab_op[7:3],ab_op[9:8]};

always @*
    case( state )
       ABS0: ab_op = 10'b00_1_01_01_00_0;			//
       ABS1: ab_op = 10'b10_1_00_10_01_0;			//
       ABW0: ab_op = 10'b00_1_01_01_00_0;			//
       ABW1: ab_op = 10'b10_1_00_10_01_0;			//
       BRA0: ab_op = {cond, cond & DI[7], 5'b1_01_01, cond, 2'b0_0};			//
       BRK0: ab_op = 10'b01_1_00_00_00_0;			//
       BRK1: ab_op = 10'b01_1_00_00_00_0;			//
       BRK2: ab_op = 10'b01_1_11_00_00_0;			//
       BRK3: ab_op = 10'b00_1_01_01_00_0;			//
       DATA: ab_op = 10'b00_1_01_01_00_0;			//
       IDX0: ab_op = 10'b00_1_00_00_11_0;			//
       IDX1: ab_op = 10'b10_1_00_11_00_1;			//
       IDX2: ab_op = 10'b10_1_00_10_01_0;			//
       IMM0: ab_op = 10'b00_1_01_01_00_0;			//
       IND0: ab_op = 10'b00_1_00_01_00_0;			//
       IND1: ab_op = 10'b00_1_01_10_00_0;			//
       JMP0: ab_op = 10'b00_1_00_01_00_0;			//
       JMP1: ab_op = 10'b00_1_01_10_00_0;			//
       JSR0: ab_op = 10'b01_1_00_00_00_0;			//
       JSR1: ab_op = 10'b01_1_00_00_00_0;			//
       JSR2: ab_op = 10'b00_1_00_01_00_0;			//
       PHA0: ab_op = 10'b01_1_00_00_00_0;			//
       PLA0: ab_op = 10'b01_1_00_00_00_1;			//
       RMW0: ab_op = 10'b00_0_01_01_00_0;			//
       RMW1: ab_op = 10'b00_1_00_11_00_0;			//
       RTI0: ab_op = 10'b01_1_00_00_00_1;			//
       RTS0: ab_op = 10'b01_1_00_00_00_1;			//
       RTS1: ab_op = 10'b01_1_00_00_00_1;			//
       RTS2: ab_op = {9'b10_1_01_10_00, !rti};		//
       SYNC: ab_op = 10'b00_1_01_01_00_0;			//
       ZPG0: ab_op = 10'b00_1_00_00_11_0;			//
       ZPW0: ab_op = 10'b00_1_00_00_11_0;			//
    endcase

ab ab(
    .clk(clk),
    .RST(RST),
    .ab_op(ab_op),
    .S(S),
    .DI(DI),
    .DR(DR),
    .XY(XY),
    .AB(AB),
    .PC(PC) );

/*
 * write enable
 */

always @*
    case( state )
       ZPG0: WE = store;
       ABS1: WE = store;
       IDX2: WE = store;
       RMW1: WE = 1;
       JSR0: WE = 1;
       JSR1: WE = 1;
       BRK0: WE = 1;
       BRK1: WE = 1;
       BRK2: WE = 1;
       PHA0: WE = 1;
    default: WE = 0;
    endcase

/*
 * data output
 */
always @*
    case( state )
       PHA0: DO = php ? P : alu_out;
       ZPG0: DO = alu_out;
       ABS1: DO = alu_out;
       RMW1: DO = alu_out;
       IDX2: DO = alu_out;
       JSR0: DO = PC[15:8];
       JSR1: DO = PC[7:0];
       BRK0: DO = PC[15:8];
       BRK1: DO = PC[7:0];
       BRK2: DO = P;
    default: DO = 8'h55;
    endcase

/*
 * flags update
 * NV_BDIZC
 */

/*
 * negative flag
 */
always @(posedge clk)
    case( state )
        RTS0: if( rti )                 N <= DI[7];
        SYNC: if( plp )                 N <= DI[7];
              else if( bit_isn )        N <= DR[7];
              else if( load & ~rmw )    N <= alu_N;
              else if( cmp )            N <= alu_N;
              else if( bit_isn )        N <= alu_N;
        RMW1:                           N <= alu_N;
    endcase


/*
 * overflow flag
 */
always @(posedge clk)
    case( state )
        RTS0: if( rti )                 V <= DI[6];
        SYNC: if( plp )                 V <= DI[6];
              else if( clv )            V <= 0;
              else if( bit_isn )        V <= DR[6];
              else if( adc_sbc )        V <= alu_V;
    endcase

/*
 * decimal flag
 */
always @(posedge clk)
    case( state )
        RTS0: if( rti )                 D <= DI[3];
        SYNC: if( plp )                 D <= DI[3];
              else if( cld )            D <= 0;
              else if( sed )            D <= 1;
    endcase

/*
 * interrupt flag 
 */
always @(posedge clk)
    case( state )
        BRK3:                           I <= 1;
        RTS0: if( rti )                 I <= DI[2];
        SYNC: if( plp )                 I <= DI[2]; 
              else if( cli )            I <= 0;
              else if( sei )            I <= 1;
    endcase

/*
 * zero flag 
 */
always @(posedge clk)
    case( state )
        RTS0: if( rti )                 Z <= DI[1];
        SYNC: if( plp )                 Z <= DI[1]; 
              else if( load & ~rmw )    Z <= alu_Z;
              else if( cmp )            Z <= alu_Z;
              else if( bit_isn )        Z <= alu_Z;
        RMW1:                           Z <= alu_Z;
    endcase

/*
 * carry flag
 */
always @(posedge clk)
    case( state )
        RTS0: if( rti )                 C <= DI[0];
        SYNC: if( plp )                 C <= DI[0];
              else if( clc )            C <= 0;
              else if( sec )            C <= 1;
              else if( cmp )            C <= alu_C;
              else if( shift & ~rmw )   C <= alu_C;
              else if( adc_sbc )        C <= alu_C;
        RMW1: if( shift )               C <= alu_C;
    endcase

/*
 * state machine
 */

reg [8:0] DIHOLD = {9'h1ea};

always @(posedge clk)
    case( state )
        PLA0: DIHOLD <= {1'b1, DI};
        PHA0: DIHOLD <= {1'b1, DI};
        RMW1: DIHOLD <= {1'b1, DI};
    default:  DIHOLD <= {1'b0, DI};
    endcase

assign IR = DIHOLD[8] ? DIHOLD[7:0] : DI;

/*
 * condition code
 */
always @(posedge clk) 
    cond_code <= IR[7:4];

always @*
    casez( cond_code )
        4'b000?: cond = ~N;
        4'b001?: cond =  N;
        4'b010?: cond = ~V;
        4'b011?: cond =  V;
        4'b1000: cond =  1;
        4'b1001: cond = ~C;
        4'b101?: cond =  C;
        4'b110?: cond = ~Z;
        4'b111?: cond =  Z;
    endcase

always @(posedge clk)
    if( RST )
        state <= BRK3;
    else case( state )
        SYNC:  state <= init_state;
        IMM0:  state <= SYNC;
        PHA0:  state <= SYNC;
        PLA0:  state <= SYNC;
        ZPG0:  state <= DATA;
        ZPW0:  state <= RMW0;
        DATA:  state <= SYNC;
        RMW0:  state <= RMW1;
        RMW1:  state <= SYNC;
        ABS0:  state <= ABS1;
        ABS1:  state <= DATA;
        ABW0:  state <= ABW1;
        ABW1:  state <= RMW0;
        BRA0:  state <= SYNC;
        JSR0:  state <= JSR1;
        JSR1:  state <= JSR2;
        JSR2:  state <= JMP1;
        RTS0:  state <= RTS1;
        RTS1:  state <= RTS2;
        RTS2:  state <= SYNC;
        JMP0:  state <= JMP1;
        JMP1:  state <= SYNC;
        IDX0:  state <= IDX1;
        IDX1:  state <= IDX2;
        IDX2:  state <= DATA;
        RMW1:  state <= SYNC;
        BRK0:  state <= BRK1;
        BRK1:  state <= BRK2;
        BRK2:  state <= BRK3;
        BRK3:  state <= JMP0;
        RTI0:  state <= RTS0;
        IND0:  state <= IND1;
        IND1:  state <= JMP0;
    endcase

/*
 * decode vector
 */
reg [24:0] decode;

assign init_state = decode[4:0];

/*
 * control vector
 */
always @(posedge clk)
    if( sync ) begin
        control <= decode[24:5];
        clc <= (IR == 8'h18);
        sec <= (IR == 8'h38);
        cld <= (IR == 8'hD8);
        sed <= (IR == 8'hF8);
        cli <= (IR == 8'h58);
        sei <= (IR == 8'h78);
        rti <= (IR == 8'h40);
        clv <= (IR == 8'hB8);
        php <= (IR == 8'h08);
        plp <= (IR == 8'h28);
        txs <= (IR == 8'h9A);
    end
/*
 * decode vector
 */
always @*
    case( IR )
         //                   +=_B  L/S   SRC    DST    ALU      YX  MODE
         8'h6D: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_ADC , IZ, ABS0 }; // ADC ABS
         8'h7D: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_ADC , IX, ABS0 }; // ADC ABS,X
         8'h79: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_ADC , IY, ABS0 }; // ADC ABS,Y
         8'h69: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_ADC , IZ, IMM0 }; // ADC #IMM
         8'h65: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_ADC , IZ, ZPG0 }; // ADC ZP
         8'h72: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_ADC , IZ, IDX0 }; // ADC (ZP)
         8'h61: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_ADC , IX, IDX0 }; // ADC (ZP,X)
         8'h75: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_ADC , IX, ZPG0 }; // ADC ZP,X
         8'h71: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_ADC , IY, IDX0 }; // ADC (ZP),Y

         8'hED: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_SBC , IZ, ABS0 }; // SBC ABS
         8'hFD: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_SBC , IX, ABS0 }; // SBC ABS,X
         8'hF9: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_SBC , IY, ABS0 }; // SBC ABS,Y
         8'hE9: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_SBC , IZ, IMM0 }; // SBC #IMM
         8'hE5: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_SBC , IZ, ZPG0 }; // SBC ZP
         8'hF2: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_SBC , IZ, IDX0 }; // SBC (ZP)
         8'hE1: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_SBC , IX, IDX0 }; // SBC (ZP,X)
         8'hF5: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_SBC , IX, ZPG0 }; // SBC ZP,X
         8'hF1: decode = { 3'b10_0, LDA, SRC_A, DST_A, ALU_SBC , IY, IDX0 }; // SBC (ZP),Y

         8'h2D: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_AND , IZ, ABS0 }; // AND ABS
         8'h3D: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_AND , IX, ABS0 }; // AND ABS,X
         8'h39: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_AND , IY, ABS0 }; // AND ABS,Y
         8'h29: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_AND , IZ, IMM0 }; // AND #IMM
         8'h25: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_AND , IZ, ZPG0 }; // AND ZP
         8'h32: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_AND , IZ, IDX0 }; // AND (ZP)
         8'h21: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_AND , IX, IDX0 }; // AND (ZP,X)
         8'h35: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_AND , IX, ZPG0 }; // AND ZP,X
         8'h31: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_AND , IY, IDX0 }; // AND (ZP),Y

         8'h0D: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ORA , IZ, ABS0 }; // ORA ABS
         8'h1D: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ORA , IX, ABS0 }; // ORA ABS,X
         8'h19: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ORA , IY, ABS0 }; // ORA ABS,Y
         8'h09: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ORA , IZ, IMM0 }; // ORA #IMM
         8'h05: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ORA , IZ, ZPG0 }; // ORA ZP
         8'h12: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ORA , IZ, IDX0 }; // ORA (ZP)
         8'h01: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ORA , IX, IDX0 }; // ORA (ZP,X)
         8'h15: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ORA , IX, ZPG0 }; // ORA ZP,X
         8'h11: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ORA , IY, IDX0 }; // ORA (ZP),Y

         8'hAD: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_LDA , IZ, ABS0 }; // LDA ABS
         8'hBD: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_LDA , IX, ABS0 }; // LDA ABS,X
         8'hB9: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_LDA , IY, ABS0 }; // LDA ABS,Y
         8'hA9: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_LDA , IZ, IMM0 }; // LDA #IMM
         8'hA5: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_LDA , IZ, ZPG0 }; // LDA ZP
         8'hB2: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_LDA , IZ, IDX0 }; // LDA (ZP)
         8'hA1: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_LDA , IX, IDX0 }; // LDA (ZP,X)
         8'hB5: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_LDA , IX, ZPG0 }; // LDA ZP,X
         8'hB1: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_LDA , IY, IDX0 }; // LDA (ZP),Y

         8'hCD: decode = { 3'b01_0, NOP, SRC_A, DST__, ALU_CMP , IZ, ABS0 }; // CMP ABS
         8'hDD: decode = { 3'b01_0, NOP, SRC_A, DST__, ALU_CMP , IX, ABS0 }; // CMP ABS,X
         8'hD9: decode = { 3'b01_0, NOP, SRC_A, DST__, ALU_CMP , IY, ABS0 }; // CMP ABS,Y
         8'hC9: decode = { 3'b01_0, NOP, SRC_A, DST__, ALU_CMP , IZ, IMM0 }; // CMP #IMM
         8'hC5: decode = { 3'b01_0, NOP, SRC_A, DST__, ALU_CMP , IZ, ZPG0 }; // CMP ZP
         8'hD2: decode = { 3'b01_0, NOP, SRC_A, DST__, ALU_CMP , IZ, IDX0 }; // CMP (ZP)
         8'hC1: decode = { 3'b01_0, NOP, SRC_A, DST__, ALU_CMP , IX, IDX0 }; // CMP (ZP,X)
         8'hD5: decode = { 3'b01_0, NOP, SRC_A, DST__, ALU_CMP , IX, ZPG0 }; // CMP ZP,X
         8'hD1: decode = { 3'b01_0, NOP, SRC_A, DST__, ALU_CMP , IY, IDX0 }; // CMP (ZP),Y

         8'h4D: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_EOR , IZ, ABS0 }; // EOR ABS
         8'h5D: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_EOR , IX, ABS0 }; // EOR ABS,X
         8'h59: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_EOR , IY, ABS0 }; // EOR ABS,Y
         8'h49: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_EOR , IZ, IMM0 }; // EOR #IMM
         8'h45: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_EOR , IZ, ZPG0 }; // EOR ZP
         8'h52: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_EOR , IZ, IDX0 }; // EOR (ZP)
         8'h41: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_EOR , IX, IDX0 }; // EOR (ZP,X)
         8'h55: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_EOR , IX, ZPG0 }; // EOR ZP,X
         8'h51: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_EOR , IY, IDX0 }; // EOR (ZP),Y

         8'h8D: decode = { 3'b00_0, STA, SRC_A, DST__, ALU_REG , IZ, ABS0 }; // STA ABS
         8'h9D: decode = { 3'b00_0, STA, SRC_A, DST__, ALU_REG , IX, ABS0 }; // STA ABS,X
         8'h99: decode = { 3'b00_0, STA, SRC_A, DST__, ALU_REG , IY, ABS0 }; // STA ABS,Y
         8'h85: decode = { 3'b00_0, STA, SRC_A, DST__, ALU_REG , IZ, ZPG0 }; // STA ZP
         8'h92: decode = { 3'b00_0, STA, SRC_A, DST__, ALU_REG , IZ, IDX0 }; // STA (ZP)
         8'h81: decode = { 3'b00_0, STA, SRC_A, DST__, ALU_REG , IX, IDX0 }; // STA (ZP,X)
         8'h95: decode = { 3'b00_0, STA, SRC_A, DST__, ALU_REG , IX, ZPG0 }; // STA ZP,X
         8'h91: decode = { 3'b00_0, STA, SRC_A, DST__, ALU_REG , IY, IDX0 }; // STA (ZP),Y

         8'h0A: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ASLA, IZ, SYNC }; // ASL A
         8'h4A: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_LSRA, IZ, SYNC }; // LSR A
         8'h2A: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_ROLA, IZ, SYNC }; // ROL A
         8'h6A: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_RORA, IZ, SYNC }; // ROR A

         8'h0E: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_ASLM, IZ, ABW0 }; // ASL ABS
         8'h1E: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_ASLM, IX, ABW0 }; // ASL ABS,X
         8'h06: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_ASLM, IZ, ZPW0 }; // ASL ZP
         8'h16: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_ASLM, IX, ZPW0 }; // ASL ZP,X

         8'h4E: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_LSRM, IZ, ABW0 }; // LSR ABS
         8'h5E: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_LSRM, IX, ABW0 }; // LSR ABS,X
         8'h46: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_LSRM, IZ, ZPW0 }; // LSR ZP
         8'h56: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_LSRM, IX, ZPW0 }; // LSR ZP,X

         8'h2E: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_ROLM, IZ, ABW0 }; // ROL ABS
         8'h3E: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_ROLM, IX, ABW0 }; // ROL ABS,X
         8'h26: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_ROLM, IZ, ZPW0 }; // ROL ZP
         8'h36: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_ROLM, IX, ZPW0 }; // ROL ZP,X

         8'h6E: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_RORM, IZ, ABW0 }; // ROR ABS
         8'h7E: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_RORM, IX, ABW0 }; // ROR ABS,X
         8'h66: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_RORM, IZ, ZPW0 }; // ROR ZP
         8'h76: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_RORM, IX, ZPW0 }; // ROR ZP,X

         8'h90: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BCC
         8'hB0: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BCS
         8'hF0: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BEQ
         8'h30: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BMI
         8'hD0: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BNE
         8'h10: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BPL
         8'h80: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BRA
         8'h50: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BVC
         8'h70: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BVS

         8'h2C: decode = { 3'b00_1, NOP, SRC_A, DST__, ALU_AND , IZ, ABS0 }; // BIT ABS
         8'h3C: decode = { 3'b00_1, NOP, SRC_A, DST__, ALU_AND , IX, ABS0 }; // BIT ABS,X
         8'h89: decode = { 3'b00_1, NOP, SRC_A, DST__, ALU_AND , IZ, IMM0 }; // BIT #IMM
         8'h24: decode = { 3'b00_1, NOP, SRC_A, DST__, ALU_AND , IZ, ZPG0 }; // BIT ZP
         8'h34: decode = { 3'b00_1, NOP, SRC_A, DST__, ALU_AND , IX, ZPG0 }; // BIT ZP,X

         8'h18: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // CLC
         8'hD8: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // CLD
         8'h58: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // CLI
         8'hB8: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // CLV
         8'h38: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // SEC
         8'hF8: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // SED
         8'h78: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // SEI
         8'hEA: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // NOP

         8'hEC: decode = { 3'b01_0, NOP, SRC_X, DST__, ALU_CMP , IZ, ABS0 }; // CPX ABS
         8'hE0: decode = { 3'b01_0, NOP, SRC_X, DST__, ALU_CMP , IZ, IMM0 }; // CPX #IMM
         8'hE4: decode = { 3'b01_0, NOP, SRC_X, DST__, ALU_CMP , IZ, ZPG0 }; // CPX ZP
         8'hCC: decode = { 3'b01_0, NOP, SRC_Y, DST__, ALU_CMP , IZ, ABS0 }; // CPY ABS
         8'hC0: decode = { 3'b01_0, NOP, SRC_Y, DST__, ALU_CMP , IZ, IMM0 }; // CPY #IMM
         8'hC4: decode = { 3'b01_0, NOP, SRC_Y, DST__, ALU_CMP , IZ, ZPG0 }; // CPY ZP

         8'hCE: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_DECM, IZ, ABW0 }; // DEC ABS
         8'hDE: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_DECM, IX, ABW0 }; // DEC ABS,X
         8'hC6: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_DECM, IZ, ZPW0 }; // DEC ZP
         8'hD6: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_DECM, IX, ZPW0 }; // DEC ZP,X

         8'hEE: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_INCM, IZ, ABW0 }; // INC ABS
         8'hFE: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_INCM, IX, ABW0 }; // INC ABS,X
         8'hE6: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_INCM, IZ, ZPW0 }; // INC ZP
         8'hF6: decode = { 3'b00_0, RMW, SRC__, DST__, ALU_INCM, IX, ZPW0 }; // INC ZP,X

         8'h3A: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_DECA, IZ, SYNC }; // DEA
         8'hCA: decode = { 3'b00_0, LDA, SRC_X, DST_X, ALU_DECA, IZ, SYNC }; // DEX
         8'h88: decode = { 3'b00_0, LDA, SRC_Y, DST_Y, ALU_DECA, IZ, SYNC }; // DEY
         8'h1A: decode = { 3'b00_0, LDA, SRC_A, DST_A, ALU_INCA, IZ, SYNC }; // INA
         8'hE8: decode = { 3'b00_0, LDA, SRC_X, DST_X, ALU_INCA, IZ, SYNC }; // INX
         8'hC8: decode = { 3'b00_0, LDA, SRC_Y, DST_Y, ALU_INCA, IZ, SYNC }; // INY
         8'hAA: decode = { 3'b00_0, LDA, SRC_A, DST_X, ALU_REG , IZ, SYNC }; // TAX
         8'hA8: decode = { 3'b00_0, LDA, SRC_A, DST_Y, ALU_REG , IZ, SYNC }; // TAY
         8'hBA: decode = { 3'b00_0, LDA, SRC__, DST_X, ALU_TSX , IZ, SYNC }; // TSX
         8'h8A: decode = { 3'b00_0, LDA, SRC_X, DST_A, ALU_REG , IZ, SYNC }; // TXA
         8'h9A: decode = { 3'b00_0, NOP, SRC_X, DST__, ALU_REG , IZ, SYNC }; // TXS
         8'h98: decode = { 3'b00_0, LDA, SRC_Y, DST_A, ALU_REG , IZ, SYNC }; // TYA

         8'h00: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, BRK0 }; // BRK
         8'h4C: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, JMP0 }; // JMP ABS
         8'h6C: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, IND0 }; // JMP (IDX)
         8'h7C: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IX, IND0 }; // JMP (IDX,X)
         8'h20: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, JSR0 }; // JSR ABS
         8'h40: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, RTI0 }; // RTI
         8'h60: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, RTS0 }; // RTS

         8'hAE: decode = { 3'b00_0, LDA, SRC__, DST_X, ALU_LDA , IZ, ABS0 }; // LDX ABS
         8'hBE: decode = { 3'b00_0, LDA, SRC__, DST_X, ALU_LDA , IY, ABS0 }; // LDX ABS,Y
         8'hA2: decode = { 3'b00_0, LDA, SRC__, DST_X, ALU_LDA , IZ, IMM0 }; // LDX #IMM
         8'hA6: decode = { 3'b00_0, LDA, SRC__, DST_X, ALU_LDA , IZ, ZPG0 }; // LDX ZP
         8'hB6: decode = { 3'b00_0, LDA, SRC__, DST_X, ALU_LDA , IY, ZPG0 }; // LDX ZP,Y
         8'hAC: decode = { 3'b00_0, LDA, SRC__, DST_Y, ALU_LDA , IZ, ABS0 }; // LDY ABS
         8'hBC: decode = { 3'b00_0, LDA, SRC__, DST_Y, ALU_LDA , IX, ABS0 }; // LDY ABS,X
         8'hA0: decode = { 3'b00_0, LDA, SRC__, DST_Y, ALU_LDA , IZ, IMM0 }; // LDY #IMM
         8'hA4: decode = { 3'b00_0, LDA, SRC__, DST_Y, ALU_LDA , IZ, ZPG0 }; // LDY ZP
         8'hB4: decode = { 3'b00_0, LDA, SRC__, DST_Y, ALU_LDA , IX, ZPG0 }; // LDY ZP,X

         8'h48: decode = { 3'b00_0, NOP, SRC_A, DST__, ALU_REG , IZ, PHA0 }; // PHA
         8'hDA: decode = { 3'b00_0, NOP, SRC_X, DST__, ALU_REG , IZ, PHA0 }; // PHX
         8'h5A: decode = { 3'b00_0, NOP, SRC_Y, DST__, ALU_REG , IZ, PHA0 }; // PHY
         8'h68: decode = { 3'b00_0, LDA, SRC__, DST_A, ALU_PLA , IZ, PLA0 }; // PLA
         8'hFA: decode = { 3'b00_0, LDA, SRC__, DST_X, ALU_PLA , IZ, PLA0 }; // PLX
         8'h7A: decode = { 3'b00_0, LDA, SRC__, DST_Y, ALU_PLA , IZ, PLA0 }; // PLY
         8'h08: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_REG , IZ, PHA0 }; // PHP
         8'h28: decode = { 3'b00_0, NOP, SRC__, DST__, ALU_____, IZ, PLA0 }; // PLP

         8'h8E: decode = { 3'b00_0, STA, SRC_X, DST__, ALU_REG , IZ, ABS0 }; // STX ABS
         8'h86: decode = { 3'b00_0, STA, SRC_X, DST__, ALU_REG , IZ, ZPG0 }; // STX ZP
         8'h96: decode = { 3'b00_0, STA, SRC_X, DST__, ALU_REG , IY, ZPG0 }; // STX ZP,Y
         8'h8C: decode = { 3'b00_0, STA, SRC_Y, DST__, ALU_REG , IZ, ABS0 }; // STY ABS
         8'h84: decode = { 3'b00_0, STA, SRC_Y, DST__, ALU_REG , IZ, ZPG0 }; // STY ZP
         8'h94: decode = { 3'b00_0, STA, SRC_Y, DST__, ALU_REG , IX, ZPG0 }; // STY ZP,X

         8'h9C: decode = { 3'b00_0, STA, SRC_Z, DST__, ALU_REG , IZ, ABS0 }; // STZ ABS
         8'h9E: decode = { 3'b00_0, STA, SRC_Z, DST__, ALU_REG , IX, ABS0 }; // STZ ABS,X
         8'h64: decode = { 3'b00_0, STA, SRC_Z, DST__, ALU_REG , IZ, ZPG0 }; // STZ ZP
         8'h74: decode = { 3'b00_0, STA, SRC_Z, DST__, ALU_REG , IX, ZPG0 }; // STZ ZP,X

         8'h1C: decode = { 25'b00_0_00_00_00_000000000_00, ABS0 }; // TRB ABS
         8'h14: decode = { 25'b00_0_00_00_00_000000000_00, ZPG0 }; // TRB ZP
         8'h0C: decode = { 25'b00_0_00_00_00_000000000_00, ABS0 }; // TSB ABS
         8'h04: decode = { 25'b00_0_00_00_00_000000000_00, ZPG0 }; // TSB ZP
       default: decode = { 25'bxx_x_xx_xx_xx_xxxxxxxxx_xx, SYNC }; 
    endcase
/*
 *****************************************************************************
 * debug section
 *****************************************************************************
 */

`ifdef SIM

reg [39:0] statename;
always @*
    case( state )
        SYNC: statename = "SYNC";
        IMM0: statename = "IMM0";
        PHA0: statename = "PHA0";
        PLA0: statename = "PLA0";
        ZPG0: statename = "ZPG0";
        ZPW0: statename = "ZPW0";
        DATA: statename = "DATA";
        ABS0: statename = "ABS0";
        ABS1: statename = "ABS1";
        ABW0: statename = "ABW0";
        ABW1: statename = "ABW1";
        BRA0: statename = "BRA0";
        IND0: statename = "IND0";
        IND1: statename = "IND1";
        JMP0: statename = "JMP0";
        JMP1: statename = "JMP1";
        JSR0: statename = "JSR0";
        JSR1: statename = "JSR1";
        JSR2: statename = "JSR2";
        RTS0: statename = "RTS0";
        RTS1: statename = "RTS1";
        RTS2: statename = "RTS2";
        IDX0: statename = "IDX0";
        IDX1: statename = "IDX1";
        IDX2: statename = "IDX2";
        RMW0: statename = "RMW0";
        RMW1: statename = "RMW1";
        BRK0: statename = "BRK0";
        BRK1: statename = "BRK1";
        BRK2: statename = "BRK2";
        BRK3: statename = "BRK3";
        RTI0: statename = "RTI0";
    default : statename = "?";
    endcase

reg [7:0] instruction;
reg [23:0] opcode;

always @*
    casez( instruction )
            8'b0000_0000: opcode = "BRK";
            8'b0000_1000: opcode = "PHP";
            8'b0001_0010: opcode = "ORA";
            8'b0011_0010: opcode = "AND";
            8'b0101_0010: opcode = "EOR";
            8'b0111_0010: opcode = "ADC";
            8'b1001_0010: opcode = "STA";
            8'b1011_0010: opcode = "LDA";
            8'b1101_0010: opcode = "CMP";
            8'b1111_0010: opcode = "SBC";
            8'b011?_0100: opcode = "STZ";
            8'b1001_11?0: opcode = "STZ";
            8'b0101_1010: opcode = "PHY";
            8'b1101_1010: opcode = "PHX";
            8'b0111_1010: opcode = "PLY";
            8'b1111_1010: opcode = "PLX";
            8'b000?_??01: opcode = "ORA";
            8'b0001_0000: opcode = "BPL";
            8'b0001_1010: opcode = "INA";
            8'b000?_??10: opcode = "ASL";
            8'b0001_1000: opcode = "CLC";
            8'b0010_0000: opcode = "JSR";
            8'b0010_1000: opcode = "PLP";
            8'b001?_?100: opcode = "BIT";
            8'b1000_1001: opcode = "BIT";
            8'b001?_??01: opcode = "AND";
            8'b0011_0000: opcode = "BMI";
            8'b0011_1010: opcode = "DEA";
            8'b001?_??10: opcode = "ROL";
            8'b0011_1000: opcode = "SEC";
            8'b0100_0000: opcode = "RTI";
            8'b0100_1000: opcode = "PHA";
            8'b010?_??01: opcode = "EOR";
            8'b0101_0000: opcode = "BVC";
            8'b010?_??10: opcode = "LSR";
            8'b0101_1000: opcode = "CLI";
            8'b01??_1100: opcode = "JMP";
            8'b0110_0000: opcode = "RTS";
            8'b0110_1000: opcode = "PLA";
            8'b011?_??01: opcode = "ADC";
            8'b0111_0000: opcode = "BVS";
            8'b011?_??10: opcode = "ROR";
            8'b0111_1000: opcode = "SEI";
            8'b1000_0000: opcode = "BRA";
            8'b1000_1000: opcode = "DEY";
            8'b1000_?100: opcode = "STY";
            8'b1001_0100: opcode = "STY";
            8'b1000_1010: opcode = "TXA";
            8'b1001_0010: opcode = "STA";
            8'b100?_??01: opcode = "STA";
            8'b1001_0000: opcode = "BCC";
            8'b1001_1000: opcode = "TYA";
            8'b1001_1010: opcode = "TXS";
            8'b100?_?110: opcode = "STX";
            8'b1010_0000: opcode = "LDY";
            8'b1010_1000: opcode = "TAY";
            8'b1010_1010: opcode = "TAX";
            8'b101?_??01: opcode = "LDA";
            8'b1011_0000: opcode = "BCS";
            8'b101?_?100: opcode = "LDY";
            8'b1011_1000: opcode = "CLV";
            8'b1011_1010: opcode = "TSX";
            8'b101?_?110: opcode = "LDX";
            8'b1010_0010: opcode = "LDX";
            8'b1100_0000: opcode = "CPY";
            8'b1100_1000: opcode = "INY";
            8'b1100_?100: opcode = "CPY";
            8'b1100_1010: opcode = "DEX";
            8'b110?_??01: opcode = "CMP";
            8'b1101_0000: opcode = "BNE";
            8'b1101_1000: opcode = "CLD";
            8'b110?_?110: opcode = "DEC";
            8'b1110_0000: opcode = "CPX";
            8'b1110_1000: opcode = "INX";
            8'b1110_?100: opcode = "CPX";
            8'b1110_1010: opcode = "NOP";
            8'b111?_??01: opcode = "SBC";
            8'b1111_0000: opcode = "BEQ";
            8'b1111_1000: opcode = "SED";
            8'b111?_?110: opcode = "INC";
            8'b1101_1011: opcode = "STP";
            8'b0000_?100: opcode = "TSB";
            8'b0001_?100: opcode = "TRB";

            default:      opcode = "___";
    endcase

wire [7:0] R_ = RST ? "R" : "-";

integer cycle;

always @( posedge clk )
    if( sync )
        instruction <= IR;

always @( posedge clk )
    cycle <= cycle + 1;

wire [7:0] B_ = B ? "B" : "-";
wire [7:0] C_ = C ? "C" : "-";
wire [7:0] D_ = D ? "D" : "-";
wire [7:0] I_ = I ? "I" : "-";
wire [7:0] N_ = N ? "N" : "-";
wire [7:0] V_ = V ? "V" : "-";
wire [7:0] Z_ = Z ? "Z" : "-";

wire [7:0] X = regs[SEL_X];
wire [7:0] Y = regs[SEL_Y];
wire [7:0] A = regs[SEL_A];

always @( posedge clk ) begin
    if( !debug || cycle < 5000 || cycle[10:0] == 0 )
      $display( "%4d %s %s %s PC:%h AB:%h DI:%h HOLD:%h DO:%h DR:%h IR:%h WE:%d ALU:%h S:%02x A:%h X:%h Y:%h R:%h P:%s%s1%s%s%s%s%s %d", 
                 cycle, R_, opcode, statename, PC, AB, DI, DIHOLD, DO, DR, IR, WE, alu_out, S, A, X, Y, R, N_, V_, B_, D_, I_, Z_, C_, alu_C );
      if( instruction == 8'hdb )
        $finish( );
end
`endif

endmodule
