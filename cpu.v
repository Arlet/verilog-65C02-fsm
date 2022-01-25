/*
 * verilog model of 65C02 CPU.
 *
 * (C) Arlet Ottens, <arlet@c-scape.nl>
 *
 */

module cpu( 
    input clk,                          // CPU clock
    input RST,                          // RST signal
    output [15:0] AD,                   // address bus (combinatorial) 
    output sync,                        // start of new instruction
    input [7:0] DI,                     // data bus input
    output reg [7:0] DO,                // data bus output 
    output reg WE,                      // write enable
    input IRQ,                          // interrupt request
    input NMI,                          // non-maskable interrupt request
    input RDY,                          // Ready signal. Pauses CPU when RDY=0
    input debug );                      // debug for simulation

reg [15:0] PC = 16'hf800;               // program counter high
wire [7:0] PCH = PC[15:8];
wire [7:0] PCL = PC[7:0];

reg [8:0] ADL;                          // ADL[8] is carry for ADH 
reg [7:0] ADH;
assign AD = { ADH, ADL[7:0] };

reg [15:0] ADR;                         // registered address
reg [7:0] DR;                           // data register, registered DI 
wire [7:0] IR;                          // instruction register

wire B = 1;
reg N, V, D, I, Z, C;                   // processor status flags 
wire [7:0] P = { N, V, 1'b1, B, D, I, Z, C };

/*
 * state
 */

parameter
    SYNC  = 5'd0,
    IMMI  = 5'd1,
    PHA0  = 5'd2,
    PLA0  = 5'd3,
    ZPG0  = 5'd4,
    DATA  = 5'd5,
    ABS0  = 5'd6,
    ABS1  = 5'd7,
    BRA0  = 5'd8,
    JSR0  = 5'd9,
    JSR1  = 5'd10,
    JSR2  = 5'd11,
    RTS0  = 5'd12,
    RTS1  = 5'd13,
    RTS2  = 5'd14,
    JMP0  = 5'd15,
    JMP1  = 5'd16,
    IDX0  = 5'd17,
    IDX1  = 5'd18,
    IDX2  = 5'd19,
    BRK0  = 5'd20,
    BRK1  = 5'd21,
    BRK2  = 5'd22,
    BRK3  = 5'd23,
    RTI0  = 5'd24,
    IND0  = 5'd25,
    IND1  = 5'd26,
    ZPW0  = 5'd27,
    ABW0  = 5'd28,
    ABW1  = 5'd29,
    RMW0  = 5'd30,
    RMW1  = 5'd31;

/*
 * control bits
 */

reg [4:0] state;
assign sync = (state == SYNC);
reg [29:0] control;
wire adc_sbc = control[28];
wire shift = control[27];
wire right = control[26];
wire cmp = control[25];
wire cli_sei = control[24];
wire cld_sed = control[23];
wire clc_sec = control[22];
wire bit_isn = control[21];
wire clv = control[20];
wire php = control[19];
wire plp = control[18];
wire rti = control[17];
wire txs = control[16];
wire rmw = control[15];
wire sta = control[14];
wire ld = control[13];
wire [1:0] ctl_src = control[12:11];
wire [1:0] ctl_dst = control[10:9];
wire [6:0] alu_op = control[8:2];
reg set;
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
    if( ld & sync )
        regs[ctl_dst] <= alu_out;

/*
 * Data register
 */


always @(posedge clk)
    case( state )
        IMMI: DR <= DI;
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

reg [7:0] alu_ai;
reg [7:0] alu_bi;
reg alu_ci;
reg alu_si;
reg [7:0] alu_out;

reg alu_C;
wire alu_Z = !alu_out;
wire alu_N = alu_out[7];
wire alu_V = alu_ai[7] ^  alu_bi[7] ^ alu_C ^ alu_N; 

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

always @*
    case( state )
           BRK0: ADL = S;
           BRK1: ADL = S;
           BRK2: ADL = S;
           BRK3: ADL = PCL;
           JSR0: ADL = S;
           JSR1: ADL = S;
           JSR2: ADL = PCL;
           ZPG0: ADL = DI + XY;
           ZPW0: ADL = DI + XY;
           IDX0: ADL = DI + XY;
           IDX1: ADL = ADR + 1;
           IDX2: ADL = DR + XY;
           DATA: ADL = PCL;
           ABS0: ADL = PCL;
           ABS1: ADL = DR + XY;
           ABW0: ADL = PCL;
           ABW1: ADL = DR + XY;
           JMP0: ADL = PCL;
           JMP1: ADL = DR;
           IMMI: ADL = PCL;
           SYNC: ADL = PCL; 
           RMW0: ADL = PCL;
           RMW1: ADL = ADR;
           PHA0: ADL = S;
           PLA0: ADL = S+1;
           RTI0: ADL = S+1;
           RTS0: ADL = S+1;
           RTS1: ADL = S+1;
           RTS2: ADL = DR + !rti;
           BRA0: if( !cond )      ADL = PCL;
                 else if( DI[7] ) ADL = PCL + DI;
                 else             ADL = PCL + DI;
           IND0: ADL = PCL;
           IND1: ADL = DR;
    endcase

always @*
    case( state )
           BRK0: ADH = 8'h01;
           BRK1: ADH = 8'h01;
           BRK2: ADH = 8'h01;
           BRK3: ADH = PCH;
           JSR0: ADH = 8'h01;
           JSR1: ADH = 8'h01;
           JSR2: ADH = PCH;
           ZPG0: ADH = 8'h00;
           ZPW0: ADH = 8'h00;
           IDX0: ADH = 8'h00;
           IDX1: ADH = ADR[15:8] + ADL[8]; 
           IDX2: ADH = DI + ADL[8];
           DATA: ADH = PCH;
           ABS0: ADH = PCH;
           ABS1: ADH = DI + ADL[8];
           ABW0: ADH = PCH;
           ABW1: ADH = DI + ADL[8];
           JMP0: ADH = PCH;
           JMP1: ADH = DI;
           IMMI: ADH = PCH;
           SYNC: ADH = PCH; 
           RMW0: ADH = PCH;
           RMW1: ADH = ADR[15:8];
           PHA0: ADH = 8'h01;
           PLA0: ADH = 8'h01;
           RTI0: ADH = 8'h01;
           RTS0: ADH = 8'h01;
           RTS1: ADH = 8'h01;
           RTS2: ADH = DI + ADL[8];
           BRA0: if( !cond )      ADH = PCH;
                 else if( DI[7] ) ADH = PCH + 8'hff + ADL[8];
                 else             ADH = PCH + 8'h00 + ADL[8];
           IND0: ADH = PCH;
           IND1: ADH = DI;
    endcase

/* 
 * make copy of current address for read-modify-write
 */
always @(posedge clk)
    if( state != RMW0 )
        ADR <= AD;


always @(posedge clk)
    if( RST )
        PC <= 16'hfffc;
    else case( state )
        SYNC: PC <= AD + 1;
        DATA: PC <= AD + 1;
        RMW0: PC <= AD + 1;
        IMMI: PC <= AD + 1;
        ABS0: PC <= AD + 1;
        ABW0: PC <= AD + 1;
        BRA0: PC <= AD + 1;
        RTS2: PC <= AD + 1;
        JMP1: PC <= AD + 1;
        IND1: PC <= AD + 1;
        BRK2: PC <= 16'hfffe;
        BRK3: PC <= AD + 1;
    endcase

/*
 * write enable
 */

always @*
    case( state )
       ZPG0: WE = sta;
       ABS1: WE = sta;
       IDX2: WE = sta;
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
       JSR0: DO = PCH;
       JSR1: DO = PCL;
       BRK0: DO = PCH;
       BRK1: DO = PCL;
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
              else if( ld )             N <= alu_N;
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
              else if( cld_sed )        D <= set;
    endcase

/*
 * interrupt flag 
 */
always @(posedge clk)
    case( state )
        BRK3:                           I <= 1;
        RTS0: if( rti )                 I <= DI[2];
        SYNC: if( plp )                 I <= DI[2]; 
              else if( cli_sei )        I <= set;
    endcase

/*
 * zero flag 
 */
always @(posedge clk)
    case( state )
        RTS0: if( rti )                 Z <= DI[1];
        SYNC: if( plp )                 Z <= DI[1]; 
              else if( ld )             Z <= alu_Z;
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
              else if( clc_sec )        C <= set;
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
 * flag set bit to distinguish CLC/SEC and friends
 */
always @(posedge clk) 
    set <= IR[5];

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
        SYNC: case( IR )
                  8'h00: state <= BRK0; // BRK
                  8'h01: state <= IDX0; // ORA (ZP,X)
                  8'h04: state <= ZPG0; // TSB ZP
                  8'h05: state <= ZPG0; // ORA ZP
                  8'h06: state <= ZPW0; // ASL ZP
                  8'h08: state <= PHA0; // PHP
                  8'h09: state <= IMMI; // ORA #IMM
                  8'h0A: state <= SYNC; // ASL A
                  8'h0C: state <= ABS0; // TSB ABS
                  8'h0D: state <= ABS0; // ORA ABS
                  8'h0E: state <= ABW0; // ASL ABS
                  8'h10: state <= BRA0; // BPL
                  8'h11: state <= IDX0; // ORA (ZP),Y
                  8'h12: state <= IDX0; // ORA (ZP)
                  8'h14: state <= ZPG0; // TRB ZP
                  8'h15: state <= ZPG0; // ORA ZP,X
                  8'h16: state <= ZPW0; // ASL ZP,X
                  8'h18: state <= SYNC; // CLC
                  8'h19: state <= ABS0; // ORA ABS,Y
                  8'h1A: state <= SYNC; // INC A
                  8'h1C: state <= ABS0; // TRB ABS
                  8'h1D: state <= ABS0; // ORA ABS,X
                  8'h1E: state <= ABW0; // ASL ABS,X
                  8'h20: state <= JSR0; // JSR
                  8'h21: state <= IDX0; // AND (ZP,X)
                  8'h24: state <= ZPG0; // BIT ZP
                  8'h25: state <= ZPG0; // AND ZP
                  8'h26: state <= ZPW0; // ROL ZP
                  8'h28: state <= PLA0; // PLP
                  8'h29: state <= IMMI; // AND #IMM
                  8'h2A: state <= SYNC; // ROL A
                  8'h2C: state <= ABS0; // BIT ABS
                  8'h2D: state <= ABS0; // AND ABS
                  8'h2E: state <= ABW0; // ROL ABS
                  8'h30: state <= BRA0; // BMI
                  8'h31: state <= IDX0; // AND (ZP),Y
                  8'h32: state <= IDX0; // AND (ZP)
                  8'h34: state <= ZPG0; // BIT ZP,X
                  8'h35: state <= ZPG0; // AND ZP,X
                  8'h36: state <= ZPW0; // ROL ZP,X
                  8'h38: state <= SYNC; // SEC
                  8'h39: state <= ABS0; // AND ABS,Y
                  8'h3A: state <= SYNC; // DEC A
                  8'h3C: state <= ABS0; // BIT ABS,X
                  8'h3D: state <= ABS0; // AND ABS,X
                  8'h3E: state <= ABW0; // ROL ABS,X
                  8'h40: state <= RTI0; // RTI
                  8'h41: state <= IDX0; // EOR (ZP,X)
                  8'h45: state <= ZPG0; // EOR ZP
                  8'h46: state <= ZPW0; // LSR ZP
                  8'h48: state <= PHA0; // PHA
                  8'h49: state <= IMMI; // EOR #IMM
                  8'h4A: state <= SYNC; // LSR A
                  8'h4C: state <= JMP0; // JMP
                  8'h4D: state <= ABS0; // EOR ABS
                  8'h4E: state <= ABW0; // LSR ABS
                  8'h50: state <= BRA0; // BVC
                  8'h51: state <= IDX0; // EOR (ZP),Y
                  8'h52: state <= IDX0; // EOR (ZP)
                  8'h55: state <= ZPG0; // EOR ZP,X
                  8'h56: state <= ZPW0; // LSR ZP,X
                  8'h58: state <= SYNC; // CLI
                  8'h59: state <= ABS0; // EOR ABS,Y
                  8'h5A: state <= PHA0; // PHY
                  8'h5D: state <= ABS0; // EOR ABS,X
                  8'h5E: state <= ABW0; // LSR ABS,X
                  8'h60: state <= RTS0; // RTS
                  8'h61: state <= IDX0; // ADC (ZP,X)
                  8'h64: state <= ZPG0; // STZ ZP
                  8'h65: state <= ZPG0; // ADC ZP
                  8'h66: state <= ZPW0; // ROR ZP
                  8'h68: state <= PLA0; // PLA
                  8'h69: state <= IMMI; // ADC #IMM
                  8'h6A: state <= SYNC; // ROR A
                  8'h6C: state <= IND0; // JMP (IDX)
                  8'h6D: state <= ABS0; // ADC ABS
                  8'h6E: state <= ABW0; // ROR ABS
                  8'h70: state <= BRA0; // BVS
                  8'h71: state <= IDX0; // ADC (ZP),Y
                  8'h72: state <= IDX0; // ADC (ZP)
                  8'h74: state <= ZPG0; // STZ ZP,X
                  8'h75: state <= ZPG0; // ADC ZP,X
                  8'h76: state <= ZPW0; // ROR ZP,X
                  8'h78: state <= SYNC; // SEI
                  8'h79: state <= ABS0; // ADC ABS,Y
                  8'h7A: state <= PLA0; // PLY
                  8'h7C: state <= IND0; // JMP (IDX,X)
                  8'h7D: state <= ABS0; // ADC ABS,X
                  8'h7E: state <= ABW0; // ROR ABS,X
                  8'h80: state <= BRA0; // BRA
                  8'h81: state <= IDX0; // STA (ZP,X)
                  8'h84: state <= ZPG0; // STY ZP
                  8'h85: state <= ZPG0; // STA ZP
                  8'h86: state <= ZPG0; // STX ZP
                  8'h88: state <= SYNC; // DEY
                  8'h89: state <= IMMI; // BIT #IMM
                  8'h8A: state <= SYNC; // TXA
                  8'h8C: state <= ABS0; // STY ABS
                  8'h8D: state <= ABS0; // STA ABS
                  8'h8E: state <= ABS0; // STX ABS
                  8'h90: state <= BRA0; // BCC
                  8'h91: state <= IDX0; // STA (ZP),Y
                  8'h92: state <= IDX0; // STA (ZP)
                  8'h94: state <= ZPG0; // STY ZP,X
                  8'h95: state <= ZPG0; // STA ZP,X
                  8'h96: state <= ZPG0; // STX ZP,Y
                  8'h98: state <= SYNC; // TYA
                  8'h99: state <= ABS0; // STA ABS,Y
                  8'h9A: state <= SYNC; // TXS
                  8'h9C: state <= ABS0; // STZ ABS
                  8'h9D: state <= ABS0; // STA ABS,X
                  8'h9E: state <= ABS0; // STZ ABS,X
                  8'hA0: state <= IMMI; // LDY #IMM
                  8'hA1: state <= IDX0; // LDA (ZP,X)
                  8'hA2: state <= IMMI; // LDX #IMM
                  8'hA4: state <= ZPG0; // LDY ZP
                  8'hA5: state <= ZPG0; // LDA ZP
                  8'hA6: state <= ZPG0; // LDX ZP
                  8'hA8: state <= SYNC; // TAY
                  8'hA9: state <= IMMI; // LDA #IMM
                  8'hAA: state <= SYNC; // TAX
                  8'hAC: state <= ABS0; // LDY ABS
                  8'hAD: state <= ABS0; // LDA ABS
                  8'hAE: state <= ABS0; // LDX ABS
                  8'hB0: state <= BRA0; // BCS
                  8'hB1: state <= IDX0; // LDA (ZP),Y
                  8'hB2: state <= IDX0; // LDA (ZP)
                  8'hB4: state <= ZPG0; // LDY ZP,X
                  8'hB5: state <= ZPG0; // LDA ZP,X
                  8'hB6: state <= ZPG0; // LDX ZP,Y
                  8'hB8: state <= SYNC; // CLV
                  8'hB9: state <= ABS0; // LDA ABS,Y
                  8'hBA: state <= SYNC; // TSX
                  8'hBC: state <= ABS0; // LDY ABS,X
                  8'hBD: state <= ABS0; // LDA ABS,X
                  8'hBE: state <= ABS0; // LDX ABS,Y
                  8'hC0: state <= IMMI; // CPY #IMM
                  8'hC1: state <= IDX0; // CMP (ZP,X)
                  8'hC4: state <= ZPG0; // CPY ZP
                  8'hC5: state <= ZPG0; // CMP ZP
                  8'hC6: state <= ZPW0; // DEC ZP
                  8'hC8: state <= SYNC; // INY
                  8'hC9: state <= IMMI; // CMP #IMM
                  8'hCA: state <= SYNC; // DEX
                  8'hCC: state <= ABS0; // CPY ABS
                  8'hCD: state <= ABS0; // CMP ABS
                  8'hCE: state <= ABW0; // DEC ABS
                  8'hD0: state <= BRA0; // BNE
                  8'hD1: state <= IDX0; // CMP (ZP),Y
                  8'hD2: state <= IDX0; // CMP (ZP)
                  8'hD5: state <= ZPG0; // CMP ZP,X
                  8'hD6: state <= ZPW0; // DEC ZP,X
                  8'hD8: state <= SYNC; // CLD
                  8'hD9: state <= ABS0; // CMP ABS,Y
                  8'hDA: state <= PHA0; // PHX
                  8'hDD: state <= ABS0; // CMP ABS,X
                  8'hDE: state <= ABW0; // DEC ABS,X
                  8'hE0: state <= IMMI; // CPX #IMM
                  8'hE1: state <= IDX0; // SBC (ZP,X)
                  8'hE4: state <= ZPG0; // CPX ZP
                  8'hE5: state <= ZPG0; // SBC ZP
                  8'hE6: state <= ZPW0; // INC ZP
                  8'hE8: state <= SYNC; // INX
                  8'hE9: state <= IMMI; // SBC #IMM
                  8'hEA: state <= SYNC; // NOP
                  8'hEC: state <= ABS0; // CPX ABS
                  8'hED: state <= ABS0; // SBC ABS
                  8'hEE: state <= ABW0; // INC ABS
                  8'hF0: state <= BRA0; // BEQ
                  8'hF1: state <= IDX0; // SBC (ZP),Y
                  8'hF2: state <= IDX0; // SBC (ZP)
                  8'hF5: state <= ZPG0; // SBC ZP,X
                  8'hF6: state <= ZPW0; // INC ZP,X
                  8'hF8: state <= SYNC; // SED
                  8'hF9: state <= ABS0; // SBC ABS,Y
                  8'hFA: state <= PLA0; // PLX
                  8'hFD: state <= ABS0; // SBC ABS,X
                  8'hFE: state <= ABW0; // INC ABS,X
               endcase

        IMMI:  state <= SYNC;
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
 * control vector
 */
always @(posedge clk)
    if( sync )
        case( IR )
             //                     +<>=IDC_BVPPIS_WSL_SR_DS ALU  C  IDX
             8'h6D: control <= 29'b1000000_000000_001_11_11_0100110_00; // ADC ABS
             8'h7D: control <= 29'b1000000_000000_001_11_11_0100110_01; // ADC ABS,X
             8'h79: control <= 29'b1000000_000000_001_11_11_0100110_10; // ADC ABS,Y
             8'h69: control <= 29'b1000000_000000_001_11_11_0100110_00; // ADC #IMM
             8'h65: control <= 29'b1000000_000000_001_11_11_0100110_00; // ADC ZP
             8'h72: control <= 29'b1000000_000000_001_11_11_0100110_00; // ADC (ZP)
             8'h61: control <= 29'b1000000_000000_001_11_11_0100110_01; // ADC (ZP,X)
             8'h75: control <= 29'b1000000_000000_001_11_11_0100110_01; // ADC ZP,X
             8'h71: control <= 29'b1000000_000000_001_11_11_0100110_10; // ADC (ZP),Y

             8'hED: control <= 29'b1000000_000000_001_11_11_0101110_00; // SBC ABS
             8'hFD: control <= 29'b1000000_000000_001_11_11_0101110_01; // SBC ABS,X
             8'hF9: control <= 29'b1000000_000000_001_11_11_0101110_10; // SBC ABS,Y
             8'hE9: control <= 29'b1000000_000000_001_11_11_0101110_00; // SBC #IMM
             8'hE5: control <= 29'b1000000_000000_001_11_11_0101110_00; // SBC ZP
             8'hF2: control <= 29'b1000000_000000_001_11_11_0101110_00; // SBC (ZP)
             8'hE1: control <= 29'b1000000_000000_001_11_11_0101110_01; // SBC (ZP,X)
             8'hF5: control <= 29'b1000000_000000_001_11_11_0101110_01; // SBC ZP,X
             8'hF1: control <= 29'b1000000_000000_001_11_11_0101110_10; // SBC (ZP),Y

             8'h2D: control <= 29'b0000000_000000_001_11_11_1010000_00; // AND ABS
             8'h3D: control <= 29'b0000000_000000_001_11_11_1010000_01; // AND ABS,X
             8'h39: control <= 29'b0000000_000000_001_11_11_1010000_10; // AND ABS,Y
             8'h29: control <= 29'b0000000_000000_001_11_11_1010000_00; // AND #IMM
             8'h25: control <= 29'b0000000_000000_001_11_11_1010000_00; // AND ZP
             8'h32: control <= 29'b0000000_000000_001_11_11_1010000_00; // AND (ZP)
             8'h21: control <= 29'b0000000_000000_001_11_11_1010000_01; // AND (ZP,X)
             8'h35: control <= 29'b0000000_000000_001_11_11_1010000_01; // AND ZP,X
             8'h31: control <= 29'b0000000_000000_001_11_11_1010000_10; // AND (ZP),Y

             8'h0D: control <= 29'b0000000_000000_001_11_11_1000000_00; // ORA ABS
             8'h1D: control <= 29'b0000000_000000_001_11_11_1000000_01; // ORA ABS,X
             8'h19: control <= 29'b0000000_000000_001_11_11_1000000_10; // ORA ABS,Y
             8'h09: control <= 29'b0000000_000000_001_11_11_1000000_00; // ORA #IMM
             8'h05: control <= 29'b0000000_000000_001_11_11_1000000_00; // ORA ZP
             8'h12: control <= 29'b0000000_000000_001_11_11_1000000_00; // ORA (ZP)
             8'h01: control <= 29'b0000000_000000_001_11_11_1000000_01; // ORA (ZP,X)
             8'h15: control <= 29'b0000000_000000_001_11_11_1000000_01; // ORA ZP,X
             8'h11: control <= 29'b0000000_000000_001_11_11_1000000_10; // ORA (ZP),Y
 
             8'hAD: control <= 29'b0000000_000000_001_11_11_0010000_00; // LDA ABS
             8'hBD: control <= 29'b0000000_000000_001_11_11_0010000_01; // LDA ABS,X
             8'hB9: control <= 29'b0000000_000000_001_11_11_0010000_10; // LDA ABS,Y
             8'hA9: control <= 29'b0000000_000000_001_11_11_0010000_00; // LDA #IMM
             8'hA5: control <= 29'b0000000_000000_001_11_11_0010000_00; // LDA ZP
             8'hB2: control <= 29'b0000000_000000_001_11_11_0010000_00; // LDA (ZP)
             8'hA1: control <= 29'b0000000_000000_001_11_11_0010000_01; // LDA (ZP,X)
             8'hB5: control <= 29'b0000000_000000_001_11_11_0010000_01; // LDA ZP,X
             8'hB1: control <= 29'b0000000_000000_001_11_11_0010000_10; // LDA (ZP),Y

             8'hCD: control <= 29'b0001000_000000_000_11_11_0101101_00; // CMP ABS
             8'hDD: control <= 29'b0001000_000000_000_11_11_0101101_01; // CMP ABS,X
             8'hD9: control <= 29'b0001000_000000_000_11_11_0101101_10; // CMP ABS,Y
             8'hC9: control <= 29'b0001000_000000_000_11_11_0101101_00; // CMP #IMM
             8'hC5: control <= 29'b0001000_000000_000_11_11_0101101_00; // CMP ZP
             8'hD2: control <= 29'b0001000_000000_000_11_11_0101101_00; // CMP (ZP)
             8'hC1: control <= 29'b0001000_000000_000_11_11_0101101_01; // CMP (ZP,X)
             8'hD5: control <= 29'b0001000_000000_000_11_11_0101101_01; // CMP ZP,X
             8'hD1: control <= 29'b0001000_000000_000_11_11_0101101_10; // CMP (ZP),Y

             8'h4D: control <= 29'b0000000_000000_001_11_11_1100000_00; // EOR ABS
             8'h5D: control <= 29'b0000000_000000_001_11_11_1100000_01; // EOR ABS,X
             8'h59: control <= 29'b0000000_000000_001_11_11_1100000_10; // EOR ABS,Y
             8'h49: control <= 29'b0000000_000000_001_11_11_1100000_00; // EOR #IMM
             8'h45: control <= 29'b0000000_000000_001_11_11_1100000_00; // EOR ZP
             8'h52: control <= 29'b0000000_000000_001_11_11_1100000_00; // EOR (ZP)
             8'h41: control <= 29'b0000000_000000_001_11_11_1100000_01; // EOR (ZP,X)
             8'h55: control <= 29'b0000000_000000_001_11_11_1100000_01; // EOR ZP,X
             8'h51: control <= 29'b0000000_000000_001_11_11_1100000_10; // EOR (ZP),Y

             8'h8D: control <= 29'b0000000_000000_010_11_11_0000000_00; // STA ABS
             8'h9D: control <= 29'b0000000_000000_010_11_11_0000000_01; // STA ABS,X
             8'h99: control <= 29'b0000000_000000_010_11_11_0000000_10; // STA ABS,Y
             8'h85: control <= 29'b0000000_000000_010_11_11_0000000_00; // STA ZP
             8'h92: control <= 29'b0000000_000000_010_11_11_0000000_00; // STA (ZP)
             8'h81: control <= 29'b0000000_000000_010_11_11_0000000_01; // STA (ZP,X)
             8'h95: control <= 29'b0000000_000000_010_11_11_0000000_01; // STA ZP,X
             8'h91: control <= 29'b0000000_000000_010_11_11_0000000_10; // STA (ZP),Y

             8'h0A: control <= 29'b0100000_000000_001_11_11_0000000_00; // ASL A
             8'h0E: control <= 29'b0100000_000000_100_00_00_0010000_00; // ASL ABS
             8'h1E: control <= 29'b0100000_000000_100_00_00_0010000_01; // ASL ABS,X
             8'h06: control <= 29'b0100000_000000_100_00_00_0010000_00; // ASL ZP
             8'h16: control <= 29'b0100000_000000_100_00_00_0010000_01; // ASL ZP,X

             8'h4A: control <= 29'b0110000_000000_001_11_11_0000000_00; // LSR A
             8'h4E: control <= 29'b0110000_000000_100_00_00_0010000_00; // LSR ABS
             8'h5E: control <= 29'b0110000_000000_100_00_00_0010000_01; // LSR ABS,X
             8'h46: control <= 29'b0110000_000000_100_00_00_0010000_00; // LSR ZP
             8'h56: control <= 29'b0110000_000000_100_00_00_0010000_01; // LSR ZP,X

             8'h2A: control <= 29'b0100000_000000_001_11_11_0000011_00; // ROL A
             8'h2E: control <= 29'b0100000_000000_100_00_00_0010011_00; // ROL ABS
             8'h3E: control <= 29'b0100000_000000_100_00_00_0010011_01; // ROL ABS,X
             8'h26: control <= 29'b0100000_000000_100_00_00_0010011_00; // ROL ZP
             8'h36: control <= 29'b0100000_000000_100_00_00_0010011_01; // ROL ZP,X

             8'h6A: control <= 29'b0110000_000000_001_11_11_0000011_00; // ROR A
             8'h6E: control <= 29'b0110000_000000_100_00_00_0010011_00; // ROR ABS
             8'h7E: control <= 29'b0110000_000000_100_00_00_0010011_01; // ROR ABS,X
             8'h66: control <= 29'b0110000_000000_100_00_00_0010011_00; // ROR ZP
             8'h76: control <= 29'b0110000_000000_100_00_00_0010011_01; // ROR ZP,X

             8'h90: control <= 29'b0000000_000000_000_00_00_0000000_00; // BCC
             8'hB0: control <= 29'b0000000_000000_000_00_00_0000000_00; // BCS
             8'hF0: control <= 29'b0000000_000000_000_00_00_0000000_00; // BEQ
             8'h30: control <= 29'b0000000_000000_000_00_00_0000000_00; // BMI
             8'hD0: control <= 29'b0000000_000000_000_00_00_0000000_00; // BNE
             8'h10: control <= 29'b0000000_000000_000_00_00_0000000_00; // BPL
             8'h80: control <= 29'b0000000_000000_000_00_00_0000000_00; // BRA
             8'h00: control <= 29'b0000000_000000_000_00_00_0000000_00; // BRK
             8'h50: control <= 29'b0000000_000000_000_00_00_0000000_00; // BVC
             8'h70: control <= 29'b0000000_000000_000_00_00_0000000_00; // BVS

             8'h2C: control <= 29'b0000000_100000_000_11_11_1010000_00; // BIT ABS
             8'h3C: control <= 29'b0000000_100000_000_11_11_1010000_01; // BIT ABS,X
             8'h89: control <= 29'b0000000_100000_000_11_11_1010000_00; // BIT #IMM
             8'h24: control <= 29'b0000000_100000_000_11_11_1010000_00; // BIT ZP
             8'h34: control <= 29'b0000000_100000_000_11_11_1010000_01; // BIT ZP,X

             8'h18: control <= 29'b0000001_000000_000_00_00_0000000_00; // CLC
             8'hD8: control <= 29'b0000010_000000_000_00_00_0000000_00; // CLD
             8'h58: control <= 29'b0000100_000000_000_00_00_0000000_00; // CLI
             8'hB8: control <= 29'b0000000_010000_000_00_00_0000000_00; // CLV
             8'h38: control <= 29'b0000001_000000_000_00_00_0000000_00; // SEC
             8'hF8: control <= 29'b0000010_000000_000_00_00_0000000_00; // SED
             8'h78: control <= 29'b0000100_000000_000_00_00_0000000_00; // SEI

             8'hEC: control <= 29'b0001000_000000_000_01_01_0101101_00; // CPX ABS
             8'hE0: control <= 29'b0001000_000000_000_01_01_0101101_00; // CPX #IMM
             8'hE4: control <= 29'b0001000_000000_000_01_01_0101101_00; // CPX ZP
             8'hCC: control <= 29'b0001000_000000_000_10_10_0101101_00; // CPY ABS
             8'hC0: control <= 29'b0001000_000000_000_10_10_0101101_00; // CPY #IMM
             8'hC4: control <= 29'b0001000_000000_000_10_10_0101101_00; // CPY ZP

             8'hCE: control <= 29'b0000000_000000_100_00_00_0011000_00; // DEC ABS
             8'hDE: control <= 29'b0000000_000000_100_00_00_0011000_01; // DEC ABS,X
             8'hC6: control <= 29'b0000000_000000_100_00_00_0011000_00; // DEC ZP
             8'hD6: control <= 29'b0000000_000000_100_00_00_0011000_01; // DEC ZP,X

             8'hEE: control <= 29'b0000000_000000_100_00_00_0010001_00; // INC ABS
             8'hFE: control <= 29'b0000000_000000_100_00_00_0010001_01; // INC ABS,X
             8'hE6: control <= 29'b0000000_000000_100_00_00_0010001_00; // INC ZP
             8'hF6: control <= 29'b0000000_000000_100_00_00_0010001_01; // INC ZP,X

             8'h3A: control <= 29'b0000000_000000_001_11_11_0001000_00; // DEA
             8'h1A: control <= 29'b0000000_000000_001_11_11_0000001_00; // INA
             8'hCA: control <= 29'b0000000_000000_001_01_01_0001000_00; // DEX
             8'h88: control <= 29'b0000000_000000_001_10_10_0001000_00; // DEY
             8'hE8: control <= 29'b0000000_000000_001_01_01_0000001_00; // INX
             8'hC8: control <= 29'b0000000_000000_001_10_10_0000001_00; // INY
             8'hAA: control <= 29'b0000000_000000_001_11_01_0000000_00; // TAX
             8'hA8: control <= 29'b0000000_000000_001_11_10_0000000_00; // TAY
             8'hBA: control <= 29'b0000000_000000_001_00_01_1110000_00; // TSX
             8'h8A: control <= 29'b0000000_000000_001_01_11_0000000_00; // TXA
             8'h9A: control <= 29'b0000000_000001_000_01_00_0000000_00; // TXS
             8'h98: control <= 29'b0000000_000000_001_10_11_0000000_00; // TYA
             8'hEA: control <= 29'b0000000_000000_000_00_00_0000000_00; // NOP

             8'h4C: control <= 29'b0000000_000000_000_00_00_0000000_00; // JMP
             8'h6C: control <= 29'b0000000_000000_000_00_00_0000000_00; // JMP (IDX)
             8'h7C: control <= 29'b0000000_000000_000_00_00_0000000_01; // JMP (IDX,X)
             8'h20: control <= 29'b0000000_000000_000_00_00_0000000_00; // JSR
             8'h40: control <= 29'b0000000_000010_000_00_00_0000000_00; // RTI
             8'h60: control <= 29'b0000000_000000_000_00_00_0000000_00; // RTS

             8'hAE: control <= 29'b0000000_000000_001_00_01_0010000_00; // LDX ABS
             8'hBE: control <= 29'b0000000_000000_001_00_01_0010000_10; // LDX ABS,Y
             8'hA2: control <= 29'b0000000_000000_001_00_01_0010000_00; // LDX #IMM
             8'hA6: control <= 29'b0000000_000000_001_00_01_0010000_00; // LDX ZP
             8'hB6: control <= 29'b0000000_000000_001_00_01_0010000_10; // LDX ZP,Y
             8'hAC: control <= 29'b0000000_000000_001_00_10_0010000_00; // LDY ABS
             8'hBC: control <= 29'b0000000_000000_001_00_10_0010000_01; // LDY ABS,X
             8'hA0: control <= 29'b0000000_000000_001_00_10_0010000_00; // LDY #IMM
             8'hA4: control <= 29'b0000000_000000_001_00_10_0010000_00; // LDY ZP
             8'hB4: control <= 29'b0000000_000000_001_00_10_0010000_01; // LDY ZP,X

             8'h48: control <= 29'b0000000_000000_000_11_00_0000000_00; // PHA
             8'h08: control <= 29'b0000000_001000_000_00_00_0000000_00; // PHP
             8'hDA: control <= 29'b0000000_000000_000_01_00_0000000_00; // PHX
             8'h5A: control <= 29'b0000000_000000_000_10_00_0000000_00; // PHY
             8'h68: control <= 29'b0010000_000000_001_00_11_0000000_00; // PLA
             8'h28: control <= 29'b00x0000_000100_000_00_00_0000000_00; // PLP
             8'hFA: control <= 29'b0010000_000000_001_00_01_0000000_00; // PLX
             8'h7A: control <= 29'b0010000_000000_000_00_10_0000000_00; // PLY

             8'h8E: control <= 29'b0000000_000000_010_01_00_0000000_00; // STX ABS
             8'h86: control <= 29'b0000000_000000_010_01_00_0000000_00; // STX ZP
             8'h96: control <= 29'b0000000_000000_010_01_00_0000000_10; // STX ZP,Y
             8'h8C: control <= 29'b0000000_000000_010_10_00_0000000_00; // STY ABS
             8'h84: control <= 29'b0000000_000000_010_10_00_0000000_00; // STY ZP
             8'h94: control <= 29'b0000000_000000_010_10_00_0000000_01; // STY ZP,X

             8'h9C: control <= 29'b0000000_000000_010_00_00_0000000_00; // STZ ABS
             8'h9E: control <= 29'b0000000_000000_010_00_00_0000000_01; // STZ ABS,X
             8'h64: control <= 29'b0000000_000000_010_00_00_0000000_00; // STZ ZP
             8'h74: control <= 29'b0000000_000000_010_00_00_0000000_01; // STZ ZP,X

             8'h1C: control <= 29'b0000000_000000_000_00_00_0000000_00; // TRB ABS
             8'h14: control <= 29'b0000000_000000_000_00_00_0000000_00; // TRB ZP
             8'h0C: control <= 29'b0000000_000000_000_00_00_0000000_00; // TSB ABS
             8'h04: control <= 29'b0000000_000000_000_00_00_0000000_00; // TSB ZP
           default: control <= 29'bxxxxxxx_xxxxxx_xxx_xx_xx_xxxxxxx_xx; 
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
        IMMI: statename = "IMMI";
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
    if( !debug || cycle < 1000 || cycle[10:0] == 0 )
      $display( "%4d %s %s %s PC:%h AD:%h DI:%h HOLD:%h DO:%h DR:%h IR:%h WE:%d ALU:%h S:%02x A:%h X:%h Y:%h R:%h LD:%h P:%s%s1%s%s%s%s%s %d", 
                 cycle, R_, opcode, statename, PC, AD, DI, DIHOLD, DO, DR, IR, WE, alu_out, S, A, X, Y, R, ld, N_, V_, B_, D_, I_, Z_, C_, alu_C );
      if( instruction == 8'hdb )
        $finish( );
end
`endif

endmodule
