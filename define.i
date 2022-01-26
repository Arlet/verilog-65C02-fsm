parameter
    SYNC  = 5'd0,
    IMM0  = 5'd1,
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

parameter
    NOP   = 2'd00,
    LDA   = 2'd01,
    STA   = 2'd10,
    RMW   = 2'd11;

parameter
    DST__ = 2'bxx,
    DST_X = 2'b01,
    DST_Y = 2'b10,
    DST_A = 2'b11;

parameter
    SRC__ = 2'bxx,
    SRC_Z = 2'b00,
    SRC_X = 2'b01,
    SRC_Y = 2'b10,
    SRC_A = 2'b11;

parameter
    IZ = 2'b00,
    IX = 2'b01,
    IY = 2'b10;

parameter //      SR__A__B__C      SR   A     B   C
          // -----------------------------------------
    ALU_____ = 9'bxx_xxx_xx_xx,
    ALU_REG  = 9'b00_000_00_00, //      R  +  0 + 0
    ALU_INCA = 9'b00_000_00_01, //      R  +  0 + 1
    ALU_DECA = 9'b00_000_10_00, //      R  + -1 + 0
    ALU_LDA  = 9'b00_001_00_00, //      M  +  0 + 0
    ALU_INCM = 9'b00_001_00_01, //      M  +  0 + 1
    ALU_DECM = 9'b00_001_10_00, //      M  + -1 + 0
    ALU_ADC  = 9'b00_000_01_10, //      R  +  M + C
    ALU_CMP  = 9'b00_000_11_01, //      R  + ~M + 1
    ALU_SBC  = 9'b00_000_11_10, //      R  + ~M + C
    ALU_ORA  = 9'b00_100_00_00, //     R|M +  0 + 0
    ALU_AND  = 9'b00_101_00_00, //     R&M +  0 + 0
    ALU_EOR  = 9'b00_110_00_00, //     R^M +  0 + 0
    ALU_TSX  = 9'b00_111_00_00, //      S  +  0 + 0
    ALU_PLA  = 9'b01_xxx_xx_xx, //         DI   
    ALU_ASLA = 9'b10_000_00_00, // asl( R  +  0 + 0)
    ALU_ROLA = 9'b10_000_00_11, // rol( R  +  0 + 0)
    ALU_LSRA = 9'b11_000_00_00, // lsr( R  +  0 + 0)
    ALU_RORA = 9'b11_000_00_11, // ror( R  +  0 + 0)
    ALU_ASLM = 9'b10_001_00_00, // asl( M  +  0 + 0)
    ALU_ROLM = 9'b10_001_00_11, // rol( M  +  0 + 0)
    ALU_LSRM = 9'b11_001_00_00, // lsr( M  +  0 + 0)
    ALU_RORM = 9'b11_001_00_11; // ror( M  +  0 + 0)
