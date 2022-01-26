module disas(
    input [7:0] opcode,
    output reg [23:0] mnemonic );

always @*
    casez( opcode )
            8'b0000_0000: mnemonic = "BRK";
            8'b0000_1000: mnemonic = "PHP";
            8'b0001_0010: mnemonic = "ORA";
            8'b0011_0010: mnemonic = "AND";
            8'b0101_0010: mnemonic = "EOR";
            8'b0111_0010: mnemonic = "ADC";
            8'b1001_0010: mnemonic = "STA";
            8'b1011_0010: mnemonic = "LDA";
            8'b1101_0010: mnemonic = "CMP";
            8'b1111_0010: mnemonic = "SBC";
            8'b011?_0100: mnemonic = "STZ";
            8'b1001_11?0: mnemonic = "STZ";
            8'b0101_1010: mnemonic = "PHY";
            8'b1101_1010: mnemonic = "PHX";
            8'b0111_1010: mnemonic = "PLY";
            8'b1111_1010: mnemonic = "PLX";
            8'b000?_??01: mnemonic = "ORA";
            8'b0001_0000: mnemonic = "BPL";
            8'b0001_1010: mnemonic = "INA";
            8'b000?_??10: mnemonic = "ASL";
            8'b0001_1000: mnemonic = "CLC";
            8'b0010_0000: mnemonic = "JSR";
            8'b0010_1000: mnemonic = "PLP";
            8'b001?_?100: mnemonic = "BIT";
            8'b1000_1001: mnemonic = "BIT";
            8'b001?_??01: mnemonic = "AND";
            8'b0011_0000: mnemonic = "BMI";
            8'b0011_1010: mnemonic = "DEA";
            8'b001?_??10: mnemonic = "ROL";
            8'b0011_1000: mnemonic = "SEC";
            8'b0100_0000: mnemonic = "RTI";
            8'b0100_1000: mnemonic = "PHA";
            8'b010?_??01: mnemonic = "EOR";
            8'b0101_0000: mnemonic = "BVC";
            8'b010?_??10: mnemonic = "LSR";
            8'b0101_1000: mnemonic = "CLI";
            8'b01??_1100: mnemonic = "JMP";
            8'b0110_0000: mnemonic = "RTS";
            8'b0110_1000: mnemonic = "PLA";
            8'b011?_??01: mnemonic = "ADC";
            8'b0111_0000: mnemonic = "BVS";
            8'b011?_??10: mnemonic = "ROR";
            8'b0111_1000: mnemonic = "SEI";
            8'b1000_0000: mnemonic = "BRA";
            8'b1000_1000: mnemonic = "DEY";
            8'b1000_?100: mnemonic = "STY";
            8'b1001_0100: mnemonic = "STY";
            8'b1000_1010: mnemonic = "TXA";
            8'b1001_0010: mnemonic = "STA";
            8'b100?_??01: mnemonic = "STA";
            8'b1001_0000: mnemonic = "BCC";
            8'b1001_1000: mnemonic = "TYA";
            8'b1001_1010: mnemonic = "TXS";
            8'b100?_?110: mnemonic = "STX";
            8'b1010_0000: mnemonic = "LDY";
            8'b1010_1000: mnemonic = "TAY";
            8'b1010_1010: mnemonic = "TAX";
            8'b101?_??01: mnemonic = "LDA";
            8'b1011_0000: mnemonic = "BCS";
            8'b101?_?100: mnemonic = "LDY";
            8'b1011_1000: mnemonic = "CLV";
            8'b1011_1010: mnemonic = "TSX";
            8'b101?_?110: mnemonic = "LDX";
            8'b1010_0010: mnemonic = "LDX";
            8'b1100_0000: mnemonic = "CPY";
            8'b1100_1000: mnemonic = "INY";
            8'b1100_?100: mnemonic = "CPY";
            8'b1100_1010: mnemonic = "DEX";
            8'b110?_??01: mnemonic = "CMP";
            8'b1101_0000: mnemonic = "BNE";
            8'b1101_1000: mnemonic = "CLD";
            8'b110?_?110: mnemonic = "DEC";
            8'b1110_0000: mnemonic = "CPX";
            8'b1110_1000: mnemonic = "INX";
            8'b1110_?100: mnemonic = "CPX";
            8'b1110_1010: mnemonic = "NOP";
            8'b111?_??01: mnemonic = "SBC";
            8'b1111_0000: mnemonic = "BEQ";
            8'b1111_1000: mnemonic = "SED";
            8'b111?_?110: mnemonic = "INC";
            8'b1101_1011: mnemonic = "STP";
            8'b0000_?100: mnemonic = "TSB";
            8'b0001_?100: mnemonic = "TRB";

            default:      mnemonic = "___";
    endcase

endmodule
