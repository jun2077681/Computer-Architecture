`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg, branch;
  wire        pcsrc, zero;
  wire        alusrc, regdst, regwrite, jump, jrcontrol;
  wire [3:0]  alucontrol;

  // Instantiate Controller
  controller c(
    .op         (instr[31:26]), //input
		.funct      (instr[5:0]), //input
		.zero       (zero), //input : zero
		.signext    (signext), //output
		.shiftl16   (shiftl16), //output
		.memtoreg   (memtoreg), //output
		.memwrite   (memwrite), //output
		.pcsrc      (pcsrc), //output
		.alusrc     (alusrc), //output
		.regdst     (regdst), //output
		.regwrite   (regwrite), //output
		.jump       (jump), //output
		.jrcontrol  (jrcontrol), //output
		.alucontrol (alucontrol)); //output

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump), //input
	.jrcontrol  (jrcontrol), //input
    .alucontrol (alucontrol), //input
    .zero       (zero), //output : zero
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata));

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump, jrcontrol,
                  output [3:0] alucontrol);

  wire [2:0] aluop;
  wire       branch;
  wire		 branchnot;

  maindec md(
    .op       (op), //input
    .signext  (signext), //output
    .shiftl16 (shiftl16), //output
    .memtoreg (memtoreg), //output
    .memwrite (memwrite), //output
    .branch   (branch), //output
    .alusrc   (alusrc), //output
    .regdst   (regdst), //output
    .regwrite (regwrite), //output
    .jump     (jump), //output
	.branchnot(branchnot), //output
    .aluop    (aluop)); //output

  aludec ad( 
    .funct      (funct), //input
    .aluop      (aluop), //input
    .alucontrol (alucontrol), //output
	.jrcontrol  (jrcontrol)); //output

  assign pcsrc = (branch&(~zero)&branchnot)|(branch&zero&(~branchnot));

endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump,
			   output		branchnot,
               output [2:0] aluop);

  reg [12:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, branchnot, memwrite,
          memtoreg, jump, aluop} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 13'b0011000000011; // Rtype
      6'b100011: controls <= #`mydelay 13'b1010100010000; // LW
      6'b101011: controls <= #`mydelay 13'b1000100100000; // SW
      6'b000100: controls <= #`mydelay 13'b1000010000001; // BEQ
	  6'b000101: controls <= #`mydelay 13'b1000011000001; // BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 13'b1010100000000; // ADDI, ADDIU: only difference is exception
	  6'b001010: controls <= #`mydelay 13'b1010100000110; // SLTI
	  6'b001011: controls <= #`mydelay 13'b1010100000111; // SLTIU
      6'b001101: controls <= #`mydelay 13'b0010100000010; // ORI
      6'b001111: controls <= #`mydelay 13'b0110100000000; // LUI
      6'b000010: controls <= #`mydelay 13'b0000000001000; // J
	  6'b000011: controls <= #`mydelay 13'b0010000001000; // JAL
      default:   controls <= #`mydelay 13'bxxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [2:0] aluop,
              output reg [3:0] alucontrol,
			  output           jrcontrol);

  always @(*)
    case(aluop)
      3'b000: alucontrol <= #`mydelay 4'b0010;  // add
      3'b001: alucontrol <= #`mydelay 4'b0110;  // sub
      3'b010: alucontrol <= #`mydelay 4'b0001;  // or
	  3'b110: alucontrol <= #`mydelay 4'b0111;  // SLTI
	  3'b111: alucontrol <= #`mydelay 4'b1111;  // SLTIU
      default: case(funct)          // RTYPE
          6'b100000,
          6'b100001: alucontrol <= #`mydelay 4'b0010; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 4'b0110; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 4'b0000; // AND
          6'b100101: alucontrol <= #`mydelay 4'b0001; // OR
          6'b101010: alucontrol <= #`mydelay 4'b0111; // SLT
		  6'b101011: alucontrol <= #`mydelay 4'b1111; // SLTU
		  6'b001000: alucontrol <= #`mydelay 4'b1010; // JR
          default:   alucontrol <= #`mydelay 4'bxxxx; // ???
        endcase
    endcase
	
	assign jrcontrol = aluop[0]&aluop[1]&(~aluop[2])&funct[3]&(~(funct[0]|funct[1]|funct[2]|funct[4]|funct[5]));
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pcsrc,
                input         alusrc, regdst,
                input         regwrite, jump, jrcontrol,
                input  [3:0]  alucontrol,
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata);

  wire [4:0]  writeregtemp, writereg;
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch, pcnextjr, pcnexttemp;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] resulttemp, result;
  wire        shift;

  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4)); // pcplus4 = pc + 0b100(4)

  sl2 immsh(
    .a (signimm),
    .y (signimmsh)); //sign immediate shift left 2
				 
  adder pcadd2(
    .a (pcplus4),
    .b (signimmsh),
    .y (pcbranch)); // pcbranch = pcplus4 + signimmsh = (pc + 4) + (imm<<2) : branch target

  mux2 #(32) pcbrmux(
    .d0  (pcplus4), // pc + 4
    .d1  (pcbranch), // branch target
    .s   (pcsrc), //branch & zero
    .y   (pcnextbr)); // branch&zero is 1 then branch target else pc+4(next addr)

  mux2 #(32) pcmux(
    .d0   (pcnexttemp),
    .d1   ({pcplus4[31:28], instr[25:0], 2'b00}), // jump target
    .s    (jump), //jump flag
    .y    (pcnext)); // if jump then d1 else d0

  adder pcaddjr(
    .a (srca),
	.b (32'b0),
	.y (pcnextjr)
  );
  mux2 #(32) pcjrmux(
	.d0   (pcnextbr),
	.d1   (pcnextjr),
	.s    (jrcontrol),
	.y    (pcnexttemp));
	
  // register file logic
  regfile rf(
    .clk     (clk), //positive edge
    .we      (regwrite), // write result flag
    .ra1     (instr[25:21]), //rs
    .ra2     (instr[20:16]), //rt
    .wa      (writereg), // write result to case by case register
    .wd      (result), // data for write
    .rd1     (srca), // read rs
    .rd2     (writedata)); // read rt

  mux2 #(5) wrmux(
    .d0  (instr[20:16]), //rt
    .d1  (instr[15:11]), //rd
    .s   (regdst),
    .y   (writeregtemp));
	
  mux2 #(5) jalwrmux(
	.d0  (writeregtemp),
	.d1  (5'b11111),
	.s   (jump),
	.y   (writereg));

  mux2 #(32) resmux(
    .d0 (aluout),
    .d1 (readdata),
    .s  (memtoreg),
    .y  (resulttemp));
	
  mux2 #(32) jalresmux(
	.d0 (resulttemp),
	.d1 (pcplus4),
	.s	(jump),
	.y	(result)); // if jump flag is 1, result is pc+4 (-> if jal then we=1, write pc+4 else if jump not write) else result is resulttemp(original result)

  sign_zero_ext sze(
    .a       (instr[15:0]), //immediate
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm[31:0]),
    .shiftl16  (shiftl16), //shiftl16 is 1 then shift else a
    .y         (shiftedimm[31:0])); // shiftl16 is 1 then (imm<<16) else sign immediate

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (writedata), //read rt
    .d1 (shiftedimm[31:0]), // shift or not shift immediate
    .s  (alusrc),
    .y  (srcb)); // read rt or imm

  alu alu(
    .a       (srca),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout), // alucont[1:0] == (00 then a&b) (01 then a|b) (10 then alucont[2]==0 then a+b else a-b) (11 then a>b then 0 else then 1)
    .zero    (zero)); // aluout == 0 then 1 else 0
	
endmodule
