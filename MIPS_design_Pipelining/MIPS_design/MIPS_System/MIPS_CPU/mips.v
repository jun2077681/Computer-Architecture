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

  /* all logic in datapath. -Junhyeok Lee-*/
  
  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
	.memwrite	(memwrite),
    .readdata   (memreaddata));

endmodule

module controller(input  [5:0] op, funct,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       alusrc,
                  output       regdst, regwrite,
                  output       jump, jrcontrol, branch,branchnot,
                  output [3:0] alucontrol);

  wire [2:0] aluop;

  maindec md(
    .op       (op),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
	.branchnot(branchnot),
    .aluop    (aluop));

  aludec ad(
    .funct      (funct), //input
    .aluop      (aluop), //input
    .alucontrol (alucontrol), //output
	.jrcontrol  (jrcontrol)); //output

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
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
				output		  memwrite,
                input  [31:0] readdata);

  wire [4:0]  writeregtemp;
  wire [31:0] pcnext, pcnexttemp;
  wire [31:0] resulttemp, result;
  wire        shift;
  wire		  signext, signext_t, zero;
    
  wire 	[31:0] 	pcplus4_if, pcplus4_id, pcplus4_ex, pcplus4_ma, pcplus4_wb;
  wire	[31:0] 	instr_id, instr_ex, instr_ma, instr_wb;
  wire 	[31:0] 	pc_id, pc_ex, pc_ma, pc_wb;
  
  wire 			shiftl16_id, shiftl16_ex, shiftl16_t;
  wire 			memtoreg_id, memtoreg_ex, memtoreg_ma, memtoreg_wb, memtoreg_t;
  wire 			memwrite_id, memwrite_ex, memwrite_ma, memwrite_t;
  wire 			alusrc_id, alusrc_ex, alusrc_t;
  wire 			regdst_id, regdst_ex, regdst_t;
  wire 			regwrite_id, regwrite_ex, regwrite_ma, regwrite_wb, regwrite_t;
  wire 			jump_id, jump_ex, jump_ma, jump_wb, jump_t;
  wire 			jrcontrol_id, jrcontrol_ex, jrcontrol_t;
  wire 			branch_id, branch_ex, branch_t;
  wire 			branchnot_id, branchnot_ex, branchnot_t;
  wire 	[3:0] 	alucontrol_id, alucontrol_ex, alucontrol_t;
  wire	[31:0] 	signimm_id, signimm_ex;
  wire	[31:0]	signimmsh_ex;
  wire	[31:0]	srca_t, srca_id, srca_ex, srca_ex_fwd;
  wire	[31:0]	srcb_ex;
  wire	[31:0]	writedata_t, writedata_id, writedata_ex, writedata_ex_fwd, writedata_ma;
  wire	[31:0]	shiftedimm_ex;
  wire  [31:0]  aluout_ex, aluout_ma, aluout_wb;
  wire	[4:0]	writereg_ex, writereg_ma, writereg_wb;
  wire	[31:0]	pcjtarget_ex;
  wire	[31:0]	readdata_ma, readdata_wb;
  wire	[31:0]	pcnextbr_ex, pcbranch_ex;
  wire			hazard, stall;
  
  wire			forwardA, forwardB;
  wire	[1:0]	forwardAE, forwardBE;
  
  wire			pcsrc;
  
  wire	[14:0]	controller_temp, controller_flush;
  
  assign pcsrc = (branch_ex&(~zero)&branchnot_ex)|(branch_ex&zero&(~branchnot_ex));
  assign pcjtarget_ex = {pcplus4_ex[31:28], instr_ex[25:0], 2'b00};
  
  assign memwrite = memwrite_ma;
  assign aluout = aluout_ma;
  assign writedata = writedata_ma;
  assign readdata_ma = readdata;
  
  assign forwardA = regwrite_wb && (writereg_wb == instr_id[25:21]);
  assign forwardB = regwrite_wb && (writereg_wb == instr_id[20:16]);
  
  /* 
		why I need forwardA, forwardB? -hasu-
		
		pc10	or 2,3,2
		pc14	lui 29, 0
		pc18	addiu 29,29,424
		pc1c	sw 2,0,29
		pc20	lui 9,0
		
		if	|	id	|	ex	|	ma	|	wb
		10
		14		10
		18		14		10
		1c		18		14		10
		20		1c		18		14		10 -> here!
		
		in id, <1c : sw 2,0,29> read register 2, but in wb, <10: or 2,3,2> write register 2.
		so, if writereg_wb == rs_id or writereg_wb == rt_id, we need forwarding.
		
		if writereg_wb == rs_id && regwrite_wb(whether write to register or not) == true
		==> rd1 <= result (if forwardA = 1) / rd2 <= result (if forwardB = 1)
  */	
  
  assign controller_temp = {signext_t, shiftl16_t, memtoreg_t, memwrite_t, alusrc_t, regdst_t, regwrite_t, jump_t, jrcontrol_t, branch_t, branchnot_t, alucontrol_t};		
  assign {signext, shiftl16_id, memtoreg_id, memwrite_id, alusrc_id, regdst_id, regwrite_id, jump_id, jrcontrol_id, branch_id, branchnot_id, alucontrol_id} = controller_flush;
		
  // next PC logic
  flopenr #(32) pcreg( 
    .clk   (clk),
    .reset (reset),
	.en    (stall),
    .d     (pcnext),
    .q     (pc));
  
  adder pcadd1( 
    .a (pc),
    .b (32'b100),
    .y (pcplus4_if)); // pcplus4 = pc + 0b100(4)

  sl2 immsh( 
    .a (signimm_ex),
    .y (signimmsh_ex)); //sign immediate shift left 2
				 
  adder pcadd2( 
    .a (pcplus4_ex),
    .b (signimmsh_ex),
    .y (pcbranch_ex)); // pcbranch = pcplus4 + signimmsh = (pc + 4) + (imm<<2) : branch target

  mux2 #(32) pcbrmux( 
    .d0  (pcplus4_if), // pc + 4
    .d1  (pcbranch_ex), // branch target
    .s   (pcsrc), //branch & zero
    .y   (pcnextbr_ex)); // branch&zero is 1 then branch target else pc+4(next addr)

  mux2 #(32) pcjrmux( 
	.d0   (pcnextbr_ex),
	.d1   (srca_ex_fwd), //jr target
	.s    (jrcontrol_ex),
	.y    (pcnexttemp));	
	
  mux2 #(32) pcmux( 
    .d0   (pcnexttemp),
    .d1   (pcjtarget_ex), // jump target
    .s    (jump_ex), //jump flag
    .y    (pcnext)); // if jump then d1 else d0
	
  // register file logic
  regfile rf(
    .clk     (clk), //positive edge
    .we      (regwrite_wb), // write result flag
    .ra1     (instr_id[25:21]), //rs
    .ra2     (instr_id[20:16]), //rt
    .wa      (writereg_wb), // write result to case by case register
    .wd      (result), // data for write
    .rd1     (srca_t), // read rs
    .rd2     (writedata_t)); // read rt
	
  mux2 #(32) forwardAmux(
	.d0	(srca_t),
	.d1	(result),
	.s	(forwardA),
	.y	(srca_id));
	
  mux2 #(32) forwardBmux(
    .d0 (writedata_t),
	.d1 (result),
	.s  (forwardB),
	.y  (writedata_id));

  hazard_unit hazard_unit(
	.regwrite_ma			(regwrite_ma),
	.regwrite_wb			(regwrite_wb),
	.rs_id					(instr_id[25:21]),
	.rt_id					(instr_id[20:16]),
	.rs_ex					(instr_ex[25:21]),
	.rt_ex					(instr_ex[20:16]),
	.writereg_ma			(writereg_ma),
	.writereg_wb			(writereg_wb),
	.forwardAE				(forwardAE),
	.forwardBE				(forwardBE),
	.memtoreg_ex			(memtoreg_ex),
	.hazard					(hazard),
	.stall					(stall),
	.pcsrc					(pcsrc),
	.jump_ex				(jump_ex),
	.jrcontrol_ex			(jrcontrol_ex));
	
  mux4 #(32) forwardAEmux( 
	.d0  (srca_ex),
	.d1  (aluout_ma),
	.d2  (result),
	.d3  (32'b0),
	.s   (forwardAE),
	.y   (srca_ex_fwd));

  mux4 #(32) forwardBEmux( 
	.d0  (writedata_ex),
	.d1  (aluout_ma),
	.d2  (result),
	.d3  (32'b0),
	.s   (forwardBE),
	.y   (writedata_ex_fwd));
	
  mux2 #(5) wrmux( 
    .d0  (instr_ex[20:16]), //rt
    .d1  (instr_ex[15:11]), //rd
    .s   (regdst_ex),
    .y   (writeregtemp));
	
  mux2 #(5) jalwrmux( 
	.d0  (writeregtemp),
	.d1  (5'b11111),
	.s   (jump_ex),
	.y   (writereg_ex));

  mux2 #(32) resmux(
    .d0 (aluout_wb),
    .d1 (readdata_wb),
    .s  (memtoreg_wb),
    .y  (resulttemp));
	
  mux2 #(32) jalresmux(
	.d0 (resulttemp),
	.d1 (pcplus4_wb),
	.s	(jump_wb),
	.y	(result)); // if jump flag is 1, result is pc+4 (-> if jal then we=1, write pc+4 else if jump not write) else result is resulttemp(original result)

  sign_zero_ext sze( 
    .a       (instr_id[15:0]), //immediate
    .signext (signext),
    .y       (signimm_id[31:0]));

  shift_left_16 sl16( 
    .a         (signimm_ex[31:0]),
    .shiftl16  (shiftl16_ex), //shiftl16 is 1 then shift else a
    .y         (shiftedimm_ex[31:0])); // shiftl16 is 1 then (imm<<16) else sign immediate

  // ALU logic
  mux2 #(32) srcbmux( 
    .d0 (writedata_ex_fwd), //read rt
    .d1 (shiftedimm_ex[31:0]), // shift or not shift immediate
    .s  (alusrc_ex),
    .y  (srcb_ex)); // read rt or imm

  alu alu( 
    .a       (srca_ex_fwd),
    .b       (srcb_ex),
	.alucont (alucontrol_ex),
    .result  (aluout_ex), // alucont[1:0] == (00 then a&b) (01 then a|b) (10 then alucont[2]==0 then a+b else a-b) (11 then a>b then 0 else then 1)
    .zero    (zero)); // aluout == 0 then 1 else 0
	

  // Instantiate Controller
  controller c(
    .op         (instr_id[31:26]), //input
		.funct      (instr_id[5:0]), //input
		.signext    (signext_t), //output
		.shiftl16   (shiftl16_t), //output
		.memtoreg   (memtoreg_t), //output
		.memwrite   (memwrite_t), //output
		.alusrc     (alusrc_t), //output
		.regdst     (regdst_t), //output
		.regwrite   (regwrite_t), //output
		.jump       (jump_t), //output
		.jrcontrol  (jrcontrol_t), //output
		.branch		(branch_t),
		.branchnot	(branchnot_t),
		.alucontrol (alucontrol_t)); //output

		
  //if hazard detected, flush controller.
  mux2 #(15) controller_flushmux(
	.d0 (controller_temp),
	.d1	(15'b0),
	.s	(hazard),
	.y	(controller_flush));

	
		
  iftoid iftoid(
	.clk			(clk),
	.reset			(reset),
	.stall			(stall),
	.pcsrc			(pcsrc),
	.jump_ex		(jump_ex),
	.jrcontrol_ex	(jrcontrol_ex),
	.pcplus4_if		(pcplus4_if),
	.instr_if		(instr),
	.pc_if			(pc),
	.pcplus4_id		(pcplus4_id),
	.instr_id		(instr_id),
	.pc_id			(pc_id));
  
  idtoex idtoex(
	.clk			(clk),
	.reset			(reset),
	.alucontrol_id	(alucontrol_id),
	.memtoreg_id	(memtoreg_id),
	.memwrite_id	(memwrite_id),
	.alusrc_id		(alusrc_id),
	.regdst_id		(regdst_id),
	.regwrite_id	(regwrite_id),
	.signimm_id		(signimm_id),
	.pcplus4_id		(pcplus4_id),
	.srca_id		(srca_id),
	.writedata_id	(writedata_id),
	.branch_id		(branch_id),
	.branchnot_id	(branchnot_id),
	.shiftl16_id	(shiftl16_id),
	.jump_id		(jump_id),
	.pc_id			(pc_id),
	.instr_id		(instr_id),
	.jrcontrol_id	(jrcontrol_id),
	.alucontrol_ex	(alucontrol_ex),
	.memtoreg_ex	(memtoreg_ex),
	.memwrite_ex	(memwrite_ex),
	.alusrc_ex		(alusrc_ex),
	.regdst_ex		(regdst_ex),
	.regwrite_ex	(regwrite_ex),
	.signimm_ex		(signimm_ex),
	.pcplus4_ex		(pcplus4_ex),
	.srca_ex		(srca_ex),
	.writedata_ex	(writedata_ex),
	.branch_ex		(branch_ex),
	.branchnot_ex	(branchnot_ex),
	.shiftl16_ex	(shiftl16_ex),
	.jump_ex		(jump_ex),
	.pc_ex			(pc_ex),
	.instr_ex		(instr_ex),
	.jrcontrol_ex	(jrcontrol_ex));
	
  extoma extoma(
	.clk			(clk),
	.reset			(reset),
	.regwrite_ex	(regwrite_ex),
	.memtoreg_ex	(memtoreg_ex),
	.memwrite_ex	(memwrite_ex),
	.aluout_ex		(aluout_ex),
	.writedata_ex_fwd(writedata_ex_fwd),
	.writereg_ex	(writereg_ex),
	.pcplus4_ex		(pcplus4_ex),
	.jump_ex		(jump_ex),
	.pc_ex			(pc_ex),
	.instr_ex		(instr_ex),
	.regwrite_ma	(regwrite_ma),
	.memtoreg_ma	(memtoreg_ma),
	.memwrite_ma	(memwrite_ma),
	.aluout_ma		(aluout_ma),
	.writedata_ma	(writedata_ma),
	.writereg_ma	(writereg_ma),
	.pcplus4_ma		(pcplus4_ma),
	.jump_ma		(jump_ma),
	.pc_ma			(pc_ma),
	.instr_ma		(instr_ma));
	
  matowb matowb(
	.clk			(clk),
	.reset			(reset),
	.regwrite_ma	(regwrite_ma),
	.memtoreg_ma	(memtoreg_ma),
	.readdata_ma	(readdata_ma),
	.aluout_ma		(aluout_ma),
	.pcplus4_ma		(pcplus4_ma),
	.jump_ma		(jump_ma),
	.pc_ma			(pc_ma),
	.instr_ma		(instr_ma),
	.writereg_ma	(writereg_ma),
	.regwrite_wb	(regwrite_wb),
	.memtoreg_wb	(memtoreg_wb),
	.readdata_wb	(readdata_wb),
	.aluout_wb		(aluout_wb),
	.pcplus4_wb		(pcplus4_wb),
	.jump_wb		(jump_wb),
	.pc_wb			(pc_wb),
	.instr_wb		(instr_wb),
	.writereg_wb	(writereg_wb));

endmodule

module hazard_unit(input				regwrite_ma, regwrite_wb,
				   input		[4:0]	rs_id, rt_id, rs_ex, rt_ex,
				   input		[4:0]	writereg_ma, writereg_wb,
				   output reg	[1:0]	forwardAE, forwardBE,
				   input				memtoreg_ex,
				   output reg			hazard, stall,
				   input				pcsrc, jump_ex, jrcontrol_ex);
				   
  always @ (regwrite_ma, regwrite_wb, writereg_ma, writereg_wb, rs_ex, rt_ex)
	begin
		if (regwrite_ma && (writereg_ma != 0) && (writereg_ma == rs_ex)) forwardAE = 2'b01;
		else if (regwrite_wb && (writereg_wb != 0) && (writereg_wb == rs_ex)) forwardAE = 2'b10;
		else forwardAE = 2'b00;
		
		if (regwrite_ma && (writereg_ma != 0) && (writereg_ma == rt_ex)) forwardBE = 2'b01;
		else if (regwrite_wb && (writereg_wb != 0) && (writereg_wb == rt_ex)) forwardBE = 2'b10;
		else forwardBE = 2'b00;		
	end
	
/*
		Why I need forwardAE, forwardBE? -hasu-
		=> Read After Write forwarding
		
		00	addi 3,0,800
		04	add 3,3,3
		08	add 3,3,3
		0c	lu 2,-1
		10	or 2,3,2
		
		if	|	id	|	ex	|	ma	|	wb
		00
		04		00
		08		04		00
		0c		08		04		00			-> case 1
		10		0c		08		04		00	-> case 2
		
		we already calculate 00 instruction in ex.
		but, we not upload memory, so in 04 instruction's ex line, can't accurate calculate.
		So, we should make forwardAE.
		
		In case 1, 04's ex want 00's result. => aluout_ma
		In case 2, 08's ex want 00's result. => aluout_wb(x) result(o) ==> WHY???
		
		I think in case 2, using aluout_wb.
		But, it happens error.
		
		38	lw 3,0,29
		3c	addi 3,3,10 <- Load-Use
		40	sw 10,0,3
		44	lw 3,0,29
		
		if	|	id	|	ex	|	ma	|	wb
		3c		38		34		30		30
		40		3c		38		34		30
		40		3c		3c		38		34
		44		40		3c		3c		38
		
		in instruction 38 and 3c, addi want reg3 and lw bring reg3.
		lw's load value is in result, so not aluout_wb but result.
		
*/	
	
  always @(memtoreg_ex, rs_id, rt_id, rt_ex, pcsrc, jump_ex, jrcontrol_ex)
    begin
    if (memtoreg_ex && (rt_ex != 0) && ((rt_ex == rs_id) || (rt_ex == rt_id)))
	  begin
	  //hazard = 1'b1;
	  stall = 1'b0;
	  hazard = (~stall) | pcsrc | jump_ex | jrcontrol_ex;
	  end
	else
	  begin
	  //hazard = 1'b0;
	  stall = 1'b1;
	  hazard = (~stall) | pcsrc | jump_ex | jrcontrol_ex;
	  end
	end	
  /*
		why I need hazard, stall? -hasu-
		=> Load-Use Hazard
		
		38	lw 3,0,29
		3c	addi 3,3,10
		40	sw 10,0,3
		44	lw lw 3,0,29
		48	addi 3,3,14
		
		if	|	id	|	ex	|	ma	|	wb
		38
		3c		38
		40		3c(*1)	38
		44		40		3c(*2)	38(*3) ->here!
		48		44		40		3c		38
  
		we read memory in ma, so <38 : lw 3,0,29> read memory in "here!"
		but, addi load register in id, and calculate in ex.
		so, we need stall.
		
		use rt register after then <lw rt, imm, rs> instr, hazard detect.
		use rt => regfile read rs or rt.
		
		so, memtoreg_ex == true (only lw make memtoreg true) and (rs_id == rt_ex) or (rt_id == rt_ex)
		=> hazard = true
		
		I use flopenr for maintain pc and instr, so stall = !hazard because stall use for en.
  */	

	
endmodule