// Junhyoung Lee & Sangmin Lee
// 05/05/2023
// EE 469
// Lab #3
/* 
arm is the spotlight of the show and contains the bulk of the datapath and control logic. 
This module is split into two parts, the datapath and control.
Lab 3 is the processor with 5-stage pipelining which has 4 pipeline registers total.
This processor will deal with any hazards that come up
5 stages: Fetch - Decode - Execute - Memory - Writeback
*/

// clk - system clock
// rst - system reset
// Instr - incoming 32 bit instruction from imem, contains opcode, condition, addresses and or immediates
// ReadData - data read out of the dmem
// WriteData - data to be written to the dmem
// MemWrite - write enable to allowed WriteData to overwrite an existing dmem word
// PC - the current program count value, goes to imem to fetch instruciton
// ALUResult - result of the ALU operation, sent as address to the dmem

module arm (input  logic        clk, rst,
				input  logic [31:0] InstrF,
				input  logic [31:0] ReadDataM,
				output logic [31:0] WriteDataM, 
				output logic [31:0] PCF, ALUOutM,
				output logic        MemWriteM);
	 
    // datapath buses and signals
    logic [31:0] PCPrime, PCPrime2, PCPlus4F, PCPlus8D, PCPlus8E; // pc signals
    logic [ 3:0] RA1D, RA2D, RA1E, RA2E;                				// regfile input addresses
    logic [31:0] RD1, RD2, RD1E, RD2E;  	              				// raw regfile outputs
    logic [ 3:0] ALUFlags;   					              				// alu combinational flag outputs
    logic [31:0] ExtImmD, ExtImmE;							  				// extended immediate inputs
	 logic [31:0] SrcAE, SrcBE, SrcAEPrime, SrcBEPrime;  				// alu source inputs 
    logic [31:0] ResultW;           				        				// computed or fetched value to be written into regfile or pc
	 
	 // more data logics for pipeline
	 logic [31:0] InstrD;						// 31-bit Instruction for Decode
	 logic [3:0]  WA3E, WA3M, WA3W;			// 4-bit WA3 must arrive at same time as Result
	 logic [31:0] WriteDataE;					// 31-bit Write Data for Execute
	 logic [31:0] ALUResultE;					// 31-bit ALU Result for Execute
	 logic [31:0] ReadDataW;					// 31-bit Read Data for Writeback
	 logic [31:0] ALUOutW;						// 31-bit ALU Out for Writeback
	 
	 //-------------------------------------------------------------------------------
	 
    // control signals (Decode, Execute, Memory, and Writeback)
    logic PCSrcD, MemtoRegD, ALUSrcD, MemWriteD, RegWriteD, BranchD;
	 logic PCSrcE, MemtoRegE, ALUSrcE, MemWriteE, RegWriteE, BranchE;
	 logic PCSrcM, MemtoRegM, RegWriteM;
	 logic PCSrcW, MemtoRegW, RegWriteW;
	 logic PCSrcEPrime, RegWriteEPrime, MemWriteEPrime;
    logic [1:0] RegSrcD, ImmSrcD, ALUControlD, ALUControlE;
	 
	 // more control logics for pipeline
	 logic ENF, END;								// 1-bit enable signals for stalling
	 logic CLRD, CLRE;							// 1-bit clear signals for flushing
	 logic FlagWriteD, FlagWriteE;			// 1-bit whether to hold flag or not based on CMP
	 logic [3:0] Flags, FlagsE;				// 4-bit logic stores the result of aluflags
	 logic CondExE;								// 1-bit logic determines if branch happens or not
	 logic BranchTakenE;							// 1-bit logic for Control Hazard (branch)
    logic [3:0] CondE;							// 4-bit logic for conditional execution (cmd)
	 logic [1:0] ForwardAE, ForwardBE;		// 2-bit data forwarding control signals
	 logic StallF, StallD, FlushD, FlushE; // 1-bit stalling and flushing control signals
	 logic ldrStallD;								// 1-bit load stalling detection signal
	 logic PCWrPendingF;							// 1-bit logic if write to PC in Decode, Execute, or Memory
	 
	 //-------------------------------------------------------------------------------
	   
    /* The datapath consists of a PC as well as a series of muxes to make decisions about which data words to pass forward and operate on. It is 
    ** noticeably missing the register file and alu, which you will fill in using the modules made in lab 1. To correctly match up signals to the 
    ** ports of the register file and alu take some time to study and understand the logic and flow of the datapath.
    */
    //-------------------------------------------------------------------------------
    //                                      DATAPATH
    //-------------------------------------------------------------------------------
	 
	 // Fetch Stage-------------------------------------------------------------------
	 assign PCPrime2 = PCSrcW ? ResultW : PCPlus4F;				// first mux, use either default or newly computed value
    assign PCPrime = BranchTakenE ? ALUResultE : PCPrime2;	// second mux, choose either ALU Result or temp
    assign PCPlus4F = PCF + 'd4;                  				// default value to access next instruction
	 assign ENF = ~StallF;  											// enable signals for stalling before Fetch
	 assign END = ~StallD;  											// enable signals for stalling before Decode
	 assign CLRD = FlushD;												// clear signals for flushing before Decode
	 
    // update the PC, at rst initialize to 0
    always_ff @(posedge clk) begin
        if 						(rst) PCF <= '0;
		  else if (ENF==0)	PCF <= PCF;
        else     				PCF <= PCPrime;
    end
	 
	 // register between Fetch and Decode
	 always_ff @(posedge clk) begin
		if (rst) begin
			InstrD 	<= 32'd0;
			PCPlus8D <= 32'd0;
		end
		
		else if (END==0) begin		// stalling
			InstrD 	<= InstrD;
			PCPlus8D <= PCPlus8D;
		end
		
		else if (CLRD) begin			// flushing
			InstrD 	<= 32'd0;
			PCPlus8D <= 32'd0;
		end
		
		else begin
			InstrD 	<= InstrF;
			PCPlus8D <= PCPlus4F + 32'd4;
		end
	 end
	 
	 //--------------------------------------------------------------------------------
	 
	 // Decode Stage-------------------------------------------------------------------
	 // determine the register addresses based on control signals
    // RegSrc[0] is set if doing a branch instruction
    // RefSrc[1] is set when doing memory instructions
    assign RA1D = RegSrcD[0] ? 4'd15         : InstrD[19:16];
    assign RA2D = RegSrcD[1] ? InstrD[15:12] : InstrD[ 3: 0];

    // reg_file module works as the 16x32 register file with two read port, one write port, and asynchronous
	 // Input: clock clk (1-bit), write enable wr_en (1-bit), write address write_addr (4-bit)
	 //		  write_data (32-bit), read_addr1 and read_addr2 (4-bit)
	 // Output: read_data1 and read_data2 (32-bit)
	 reg_file u_reg_file (
        .clk       (clk), 
        .wr_en     (RegWriteW),
        .write_data(ResultW),
        .write_addr(WA3W),
        .read_addr1(RA1D), 
        .read_addr2(RA2D),
        .read_data1(RD1), 
        .read_data2(RD2)
    );
	 
    // two muxes, put together into an always_comb for clarity
    // determines which set of instruction bits are used for the immediate
    always_comb begin
        if      (ImmSrcD == 'b00) ExtImmD = {{24{InstrD[7]}},InstrD[7:0]};          // 8 bit immediate - reg operations
        else if (ImmSrcD == 'b01) ExtImmD = {20'b0, InstrD[11:0]};                  // 12 bit immediate - mem operations
        else                      ExtImmD = {{6{InstrD[23]}}, InstrD[23:0], 2'b00}; // 24 bit immediate - branch operation
    end
	 
	 assign CLRE = FlushE;						// clear signals for flushing before Execute
	 
	 // register between Decode and Execute
	 always_ff @(posedge clk) begin
		if (CLRE==1 || rst==1) begin			// reset or flushing happen
			// datapath signals
			RD1E 			<= 32'd0;
			RD2E 			<= 32'd0;
			WA3E 			<= 4'd0;
			ExtImmE 		<= 32'd0;
			PCPlus8E 	<= 32'd0;
			RA1E 			<= 4'd0;
			RA2E 			<= 4'd0;
			// control signals
			PCSrcE 		<= 0;
			RegWriteE 	<= 0;
			MemtoRegE 	<= 0;
			MemWriteE 	<= 0;
			ALUControlE <= 2'd0;
			BranchE 		<= 0;
			ALUSrcE 		<= 0;
			FlagWriteE  <= 0;
			CondE 		<= 4'd0;
			FlagsE 		<= 4'd0;
		end
		
		else begin
			// datapath signals
			RD1E 			<= RD1;
			RD2E 			<= RD2;
			WA3E 			<= InstrD[15:12];
			ExtImmE 		<= ExtImmD;
			PCPlus8E 	<= PCPlus8D;
			RA1E 			<= RA1D;
			RA2E 			<= RA2D;
			// control signals
			PCSrcE 		<= PCSrcD;
			RegWriteE 	<= RegWriteD;
			MemtoRegE 	<= MemtoRegD;
			MemWriteE 	<= MemWriteD;
			ALUControlE <= ALUControlD;
			BranchE 		<= BranchD;
			ALUSrcE 		<= ALUSrcD;
			FlagWriteE 	<= FlagWriteD;
			CondE 		<= InstrD[31:28];
			FlagsE 		<= Flags;
		end
	 end
	 
	 //--------------------------------------------------------------------------------
	 
	 // Execute Stage-------------------------------------------------------------------
	 // logic for SrcAE and SrcBE before alu
	 assign SrcAEPrime = (RA1E == 'd15) ? PCPlus8E : RD1E;		// substitute the 15th regfile register for PC
	 assign SrcBEPrime = (RA2E == 'd15) ? PCPlus8E : RD2E;		// substitute the 15th regfile register for PC
	 
	 // Hazard Unit selects forwarding signal
	 always_comb begin
		if 	  (ForwardAE == 2'b00) SrcAE = SrcAEPrime;
		else if (ForwardAE == 2'b01) SrcAE = ResultW;
		else 	  							  SrcAE = ALUOutM;
	 end
	 
	 always_comb begin
		if 	  (ForwardBE == 2'b00) WriteDataE = SrcBEPrime;
		else if (ForwardBE == 2'b01) WriteDataE = ResultW;
		else 	  							  WriteDataE = ALUOutM;
	 end
	 
	 assign SrcBE = ALUSrcE ? ExtImmE : WriteDataE;					// mux, for SrcBE either from reg file or from immediate
	 
	 // alu module implement the ALU system with ALU control input and shows the ALU Flags
	 // Input: input a and b (32-bit), ALU Control (2-bit)
	 // Output: Result after the operations (32-bit), ALU Flags (4-bit)
    alu u_alu (
        .a          (SrcAE), 
        .b          (SrcBE),
        .ALUControl (ALUControlE),
        .Result     (ALUResultE),
        .ALUFlags   (ALUFlags)
    );
	 
	 // control signal mux before register between Execute and Memory
	 assign PCSrcEPrime = PCSrcE & CondExE;
	 assign RegWriteEPrime = RegWriteE & CondExE;
	 assign MemWriteEPrime = MemWriteE & CondExE;
	 assign BranchTakenE = BranchE & CondExE;
	 
	 // register between Execute and Memory
	 always_ff @(posedge clk) begin
		if (rst) begin
			// datapath signals
			ALUOutM 	  <= 32'd0;
			WriteDataM <= 32'd0;
			WA3M 		  <= 4'd0;
			// control signals
			PCSrcM 	  <= 0;
			RegWriteM  <= 0;
			MemtoRegM  <= 0;
			MemWriteM  <= 0;
		end
		
		else begin
			// datapath signals
			ALUOutM 	  <= ALUResultE;
			WriteDataM <= WriteDataE;
			WA3M 		  <= WA3E;
			// control signals
			PCSrcM 	  <= PCSrcEPrime;
			RegWriteM  <= RegWriteEPrime;
			MemtoRegM  <= MemtoRegE;
			MemWriteM  <= MemWriteEPrime;
		end
	 end
	 
	 //--------------------------------------------------------------------------------
	 
	 // Memory Stage-------------------------------------------------------------------
	 // register between Memory and Writeback
	 always_ff @(posedge clk) begin
		if (rst) begin
			// datapath signals
			ALUOutW 	 <= 32'd0;
			WA3W 		 <= 4'd0;
			ReadDataW <= 32'd0;
			// control signals
			PCSrcW 	 <= 0;
			RegWriteW <= 0;
			MemtoRegW <= 0;
		end
		
		else begin
			// datapath signals
			ALUOutW 	 <= ALUOutM;
			WA3W 		 <= WA3M;
			ReadDataW <= ReadDataM;
			// control signals
			PCSrcW 	 <= PCSrcM;
			RegWriteW <= RegWriteM;
			MemtoRegW <= MemtoRegM;
		end
	 end
	 
	 //--------------------------------------------------------------------------------
	 
	 // Writeback Stage----------------------------------------------------------------
	 // determine the result to run back to PC or the register file based on whether we used a memory instruction
    assign ResultW = MemtoRegW ? ReadDataW : ALUOutW;    // determine whether final writeback result is from dmemory or alu
	 
	 //--------------------------------------------------------------------------------

    /* The control conists of a large decoder, which evaluates the top bits of the instruction and produces the control bits 
    ** which become the select bits and write enables of the system. The write enables (RegWrite, MemWrite and PCSrc) are 
    ** especially important because they are representative of your processors current state. 
    */
    //-------------------------------------------------------------------------------
    //                                      CONTROL
    //-------------------------------------------------------------------------------
    
	 // Decode Stage------------------------------------------------------------------
	 
	 
	 always_comb begin
	
		casez (InstrD[27:20])

			// ADD (Imm or Reg)
         8'b00?_0100_0 : begin			// note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we add
				PCSrcD    	= 0;
            MemtoRegD 	= 0; 
            MemWriteD 	= 0; 
            ALUSrcD   	= InstrD[25]; 	// may use immediate
            RegWriteD 	= 1;
            RegSrcD   	= 'b00;
            ImmSrcD   	= 'b00; 
            ALUControlD = 'b00;
				FlagWriteD 	= 0;
				BranchD 		= 0;
			end
 
			// SUB (Imm or Reg) or SUBS (CMP)
			8'b00?_0010_? : begin			// note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we add
				PCSrcD    	= 0;
            MemtoRegD 	= 0; 
            MemWriteD 	= 0; 
            ALUSrcD   	= InstrD[25]; 	// may use immediate
            RegWriteD 	= 1;
            RegSrcD   	= 'b00;
            ImmSrcD   	= 'b00; 
            ALUControlD = 'b01;
				FlagWriteD 	= InstrD[20];	// condition flag (CMP) if bit 20 == 1, execute CMP
				BranchD 		= 0;
			end

			// AND
			8'b000_0000_0 : begin
				PCSrcD    	= 0;
            MemtoRegD 	= 0; 
            MemWriteD 	= 0; 
            ALUSrcD   	= 0;
            RegWriteD 	= 1;
            RegSrcD   	= 'b00;
            ImmSrcD   	= 'b00;			// doesn't matter
            ALUControlD = 'b10;
				FlagWriteD 	= 0;
				BranchD 		= 0;  
			end

			// ORR
			8'b000_1100_0 : begin
				PCSrcD    	= 0;
            MemtoRegD 	= 0; 
            MemWriteD 	= 0; 
            ALUSrcD   	= 0;
            RegWriteD 	= 1;
            RegSrcD   	= 'b00;
            ImmSrcD   	= 'b00;			// doesn't matter
            ALUControlD = 'b11;
				FlagWriteD 	= 0;
				BranchD 		= 0;
			end

			// LDR
			8'b010_1100_1 : begin
				PCSrcD    	= 0;
            MemtoRegD 	= 1; 
            MemWriteD 	= 0; 
            ALUSrcD   	= 1;
            RegWriteD 	= 1;
            RegSrcD   	= 'b10;			// msb doesn't matter
            ImmSrcD   	= 'b01;
            ALUControlD = 'b00;			// do an add
				FlagWriteD 	= 0;
				BranchD 		= 0;
			end

			// STR
			8'b010_1100_0 : begin
				PCSrcD    	= 0;
            MemtoRegD 	= 0;				// doesn't matter
            MemWriteD 	= 1; 
            ALUSrcD   	= 1;
            RegWriteD 	= 0;
            RegSrcD   	= 'b10;			// msb doesn't matter
            ImmSrcD   	= 'b01;
            ALUControlD = 'b00;			// do an add
				FlagWriteD 	= 0;
				BranchD 		= 0;
			end

			// B
			8'b1010_???? : begin
				PCSrcD    	= 1;
            MemtoRegD 	= 0;
            MemWriteD 	= 0; 
            ALUSrcD   	= 1;
            RegWriteD 	= 0;
            RegSrcD   	= 'b01;
            ImmSrcD   	= 'b10;
            ALUControlD = 'b00;			// do an add
				FlagWriteD 	= 0;
				BranchD 		= 1;
			end

			default: begin
				PCSrcD    	= 0;
            MemtoRegD 	= 0;				// doesn't matter
            MemWriteD 	= 0; 
            ALUSrcD   	= 0;
            RegWriteD 	= 0;
            RegSrcD   	= 'b00;
            ImmSrcD   	= 'b00;
            ALUControlD = 'b00;			// do an add
				FlagWriteD 	= 0;
				BranchD 		= 0;
			end
			
		endcase
		  
	end
	
	// flag reg keeps the aluflags
	assign Flags = FlagWriteE ? ALUFlags : 4'd0;
	
	// Cond Unit
	// CondExE is 1 unless it doesn't take branch
	always_comb begin
		casez (CondE)
			// unconditional
			4'b1110 : begin
				CondExE = 1;
			end
			
			// equal
			4'b0000 : begin
				if (FlagsE[2]==1) begin
					CondExE = 1;
				end
				else begin
					CondExE = 0;
				end
			end
			
			// not equal
			4'b0001 : begin
				if (FlagsE[2]==0) begin
					CondExE = 1;
				end
				else begin
					CondExE = 0;
				end
			end
			
			// greater or equal
			4'b1010 : begin
				if (FlagsE[2]==1 || FlagsE[3]==0) begin
					CondExE = 1;
				end
				else begin
					CondExE = 0;
				end
			end
			
			// greater
			4'b1100 : begin
				if (FlagsE[3]==0) begin
					CondExE = 1;
				end
				else begin
					CondExE = 0;
				end
			end
			
			// less or equal
			4'b1101 : begin
				if (FlagsE[2]==1 || FlagsE[3]==1) begin
					CondExE = 1;
				end
				else begin
					CondExE = 0;
				end
			end
			
			// less
			4'b1011 : begin
				if (FlagsE[3]==1) begin
					CondExE = 1;
				end
				else begin
					CondExE = 0;
				end
			end
			
			// no branch
			default : begin
				CondExE = 1;
			end
		endcase
	end
	
	// Hazard Unit - Data Forwarding Logic
	// for ForwardAE
	always_comb begin
		if 		((RA1E==WA3M) && (RegWriteM==1))			// Match_1E_M AND RegWriteM
			ForwardAE = 2'b10;
		else if	((RA1E==WA3W) && (RegWriteW==1))			// Match_1E_W AND RegWriteW
			ForwardAE = 2'b01;
		else
			ForwardAE = 2'b00;
	end
	
	// for ForwardBE
	always_comb begin
		if 		((RA2E==WA3M) && (RegWriteM==1))			// Match_2E_M AND RegWriteM
			ForwardBE = 2'b10;
		else if	((RA2E==WA3W) && (RegWriteW==1))			// Match_2E_W AND RegWriteW
			ForwardBE = 2'b01;
		else
			ForwardBE = 2'b00;
	end
	
	// Hazard Unit - Stalling Logic
	always_comb begin
		if (rst) begin
			StallF = 0;
			StallD = 0;
			FlushD = 0;
			FlushE = 0;
			ldrStallD = 0;
			PCWrPendingF = 0;
		end
		
		else begin
			PCWrPendingF = PCSrcD + PCSrcE + PCSrcM;
			ldrStallD = ((RA1D==WA3E) || (RA2D==WA3E)) && (MemtoRegE==1);		// Match_12D_E AND MemtoRegE
			StallF = ldrStallD + PCWrPendingF; 
			FlushD = PCWrPendingF + PCSrcW + BranchTakenE;
			FlushE = ldrStallD + BranchTakenE;
			StallD = ldrStallD;
		end
	end

endmodule	
