// Junhyoung Lee & Sangmin Lee
// 04/21/2023
// EE 469
// Lab #2 Task #2
// This program works as the ALU with several operations

// This module implement the ALU system with ALU control input and shows the ALU Flags
// Input: input a and b (32-bit), ALU Control (2-bit)
// Output: Result after the operations (32-bit), ALU Flags (4-bit)

// ALUControl: 00 (Add), 01 (Subtract), 10 (AND), 11 (OR)
// ALUFlags: 1000 (Negative), 0100 (Zero), 0010 (Carry out), 0001 (Overflow)

module alu (input logic [31:0] a, b,
				input logic [1:0] ALUControl,
				output logic [31:0] Result,
				output logic [3:0] ALUFlags);
	
	// 1-bit carry out logic after the add operation
	logic cout;
	
	// 32-bit temporary result
	logic [31:0] temp;		// temporary for ADD and SUB
	logic [31:0] andtemp;	// temporary for AND
	logic [31:0] ortemp;		// temporary for OR
	
	// 32-bit logic holds the inverse of b for subtraction
	logic [31:0] inverse_b;
	assign inverse_b = ~b;
	
	always_comb begin
			
		// ADD
		if (ALUControl[0] == 1'b0) begin
			{cout, temp} = a + b + 1'b0;
		end
		// SUB
		else begin
			{cout, temp} = a + inverse_b + 1'b1;
		end
	
	end
	
	always_comb begin
		//AND
		andtemp = a & b;
	end

	always_comb begin
		//OR
		ortemp = a | b;
	end
	
	// decide the results with temp, andtemp, ortemp using MUX
	always_comb begin
			  
		case (ALUControl)
		
			2'd0: // ADD
			Result = temp;
			
			2'd1: // SUB
			Result = temp;
			
			2'd2: // AND
			Result = andtemp;
			
			2'd3: // OR
			Result = ortemp;
		
		endcase
		
	end
	
	// Overflow
	assign ALUFlags[0] = ((ALUControl[1]==0) && (a[31]^Result[31]==1) && (a[31]^b[31]^ALUControl[0]==0)) ? 1 : 0;
	// Carry-out
	assign ALUFlags[1] = ((ALUControl[1]==0) && (cout==1'b1)) ? 1 : 0;
	// Zero
	assign ALUFlags[2] = (Result==32'd0) ? 1 : 0;
	// Negative
	assign ALUFlags[3] = (Result[31]) 	 ? 1 : 0;

endmodule	