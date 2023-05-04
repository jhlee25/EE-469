// Junhyoung Lee & Sangmin Lee
// 04/07/2023
// EE 469
// Lab #1 Task #3
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
	
	// 32-bit logic B which is depending on ALU Control operation (ADD or Sub)
	logic [31:0] B;
	assign B = ALUControl[0] ? ~b : b;
	
	// 1-bit small parts of the overflow flag
	logic overflow1, overflow2, overflow3;
	assign overflow1 = ~ALUControl[1];						// NOT ALUControl[1]
	assign overflow2 = (a[31]^Result[31]);					// a[31] XOR Result[31]
	assign overflow3 = ~(ALUControl[0]^a[31]^b[31]);	// XNOR of (ALUControl[0], a[31], b[31])
	
	always_comb begin
	
		case (ALUControl)
			
			// Add
			2'b00: begin {cout, Result} = a + B;
					 ALUFlags[0] = (overflow1 & overflow2 & overflow3) ? 1 : 0; 		// Overflow
					 ALUFlags[1] = (~ALUControl[1] & cout) ? 1 : 0;							// Carry out
					 end
			
			// Sub
			2'b01: begin {cout, Result} = a + B + 1;
					 ALUFlags[0] = (overflow1 & overflow2 & overflow3) ? 1 : 0;			// Overflow
					 ALUFlags[1] = (~ALUControl[1] & cout) ? 1 : 0;							// Carry out
					 end
			
			// AND
			2'b10: begin Result = a & b; cout = 0;
					 ALUFlags[0] = 0; 																// Overflow
					 ALUFlags[1] = 0;																	// Carry out
					 end
			
			// OR
			2'b11: begin Result = a | b; cout = 0;
					 ALUFlags[0] = 0; 																// Overflow
					 ALUFlags[1] = 0;																	// Carry out
					 end			
		endcase
	
	end
	
	assign ALUFlags[2] = (Result==32'b0) ? 1 : 0;			// Zero
	assign ALUFlags[3] = (Result[31]) 	 ? 1 : 0;			// Negative

endmodule	