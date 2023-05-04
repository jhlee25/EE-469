// Junhyoung Lee & Sangmin Lee
// 04/07/2023
// EE 469
// Lab #1 Task #2
// This program implement 16x32 register file 

// This module works as the 16x32 register file with two read port, one write port, and asynchronous
// Input: clock clk (1-bit), write enable wr_en (1-bit), write address write_addr (4-bit)
//			 write_data (32-bit), read_addr1 and read_addr2 (4-bit)
// Output: read_data1 and read_data2 (32-bit)

module reg_file (input logic clk, wr_en,
					  input logic [3:0] write_addr,
					  input logic [31:0] write_data,
					  input logic [3:0] read_addr1, read_addr2,
					  output logic [31:0] read_data1, read_data2);
					  
	// 16x32 memory array (16 words with 32 bits per word)
	logic [31:0] memory [15:0];
	
	// write the data to the write_address
	always_ff @ (posedge clk) begin
		if (wr_en) begin
			memory[write_addr] <= write_data;
		end
	end
	
	// read data corresponding to the read_address asynchronously
	assign read_data1 = memory[read_addr1];
	assign read_data2 = memory[read_addr2];
	
endmodule	