`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Isaac Muyu Zhang
// Engineer: Isaac Muyu Zhang
// 
// Create Date: 08/14/2024 05:00:19 PM
// Design Name: MIPS CPU
// Module Name: testbench
// Project Name: MIPS Single Cycle CPU
// Target Devices: N/A
// Tool Versions: 2024.2
// Description: Implementation of a MIPS Single Cycle CPU
// 
// Dependencies: Ubuntu/Windows OS
// 
// Revision 0.2 Updated Comments
// Additional Comments: None
// 
//////////////////////////////////////////////////////////////////////////////////


module testbench(

    );
    reg clk;
    
    datapath dp(.clk(clk));
    
    initial begin
        clk = 0;
        forever begin
            #10
            clk = ~clk;
        end
    end
endmodule
