`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Isaac Muyu Zhang
// Engineer: Isaac Muyu Zhang
// 
// Create Date: 08/14/2024 05:07:58 PM
// Design Name: MIPS CPU
// Module Name: datapath
// Project Name: MIPS Single Cycle CPU
// Target Devices: N/A
// Tool Versions: 2024.2
// Description: Implementation of a MIPS Single Cycle CPU
// 
// Dependencies: Ubuntu/Windows OS
// 
// Revision 5.6 Updated Comments
// Additional Comments: None
// 
//////////////////////////////////////////////////////////////////////////////////
module datapath(
    input clk
    );
 
    // Misc
    wire [31:0] pc, nextPc;
    wire [31:0] inst;
    wire [4:0] readAddr1, readAddr2, writeAddr;
    wire [31:0] regOut1, regOut2, writeData;
    wire regWrite, memToReg, memWrite, aluSrc, regDst, memRead;
    wire [3:0] aluControl;
    wire [31:0] imm32;
    wire [31:0] memOut;
    
    // Hazard Unit
    wire stall;
    // Fowarding Unit
    wire [1:0] forwardA, forwardB;
    
    // ALU Mux
    wire [31:0] alu_mux_in0;
    
    // ALU
    wire [31:0] aluIn1;
    wire [31:0] aluIn2;
    wire [31:0] aluOut;
    
    // IF/ID
    wire[31:0] inst_d;
    
    // ID/EX
    wire [3:0] aluControl_x;
    wire [4:0] rs_x, rt_x, rd_x;
    wire [31:0] regOut1_x, regOut2_x, imm32_x;
    wire regWrite_x, memToReg_x, memWrite_x, aluSrc_x, regDst_x, memRead_x;
    
    // EX/MEM
    wire [4:0]writeAddr_m;
    wire [31:0] aluOut_m, regOut2_m;
    wire regWrite_m, memToReg_m, memWrite_m, memRead_m;
    
    // MEM/WB
    wire [4:0] writeAddr_b;
    wire [31:0] aluOut_b, memOut_b;
    wire regWrite_b, memToReg_b;
    
    
    // Program Counter
    program_counter pcmodule(
        .clk(clk),
        .stall(stall),
        .nextPc(nextPc),
        .pc(pc)
    );
    
    // PC Adder
    pc_adder pa(
        .pc(pc),
        .offset(32'd4),
        .nextPc(nextPc)
    );
    
    
    // Instantiate instruction memory and connect it to the pc
    inst_mem im(
        .pc(pc),
        .inst(inst) 
    ); 
    
    // IF/ID Register
    IF_ID_register IF_ID(
        .clk(clk),
        .stall(stall),
        .inst(inst),
        .inst_d(inst_d)
    );
    
    // Hazard Unit
    hazard_unit hazard_unit(
        .rt_x(rt_x),
        .rt_d(inst_d[20:16]),
        .rs_d(inst_d[25:21]),
        .memRead_x(memRead_x),
        .stall(stall)
    ); 
    
    
    // Register File
    //assign readAddr1 = inst_d[25:21]; // rs
    //assign readAddr2 = inst_d[20:16]; // rt

    register_file rf(
        .clk(clk),
        .readAddr1(inst_d[25:21]), 
        .readAddr2(inst_d[20:16]), 
        .writeAddr(writeAddr_b), 
        .writeData(writeData), 
        .regWrite(regWrite),
        .regOut1(regOut1), 
        .regOut2(regOut2)
    );
    
    // Control Unit
    control_unit ctrl(
        .stall(stall),
        .op(inst_d[31:26]), 
        .func(inst_d[5:0]),
        .regWrite(regWrite), 
        .memToReg(memToReg), 
        .memWrite(memWrite), 
        .aluSrc(aluSrc), 
        .regDst(regDst), 
        .memRead(memRead),
        .aluControl(aluControl)
    );
    
    
    // Immediate Extension
    imm_extend imm_ext(
        .imm(inst_d[15:0]),
        .imm32(imm32)
    );
    
    
    // ID/EX Register
    ID_EX_register ID_EX(
        .clk(clk),
        .aluControl(aluControl),
        .rs(inst_d[25:21]),
        .rt(inst_d[20:16]),
        .rd(inst_d[15:11]),
        .regOut1(regOut1),
        .regOut2(regOut2),
        .imm32(imm32),
        .regWrite(regWrite),
        .memToReg(memToReg),
        .memWrite(memWrite),
        .aluSrc(aluSrc),
        .regDst(regDst),
        .memRead(memRead),
        .aluControl_x(aluControl_x),
        .rs_x(rs_x),
        .rt_x(rt_x),
        .rd_x(rd_x),
        .regOut1_x(regOut1_x),
        .regOut2_x(regOut2_x),
        .imm32_x(imm32_x),
        .regWrite_x(regWrite_x),
        .memToReg_x(memToReg_x),
        .memWrite_x(memWrite_x),
        .aluSrc_x(aluSrc_x),
        .regDst_x(regDst_x),
        .memRead_x(memRead_x)
    );
    
    // Write addr mux
    mux_2x1_5b write_addr_mux(
        .in0(rt_x),
        .in1(rd_x),
        .sel(regDst_x),
        .out(writeAddr)
    );
    
    // Forwarding Unit
    forwarding_unit forwarding_unit(
        .writeAddr_m(writeAddr_m),
        .writeAddr_b(writeAddr_b),
        .rs_x(rs_x),
        .rt_x(rt_x),
        .regWrite_m(regWrite_m),
        .regWrite_b(regWrite_b),
        .forwardA(forwardA),
        .forwardB(forwardB)
    );
    
    // Forward mux A (3x1 32b)
    mux_3x1_32b forwardA_mux(
        .in0(regOut1_x),
        .in1(writeData),
        .in2(aluOut_m),
        .sel(forwardA),
        .out(aluIn1)
    );
    
    // Forward mux B (3x1 32b)
    mux_3x1_32b forwardB_mux(
        .in0(regOut2_x),
        .in1(writeData),
        .in2(aluOut_m),
        .sel(forwardB),
        .out(alu_mux_in0)
    );
    

    // ALU Input Multiplexer (selects between register data and immediate value)
    mux_2x1_32b alu_src_mux(
        .in0(alu_mux_in0),
        .in1(imm32_x),
        .sel(aluSrc_x),
        .out(aluIn2)
    );


    // ALU
    alu alu_unit(
        .aluIn1(aluIn1),
        .aluIn2(aluIn2),
        .aluControl(aluControl_x),
        .aluOut(aluOut)
    );
    
    
    // EX/MEM Register
    EX_MEM_register EX_MEM(
        .clk(clk),
        .writeAddr(writeAddr),
        .aluOut(aluOut),
        .regOut2_x(alu_mux_in0),
        .regWrite_x(regWrite_x),
        .memToReg_x(memToReg_x),
        .memWrite_x(memWrite_x),
        .memRead_x(memRead_x),
        .writeAddr_m(writeAddr_m),
        .aluOut_m(aluOut_m),
        .regOut2_m(regOut2_m),
        .regWrite_m(regWrite_m),
        .memToReg_m(memToReg_m),
        .memWrite_m(memWrite_m),
        .memRead_m(memRead_m)
    );
    
    
    // Data Memory
    data_mem data_memory(
        .clk(clk),
        .memWrite(memWrite_m),
        .memRead(memRead_m),
        .addr(aluOut_m),
        .memIn(regOut2_m),
        .memOut(memOut) 
    );
    
    
    // MEM/WB Register
    MEM_WB_register MEM_WB(
        .clk(clk),
        .writeAddr_m(writeAddr_m),
        .aluOut_m(aluOut_m),
        .memOut(memOut),
        .regWrite_m(regWrite_m),
        .memToReg_m(memToReg_m),
        .writeAddr_b(writeAddr_b),
        .aluOut_b(aluOut_b),
        .memOut_b(memOut_b),
        .regWrite_b(regWrite_b),
        .memToReg_b(memToReg_b)
    );
    
    
    // WB Stage: Mux for Write Data (ALU Result or Memory Output)
    mux_2x1_32b wb_mux(
        .in0(aluOut_b),   // ALU Result
        .in1(memOut_b),      // Memory Output
        .sel(memToReg_b),    // Control Signal for Write-back
        .out(writeData)    // Data to Write into Register
    );

endmodule
/* ================= Modules to implement for HW1 =====================*/
module program_counter(
    input clk, 
    input stall,
    input [31:0] nextPc,
    output reg [31:0] pc
    );
    initial begin
        pc = 32'd96; // PC initialized to start from 100.
    end
    // ==================== Students fill here BEGIN ====================
    always@(posedge clk) begin // Update on every positive edge of clock
        if (stall == 0) begin // Check if stall
            pc <= nextPc; // setting pc to nextPc
        end
    end
    // ==================== Students fill here END ======================
endmodule

module pc_adder(
    input [31:0] pc, offset,
    output reg [31:0] nextPc
    );
    // ==================== Students fill here BEGIN ====================
    always@(*) begin // Updatte on every input
        nextPc = pc + offset; // Adding offset to pc and then setting that value to nextPc
    end
    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW3 =====================*/
module inst_mem(
    input [31:0] pc,
    output reg [31:0] inst
    );
    
    // This is an instruction memory that holds 64 instructions, 32b each.
    reg [31:0] memory [0:63];
    
    // Initializing instruction memory.
    initial begin       
        /* memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};
        memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};
        memory[27] = {6'b000000, 5'd1, 5'd2, 5'd3, 11'b00000100010};
        memory[28] = {6'b100011, 5'd3, 5'd4, 16'hFFFC}; */
       /* memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};
        memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};
        memory[27] = {6'b100011, 5'd0, 5'd3, 16'd8};
        memory[28] = {6'b100011, 5'd0, 5'd4, 16'd16};
        memory[29] = {6'b000000, 5'd1, 5'd2, 5'd5, 11'b00000100000};
        memory[30] = {6'b100011, 5'd3, 5'd6, 16'hFFFC};
        memory[31] = {6'b000000, 5'd4, 5'd3, 5'd7, 11'b00000100010}; */
        memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};
        memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};
        memory[27] = {6'b100011, 5'd0, 5'd4, 16'd16};
        memory[28] = {6'b000000, 5'd1, 5'd2, 5'd3, 11'b00000100010};
        memory[29] = {6'b100011, 5'd3, 5'd4, 16'hFFFC}; 
    end
    // ==================== Students fill here BEGIN ====================
    always @(*) begin
        inst = memory[pc[31:2]]; // only use upper bits, ie ignore 2 least sig
    end
    // ==================== Students fill here END ======================
endmodule

module register_file(
    input [4:0] readAddr1, readAddr2, writeAddr,
    input [31:0] writeData,
    input regWrite, clk,
    output reg [31:0] regOut1, regOut2
    );
    
    // Initializing registers. Do not touch here.
    reg [31:0] register [0:31]; // 32 registers, 32b each.
    integer i;
    initial begin
        for (i=0; i<32; i=i+1) begin
            register[i] = 32'd0; // Initialize to zero
        end
    end
    // ==================== Students fill here BEGIN ====================
    always @(posedge clk) begin
        regOut1 = register[readAddr1]; // read first address from register
        regOut2 = register[readAddr2]; // read second address from register
    end
    
    always@(negedge clk) begin // neg edge clock write
        if (regWrite) begin
            register[writeAddr] <= writeData;
        end
    end
        

    // ==================== Students fill here END ======================
endmodule

module control_unit(
    input [5:0] op, func,
    input stall,
    output reg regWrite, memToReg, memWrite, aluSrc, regDst, memRead,
    output reg [3:0] aluControl
    );
    // ==================== Students fill here BEGIN ====================
     always @(op, func, stall) begin
        if (stall == 0) begin // Check if stall
        // Initialize default signal values
        regWrite = 0;
        memToReg = 0;
        memWrite = 0;
        aluSrc = 0;
        regDst = 0;
        memRead = 0;
        aluControl = 4'b0000;

        case (op)
            6'b100011: begin // check for lw
                regWrite = 1; // regWrite is 1
                memToReg = 1; // memToReg is 1
                memRead = 1; // memRead is 1
                aluSrc = 1; // Use immediate value
                aluControl = 4'b0010; // select ADD for address calculation with offset
            end
            6'b101011: begin // check for sw
                memWrite = 1; // Only memWrite is 1 for sw
                aluSrc = 1; // use immediate value
                aluControl = 4'b0010; // select ADD for address calculation with offset
            end
            6'b000000: begin // R-type (implementing add and sub for now)
                case (func)
                    6'b100000: begin // check for add
                        regWrite = 1; // regWrite is 1
                        regDst = 1; // write to rd
                        aluControl = 4'b0010; // select ADD from ALU
                    end
                    6'b100010: begin // check for sub
                        regWrite = 1; // regWrite is 1
                        regDst = 1; // write to rd
                        aluControl = 4'b0110; // select SUB from ALU
                    end
                    // add more funcs later
                endcase
            end
            default: begin
                // for handling other cases or leave default alone
            end
        endcase 
        end else if (stall == 1) begin // Set all signals to 0
            regWrite = 0;
            memToReg = 0;
            memWrite = 0;
            aluSrc = 0;
            regDst = 0;
            memRead = 0;
            aluControl = 4'b0000;
        end
    end
    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW4 =====================*/
module imm_extend(
    input [15:0] imm,
    output reg [31:0] imm32
    );
    // ==================== Students fill here BEGIN ====================
    always @(*) begin
        imm32 = {{16{imm[15]}}, imm}; // Sign-extension
    end
    // ==================== Students fill here END ======================
endmodule

module mux_2x1_32b(
    input [31:0] in0, in1,
    input sel,
    output reg [31:0] out
    );
    // ==================== Students fill here BEGIN ====================
    always @(in0, in1, sel) begin
        out = sel ? in1 : in0; // Formatted if-else to select in0 if sel == 0 else in1 is selected
    end
    // ==================== Students fill here END ======================
endmodule

module alu(
    input [31:0] aluIn1, aluIn2,
    input [3:0] aluControl,
    output reg [31:0] aluOut
    );

    // ==================== Students fill here BEGIN ====================
    always @(*) begin
        case (aluControl)
            4'b0010: aluOut = aluIn1 + aluIn2; // ADD
            4'b0110: aluOut = aluIn1 - aluIn2; // SUB
            default: aluOut = 32'b0; // // Set output to zero if no match
        endcase
    end
    // ==================== Students fill here END ======================
endmodule

module data_mem(
    input clk, memWrite, memRead,
    input [31:0] addr, memIn,
    output reg [31:0] memOut
    );
    
    reg [31:0] memory [0:63]; // 64x32 memory
    
    // Initialize data memory. Do not touch this part.
    initial begin
        memory[0] = 32'd16817;
        memory[1] = 32'd16801;
        memory[2] = 32'd16;
        memory[3] = 32'hDEAD_BEEF;
        memory[4] = 32'h4242_4242;
    end
    
    // ==================== Students fill here BEGIN ====================
    always @(*) begin
        if (memRead)
            memOut = memory[addr[31:2]]; // Read operation
        else
            memOut = 32'bx; // Undefined when not reading
    end

    always @(negedge clk) begin
        if (memWrite)
            memory[addr[31:2]] <= memIn; // Write operation on negative edge
    end
    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW5 =====================*/
module mux_2x1_5b(
    input [4:0] in0, in1,
    input sel,
    output reg [4:0] out
    );
    // ==================== Students fill here BEGIN ====================
    always @(in0, in1,sel) begin
        out = sel ? in1 : in0; // Formatted if-else to select in0 if sel == 0 else in1 is selected
    end
    // ==================== Students fill here END ======================
endmodule

module IF_ID_register(
    input clk,
    input [31:0] inst,
    input stall,
    output reg [31:0] inst_d
    );
    
    always @(posedge clk)begin
        if (stall == 0) begin // Check for stall
            inst_d = inst;
        end
    end
    
endmodule

module ID_EX_register(
    input clk,
    input[3:0] aluControl,
    input [4:0] rs, rt, rd,
    input [31:0] regOut1, regOut2, imm32,
    input regWrite, memToReg, memWrite, aluSrc, regDst, memRead,
    output reg [3:0] aluControl_x,
    output reg [4:0] rs_x, rt_x, rd_x,
    output reg [31:0] regOut1_x, regOut2_x, imm32_x,
    output reg regWrite_x, memToReg_x, memWrite_x, aluSrc_x, regDst_x, memRead_x
    );
    
    always @(posedge clk) begin // Update the outputs to inputs
        aluControl_x = aluControl;
        rs_x = rs;
        rt_x = rt;
        rd_x = rd;
        regOut1_x = regOut1;
        regOut2_x = regOut2;
        imm32_x = imm32;
        regWrite_x = regWrite;
        memToReg_x = memToReg;
        memWrite_x = memWrite;
        aluSrc_x = aluSrc;
        regDst_x = regDst;
        memRead_x = memRead;
    end
        
endmodule

module EX_MEM_register(
    input clk,
    input [4:0] writeAddr,
    input [31:0] aluOut, regOut2_x,
    input regWrite_x, memToReg_x, memWrite_x, memRead_x,
    output reg [4:0]writeAddr_m,
    output reg[31:0] aluOut_m, regOut2_m,
    output reg regWrite_m, memToReg_m, memWrite_m, memRead_m
    );
    
    always @(posedge clk)begin // Update the outputs to inputs
        writeAddr_m = writeAddr;
        aluOut_m = aluOut;
        regOut2_m = regOut2_x;
        regWrite_m = regWrite_x;
        memToReg_m = memToReg_x;
        memWrite_m = memWrite_x;
        memRead_m = memRead_x;
    end
    
endmodule

module MEM_WB_register(
    input clk,
    input [4:0] writeAddr_m,
    input [31:0] aluOut_m, memOut,
    input regWrite_m, memToReg_m,
    output reg [4:0] writeAddr_b,
    output reg [31:0] aluOut_b, memOut_b,
    output reg regWrite_b, memToReg_b
    );
    
    always @(posedge clk)begin // Update the outputs to inputs
        writeAddr_b = writeAddr_m;
        aluOut_b = aluOut_m;
        memOut_b = memOut;
        regWrite_b = regWrite_m;
        memToReg_b = memToReg_m;
    end
    
endmodule

module forwarding_unit(
    input [4:0] writeAddr_m, writeAddr_b, rs_x, rt_x,
    input regWrite_m, regWrite_b,
    output reg [1:0] forwardA, forwardB
    );
    
    always @(rs_x, rt_x, writeAddr_m, writeAddr_b, regWrite_m, regWrite_b) begin
    //always @(*) begin
		forwardA = 2'b00; // Reg out 1 from x
		forwardB = 2'b00; // Reg out 2 from x
	
		if((regWrite_m == 1'b1) && (writeAddr_m != 5'b00000) && (writeAddr_m == rs_x)) begin // mem forward c-code translated
			forwardA = 2'b10; // ALU output from mem
		end else if((regWrite_b == 1'b1) && (writeAddr_b != 5'b00000) && (writeAddr_b == rs_x)) begin // writeback forward c-code translated
			forwardA = 2'b01; // Write data
		end
		
		if((regWrite_m == 1'b1) && (writeAddr_m != 5'b00000) && (writeAddr_m == rt_x)) begin // mem forward c-code translated
			forwardB = 2'b10; // ALU output from mem
		end else if((regWrite_b == 1'b1) && (writeAddr_b != 5'b00000) && (writeAddr_b == rt_x)) begin // writeback forward c-code translated
			forwardB = 2'b01; // Write data
		end
	end
endmodule


module mux_3x1_32b(
    input [31:0] in0, in1, in2,
    input [1:0] sel,
    output reg [31:0] out
);  
    // Note, mux may look weird because I tried multiple code styles while trying to debug
    always @(in0, in1, in2, sel) begin
        case (sel)
            2'b00: out = in0; // Select input 0
            2'b01: out = in1; // Select input 1
            2'b10: out = in2; // Select input 2
            default: out = 32'b0; // Default to 0 for unexpected `sel` values
        endcase
    end
endmodule

module hazard_unit(
    input [4:0] rt_x, rt_d, rs_d, 
    input memRead_x,
    output reg stall
    );
    
    initial begin
        assign stall = 1'b0; // Initialize stall to 0 (no stall)
    end
   
    always @(rt_x, rt_d, rs_d, memRead_x) begin
        stall = 1'b0;
        if ((memRead_x == 1'b1) && ((rt_x == rs_d) || (rt_x == rt_d))) begin // hazard detection c-code translated
            stall = 1'b1; // Stall signal to 1 (stall)
        end else 
            stall = 1'b0; // Stall signal to 0 (no stall)
    end
endmodule
