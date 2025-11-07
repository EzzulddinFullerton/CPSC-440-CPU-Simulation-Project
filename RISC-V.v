// Program Counter
module Program_Counter(clk, reset, PC_in, PC_out);

input clk, reset;
input      [31:0] PC_in;
output reg [31:0] PC_out;

always @(posedge clk or posedge reset)
begin
if(reset)
	PC_out <= 32'b00;
else
	PC_out <= PC_in;
end

endmodule

// PC + 4

module PCplus4(fromPC, NextoPC);

input[31:0] fromPC;
output[31:0] NextoPC;

assign NextoPC = 4 + fromPC;

endmodule

// Instruction Memory
module Instruction_Mem(clk, reset, read_address, instruction_out);

input clk, reset;
input [31:0] read_address;
output [31:0] instruction_out;

reg [31:0] IMemory [63:0];
assign instruction_out = IMemory[read_address[7:2]];

endmodule
//Register File

module Reg_File(clk, reset, RegWrite, Rs1, Rs2, Rd, Write_data, read_data1, read_data2);

input clk, reset, RegWrite;
input [4:0] Rs1, Rs2, Rd;
input [31:0] Write_data;
output [31:0] read_data1, read_data2;
integer k;
reg [31:0] Registers[31:0];

always @(posedge clk or posedge reset)
begin
if(reset)
	begin
		for(k=0; k<32; k=k+1)begin
		Registers[k] <= 32'b00;
		end
	end
else if(RegWrite && (Rd != 5'b0))begin
	Registers[Rd] <= Write_data;
end
end

assign read_data1 = Registers[Rs1];
assign read_data2 = Registers[Rs2];

endmodule

// Immediate Generator
module ImmGen(Opcode, instruction, ImmExt);
input      [6:0] Opcode;
input      [31:0] instruction;
output reg [31:0] ImmExt;
always @(*)
begin
	case(Opcode)
	7'b0000011, 7'b0010011, 7'b1100111 : ImmExt = {{20{instruction[31]}}, instruction[31:20]}; // I-type
	
	7'b0100011 : ImmExt = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S-type
	7'b1100011 : ImmExt = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0}; // B-type
	7'b0010111, 7'b0110111 : ImmExt = {instruction[31:12], 12'b0}; // U-type
	7'b1101111 : ImmExt = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0}; // J-type
	
	default: ImmExt = 32'b0;
	endcase
end

endmodule
// Control Unit
module Control_Unit(instruction, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite, ALUSrcA, Jump, Jalr, WriteSel);

input [6:0] instruction;
output reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
output reg [1:0] ALUOp;
output reg [1:0] ALUSrcA, WriteSel;
output reg Jump, Jalr;

always @(*)
begin
	ALUSrcA = 2'b00;
	Jump = 0;
	Jalr = 0;
	WriteSel = 2'b00;
	MemtoReg = 0; // Legacy, not used
	case(instruction)
	7'b0110011 : begin {ALUSrc, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 7'b01000010; WriteSel=2'b00; end
	7'b0010011 : begin {ALUSrc, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 7'b1100011; WriteSel=2'b00; end
	7'b0000011 : begin {ALUSrc, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 7'b1110000; WriteSel=2'b01; end
	7'b0100011 : begin {ALUSrc, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 7'b1001000; WriteSel=2'b00; end
	7'b1100011 : begin {ALUSrc, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 7'b0000101; WriteSel=2'b00; end
	7'b0110111 : begin {ALUSrc, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 7'b1100000; ALUSrcA=2'b10; WriteSel=2'b00; end
	7'b0010111 : begin {ALUSrc, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 7'b1100000; ALUSrcA=2'b01; WriteSel=2'b00; end
	7'b1101111 : begin {ALUSrc, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 7'b0100000; Jump=1; Jalr=0; WriteSel=2'b10; end
	7'b1100111 : begin {ALUSrc, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 7'b1100000; Jump=1; Jalr=1; WriteSel=2'b10; end
	default:     {ALUSrc, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 7'b0000000; 
	endcase
end
endmodule

// ALU
module ALU_unit(A, B, Control_in, ALU_Result, zero);

input [31:0] A, B;
input[3:0] Control_in;
output reg zero;
output reg [31:0] ALU_Result;
always @(Control_in or A or B)
begin
	case(Control_in)
	4'b0000: begin zero = 0; ALU_Result = A & B; end
	4'b0001: begin zero = 0; ALU_Result = A | B; end 
	4'b0010: begin zero = 0; ALU_Result = A + B; end 
	4'b0011: begin zero = 0; ALU_Result = A ^ B; end
	4'b0100: begin zero = 0; ALU_Result = A << B[4:0]; end
	4'b0101: begin zero = 0; ALU_Result = A >> B[4:0]; end
	4'b0110: begin 
	         if(A==B) zero = 1; else zero = 0; 
	         ALU_Result = A - B; 
	         end
	4'b0111: begin zero = 0; ALU_Result = $signed(A) >>> B[4:0]; end
	default: begin zero = 0; ALU_Result = 32'b0; end 
	endcase
end
endmodule
// ALU Control
module ALU_Control(ALUOp, fun7, fun3, Control_out);

input fun7;
input [2:0] fun3;
input [1:0] ALUOp;
output reg [3:0] Control_out;

always @(*)
begin
  if(ALUOp == 2'b00) 
    Control_out = 4'b0010; // add
  else if(ALUOp == 2'b01)
    Control_out = 4'b0110; // sub
  else begin
    Control_out = 4'b0000;
    case({fun7, fun3})
    4'b0000: Control_out = 4'b0010; // add/i
    4'b1000: Control_out = (ALUOp==2'b10 ? 4'b0110 : 4'b0010); // sub or add
    4'b0111: Control_out = 4'b0000; // and/i
    4'b0110: Control_out = 4'b0001; // or/i
    4'b0100: Control_out = 4'b0011; // xor/i
    4'b0001: Control_out = 4'b0100; // sll/i
    4'b0101: Control_out = 4'b0101; // srl/i
    4'b1101: Control_out = 4'b0111; // sra/i
    default: Control_out = 4'b0000;
    endcase
  end
end

endmodule

// Data Memory
module Data_Memory(clk, reset, MemWrite, MemRead, read_address, Write_data, MemData_out);

input clk, reset, MemWrite, MemRead;
input[31:0] read_address, Write_data;
output [31:0] MemData_out;
integer k;
reg [31:0] D_Memory[63:0];

always @(posedge clk or posedge reset)
begin
if(reset)
	begin
		for(k=0; k<64; k=k+1)begin
		D_Memory[k] <= 32'b00;
		end
	end
else if (MemWrite) begin
	D_Memory[read_address[7:2]] <= Write_data;
	end
end
assign MemData_out = (MemRead) ? D_Memory[read_address[7:2]] : 32'b00;

endmodule

// Multiplexers
module Mux1(sel1, A1, B1, Mux1_out);

input sel1;
input [31:0] A1, B1;
output [31:0] Mux1_out;

assign Mux1_out = (sel1==1'b0) ? A1 : B1;
endmodule

// Branch logic
module Branch_logic(branch, zero, fun3, taken);

input branch, zero;
input [2:0] fun3;
output reg taken;

always @(*)
begin
  if(branch)
    case(fun3)
    3'b000: taken = zero; // beq
    3'b001: taken = ~zero; // bne
    default: taken = 0;
    endcase
  else
    taken = 0;
end

endmodule

// Adder
module Adder(in_1, in_2, Sum_out);

input [31:0] in_1, in_2;
output [31:0] Sum_out;

assign Sum_out = in_1 + in_2;


endmodule


// All modules instantiate here

module top(clk, reset);
input clk, reset;

wire [31:0] PC_top, instruction_top, Rd1_top, Rd2_top,ImmExt_top, mux1_top, Sum_out_top, NextoPC_top, PCin_top, address_top, Memdata_top, WriteBack_top;
wire RegWrite_top , ALUSrc_top, zero_top, branch_top, MemtoReg_top, MemWrite_top, MemRead_top;
wire [1:0] ALUOp_top, ALUSrcA_top, WriteSel_top;
wire Jump_top, Jalr_top;
wire [3:0] control_top;
wire sel2_top; // taken


// Program Counter
Program_Counter PC( .clk(clk), .reset(reset), .PC_in(PCin_top), .PC_out(PC_top));

// PC Adder
PCplus4 PC_Adder(.fromPC(PC_top), .NextoPC(NextoPC_top));

// Instruction Memory
Instruction_Mem Inst_Memory(.clk(clk), .reset(reset), .read_address(PC_top), .instruction_out(instruction_top));

// Register File
Reg_File Reg_File(.clk(clk), .reset(reset), .RegWrite(RegWrite_top), .Rs1(instruction_top[19:15]), .Rs2(instruction_top[24:20]), .Rd(instruction_top[11:7]), .Write_data(WriteBack_top), .read_data1(Rd1_top), .read_data2(Rd2_top));

// Immediate Generator
ImmGen ImmGen(.Opcode(instruction_top[6:0]), .instruction(instruction_top), .ImmExt(ImmExt_top));

// Control Unit
Control_Unit Control_Unit(.instruction(instruction_top[6:0]), .Branch(branch_top), .MemRead(MemRead_top), .MemtoReg(MemtoReg_top), .ALUOp(ALUOp_top), .MemWrite(MemWrite_top), .ALUSrc(ALUSrc_top), .RegWrite(RegWrite_top), .ALUSrcA(ALUSrcA_top), .Jump(Jump_top), .Jalr(Jalr_top), .WriteSel(WriteSel_top));

// ALU Control
ALU_Control ALU_Control( .ALUOp(ALUOp_top), .fun7(instruction_top[30]), .fun3(instruction_top[14:12]), .Control_out(control_top));

// ALU A Mux
wire [31:0] alu_a;
assign alu_a = (ALUSrcA_top == 2'b00) ? Rd1_top : (ALUSrcA_top == 2'b01 ? PC_top : 32'b0);

// ALU
ALU_unit ALU(.A(alu_a), .B(mux1_top), .Control_in(control_top), .ALU_Result(address_top), .zero(zero_top));

// ALU B Mux
Mux1 ALU_mux(.sel1(ALUSrc_top), .A1(Rd2_top), .B1(ImmExt_top), .Mux1_out(mux1_top));

// Adder for branch/jal target
Adder Adder(.in_1(PC_top), .in_2(ImmExt_top), .Sum_out(Sum_out_top));

// Branch Logic
Branch_logic Branch(.branch(branch_top), .zero(zero_top), .fun3(instruction_top[14:12]), .taken(sel2_top));

// PC Mux
wire [31:0] jalr_target = Rd1_top + ImmExt_top;
assign PCin_top = sel2_top ? Sum_out_top : (Jump_top ? (Jalr_top ? jalr_target : Sum_out_top) : NextoPC_top);

// Data Memory
Data_Memory Data_mem(.clk(clk), .reset(reset), .MemWrite(MemWrite_top), .MemRead(MemRead_top), .read_address(address_top), .Write_data(Rd2_top), .MemData_out(Memdata_top));

// WriteBack Mux
assign WriteBack_top = (WriteSel_top == 2'b00) ? address_top : (WriteSel_top == 2'b01 ? Memdata_top : NextoPC_top);


endmodule


// testbench
module tb_top;
reg clk, reset;

// The DUT is instantiated here
top uut(.clk(clk), .reset(reset));

initial begin
    // Load Instructions into Instruction Memory (indexed by word address)
    // Address / 4 = array index.
    uut.Inst_Memory.IMemory[0] = 32'h00500093; // addi x1, x0, 5
    uut.Inst_Memory.IMemory[1] = 32'h00A00113; // addi x2, x0, 10
    uut.Inst_Memory.IMemory[2] = 32'h002081B3; // add x3, x1, x2
    uut.Inst_Memory.IMemory[3] = 32'h40110233; // sub x4, x2, x1
    uut.Inst_Memory.IMemory[4] = 32'h000002B7; // lui x5, 0x0 (Address 0x00000000, targets memory index 0)
    uut.Inst_Memory.IMemory[5] = 32'h0022A023; // sw x3, 0(x5) (Stores 15 to memory[0])
    uut.Inst_Memory.IMemory[6] = 32'h0002A203; // lw x4, 0(x5) (Loads 15 to x4)
    uut.Inst_Memory.IMemory[7] = 32'h00418463; // beq x3, x4, label1
    uut.Inst_Memory.IMemory[8] = 32'h00100313; // addi x6, x0, 1 (skipped)
    uut.Inst_Memory.IMemory[9] = 32'h00200313; // label1: addi x6, x0, 2
    uut.Inst_Memory.IMemory[10] = 32'h0000006F; // jal x0, 0 (halt)

    // Setup simulation
    clk = 0;
    reset=1;
    #5;
    reset = 0;
    $display("------------------- RISC-V Testbench Start --------------------");
    
    // Monitor key values: PC and Registers used (x1, x2, x3, x4, x6)
    $display("------------------------------------------------------------------------------------------------");
    $display("| %4s | %8s | %8s | %8s | %8s | %8s | %8s |", "Time", "PC", "x1 (5)", "x2 (10)", "x3 (15)", "x4 (15)", "x6 (2)");
    $display("------------------------------------------------------------------------------------------------");

    $monitor("| %4d | %8h | %8d | %8d | %8d | %8d | %8d |",
             $time, uut.PC_top,
             uut.Reg_File.Registers[1], // x1
             uut.Reg_File.Registers[2], // x2
             uut.Reg_File.Registers[3], // x3
             uut.Reg_File.Registers[4], // x4
             uut.Reg_File.Registers[6]  // x6
             );

    #120; // Run for 120 time units (12 clock cycles, enough to finish)

    $display("------------------------------------------------------------------------------------------------");
    $display("Simulation finished. Final check: x6 should be 2. Current value: %0d", uut.Reg_File.Registers[6]);
    $finish;
end

always begin
#5 clk = ~clk;
end

endmodule
