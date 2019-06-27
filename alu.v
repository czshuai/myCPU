`timescale 1ns / 1ps

module alu(
    input [3:0] ALUControl,
    input [31:0] alu_src1,
    input [31:0] alu_src2,
    output [31:0] alu_result
    );

wire op_add; //加法操作
wire op_sub; //减法操作
wire op_slt; //有符号比较，小于置位
wire op_sltu; //无符号比较，小于置位
wire op_and; //按位�?
wire op_nor; //按位或非
wire op_or; //按位�?
wire op_xor; //按位异或
wire op_sll; //逻辑左移
wire op_srl; //逻辑右移
wire op_sra; //算术右移
wire op_lui; //高位加载

assign op_add = (ALUControl == 4'b0);
assign op_lui = (ALUControl == 4'b1);
assign op_sub = (ALUControl == 4'b10);
assign op_slt = (ALUControl == 4'b11);
assign op_sltu = (ALUControl == 4'b100);
assign op_and = (ALUControl == 4'b101);
assign op_or = (ALUControl == 4'b110);
assign op_xor = (ALUControl == 4'b111);
assign op_nor = (ALUControl == 4'b1000);
assign op_sll = (ALUControl == 4'b1001);
assign op_srl = (ALUControl == 4'b1010);
assign op_sra = (ALUControl == 4'b1011);

/*
assign op_sub = alu_control[10]; 
assign op_slt = alu_control[9]; 
assign op_sltu = alu_control[8];
assign op_and = alu_control[7]; 
assign op_nor = alu_control[6]; 
assign op_or = alu_control[5];
assign op_xor = alu_control[4]; 
assign op_sll = alu_control[3]; 
assign op_srl = alu_control[2]; 
assign op_sra = alu_control[1]; 
assign op_lui = alu_control[0];
*/

wire [31:0] add_sub_result; 
wire [31:0] slt_result; 
wire [31:0] sltu_result; 
wire [31:0] and_result; 
wire [31:0] nor_result; 
wire [31:0] or_result; 
wire [31:0] xor_result; 
wire [31:0] sll_result; 
wire [63:0] sr64_result;
wire [31:0] sr_result; 
wire [31:0] lui_result;

assign and_result = alu_src1 & alu_src2; 
assign or_result = alu_src1 | alu_src2; 
assign nor_result = ~or_result; 
assign xor_result = alu_src1 ^ alu_src2; 
assign lui_result = {alu_src2[15:0], 16'b0};

wire [31:0] adder_a; 
wire [31:0] adder_b; 
wire adder_cin; 
wire [31:0] adder_result; 
wire adder_cout; 
assign adder_a = alu_src1; 
assign adder_b = alu_src2 ^ {32{op_sub | op_slt | op_sltu}}; 
assign adder_cin = op_sub | op_slt | op_sltu; 
assign {adder_cout, adder_result} = adder_a + adder_b + adder_cin;

assign add_sub_result = adder_result;

assign slt_result[31:1] = 31'b0; 
assign slt_result[0] = (alu_src1[31] & ~alu_src2[31]) | (~(alu_src1[31]^alu_src2[31]) & adder_result[31]);

assign sltu_result[31:1] = 31'b0; 
assign sltu_result[0] = ~adder_cout;

assign sll_result = alu_src2 << alu_src1[4:0];

assign sr64_result = {{32{op_sra&alu_src2[31]}}, alu_src2[31:0]}>>alu_src1[4:0]; 
assign sr_result = sr64_result[31:0];

assign alu_result = ({32{op_add|op_sub }} & add_sub_result) 
                    | ({32{op_slt }} & slt_result) 
                    | ({32{op_sltu }} & sltu_result) 
                    | ({32{op_and }} & and_result) 
                    | ({32{op_nor }} & nor_result) 
                    | ({32{op_or }} & or_result) 
                    | ({32{op_xor }} & xor_result) 
                    | ({32{op_sll }} & sll_result) 
                    | ({32{op_srl|op_sra }} & sr_result) 
                    | ({32{op_lui }} & lui_result);

endmodule