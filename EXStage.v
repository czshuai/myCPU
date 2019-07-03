module EXStage (
    input clk, resetn,

    input ID_to_EX_valid,

    input ME_allowin,

    input [31:0] ID_NextPC, ID_PC, 

    input [4:0] ID_rs, ID_rt, ID_rd,

    input [31:0] ID_UnSignExt_imm106, ID_Ext_imm150,

    input [31:0] ID_rdata1, ID_rdata2,

    input [4:0] ID_ALUControl,
    input ID_RegWrite,
    input ID_MemWrite,
    input ID_MemToReg,
    input ID_RegDst,
    input ID_ALUSrc1,
    input ID_ALUSrc2,
    input ID_Jal,
    input ID_Stall,

    input ID_Div,
    input ID_DivSigned,
    input ID_Mul,
    input ID_MulSigned,

    input ID_SpecialRegWri,
    input ID_SpecialRegRead,
    input [1:0] ID_SpecialRegSel,
    input [31:0] ID_ReadHiReg, ID_ReadLoReg,

    output EX_allowin, EX_to_ME_valid,
    output reg EX_valid,

    output reg [31:0] EX_NextPC, EX_PC,

    output [4:0] EX_WriteReg,
    output EX_RegWrite,
    output [31:0] EX_aluResult,
    output reg EX_MemToReg,
    output reg EX_Jal,
    output reg EX_MemWrite,
    output [31:0] EX_WriteData,

    output [31:0] EX_LOVal, EX_HIVal,
    output reg EX_SpecialRegWri,
    output reg EX_SpecialRegRead,
    output reg [31:0] EX_ReadHiReg, EX_ReadLoReg,
    output reg EX_Mul,
    output reg [1:0] EX_SpecialRegSel,

    output [3:0] data_sram_wen,
    output [31:0] data_sram_addr, data_sram_wdata,
    
    output [63:0] ME_MulRes
    );

wire EX_ready_go;

wire EX_Stall;
reg [4:0] EX_rs;
reg [4:0] EX_rt;
reg [4:0] EX_rd;
reg [4:0] EX_ALUControl;
reg EX_OldRegWrite;
reg EX_RegDst;
reg EX_ALUSrc1;
reg EX_ALUSrc2;
reg [31:0] EX_Ext_imm150;
reg [31:0] EX_UnSignExt_imm106;
reg [31:0] EX_rdata1;
reg [31:0] EX_rdata2;
reg EX_Div;
reg EX_DivSigned; 
reg EX_MulSigned;
wire [31:0] EX_srcA;
wire [31:0] EX_srcB;
wire EX_Overflow;
wire EX_DivComplete;
wire [31:0] EX_DivResS;
wire [31:0] EX_DivResR;

assign EX_ready_go = ~EX_Stall;
assign EX_allowin = !EX_valid || EX_ready_go && ME_allowin;
assign EX_to_ME_valid = EX_valid && EX_ready_go;

always @(posedge clk) begin
    if (~resetn) begin
        EX_valid <= 1'b0;
    end
    else if (EX_allowin) begin
        EX_valid <= ID_to_EX_valid;
    end

    if (ID_to_EX_valid && EX_allowin) begin
        EX_NextPC <= ID_NextPC;
        EX_PC <= ID_PC;
        EX_rs <= ID_rs;
        EX_rt <= ID_rt;
        EX_rd <= ID_rd;
        EX_ALUControl <= ID_ALUControl;
        EX_OldRegWrite <= ID_RegWrite;
        EX_MemWrite <= ID_MemWrite;
        EX_MemToReg <= ID_MemToReg;
        EX_RegDst <= ID_RegDst;
        EX_ALUSrc1 <= ID_ALUSrc1;
        EX_ALUSrc2 <= ID_ALUSrc2;
        EX_Jal <= ID_Jal;
        EX_Ext_imm150 <= ID_Ext_imm150;
        EX_UnSignExt_imm106 <= ID_UnSignExt_imm106;
        EX_rdata1 <= ID_rdata1;
        EX_rdata2 <= ID_rdata2;
        EX_Div <= ID_Div;
        EX_DivSigned <= ID_DivSigned;
        EX_Mul <= ID_Mul;
        EX_MulSigned <= ID_MulSigned;
        EX_ReadHiReg <= ID_ReadHiReg;
        EX_ReadLoReg <= ID_ReadLoReg;
        EX_SpecialRegWri <= ID_SpecialRegWri;
        EX_SpecialRegRead <= ID_SpecialRegRead;
        EX_SpecialRegSel <= ID_SpecialRegSel;
    end
end

assign EX_LOVal = (EX_SpecialRegWri && ~EX_Div && EX_SpecialRegSel[0]) ? EX_rdata1 : EX_DivResS;
assign EX_HIVal = (EX_SpecialRegWri && ~EX_Div && EX_SpecialRegSel[1]) ? EX_rdata1 : EX_DivResR;

//数据选择
assign EX_srcA = EX_ALUSrc1 ? EX_rdata1 : EX_UnSignExt_imm106;
assign EX_srcB = EX_ALUSrc2 ? EX_rdata2 : EX_Ext_imm150;
assign EX_WriteData = EX_rdata2;
assign EX_WriteReg = EX_Jal ? 5'd31 : (EX_RegDst ? EX_rd : EX_rt); //jal�????要写31号寄存器
assign EX_RegWrite = (EX_WriteReg == 5'b0) ? 1'b0 : EX_OldRegWrite; //写寄存器0�???? 直接取消寄存器写使能 避免后续产生寄存器前�????

//出现例外直接取消写信�????
alu calculation(.ALUControl(EX_ALUControl), .alu_src1(EX_srcA), .alu_src2(EX_srcB), .alu_result(EX_aluResult), .Overflow(EX_Overflow));
div divider(.div_clk(clk), .resetn(resetn), .div(EX_Div), .div_signed(EX_DivSigned), .x(EX_rdata1), .y(EX_rdata2), .s(EX_DivResS), .r(EX_DivResR), .complete(EX_DivComplete));
mul muler(.mul_clk(clk), .resetn(resetn), .mul_signed(EX_MulSigned), .x(EX_rdata1), .y(EX_rdata2), .result(ME_MulRes));
//mult流水 跨执行阶段和访存阶段 模块内进行流�???? 在访存阶段得到结�????

//div产生阻塞信号
assign EX_Stall = EX_Div && ~EX_DivComplete;

assign data_sram_wen = {4{EX_MemWrite}};
assign data_sram_wdata = EX_WriteData;
assign data_sram_addr = EX_aluResult;

endmodule
