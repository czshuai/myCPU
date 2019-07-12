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
    input ID_WriReg31,
    input ID_WriPCPlus8,
    input ID_Stall,
    input [2:0] ID_MemDataWidth,
    input [1:0] ID_MemDataCombine,
    input ID_DelaySlot,
    input ID_ERET,

    input [4:0] ID_CP0Sel,
    input ID_CP0Wri,

    input ID_Div,
    input ID_DivSigned,
    input ID_Mul,
    input ID_MulSigned,

    input ID_SpecialRegWri,
    input ID_SpecialRegRead,
    input [1:0] ID_SpecialRegSel,
    input [31:0] ID_ReadSpecialReg,

    input ID_ExcepSYS,
    input ID_ExcepBP,
    input ID_InsExcepAdEL,
    input ID_ExcepRI,

    input WB_ExcepEN,

    output EX_allowin, EX_to_ME_valid,
    output reg EX_valid,

    output reg [31:0] EX_NextPC, EX_PC,

    output [4:0] EX_WriteReg,
    output EX_RegWrite,
    output [31:0] EX_aluResult,
    output reg EX_MemToReg,
    output reg EX_WriPCPlus8,
    output reg EX_MemWrite,
    output [31:0] EX_WriteData,
    output reg [2:0] EX_MemDataWidth,
    output reg [1:0] EX_MemDataCombine,
    output reg [31:0] EX_rdata2,
    output reg EX_DelaySlot,
    output reg EX_ERET,

    output [31:0] EX_LOVal, EX_HIVal,
    output reg EX_SpecialRegWri,
    output reg EX_SpecialRegRead,
    output reg [31:0] EX_ReadSpecialReg,
    output reg EX_Mul,
    output reg [1:0] EX_SpecialRegSel,

    output reg [4:0] EX_CP0Sel,
    output reg EX_CP0Wri,

    output reg EX_ExcepSYS,
    output reg EX_ExcepBP,
    output EX_ExcepOv,
    output EX_ExcepAdES,
    output reg EX_InsExcepAdEL,
    output EX_DataExcepAdEL,
    output reg EX_ExcepRI,
    output EX_Excep,

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
reg EX_Div;
reg EX_DivSigned; 
reg EX_MulSigned;
reg EX_WriReg31;
wire [31:0] EX_srcA;
wire [31:0] EX_srcB;
wire EX_DivComplete;
wire [31:0] EX_DivResS;
wire [31:0] EX_DivResR;
wire [31:0] EX_TrueWriteData;
wire [31:0] EX_TrueTrueWriteData;
wire [3:0] data_wen_sel; //选择写的字节
wire ExcepOv;

assign EX_ready_go = ~EX_Stall || WB_ExcepEN;
assign EX_allowin = !EX_valid || EX_ready_go && ME_allowin;
assign EX_to_ME_valid = EX_valid && EX_ready_go;

always @(posedge clk) begin
    if (~resetn) begin
        EX_valid <= 1'b0;
    end
    else if (EX_allowin) begin
        EX_valid <= ID_to_EX_valid && ~WB_ExcepEN;
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
        EX_WriReg31 <= ID_WriReg31;
        EX_WriPCPlus8 <= ID_WriPCPlus8;
        EX_Ext_imm150 <= ID_Ext_imm150;
        EX_UnSignExt_imm106 <= ID_UnSignExt_imm106;
        EX_rdata1 <= ID_rdata1;
        EX_rdata2 <= ID_rdata2;
        EX_Div <= ID_Div;
        EX_DivSigned <= ID_DivSigned;
        EX_Mul <= ID_Mul;
        EX_MulSigned <= ID_MulSigned;
        EX_ReadSpecialReg <= ID_ReadSpecialReg;
        EX_SpecialRegWri <= ID_SpecialRegWri;
        EX_SpecialRegRead <= ID_SpecialRegRead;
        EX_SpecialRegSel <= ID_SpecialRegSel;
        EX_MemDataWidth <= ID_MemDataWidth;
        EX_MemDataCombine <= ID_MemDataCombine;
        EX_CP0Sel <= ID_CP0Sel;
        EX_CP0Wri <= ID_CP0Wri;
        EX_ExcepSYS <= ID_ExcepSYS;
        EX_ExcepBP <= ID_ExcepBP;
        EX_InsExcepAdEL <= ID_InsExcepAdEL;
        EX_ExcepRI <= ID_ExcepRI;
        EX_DelaySlot <= ID_DelaySlot;
        EX_ERET <= ID_ERET;
    end
end

assign EX_LOVal = (EX_SpecialRegWri && ~EX_Div && EX_SpecialRegSel[0]) ? EX_rdata1 : EX_DivResS;
assign EX_HIVal = (EX_SpecialRegWri && ~EX_Div && EX_SpecialRegSel[1]) ? EX_rdata1 : EX_DivResR;

//数据选择
assign EX_srcA = EX_ALUSrc1 ? EX_rdata1 : EX_UnSignExt_imm106;
assign EX_srcB = EX_ALUSrc2 ? EX_rdata2 : EX_Ext_imm150;
assign EX_WriteData = EX_rdata2;
assign EX_WriteReg = EX_WriReg31 ? 5'd31 : (EX_RegDst ? EX_rd : EX_rt); //jal�??????要写31号寄存器
assign EX_RegWrite = (EX_WriteReg == 5'b0) ? 1'b0 : EX_OldRegWrite; //写寄存器0�?????? 直接取消寄存器写使能 避免后续产生寄存器前�??????

//出现例外直接取消写信�??????
alu calculation(.ALUControl(EX_ALUControl), .alu_src1(EX_srcA), .alu_src2(EX_srcB), .alu_result(EX_aluResult), .ExcepOv(ExcepOv));
div divider(.div_clk(clk), .resetn(resetn), .div(EX_Div), .div_signed(EX_DivSigned), .x(EX_rdata1), .y(EX_rdata2), .s(EX_DivResS), .r(EX_DivResR), .complete(EX_DivComplete));
mul muler(.mul_clk(clk), .resetn(resetn), .mul_signed(EX_MulSigned), .x(EX_rdata1), .y(EX_rdata2), .result(ME_MulRes));

//div产生阻塞信号
assign EX_Stall = EX_Div && ~EX_DivComplete;
//mult流水 跨执行阶段和访存阶段 模块内进行流�?????? 在访存阶段得到结�??????

assign data_wen_sel = ({4{(EX_MemDataWidth == 3'b001 && EX_aluResult[1:0] == 2'b0)}} & {3'b0, EX_MemWrite}) |
                      ({4{(EX_MemDataWidth == 3'b001 && EX_aluResult[1:0] == 2'b1)}} & {2'b0, EX_MemWrite, 1'b0}) |
                      ({4{(EX_MemDataWidth == 3'b001 && EX_aluResult[1:0] == 2'b10)}} & {1'b0, EX_MemWrite, 2'b0}) |
                      ({4{(EX_MemDataWidth == 3'b001 && EX_aluResult[1:0] == 2'b11)}} & {EX_MemWrite, 3'b0}) |
                      ({4{(EX_MemDataWidth == 3'b011 && EX_aluResult[1:0] == 2'b0)}} & {2'b0, {2{EX_MemWrite}}}) |
                      ({4{(EX_MemDataWidth == 3'b011 && EX_aluResult[1:0] == 2'b10)}} & {{2{EX_MemWrite}}, 2'b0}) |
                      ({4{(EX_MemDataWidth == 3'b101) && (EX_MemDataCombine == 2'b0)}} & {4{EX_MemWrite}}) |
                      ({4{(EX_MemDataWidth == 3'b101) && (EX_MemDataCombine == 2'b01) && (EX_aluResult[1:0] == 2'b0)}} & {3'b0, EX_MemWrite}) |
                      ({4{(EX_MemDataWidth == 3'b101) && (EX_MemDataCombine == 2'b01) && (EX_aluResult[1:0] == 2'b1)}} & {2'b0, {2{EX_MemWrite}}}) |
                      ({4{(EX_MemDataWidth == 3'b101) && (EX_MemDataCombine == 2'b01) && (EX_aluResult[1:0] == 2'b10)}} & {1'b0, {3{EX_MemWrite}}}) |
                      ({4{(EX_MemDataWidth == 3'b101) && (EX_MemDataCombine == 2'b01) && (EX_aluResult[1:0] == 2'b11)}} & {4{EX_MemWrite}}) |
                      ({4{(EX_MemDataWidth == 3'b101) && (EX_MemDataCombine == 2'b10) && (EX_aluResult[1:0] == 2'b0)}} & {4{EX_MemWrite}}) |
                      ({4{(EX_MemDataWidth == 3'b101) && (EX_MemDataCombine == 2'b10) && (EX_aluResult[1:0] == 2'b1)}} & {{3{EX_MemWrite}}, 1'b0}) |
                      ({4{(EX_MemDataWidth == 3'b101) && (EX_MemDataCombine == 2'b10) && (EX_aluResult[1:0] == 2'b10)}} & {{2{EX_MemWrite}}, 2'b0}) |
                      ({4{(EX_MemDataWidth == 3'b101) && (EX_MemDataCombine == 2'b10) && (EX_aluResult[1:0] == 2'b11)}} & {{1{EX_MemWrite}}, 3'b0});
                      

wire [7:0] regData [3:0];
wire [31:0] SWLRes;
wire [31:0] SWRRes;
wire BadAddr; //表示地址不对�?

assign regData[0] = EX_rdata2[7:0];
assign regData[1] = EX_rdata2[15:8];
assign regData[2] = EX_rdata2[23:16];
assign regData[3] = EX_rdata2[31:24];

assign EX_TrueWriteData = ({32{(EX_MemDataWidth == 3'b001 && EX_aluResult[1:0] == 2'b0)}} & {24'b0, regData[0]}) | 
                          ({32{(EX_MemDataWidth == 3'b001 && EX_aluResult[1:0] == 2'b1)}} & {16'b0, regData[0], 8'b0}) | 
                          ({32{(EX_MemDataWidth == 3'b001 && EX_aluResult[1:0] == 2'b10)}} & {8'b0, regData[0], 16'b0}) |
                          ({32{(EX_MemDataWidth == 3'b001 && EX_aluResult[1:0] == 2'b11)}} & {regData[0], 24'b0}) |
                          ({32{(EX_MemDataWidth == 3'b011 && EX_aluResult[1:0] == 2'b0)}} & {16'b0, regData[1], regData[0]}) |
                          ({32{(EX_MemDataWidth == 3'b011 && EX_aluResult[1:0] == 2'b10)}} & {regData[1], regData[0], 16'b0}) |
                          ({32{(EX_MemDataWidth == 3'b101)}} & EX_WriteData);

assign SWLRes = {32{(EX_aluResult[1:0] == 2'b0)}} & {24'b0 , regData[3]} |
                {32{(EX_aluResult[1:0] == 2'b1)}} & {16'b0, regData[3], regData[2]} |
                {32{(EX_aluResult[1:0] == 2'b10)}} & {8'b0, regData[3], regData[2], regData[1]} |
                {32{(EX_aluResult[1:0] == 2'b11)}} & {regData[3], regData[2], regData[1], regData[0]};

assign SWRRes = {32{(EX_aluResult[1:0] == 2'b0)}} &  {regData[3], regData[2], regData[1], regData[0]} |
                {32{(EX_aluResult[1:0] == 2'b1)}} & {regData[2], regData[1], regData[0], 8'b0} |
                {32{(EX_aluResult[1:0] == 2'b10)}} & {regData[1], regData[0], 16'b0} |
                {32{(EX_aluResult[1:0] == 2'b11)}} & {regData[0], 24'b0};

assign EX_TrueTrueWriteData = {32{(EX_MemDataCombine == 2'b01)}} & SWLRes | 
                          {32{(EX_MemDataCombine == 2'b10)}} & SWRRes |
                          {32{(EX_MemDataCombine == 2'b0)}} & EX_TrueWriteData;

assign BadAddr = (EX_MemDataCombine == 2'b0) && (((EX_MemDataWidth == 3'b011 || EX_MemDataWidth == 3'b100) && EX_aluResult[0] != 1'b0) || (EX_MemDataWidth == 3'b101 && EX_aluResult[1:0] != 2'b0));
assign EX_ExcepAdES = EX_MemWrite && BadAddr && EX_valid;
assign EX_DataExcepAdEL = EX_MemToReg && BadAddr && EX_valid;
assign EX_ExcepOv = ExcepOv && EX_valid;

assign data_sram_wen = data_wen_sel; 
assign data_sram_wdata = EX_TrueTrueWriteData;
assign data_sram_addr = EX_aluResult;

assign EX_Excep = EX_ExcepBP || EX_ExcepOv || EX_ExcepSYS || EX_ExcepAdES || EX_ExcepRI || EX_InsExcepAdEL || EX_DataExcepAdEL;

endmodule
