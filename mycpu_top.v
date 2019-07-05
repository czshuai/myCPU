`timescale 1ns / 1ps

module mycpu_top(
    //clock and reset
    input clk, resetn,
    //instruction
    output inst_sram_en,
    output [3:0] inst_sram_wen,
    output [31:0] inst_sram_addr, inst_sram_wdata,
    input [31:0] inst_sram_rdata,
    //data
    output data_sram_en,    
    output [3:0] data_sram_wen,
    output [31:0] data_sram_addr, data_sram_wdata,
    input [31:0] data_sram_rdata,
    //debug
    output [31:0] debug_wb_pc, debug_wb_rf_wdata,
    output [3:0] debug_wb_rf_wen,
    output [4:0] debug_wb_rf_wnum
    );

assign inst_sram_wen = 4'b0;

wire valid_in;
wire IF_ready_go, IF_allowin, IF_to_ID_valid;
reg IF_valid;

//ID parameter
wire ID_allowin, ID_to_EX_valid;
wire ID_valid;
wire ID_PCSrc; //是否跳转 1 跳转
wire [31:0] ID_PCBranch; //跳转PC地址
wire [31:0] ID_NextPC;
wire [31:0] ID_PC;
wire [4:0] ID_rs;
wire [4:0] ID_rt;
wire [4:0] ID_rd;
wire [31:0] ID_UnSignExt_imm106;
wire [31:0] ID_Ext_imm150;
wire [31:0] ID_rdata1;
wire [31:0] ID_rdata2;
wire [4:0] ID_ALUControl; //alu计算
wire [25:0] ID_InsIdx; //jal的instr_index
wire ID_RegWrite; //控制信号 是否写回reg
wire ID_MemWrite; //是否写Mem
wire ID_MemToReg; //是否将内存读结果写入reg
wire ID_RegDst; //目的寄存器sel
wire ID_ALUSrc1; //选择alusource1的val
wire ID_ALUSrc2; //选择alusource2的val
wire ID_WriReg31; //是否直接选择31号寄存器
wire ID_WriPCPlus8; //是否直接保存 PC+8 作用在分支指令中
wire ID_Stall; //阻塞译码阶段 *********************************************
wire ID_Div; //除法使能信号
wire ID_DivSigned; //是否为有符号除法
wire ID_Mul; //乘法使能信号
wire ID_MulSigned; //是否为有符号乘法
wire ID_SpecialRegWri; //是否写特殊寄存器
wire ID_SpecialRegRead; //是否读特殊寄存器
wire [1:0] ID_SpecialRegSel; //选择HI或�?�LO寄存�????? 01选择lo 10选择hi
wire [31:0] ID_ReadHiReg; //读取的特殊寄存器的�??
wire [31:0] ID_ReadLoReg;
wire [2:0] ID_MemDataWidth; //内存数据宽度
wire [1:0] ID_MemDataCombine; //拼合内存数据

//EX Parameter
wire EX_allowin, EX_to_ME_valid;
wire EX_valid;
wire EX_RegWrite;
wire [4:0] EX_WriteReg;
wire [31:0] EX_NextPC;
wire [31:0] EX_PC;
wire EX_MemWrite;
wire EX_MemToReg;
wire EX_WriPCPlus8;
wire EX_Mul;
wire EX_SpecialRegWri;
wire EX_SpecialRegRead;
wire [1:0] EX_SpecialRegSel;
wire [31:0] EX_ReadHiReg;
wire [31:0] EX_ReadLoReg;
wire [31:0] EX_WriteData;
wire [31:0] EX_aluResult;
wire [31:0] EX_LOVal; //保存LO寄存器的�?????
wire [31:0] EX_HIVal; //保存HI寄存器的�?????
wire [2:0] EX_MemDataWidth;
wire [1:0] EX_MemDataCombine;
wire [31:0] EX_rdata2;

//ME Parameter
wire ME_ready_go, ME_allowin, ME_to_WB_valid;
reg ME_valid;
wire [31:0] ME_FinalData; //寄存器写入�??
reg [4:0] ME_WriteReg; //写入寄存器号
reg ME_RegWrite; //寄存器写使能
wire [63:0] ME_MulRes; //mul计算结果
reg ME_SpecialRegWri;
reg ME_SpecialRegRead;
wire [31:0] ME_readData;
wire [31:0] ME_LOVal;
wire [31:0] ME_HIVal;
reg [1:0] ME_SpecialRegSel;

//WB parameter
wire WB_ready_go, WB_allowin;
reg WB_valid;
wire [31:0] WB_FinalData;
reg [4:0] WB_WriteReg; //写入寄存器号
reg WB_RegWrite; //寄存器写使能
reg [31:0] WB_readData;
reg [31:0] WB_LOVal;
reg [31:0] WB_HIVal;
reg WB_SpecialRegWri;
reg [1:0] WB_SpecialRegSel;

wire [2:0] ForwardA; //srcA 前�??
wire [2:0] ForwardB; //srcB

//IF
reg [31:0] PC;
wire [31:0] next_PC;
wire [31:0] ins_reg;

assign valid_in = resetn;
assign IF_allowin = !IF_valid || IF_ready_go && ID_allowin;
assign IF_ready_go = valid_in;
assign IF_to_ID_valid = IF_valid && IF_ready_go;
assign inst_sram_wen = 4'b0;
assign inst_sram_en = IF_allowin;
assign next_PC = ID_PCSrc ? ID_PCBranch : (PC + 32'd4);
assign inst_sram_addr = next_PC;
assign ins_reg = inst_sram_rdata;

always @(posedge clk) begin
    if (~resetn) begin
        IF_valid <= 1'b0;
        PC <= 32'hbfbffffc;
    end
    else if (IF_allowin) begin
        IF_valid <= valid_in;
        PC <= next_PC;
    end
end

//ID
IDStage IDInterface(
    .clk(clk),
    .resetn(resetn),

    .next_PC(next_PC),
    .PC(PC),
    .ins_reg(ins_reg),

    .ID_Stall(ID_Stall),
    .EX_allowin(EX_allowin),
    .IF_to_ID_valid(IF_to_ID_valid),

    .ME_FinalData(ME_FinalData),

    .WB_RegWrite(WB_RegWrite),
    .WB_valid(WB_valid),
    .WB_WriteReg(WB_WriteReg),
    .WB_FinalData(WB_FinalData),
    .WB_SpecialRegWri(WB_SpecialRegWri),
    .WB_SpecialRegSel(WB_SpecialRegSel),
    .WB_HIVal(WB_HIVal),
    .WB_LOVal(WB_LOVal),

    .ForwardA(ForwardA),
    .ForwardB(ForwardB),

    .ID_valid(ID_valid),
    .ID_allowin(ID_allowin),
    .ID_to_EX_valid(ID_to_EX_valid),

    .ID_NextPC(ID_NextPC),
    .ID_PC(ID_PC),

    .ID_rs(ID_rs),
    .ID_rt(ID_rt),
    .ID_rd(ID_rd),

    .ID_Ext_imm150(ID_Ext_imm150),
    .ID_UnSignExt_imm106(ID_UnSignExt_imm106),

    .ID_PCBranch(ID_PCBranch),
    .ID_PCSrc(ID_PCSrc),

    .ID_ALUControl(ID_ALUControl),
    .ID_InsIdx(ID_InsIdx),
    .ID_RegWrite(ID_RegWrite),
    .ID_MemWrite(ID_MemWrite),
    .ID_MemToReg(ID_MemToReg),
    .ID_RegDst(ID_RegDst),
    .ID_ALUSrc1(ID_ALUSrc1),
    .ID_ALUSrc2(ID_ALUSrc2),
    .ID_WriReg31(ID_WriReg31),
    .ID_WriPCPlus8(ID_WriPCPlus8),
    .ID_rdata1(ID_rdata1),
    .ID_rdata2(ID_rdata2),
    .ID_Div(ID_Div),
    .ID_DivSigned(ID_DivSigned),
    .ID_Mul(ID_Mul),
    .ID_MulSigned(ID_MulSigned),
    .ID_ReadHiReg(ID_ReadHiReg),
    .ID_ReadLoReg(ID_ReadLoReg),
    .ID_SpecialRegWri(ID_SpecialRegWri),
    .ID_SpecialRegRead(ID_SpecialRegRead),
    .ID_SpecialRegSel(ID_SpecialRegSel),
    .ID_MemDataWidth(ID_MemDataWidth),
    .ID_MemDataCombine(ID_MemDataCombine)
);

//EX
EXStage EXInterface (
    .clk(clk),
    .resetn(resetn),

    .ID_to_EX_valid(ID_to_EX_valid),

    .ME_allowin(ME_allowin),

    .ID_NextPC(ID_NextPC),
    .ID_PC(ID_PC),

    .ID_rs(ID_rs),
    .ID_rt(ID_rt),
    .ID_rd(ID_rd),

    .ID_UnSignExt_imm106(ID_UnSignExt_imm106),
    .ID_Ext_imm150(ID_Ext_imm150),

    .ID_rdata1(ID_rdata1),
    .ID_rdata2(ID_rdata2),

    .ID_ALUControl(ID_ALUControl),
    .ID_RegWrite(ID_RegWrite),
    .ID_MemWrite(ID_MemWrite),
    .ID_MemToReg(ID_MemToReg),
    .ID_RegDst(ID_RegDst),
    .ID_ALUSrc1(ID_ALUSrc1),
    .ID_ALUSrc2(ID_ALUSrc2),
    .ID_WriReg31(ID_WriReg31),
    .ID_WriPCPlus8(ID_WriPCPlus8),
    .ID_Stall(ID_Stall),
    .ID_MemDataWidth(ID_MemDataWidth),
    .ID_MemDataCombine(ID_MemDataCombine),

    .ID_Div(ID_Div),
    .ID_DivSigned(ID_DivSigned),
    .ID_Mul(ID_Mul),
    .ID_MulSigned(ID_MulSigned),
    
    .ID_SpecialRegWri(ID_SpecialRegWri),
    .ID_SpecialRegRead(ID_SpecialRegRead),
    .ID_SpecialRegSel(ID_SpecialRegSel),
    .ID_ReadHiReg(ID_ReadHiReg),
    .ID_ReadLoReg(ID_ReadLoReg),

    .EX_allowin(EX_allowin),
    .EX_to_ME_valid(EX_to_ME_valid),
    .EX_valid(EX_valid),

    .EX_NextPC(EX_NextPC),
    .EX_PC(EX_PC),

    .EX_WriteReg(EX_WriteReg),
    .EX_RegWrite(EX_RegWrite),
    .EX_aluResult(EX_aluResult),
    .EX_MemToReg(EX_MemToReg),
    .EX_WriPCPlus8(EX_WriPCPlus8),
    .EX_MemWrite(EX_MemWrite),
    .EX_WriteData(EX_WriteData),
    .EX_MemDataWidth(EX_MemDataWidth),
    .EX_MemDataCombine(EX_MemDataCombine),
    .EX_rdata2(EX_rdata2),
    
    .EX_LOVal(EX_LOVal),
    .EX_HIVal(EX_HIVal),
    .EX_SpecialRegWri(EX_SpecialRegWri),
    .EX_SpecialRegRead(EX_SpecialRegRead),
    .EX_ReadHiReg(EX_ReadHiReg),
    .EX_ReadLoReg(EX_ReadLoReg),
    .EX_Mul(EX_Mul),
    .EX_SpecialRegSel(EX_SpecialRegSel),

    .data_sram_wen(data_sram_wen),
    .data_sram_addr(data_sram_addr),
    .data_sram_wdata(data_sram_wdata),
    
    .ME_MulRes(ME_MulRes)
);

//ME
reg [31:0] ME_NextPC;
reg [31:0] ME_PC;
reg ME_MemToReg;
reg ME_WriPCPlus8;
reg [31:0] ME_aluResult;
reg ME_MemWrite;
reg [31:0] ME_WriteData;
reg [31:0] ME_OldLOVal;
reg [31:0] ME_OldHIVal;
//reg [1:0] ME_SpecialRegSel;
reg [31:0] ME_ReadHiReg;
reg [31:0] ME_ReadLoReg;
//reg ME_SpecialRegWri;
//reg ME_SpecialRegRead
reg ME_Mul;
reg [2:0] ME_MemDataWidth;
reg [1:0] ME_MemDataCombine;
reg [31:0] ME_rdata2;
//wire [31:0] ME_LOVal;
//wire [31:0] ME_HIVal;
//wire [63:0] ME_MulRes; //mul计算结果
//wire [31:0] ME_FinalData; //寄存器写入�??
//reg [4:0] ME_WriteReg; //写入寄存器号
//reg ME_RegWrite; //寄存器写使能
wire [31:0] ME_ReadSpecialReg;

assign ME_ready_go = 1'b1;
assign ME_allowin = !ME_valid || ME_ready_go && WB_allowin;
assign ME_to_WB_valid = ME_valid && ME_ready_go;

assign data_sram_en = EX_valid; //同步RAM 上一拍输入， 下一拍得到结�?????
assign ME_readData = data_sram_rdata;

always @(posedge clk) begin
    if (~resetn) begin
        ME_valid <= 1'b0;
    end
    else if (ME_allowin) begin
        ME_valid <= EX_to_ME_valid;
    end

    if (EX_to_ME_valid && ME_allowin) begin
        ME_NextPC <= EX_NextPC;
        ME_PC <= EX_PC;
        ME_WriteReg <= EX_WriteReg;
        ME_RegWrite <= EX_RegWrite;
        ME_aluResult <= EX_aluResult;
        ME_MemToReg <= EX_MemToReg;
        ME_WriPCPlus8 <= EX_WriPCPlus8;
        ME_MemWrite <= EX_MemWrite;
        ME_WriteData <= EX_WriteData;
        ME_OldLOVal <= EX_LOVal;
        ME_OldHIVal <= EX_HIVal;
        ME_SpecialRegWri <= EX_SpecialRegWri;
        ME_SpecialRegRead <= EX_SpecialRegRead;
        ME_ReadHiReg <= EX_ReadHiReg;
        ME_ReadLoReg <= EX_ReadLoReg;
        ME_Mul <= EX_Mul;
        ME_SpecialRegSel <= EX_SpecialRegSel;
        ME_MemDataWidth <= EX_MemDataWidth;
        ME_MemDataCombine <= EX_MemDataCombine;
        ME_rdata2 <= EX_rdata2;
    end
end

assign ME_ReadSpecialReg = {32{ME_SpecialRegSel[0]}} & ME_ReadLoReg |
                           {32{ME_SpecialRegSel[1]}} & ME_ReadHiReg;
                           
assign ME_FinalData = ME_SpecialRegRead ? ME_ReadSpecialReg : (ME_WriPCPlus8 ? (ME_NextPC + 32'd4) : ME_aluResult); //jal特殊处理

//乘法数据处理
assign ME_LOVal = ME_Mul ? ME_MulRes[31:0] : ME_OldLOVal;
assign ME_HIVal = ME_Mul ? ME_MulRes[63:32] : ME_OldHIVal;

//WB
reg [31:0] WB_NextPC;
reg [31:0] WB_PC;
reg WB_MemToReg;
reg [31:0] WB_aluResult;
reg [31:0] WB_OldFinalData;
reg [2:0] WB_MemDataWidth;
reg [1:0] WB_MemDataCombine;
reg [31:0] WB_rdata2;
//reg [31:0] WB_LOVal;
//reg [31:0] WB_HIVal;
//reg WB_SpecialRegWri;
//reg [1:0] WB_SpecialRegSel;
//reg WB_RegWrite;
//reg [4:0] WB_WriteReg;
//wire [31:0] WB_FinalData;
//reg [31:0] WB_readData;
wire [31:0] WB_TrueReadData;
wire [31:0] WB_TrueTrueReadData;

assign WB_ready_go = 1'b1;
assign WB_allowin = 1'b1;

always @(posedge clk) begin
    if (~resetn) begin
        WB_valid <= 1'b0;
    end
    else if (WB_allowin) begin
        WB_valid <= ME_to_WB_valid;
    end

    if (ME_to_WB_valid && WB_allowin) begin
        WB_NextPC <= ME_NextPC;
        WB_PC <= ME_PC;
        WB_WriteReg <= ME_WriteReg;
        WB_RegWrite <= ME_RegWrite;
        WB_MemToReg <= ME_MemToReg;
        WB_aluResult <= ME_aluResult;
        WB_OldFinalData <= ME_FinalData;
        WB_LOVal <= ME_LOVal;
        WB_HIVal <= ME_HIVal;
        WB_SpecialRegWri <= ME_SpecialRegWri;
        WB_SpecialRegSel <= ME_SpecialRegSel;
        WB_readData <= ME_readData;
        WB_MemDataWidth <= ME_MemDataWidth;
        WB_MemDataCombine <= ME_MemDataCombine;
        WB_rdata2 <= ME_rdata2;
    end
end

//assign WB_FinalData = ME_WriPCPlus8 ? (WB_NextPC + 32'd4) : (WB_MemToReg ? WB_readData : WB_aluResult);
assign WB_TrueReadData = ({32{(WB_MemDataWidth == 3'b001 && WB_aluResult[1:0] == 2'b0)}} & {{24{WB_readData[7]}}, WB_readData[7:0]}) | 
                         ({32{(WB_MemDataWidth == 3'b001 && WB_aluResult[1:0] == 2'b1)}} & {{24{WB_readData[15]}}, WB_readData[15:8]}) | 
                         ({32{(WB_MemDataWidth == 3'b001 && WB_aluResult[1:0] == 2'b10)}} & {{24{WB_readData[23]}}, WB_readData[23:16]}) |
                         ({32{(WB_MemDataWidth == 3'b001 && WB_aluResult[1:0] == 2'b11)}} & {{24{WB_readData[31]}}, WB_readData[31:24]}) |
                         ({32{(WB_MemDataWidth == 3'b010 && WB_aluResult[1:0] == 2'b0)}} & {24'b0, WB_readData[7:0]}) | 
                         ({32{(WB_MemDataWidth == 3'b010 && WB_aluResult[1:0] == 2'b1)}} & {24'b0, WB_readData[15:8]}) | 
                         ({32{(WB_MemDataWidth == 3'b010 && WB_aluResult[1:0] == 2'b10)}} & {24'b0, WB_readData[23:16]}) |
                         ({32{(WB_MemDataWidth == 3'b010 && WB_aluResult[1:0] == 2'b11)}} & {24'b0, WB_readData[31:24]}) |                     
                         ({32{(WB_MemDataWidth == 3'b011 && WB_aluResult[1:0] == 2'b0)}} & {{16{WB_readData[15]}}, WB_readData[15:0]}) |
                         ({32{(WB_MemDataWidth == 3'b011 && WB_aluResult[1:0] == 2'b10)}} & {{16{WB_readData[31]}}, WB_readData[31:16]}) |
                         ({32{(WB_MemDataWidth == 3'b100 && WB_aluResult[1:0] == 2'b0)}} & {16'b0, WB_readData[15:0]}) |
                         ({32{(WB_MemDataWidth == 3'b100 && WB_aluResult[1:0] == 2'b10)}} & {16'b0, WB_readData[31:16]}) |
                         ({32{(WB_MemDataWidth == 3'b101)}} & WB_readData);

wire [7:0] MemData [3:0];
wire [7:0] regData [3:0];
wire [31:0] LWLRes;
wire [31:0] LWRRes;

assign regData[0] = WB_rdata2[7:0];
assign regData[1] = WB_rdata2[15:8];
assign regData[2] = WB_rdata2[23:16];
assign regData[3] = WB_rdata2[31:24];

assign MemData[0] = WB_TrueReadData[7:0];
assign MemData[1] = WB_TrueReadData[15:8];
assign MemData[2] = WB_TrueReadData[23:16];
assign MemData[3] = WB_TrueReadData[31:24];

assign LWLRes = {32{(WB_aluResult[1:0] == 2'b0)}} & {MemData[0], regData[2], regData[1], regData[0]} |
                {32{(WB_aluResult[1:0] == 2'b1)}} & {MemData[1], MemData[0], regData[1], regData[0]} |
                {32{(WB_aluResult[1:0] == 2'b10)}} & {MemData[2], MemData[1], MemData[0], regData[0]} |
                {32{(WB_aluResult[1:0] == 2'b11)}} & {MemData[3], MemData[2], MemData[1], MemData[0]};

assign LWRRes = {32{(WB_aluResult[1:0] == 2'b0)}} & {MemData[3], MemData[2], MemData[1], MemData[0]} |
                {32{(WB_aluResult[1:0] == 2'b1)}} & {regData[3], MemData[3], MemData[2], MemData[1]} |
                {32{(WB_aluResult[1:0] == 2'b10)}} & {regData[3], regData[2], MemData[3], MemData[2]} |
                {32{(WB_aluResult[1:0] == 2'b11)}} & {regData[3], regData[2], regData[1], MemData[3]};
            

assign WB_TrueTrueReadData = {32{(WB_MemDataCombine == 2'b01)}} & LWLRes | 
                             {32{(WB_MemDataCombine == 2'b10)}} & LWRRes |
                             {32{(WB_MemDataCombine == 2'b0)}} & WB_TrueReadData;

assign WB_FinalData = WB_MemToReg ? WB_TrueTrueReadData : WB_OldFinalData;

assign debug_wb_pc = WB_PC;
assign debug_wb_rf_wen = {4{WB_RegWrite && WB_valid}};
assign debug_wb_rf_wnum = WB_WriteReg;
assign debug_wb_rf_wdata = WB_FinalData;

//冲突�?????测单�?????
//wire [2:0] ForwardA;
//wire [2:0] ForwardB;

assign ID_Stall = (ForwardA[0] || ForwardB[0]) || ((ForwardA[1] || ForwardB[1]) && ME_MemToReg) || 
                  (ID_valid && ID_SpecialRegRead && EX_SpecialRegWri && EX_valid && ((ID_SpecialRegSel[0] && EX_SpecialRegSel[0]) || (ID_SpecialRegSel[1] && EX_SpecialRegSel[1]))) ||
                  (ID_valid && ID_SpecialRegRead && ME_SpecialRegWri && ME_valid && ((ID_SpecialRegSel[0] && ME_SpecialRegSel[0]) || (ID_SpecialRegSel[1] && ME_SpecialRegSel[1])));

assign ForwardA[0] = (ID_rs == EX_WriteReg) && EX_valid && EX_RegWrite;
assign ForwardA[1] = (ID_rs == ME_WriteReg) && ME_valid && ME_RegWrite;
assign ForwardA[2] = (ID_rs == WB_WriteReg) && WB_valid && WB_RegWrite;
assign ForwardB[0] = (ID_rt == EX_WriteReg) && EX_valid && EX_RegWrite;
assign ForwardB[1] = (ID_rt == ME_WriteReg) && ME_valid && ME_RegWrite;
assign ForwardB[2] = (ID_rt == WB_WriteReg) && WB_valid && WB_RegWrite;

endmodule



