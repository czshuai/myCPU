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

reg [31:0] CP0 [15:0]; //CP0[rt/rd, sel]  sel始终为零

wire InsExcepAdEL;
wire valid_in;
wire IF_ready_go, IF_allowin, IF_to_ID_valid;
reg IF_valid;
reg IF_InsExcepAdEL; //记录取�?�是否产生例�???
wire IF_DelaySlot; //判断当前指令是否在延迟槽�??

//ID parameter
wire ID_allowin, ID_to_EX_valid;
wire ID_valid;
wire ID_PCSrc; //是否跳转 1 跳转
wire [31:0] ID_PCBranch; //跳转PC地址
wire ID_Branch; //是否为分支跳转ins
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
wire [31:0] ID_ReadSpecialReg; //特殊寄存器的�???
wire [1:0] ID_SpecialRegSel; //选择HI或�?�LO寄存�???????? 01选择lo 10选择hi
wire [2:0] ID_MemDataWidth; //内存数据宽度
wire [1:0] ID_MemDataCombine; //拼合内存数据
wire [4:0] ID_CP0Sel;
wire ID_CP0Wri;
wire ID_ExcepSYS;
wire ID_ExcepBP;
wire ID_InsExcepAdEL;
wire ID_ExcepRI;
wire ID_DelaySlot;
wire ID_ERET;

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
wire [31:0] EX_ReadSpecialReg;
wire [31:0] EX_WriteData;
wire [31:0] EX_aluResult;
wire [31:0] EX_LOVal; //保存LO寄存器的�????????
wire [31:0] EX_HIVal; //保存HI寄存器的�????????
wire [2:0] EX_MemDataWidth;
wire [1:0] EX_MemDataCombine;
wire [31:0] EX_rdata2;
wire [4:0] EX_CP0Sel;
wire EX_CP0Wri;
wire EX_ExcepSYS;
wire EX_ExcepBP;
wire EX_ExcepOv;
wire EX_ExcepAdES;
wire EX_InsExcepAdEL;
wire EX_DataExcepAdEL;
wire EX_ExcepRI;
wire EX_DelaySlot;
wire EX_Excep; //EX阶段是否出现例外
wire EX_ERET;

//ME Parameter
wire ME_ready_go, ME_allowin, ME_to_WB_valid;
reg ME_valid;
wire [31:0] ME_readData;
wire [31:0] ME_LOVal;
wire [31:0] ME_HIVal;
wire [63:0] ME_MulRes; //mul计算结果
wire [31:0] ME_FinalData; //寄存器写入�??
wire ME_Excep;
reg [31:0] ME_NextPC;
reg [31:0] ME_PC;
reg ME_MemToReg;
reg ME_WriPCPlus8;
reg [31:0] ME_aluResult;
reg ME_MemWrite;
reg [31:0] ME_WriteData;
reg [31:0] ME_OldLOVal;
reg [31:0] ME_OldHIVal;
reg [1:0] ME_SpecialRegSel;
reg [31:0] ME_ReadHiReg;
reg [31:0] ME_ReadLoReg;
reg ME_SpecialRegWri;
reg ME_SpecialRegRead
reg ME_Mul;
reg [2:0] ME_MemDataWidth;
reg [1:0] ME_MemDataCombine;
reg [31:0] ME_rdata2;
reg [4:0] ME_WriteReg; //写入寄存器号
reg ME_RegWrite; //寄存器写使能
reg [31:0] ME_ReadSpecialReg;
reg [4:0] ME_CP0Sel;
reg ME_CP0Wri;
reg ME_ExcepSYS;
reg ME_ExcepBP;
reg ME_ExcepOv;
reg ME_ExcepAdES;
reg ME_InsExcepAdEL;
reg ME_DataExcepAdEL;
reg ME_ExcepRI;
reg ME_DelaySlot;
reg ME_ERET;

//WB parameter
wire WB_ready_go, WB_allowin;
wire WB_CP0SpecWri; //产生例外时对CP0进行�???
wire WB_ExcepEN; //出现例外则无效五级流�???
wire [31:0] WB_FinalData;
wire [31:0] WB_TrueReadData;
wire [31:0] WB_TrueTrueReadData;
wire WB_ExcepInt; //中断例外
reg WB_valid;
reg [31:0] WB_NextPC;
reg [31:0] WB_PC;
reg WB_MemToReg;
reg [31:0] WB_aluResult;
reg [31:0] WB_OldFinalData;
reg [2:0] WB_MemDataWidth;
reg [1:0] WB_MemDataCombine;
reg [31:0] WB_rdata2;
reg [31:0] WB_LOVal;
reg [31:0] WB_HIVal;
reg WB_SpecialRegWri;
reg [1:0] WB_SpecialRegSel;
reg WB_RegWrite;//寄存器写使能
reg [4:0] WB_WriteReg;
reg [31:0] WB_readData;
reg [4:0] WB_CP0Sel;
reg WB_CP0Wri;
reg WB_ExcepSYS;
reg WB_ExcepBP;
reg WB_ExcepOv;
reg WB_ExcepAdES;
reg WB_InsExcepAdEL;
reg WB_DataExcepAdEL;
reg WB_ExcepRI;
reg WB_DelaySlot;
reg WB_ERET;

wire [2:0] ForwardA; //srcA 前�??
wire [2:0] ForwardB; //srcB
wire [2:0] ForwardCP0; //CP0寄存器的前递

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
assign next_PC = (WB_ExcepEN) ? 32'hbfc00380 : (ID_PCSrc ? ID_PCBranch : (PC + 32'd4));
assign inst_sram_addr = next_PC;
assign ins_reg = inst_sram_rdata;
assign InsExcepAdEL = ~(next_PC[1:0] == 2'b0);

always @(posedge clk) begin
    if (~resetn) begin
        IF_valid <= 1'b0;
        PC <= 32'hbfbffffc;
    end
    else if (IF_allowin) begin
        IF_valid <= valid_in;
        PC <= next_PC;
        IF_InsExcepAdEL <= InsExcepAdEL;
    end
end

assign IF_DelaySlot = ID_Branch;

//ID
IDStage IDInterface(
    .clk(clk),
    .resetn(resetn),

    .next_PC(next_PC),
    .PC(PC),
    .ins_reg(ins_reg),
    .IF_InsExcepAdEL(IF_InsExcepAdEL),
    .IF_DelaySlot(IF_DelaySlot),

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
    .WB_ExcepEN(WB_ExcepEN),
    .WB_ExcepInt(WB_ExcepInt),
    .WB_DelaySlot(WB_DelaySlot),
    .ME_InsExcepAdEL(ME_InsExcepAdEL),

    .ForwardA(ForwardA),
    .ForwardB(ForwardB),

    .CP0BadVAddr(CP0[8]),
    .CP0Count(CP0[9]),
    .CP0Compare(CP0[11]),
    .CP0Status(CP0[12]),
    .CP0Cause(CP0[13]),
    .CP0EPC(CP0[14]),
    
    .ForwardCP0(ForwardCP0),
    .EX_rdata2(EX_rdata2),
    .ME_rdata2(ME_rdata2),
    .WB_rdata2(WB_rdata2),

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
    .ID_Branch(ID_Branch),

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
    .ID_SpecialRegWri(ID_SpecialRegWri),
    .ID_SpecialRegRead(ID_SpecialRegRead),
    .ID_ReadSpecialReg(ID_ReadSpecialReg),
    .ID_SpecialRegSel(ID_SpecialRegSel),
    .ID_MemDataWidth(ID_MemDataWidth),
    .ID_MemDataCombine(ID_MemDataCombine),
    .ID_DelaySlot(ID_DelaySlot),
    .ID_ERET(ID_ERET),

    .ID_CP0Sel(ID_CP0Sel),
    .ID_CP0Wri(ID_CP0Wri),

    .ID_ExcepSYS(ID_ExcepSYS),
    .ID_ExcepBP(ID_ExcepBP),
    .ID_InsExcepAdEL(ID_InsExcepAdEL),
    .ID_ExcepRI(ID_ExcepRI)
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
    .ID_DelaySlot(ID_DelaySlot),
    .ID_ERET(ID_ERET),

    .ID_CP0Sel(ID_CP0Sel),
    .ID_CP0Wri(ID_CP0Wri),

    .ID_Div(ID_Div),
    .ID_DivSigned(ID_DivSigned),
    .ID_Mul(ID_Mul),
    .ID_MulSigned(ID_MulSigned),
    
    .ID_SpecialRegWri(ID_SpecialRegWri),
    .ID_SpecialRegRead(ID_SpecialRegRead),
    .ID_SpecialRegSel(ID_SpecialRegSel),
    .ID_ReadSpecialReg(ID_ReadSpecialReg),

    .ID_ExcepSYS(ID_ExcepSYS),
    .ID_ExcepBP(ID_ExcepBP),
    .ID_InsExcepAdEL(ID_InsExcepAdEL),
    .ID_ExcepRI(ID_ExcepRI),

    .WB_ExcepEN(WB_ExcepEN),

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
    .EX_DelaySlot(EX_DelaySlot),
    .EX_ERET(EX_ERET),
    
    .EX_LOVal(EX_LOVal),
    .EX_HIVal(EX_HIVal),
    .EX_SpecialRegWri(EX_SpecialRegWri),
    .EX_SpecialRegRead(EX_SpecialRegRead),
    .EX_ReadSpecialReg(EX_ReadSpecialReg),
    .EX_Mul(EX_Mul),
    .EX_SpecialRegSel(EX_SpecialRegSel),

    .EX_CP0Sel(EX_CP0Sel),
    .EX_CP0Wri(EX_CP0Wri),

    .EX_ExcepSYS(EX_ExcepSYS),
    .EX_ExcepBP(EX_ExcepBP),
    .EX_ExcepOv(EX_ExcepOv),
    .EX_ExcepAdES(EX_ExcepAdES),
    .EX_InsExcepAdEL(EX_InsExcepAdEL),
    .EX_DataExcepAdEL(EX_DataExcepAdEL),
    .EX_ExcepRI(EX_ExcepRI),
    .EX_Excep(EX_Excep),

    .data_sram_wen(data_sram_wen),
    .data_sram_addr(data_sram_addr),
    .data_sram_wdata(data_sram_wdata),
    
    .ME_MulRes(ME_MulRes)
);

//ME
assign ME_ready_go = 1'b1;
assign ME_allowin = !ME_valid || ME_ready_go && WB_allowin;
assign ME_to_WB_valid = ME_valid && ME_ready_go;

//存在例外取消内存�???
assign data_sram_en = (EX_MemToReg || EX_MemWrite) && EX_valid && ~(EX_Excep || ME_Excep || WB_ExcepEN || (EX_DelaySlot && ID_InsExcepAdEL)) ; //同步RAM 上一拍输入， 下一拍得到结�????????
assign ME_readData = data_sram_rdata;

always @(posedge clk) begin
    if (~resetn) begin
        ME_valid <= 1'b0;
    end
    else if (ME_allowin) begin
        ME_valid <= EX_to_ME_valid && ~WB_ExcepEN;
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
        ME_ReadSpecialReg <= EX_ReadSpecialReg;
        ME_Mul <= EX_Mul;
        ME_SpecialRegSel <= EX_SpecialRegSel;
        ME_MemDataWidth <= EX_MemDataWidth;
        ME_MemDataCombine <= EX_MemDataCombine;
        ME_rdata2 <= EX_rdata2;
        ME_CP0Sel <= EX_CP0Sel;
        ME_CP0Wri <= EX_CP0Wri;
        ME_ExcepSYS <= EX_ExcepSYS;
        ME_ExcepBP <= EX_ExcepBP;
        ME_ExcepOv <= EX_ExcepOv;
        ME_ExcepAdES <= EX_ExcepAdES;
        ME_InsExcepAdEL <= EX_InsExcepAdEL;
        ME_DataExcepAdEL <= EX_DataExcepAdEL;
        ME_ExcepRI <= EX_ExcepRI;
        ME_DelaySlot <= EX_DelaySlot;
        ME_ERET <= EX_ERET;
    end
end
                           
assign ME_FinalData = ME_SpecialRegRead ? ME_ReadSpecialReg : (ME_WriPCPlus8 ? (ME_NextPC + 32'd4) : ME_aluResult); //jal特殊处理

assign ME_Excep = (ME_ExcepRI || ME_DataExcepAdEL || ME_InsExcepAdEL || ME_ExcepAdES || ME_ExcepOv || ME_ExcepBP || ME_ExcepSYS) && ME_valid;

//乘法数据处理
assign ME_LOVal = ME_Mul ? ME_MulRes[31:0] : ME_OldLOVal;
assign ME_HIVal = ME_Mul ? ME_MulRes[63:32] : ME_OldHIVal;

//WB
assign WB_ready_go = 1'b1;
assign WB_allowin = 1'b1;

always @(posedge clk) begin
    if (~resetn) begin
        WB_valid <= 1'b0;
    end
    else if (WB_allowin) begin
        WB_valid <= ME_to_WB_valid && ~WB_ExcepEN;
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
        WB_CP0Sel <= ME_CP0Sel;
        WB_CP0Wri <= ME_CP0Wri;
        WB_ExcepSYS <= ME_ExcepSYS;
        WB_ExcepBP <= ME_ExcepBP;
        WB_ExcepOv <= ME_ExcepOv;
        WB_ExcepAdES <= ME_ExcepAdES;
        WB_InsExcepAdEL <= ME_InsExcepAdEL;
        WB_DataExcepAdEL <= ME_DataExcepAdEL;
        WB_ExcepRI <= ME_ExcepRI;
        WB_DelaySlot <= ME_DelaySlot;
        WB_ERET <= ME_ERET;
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

//CP0 count 寄存器以1/2频率自增
reg count;
always @(posedge clk) begin 
    if (~resetn) begin
        count <= 1'b0;
    end
    else begin
        count <= count + 1'b1;
        if (count) begin
            CP0[9] <= CP0[9] + 32'd1;
        end
    end
end

//触发计时器中�??
always @(posedge clk) begin
    if (CP0[9] == CP0[11]) begin
        CP0[13][30] <= 1'b1;
        CP0[13][15] <= 1'b1; //绑定在硬件中�??5号上
    end
    else begin
        CP0[13][30] <= 1'b0;
        CP0[13][15] <= 1'b0;
    end
end

//中断采样
assign WB_ExcepInt = (~CP0[12][1] && CP0[12][0] && (CP0[12][15:8] & CP0[13][15:8]));

//产生例外的指令会取消当前指令的所有数据写使能
assign WB_CP0SpecWri = WB_ExcepSYS || WB_ExcepBP || WB_ExcepOv || WB_ExcepAdES || WB_InsExcepAdEL || WB_DataExcepAdEL || WB_ExcepRI || WB_ExcepInt; //判断是否产生例外
assign WB_ExcepEN = WB_CP0SpecWri && WB_valid;

//CP0寄存器写
always @(posedge clk) begin
    if (~resetn) begin
        CP0[12] <= {9'b0, 1'b1, 6'b0, 8'b0, 6'b0, 1'b0, 1'b0}; //status寄存器初始化
        CP0[13] <= {1'b0, 1'b0, 14'b0, 6'b0, 2'b0, 1'b0, 5'b0, 2'b0}; //cause
    end
    else if (WB_CP0SpecWri && WB_valid) begin //例外响应
        
        if (~CP0[12][1]) begin //EXL�??1时，EPC在发生新的例外时不做更新
            CP0[14] <= WB_DelaySlot ? (WB_PC - 32'd4) : WB_PC; //
            CP0[13][31] <= WB_DelaySlot;
        end

        CP0[12][1] <= 1'b1;

        if (WB_ExcepInt) begin //控制例外的优先级
            CP0[13][6:2] <= 5'h00;
        end
        else if (WB_InsExcepAdEL) begin 
            CP0[13][6:2] <= 5'h04;
            CP0[8] <= WB_PC;
        end
        else if (WB_ExcepRI) begin
            CP0[13][6:2] <= 5'h0a;
        end
        else if (WB_ExcepOv) begin
            CP0[13][6:2] <= 5'h0c;
        end 
        else if (WB_ExcepBP) begin
            CP0[13][6:2] <= 5'h09;
        end 
        else if (WB_ExcepSYS) begin
            CP0[13][6:2] <= 5'h08;
        end 
        else if (WB_ExcepAdES) begin
            CP0[13][6:2] <= 5'h05;
            CP0[8] <= WB_aluResult;
        end
        else if (WB_DataExcepAdEL) begin
            CP0[13][6:2] <= 5'h04;
            CP0[8] <= WB_aluResult;
        end

    end
    else if (WB_CP0Wri && WB_valid) begin //由指令控制的CP0�??? 

        if (WB_CP0Sel == 5'd9) begin //控制读写权限
            CP0[9] <= WB_rdata2;
        end
        else if (WB_CP0Sel == 5'd11) begin
            CP0[11] <= WB_rdata2;
            CP0[13][30] <= 1'b0;
        end
        else if (WB_CP0Sel == 5'd12) begin
            CP0[12][15:8] <= WB_rdata2[15:8];
            CP0[12][1:0] <= WB_rdata2[1:0];
        end 
        else if (WB_CP0Sel == 5'd13) begin
            CP0[13][9:8] <= WB_rdata2[9:8];
        end
        else if (WB_CP0Sel == 5'd14) begin
            CP0[14] <= WB_rdata2;
        end 

    end
    
    if (resetn && WB_valid && WB_ERET) begin
        CP0[12][1] <= 1'b0;
    end
    
end

assign debug_wb_pc = WB_PC;
assign debug_wb_rf_wen = {4{WB_RegWrite && WB_valid && (~WB_ExcepEN) && ~(WB_DelaySlot && ME_InsExcepAdEL)}}; //跳转出现无效指令，分支延迟槽内的指令无效
assign debug_wb_rf_wnum = WB_WriteReg;
assign debug_wb_rf_wdata = WB_FinalData;

//冲突�????????测单�????????
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

assign ForwardCP0[0] = (ID_CP0Sel == EX_CP0Sel) && EX_valid && EX_CP0Wri;
assign ForwardCP0[1] = (ID_CP0Sel == ME_CP0Sel) && ME_valid && ME_CP0Wri;
assign ForwardCP0[2] = (ID_CP0Sel == WB_CP0Sel) && WB_valid && WB_CP0Wri;

endmodule



