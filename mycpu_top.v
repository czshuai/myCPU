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

wire ID_ready_go, ID_allowin, ID_to_EX_valid;
reg ID_valid;
wire ID_PCSrc; //是否跳转 1 跳转
wire [31:0] ID_PCBranch; //跳转PC地址

wire EX_ready_go, EX_allowin, EX_to_ME_valid;
reg EX_valid;
wire EX_RegWrite;
wire [4:0] EX_WriteReg;

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

//特殊寄存�??
reg [31:0] LO, HI; //保存除法的结�??

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
reg [31:0] ID_NextPC;
reg [31:0] ID_PC;
reg [31:0] ID_ins;
wire [4:0] ID_rs;
wire [4:0] ID_rt;
wire [4:0] ID_rd;
wire [31:0] ID_SignExt_imm150;
wire [31:0] ID_UnSignExt_imm150;
wire [31:0] ID_UnSignExt_imm106;
wire ID_UnSignExt150; //ins[15:0]无符号扩�??
wire [31:0] ID_Ext_imm150;
wire [63:0] ID_op_sel;
wire [31:0] ID_rdata1;
wire [31:0] ID_rdata2;
wire [31:0] rdata1;
wire [31:0] rdata2;
wire [4:0] ID_ALUControl; //alu计算
wire [25:0] ID_InsIdx; //jal的instr_index
wire ID_RegWrite; //控制信号 是否写回reg
wire ID_MemWrite; //是否写Mem
wire ID_MemToReg; //是否将内存读结果写入reg
wire ID_RegDst; //目的寄存器sel
wire ID_ALUSrc1; //选择alusource1的val
wire ID_ALUSrc2; //选择alusource2的val
wire ID_Branch; //是否为分支跳转ins
wire ID_BranchCond; //分支跳转条件
wire ID_DirectBranch; //直接跳转 无cond
wire ID_Jal; //是否为jal跳转
wire ID_Jr; //是否为jr跳转
wire [31:0] ID_NoDirectPCBranch; //非直接跳转的PC地址
wire [31:0] ID_DirectPCBranch; //直接跳转的PC地址
wire ID_Zero;
wire ID_NoZero;
wire ID_BranchEnable; //branch使能信号 跳转条件是否成立
wire ID_Stall; //阻塞执行阶段 *********************************************
wire ID_Div; //除法使能信号
wire ID_DivSigned; //是否为有符号除法
wire ID_Mul; //乘法使能信号
wire ID_MulSigned; //是否为有符号乘法
wire ID_SpecialRegWri; //是否写特殊寄存器
wire ID_SpecialRegRead; //是否读特殊寄存器
wire [1:0] ID_SpecialRegSel; //选择HI或�?�LO寄存�?? 01选择lo 10选择hi
wire [31:0] ID_ReadHiReg; //读取的特殊寄存器的�??
wire [31:0] ID_ReadLoReg;
//wire ID_PCSrc; //是否跳转 1 跳转
//wire [31:0] ID_PCBranch; //跳转PC地址

assign ID_ready_go = ~ID_Stall;
assign ID_allowin = !ID_valid || ID_ready_go && EX_allowin;
assign ID_to_EX_valid = ID_valid && ID_ready_go;

always @(posedge clk) begin
    if (~resetn) begin
        ID_valid <= 1'b0;
    end
    else if (ID_allowin) begin
        ID_valid <= IF_to_ID_valid;
    end

    if (IF_to_ID_valid && ID_allowin) begin
        ID_NextPC <= next_PC;
        ID_PC <= PC;
        ID_ins <= ins_reg;
    end
end

assign ID_rs = ID_ins[25:21];
assign ID_rt = ID_ins[20:16];
assign ID_rd = ID_ins[15:11];
assign ID_InsIdx = ID_ins[25:0];

regfile register_set(.clk(clk), .raddr1(ID_rs), .rdata1(rdata1), .raddr2(ID_rt), .rdata2(rdata2), .we(WB_RegWrite && WB_valid), .waddr(WB_WriteReg), .wdata(WB_FinalData));

//符号扩展
assign ID_SignExt_imm150 = {{16{ID_ins[15]}}, ID_ins[15:0]}; //150有符号扩�??
assign ID_UnSignExt_imm150 = {16'b0, ID_ins[15:0]}; //150无符号扩�??
assign ID_UnSignExt_imm106 = {27'b0, ID_ins[10:6]}; //106无符号扩�??
assign ID_Ext_imm150 = ID_UnSignExt150 ? ID_UnSignExt_imm150 : ID_SignExt_imm150;

//寄存器堆前�?? //如果是跳转指令，数据前�?�到译码阶段
assign ID_rdata1 = ForwardA[1] ? ME_FinalData : (ForwardA[2] ? WB_FinalData : rdata1);
assign ID_rdata2 = ForwardB[1] ? ME_FinalData : (ForwardB[2] ? WB_FinalData : rdata2);

//特殊寄存器读
assign ID_ReadHiReg = (WB_valid && WB_SpecialRegWri && WB_SpecialRegSel[1]) ? WB_HIVal : HI;
assign ID_ReadLoReg = (WB_valid && WB_SpecialRegWri && WB_SpecialRegSel[0]) ? WB_LOVal : LO;

assign ID_op_sel[0] = (ID_ins[31:26] == 6'b100011); //op译码 lw
assign ID_op_sel[1] = (ID_ins[31:26] == 6'b101011); //sw
assign ID_op_sel[2] = (ID_ins[31:26] == 6'b001111); //lui
assign ID_op_sel[3] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b100001); //addu R类型指令
assign ID_op_sel[4] = (ID_ins[31:26] == 6'b001001); //addiu
assign ID_op_sel[5] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b100011); //subu
assign ID_op_sel[6] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b101010); //slt
assign ID_op_sel[7] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b101011); //sltu
assign ID_op_sel[8] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b100100); //and
assign ID_op_sel[9] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b100101); //or
assign ID_op_sel[10] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b100110); //xor
assign ID_op_sel[11] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b100111); //nor
assign ID_op_sel[12] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b0); //sll
assign ID_op_sel[13] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b000010); //srl
assign ID_op_sel[14] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b000011); //sra
assign ID_op_sel[15] = (ID_ins[31:26] == 6'b000100); //beq
assign ID_op_sel[16] = (ID_ins[31:26] == 6'b000101); //bne
assign ID_op_sel[17] = (ID_ins[31:26] == 6'b000011); //jal
assign ID_op_sel[18] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b001000); //jr
assign ID_op_sel[19] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b100000); //add
assign ID_op_sel[20] = (ID_ins[31:26] == 6'b001000); //addi
assign ID_op_sel[21] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b100010); //sub
assign ID_op_sel[22] = (ID_ins[31:26] == 6'b001010); //slti
assign ID_op_sel[23] = (ID_ins[31:26] == 6'b001011); //sltiu
assign ID_op_sel[24] = (ID_ins[31:26] == 6'b001100); //andi
assign ID_op_sel[25] = (ID_ins[31:26] == 6'b001101); //ori
assign ID_op_sel[26] = (ID_ins[31:26] == 6'b001110); //xori
assign ID_op_sel[27] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b000100); //sllv
assign ID_op_sel[28] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b000111); //srav
assign ID_op_sel[29] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b000110); //srlv
assign ID_op_sel[30] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b011010); //div
assign ID_op_sel[31] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b011011); //divu
assign ID_op_sel[32] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b011000); //mult
assign ID_op_sel[33] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b011001); //multu
assign ID_op_sel[34] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b010000); //mfhi
assign ID_op_sel[35] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b010010); //mflo
assign ID_op_sel[36] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b010001); //mthi
assign ID_op_sel[37] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b010011); //mtlo

//有符号加减单独列出为了处理例�??
assign ID_ALUControl = ({5{ID_op_sel[0] || ID_op_sel[1] || ID_op_sel[3] || ID_op_sel[4]}} && 5'b0) //000 加法
                       | ({5{ID_op_sel[2]}} & 5'b1) //001 lui 高位加载
                       | ({5{ID_op_sel[5]}} & 5'b10) //010 减法
                       | ({5{ID_op_sel[6] || ID_op_sel[22]}} & 5'b11) //011 有符号比�??
                       | ({5{ID_op_sel[7] || ID_op_sel[23]}} & 5'b100) //100 无符号比�??
                       | ({5{ID_op_sel[8] || ID_op_sel[24]}} & 5'b101) //101 逻辑and
                       | ({5{ID_op_sel[9] || ID_op_sel[25]}} & 5'b110) //110 逻辑or
                       | ({5{ID_op_sel[10] || ID_op_sel[26]}} & 5'b111) //111 逻辑异或
                       | ({5{ID_op_sel[11]}} & 5'b1000) //1000 逻辑或非
                       | ({5{ID_op_sel[12] || ID_op_sel[27]}} & 5'b1001) //1001 逻辑左移
                       | ({5{ID_op_sel[13] || ID_op_sel[29]}} & 5'b1010) //1010 逻辑右移
                       | ({5{ID_op_sel[14] || ID_op_sel[28]}} & 5'b1011) //1011 算术右移
                       | ({5{ID_op_sel[19] || ID_op_sel[20]}} & 5'b1100) //1100 有符号加
                       | ({5{ID_op_sel[21]}} & 5'b1101); //1101 有符号减

//控制信号设置
assign ID_RegWrite = ~(ID_op_sel[1] || ID_op_sel[15] || ID_op_sel[16] || ID_op_sel[18] || ID_op_sel[30] || ID_op_sel[31] || ID_op_sel[32] || ID_op_sel[33] || ID_op_sel[36] || ID_op_sel[37]); //设置控制信号 是否写回寄存�??
assign ID_MemWrite = ID_op_sel[1]; //是否写内�??
assign ID_MemToReg = ID_op_sel[0]; // 1 选择 readData; 0 选择 aluResult
//assign ID_RegDst = ID_op_sel[3] || ID_op_sel[5] || ID_op_sel[6] || ID_op_sel[7] || ID_op_sel[8]; // 1 选择 rd; 0 选择 rt
//assign ID_ALUSrc2 = ID_op_sel[3] || ID_op_sel[5] || ID_op_sel[6] || ID_op_sel[7] || ID_op_sel[8]; // 1 选择 rdata2; 0 选择 SignExt_imm150
assign ID_RegDst = (ID_ins[31:26] == 6'b0);
assign ID_ALUSrc2 = (ID_ins[31:26] == 6'b0);
assign ID_ALUSrc1 = ~(ID_op_sel[12] || ID_op_sel[13] || ID_op_sel[14]); // 1 选择 rdata1; 0 选择 UnSignExt_imm106
assign ID_Branch = ID_op_sel[15] || ID_op_sel[16] || ID_op_sel[17] || ID_op_sel[18];
assign ID_BranchCond = ID_op_sel[16]; //1 选择 NoZero信号; 0 选择 Zero信号 �??要扩�??
assign ID_Jal = ID_op_sel[17]; //1 采用jal跳转信号的特殊处�??
assign ID_DirectBranch = ID_op_sel[17] || ID_op_sel[18]; //1 选择 DirectPCBranch; 0 选择 NoDirectPCBranch; 特殊信号 为jal设置
assign ID_Jr = ID_op_sel[18];
assign ID_UnSignExt150 = ID_op_sel[24] || ID_op_sel[25] || ID_op_sel[26]; //1 选择 UnSignExt_imm150
assign ID_Div = ID_op_sel[30] || ID_op_sel[31];
assign ID_DivSigned = ID_op_sel[30];
assign ID_Mul = ID_op_sel[32] || ID_op_sel[33];
assign ID_MulSigned = ID_op_sel[32];
assign ID_SpecialRegWri = ID_op_sel[30] || ID_op_sel[31] || ID_op_sel[32] || ID_op_sel[33] || ID_op_sel[36] || ID_op_sel[37];
assign ID_SpecialRegRead = ID_op_sel[34] || ID_op_sel[35];
assign ID_SpecialRegSel[0] = ID_op_sel[30] || ID_op_sel[31] || ID_op_sel[32] || ID_op_sel[33] || ID_op_sel[35] || ID_op_sel[37]; //选择lo特殊寄存�??
assign ID_SpecialRegSel[1] = ID_op_sel[30] || ID_op_sel[31] || ID_op_sel[32] || ID_op_sel[33] || ID_op_sel[34] || ID_op_sel[36]; //选择hi特殊寄存�??

//分支处理
assign ID_NoDirectPCBranch = (ID_SignExt_imm150 << 2) + ID_NextPC;
assign ID_DirectPCBranch = {ID_NextPC[31:28], ID_InsIdx, 2'b0};
assign ID_PCBranch = ID_Jr ? ID_rdata1 : (ID_Jal ? ID_DirectPCBranch : ID_NoDirectPCBranch); //选择jal、jr的特殊处理方�??
assign ID_Zero = (ID_rdata1 == ID_rdata2);//分支条件
assign ID_NoZero = ~ID_Zero;
assign ID_BranchEnable = (ID_BranchCond ? ID_NoZero : ID_Zero) || ID_DirectBranch;
//assign ID_BranchEnable = (ID_NoZero && (ID_BranchCond == 1'b1)) 
//                         || (ID_Zero && (ID_BranchCond == 1'b0));//跳转条件扩展，需要使用当前模�??
assign ID_PCSrc = ID_Branch && ID_BranchEnable && ID_valid;

//EX
reg [31:0] EX_NextPC;
reg [31:0] EX_PC;
reg [4:0] EX_rs;
reg [4:0] EX_rt;
reg [4:0] EX_rd;
reg [4:0] EX_ALUControl;
reg [25:0] EX_InsIdx;
reg EX_MemWrite;
reg EX_MemToReg;
reg EX_RegDst;
reg EX_ALUSrc1;
reg EX_ALUSrc2;
reg EX_Jal;
reg [31:0] EX_Ext_imm150;
reg [31:0] EX_UnSignExt_imm106;
reg [31:0] EX_rdata1;
reg [31:0] EX_rdata2;
reg EX_OldRegWrite;
reg EX_Div;
reg EX_DivSigned; 
reg EX_Mul;
reg EX_MulSigned;
reg EX_SpecialRegWri;
reg EX_SpecialRegRead;
reg [1:0] EX_SpecialRegSel;
reg [31:0] EX_ReadHiReg;
reg [31:0] EX_ReadLoReg;
//wire EX_RegWrite;
//wire [4:0] EX_WriteReg;
wire [31:0] EX_WriteData;
wire [31:0] EX_aluResult;
wire [31:0] EX_srcA;
wire [31:0] EX_srcB;
wire EX_Overflow;
wire EX_DivComplete;
wire [31:0] EX_DivResS;
wire [31:0] EX_DivResR;
wire [31:0] EX_LOVal; //保存LO寄存器的�??
wire [31:0] EX_HIVal; //保存HI寄存器的�??
wire EX_Stall;

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
        EX_InsIdx <= ID_InsIdx;
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
assign EX_WriteReg = EX_Jal ? 5'd31 : (EX_RegDst ? EX_rd : EX_rt); //jal�??要写31号寄存器
assign EX_RegWrite = (EX_WriteReg == 5'b0) ? 1'b0 : EX_OldRegWrite; //写寄存器0�?? 直接取消寄存器写使能 避免后续产生寄存器前�??

//出现例外直接取消写信�??
alu calculation(.ALUControl(EX_ALUControl), .alu_src1(EX_srcA), .alu_src2(EX_srcB), .alu_result(EX_aluResult), .Overflow(EX_Overflow));
div divider(.div_clk(clk), .resetn(resetn), .div(EX_Div), .div_signed(EX_DivSigned), .x(EX_rdata1), .y(EX_rdata2), .s(EX_DivResS), .r(EX_DivResR), .complete(EX_DivComplete));
mul muler(.mul_clk(clk), .resetn(resetn), .mul_signed(EX_MulSigned), .x(EX_rdata1), .y(EX_rdata2), .result(ME_MulRes));
//mult流水 跨执行阶段和访存阶段 模块内进行流�?? 在访存阶段得到结�??

//div产生阻塞信号
assign EX_Stall = EX_Div && ~EX_DivComplete;

//ME
reg [31:0] ME_NextPC;
reg [31:0] ME_PC;
reg ME_MemToReg;
reg ME_Jal;
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

assign data_sram_en = ME_valid; //同步RAM 上一拍输入， 下一拍得到结�??
assign data_sram_wen = {4{EX_MemWrite}};
assign data_sram_wdata = EX_WriteData;
assign data_sram_addr = EX_aluResult;
assign ME_readData = data_sram_rdata;

always @(posedge clk) begin
    if (~resetn) begin
        ME_valid <= 1'b0;
    end
    else if (EX_allowin) begin
        ME_valid <= EX_to_ME_valid;
    end

    if (EX_to_ME_valid && ME_allowin) begin
        ME_NextPC <= EX_NextPC;
        ME_PC <= EX_PC;
        ME_WriteReg <= EX_WriteReg;
        ME_RegWrite <= EX_RegWrite;
        ME_aluResult <= EX_aluResult;
        ME_MemToReg <= EX_MemToReg;
        ME_Jal <= EX_Jal;
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
    end
end

assign ME_ReadSpecialReg = {32{ME_SpecialRegSel[0]}} & ME_ReadLoReg |
                           {32{ME_SpecialRegSel[1]}} & ME_ReadHiReg;
                           
assign ME_FinalData = ME_SpecialRegRead ? ME_ReadSpecialReg : (ME_Jal ? (ME_NextPC + 32'd4) : ME_aluResult); //jal特殊处理

//乘法数据处理
assign ME_LOVal = ME_Mul ? ME_MulRes[31:0] : ME_OldLOVal;
assign ME_HIVal = ME_Mul ? ME_MulRes[63:32] : ME_OldHIVal;

//WB
reg [31:0] WB_NextPC;
reg [31:0] WB_PC;
reg WB_MemToReg;
reg WB_Jal;
reg [31:0] WB_aluResult;
reg [31:0] WB_OldFinalData;
//reg [31:0] WB_LOVal;
//reg [31:0] WB_HIVal;
//reg WB_SpecialRegWri;
//reg [1:0] WB_SpecialRegSel;
//reg WB_RegWrite;
//reg [4:0] WB_WriteReg;
//wire [31:0] WB_FinalData;
//wire [31:0] WB_readData;

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
        WB_Jal <= ME_Jal;
        WB_OldFinalData <= ME_FinalData;
        WB_LOVal <= ME_LOVal;
        WB_HIVal <= ME_HIVal;
        WB_SpecialRegWri <= ME_SpecialRegWri;
        WB_SpecialRegSel <= ME_SpecialRegSel;
        WB_readData <= ME_readData;
    end
end

//assign WB_FinalData = WB_Jal ? (WB_NextPC + 32'd4) : (WB_MemToReg ? WB_readData : WB_aluResult);
assign WB_FinalData = WB_MemToReg ? WB_readData : WB_OldFinalData;

//LO/HI特殊寄存器写
always @(posedge clk) begin
    if (WB_SpecialRegWri && WB_valid && WB_SpecialRegSel[0]) begin
        LO <= WB_LOVal;
    end

    if (WB_SpecialRegWri && WB_valid && WB_SpecialRegSel[1]) begin
        HI <= WB_HIVal;
    end
end

assign debug_wb_pc = WB_PC;
assign debug_wb_rf_wen = {4{WB_RegWrite && WB_valid}};
assign debug_wb_rf_wnum = WB_WriteReg;
assign debug_wb_rf_wdata = WB_FinalData;

//冲突�??测单�??
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



