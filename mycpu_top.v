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

wire valid_in = resetn;
assign inst_sram_wen = 4'b0;

wire IF_ready_go, IF_allowin, IF_to_ID_valid;
reg IF_valid;

wire ID_ready_go, ID_allowin, ID_to_EX_valid;
reg ID_valid;
wire ID_PCSrc; //是否跳转 1 跳转
wire [31:0] ID_PCBranch; //跳转PC地址

wire EX_ready_go, EX_allowin, EX_to_ME_valid;
reg EX_valid;

wire ME_ready_go, ME_allowin, ME_to_WB_valid;
reg ME_valid;
wire [31:0] ME_FinalData; //寄存器写入值
reg [4:0] ME_WriteReg; //写入寄存器号
reg ME_RegWrite; //寄存器写使能

wire WB_ready_go, WB_allowin;
reg WB_valid;
wire [31:0] WB_FinalData;
reg [4:0] WB_WriteReg; //写入寄存器号
reg WB_RegWrite; //寄存器写使能
wire [31:0] WB_readData;

wire [1:0] ForwardA; //srcA 前递
wire [1:0] ForwardB; //srcB

//IF
reg [31:0] PC;
wire [31:0] next_PC;
wire [31:0] ins_reg;

assign IF_allowin = !IF_valid || IF_ready_go && ID_allowin;
assign IF_ready_go = valid_in;
assign IF_to_ID_valid = IF_valid && IF_ready_go;
assign inst_sram_wen = 4'b0;
assign inst_sram_en = valid_in;
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
wire [31:0] ID_SignExt_imm;
wire [31:0] ID_UnSignExt_imm;
wire [63:0] ID_op_sel;
wire [31:0] ID_rdata1;
wire [31:0] ID_rdata2;
wire [31:0] rdata1;
wire [31:0] rdata2;
wire [3:0] ID_ALUControl; //alu计算
wire [25:0] ID_InsIdx; //jal的instr_index
wire ID_RegWrite; //控制信号 是否写回寄存�?
wire ID_MemWrite; //是否写内�?
wire ID_MemToReg; //是否将内存读结果写入寄存�?
wire ID_RegDst; //目的寄存器�?�择
wire ID_ALUSrc1; //选择alusource1的�??
wire ID_ALUSrc2; //选择alusource2的�??
wire ID_Branch; //是否为分支跳转指�?
wire ID_BranchCond; //分支跳转条件
wire ID_DirectBranch; //直接跳转 无条�?
wire ID_Jal; //是否为jal跳转
wire ID_Jr; //是否为jr跳转
wire [31:0] ID_NoDirectPCBranch; //非直接跳转的PC地址
wire [31:0] ID_DirectPCBranch; //直接跳转的PC地址
wire ID_Zero;
wire ID_NoZero;
wire ID_BranchEnable; //branch使能信号 跳转条件是否成立
//wire ID_PCSrc; //是否跳转 1 跳转
//wire [31:0] ID_PCBranch; //跳转PC地址

assign ID_ready_go = 1'b1;
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
regfile register_set(.clk(clk), .raddr1(ID_rs), .rdata1(rdata1), .raddr2(ID_rt), .rdata2(rdata2), .we(WB_RegWrite), .waddr(WB_WriteReg), .wdata(WB_FinalData));
assign ID_SignExt_imm = {{16{ID_ins[15]}}, ID_ins[15:0]}; //有符号扩�?
assign ID_UnSignExt_imm = {27'b0, ID_ins[10:6]}; //无符号扩�?

//寄存器堆前�??
assign ID_rdata1 = WB_valid ? (WB_RegWrite ? ((WB_WriteReg == ID_rs) ? WB_FinalData : rdata1) : rdata1) : rdata1;
assign ID_rdata2 = WB_valid ? (WB_RegWrite ? ((WB_WriteReg == ID_rt) ? WB_FinalData : rdata2) : rdata2) : rdata2;

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

assign ID_ALUControl = ({4{ID_op_sel[0] || ID_op_sel[1] || ID_op_sel[3] || ID_op_sel[4]}} && 4'b0) //000 加法 // 可合�?
                       | ({4{ID_op_sel[2]}} & 4'b1) //001 lui 高位加载
                       | ({4{ID_op_sel[5]}} & 4'b10) //010 减法
                       | ({4{ID_op_sel[6]}} & 4'b11) //011 有符号比�?
                       | ({4{ID_op_sel[7]}} & 4'b100) //100 无符号比�?
                       | ({4{ID_op_sel[8]}} & 4'b101) //101 逻辑�?
                       | ({4{ID_op_sel[9]}} & 4'b110) //110 逻辑�?
                       | ({4{ID_op_sel[10]}} & 4'b111) //111 逻辑异或
                       | ({4{ID_op_sel[11]}} & 4'b1000) //1000 逻辑或非
                       | ({4{ID_op_sel[12]}} & 4'b1001) //1001 逻辑左移
                       | ({4{ID_op_sel[13]}} & 4'b1010) //1010 逻辑右移
                       | ({4{ID_op_sel[14]}} & 4'b1011); //1011 算术右移

//控制信号设置
assign ID_RegWrite = ~(ID_op_sel[1] || ID_op_sel[15] || ID_op_sel[16] || ID_op_sel[18]); //设置控制信号 是否写回寄存�?
assign ID_MemWrite = ID_op_sel[1]; //是否写内�?
assign ID_MemToReg = ID_op_sel[0]; // 1 选择 readData; 0 选择 aluResult
//assign ID_RegDst = ID_op_sel[3] || ID_op_sel[5] || ID_op_sel[6] || ID_op_sel[7] || ID_op_sel[8]; // 1 选择 rd; 0 选择 rt
//assign ID_ALUSrc2 = ID_op_sel[3] || ID_op_sel[5] || ID_op_sel[6] || ID_op_sel[7] || ID_op_sel[8]; // 1 选择 rdata2; 0 选择 SignExt_imm //可简�?
assign ID_RegDst = (ID_ins[31:26] == 6'b0);
assign ID_ALUSrc2 = (ID_ins[31:26] == 6'b0);
assign ID_ALUSrc1 = ~(ID_op_sel[12] || ID_op_sel[13] || ID_op_sel[14]); // 1 选择 rdata1; 0 选择 UnSignExt_imm
assign ID_Branch = ID_op_sel[15] || ID_op_sel[16] || ID_op_sel[17] || ID_op_sel[18];
assign ID_BranchCond = ID_op_sel[16]; //1 选择 NoZero信号; 0 选择 Zero信号 �?要扩�?
assign ID_Jal = ID_op_sel[17]; //1 采用jal跳转信号的特殊处�?
assign ID_DirectBranch = ID_op_sel[17] || ID_op_sel[18]; //1 选择 DirectPCBranch; 0 选择 NoDirectPCBranch; 特殊信号 为jal设置
assign ID_Jr = ID_op_sel[18];

//分支处理
assign ID_NoDirectPCBranch = (ID_SignExt_imm << 2) + ID_NextPC;
assign ID_DirectPCBranch = {ID_NextPC[31:28], ID_InsIdx, 2'b0};
assign ID_PCBranch = ID_Jr ? ID_rdata1 : (ID_Jal ? ID_DirectPCBranch : ID_NoDirectPCBranch); //选择jal、jr的特殊处理方法
assign ID_Zero = (ID_rdata1 == ID_rdata2);//分支条件
assign ID_NoZero = ~ID_Zero;
assign ID_BranchEnable = (ID_BranchCond ? ID_NoZero : ID_Zero) || ID_DirectBranch;
//assign ID_BranchEnable = (ID_NoZero && (ID_BranchCond == 1'b1)) 
//                         || (ID_Zero && (ID_BranchCond == 1'b0));//跳转条件扩展，需要使用当前模式
assign ID_PCSrc = ID_Branch && ID_BranchEnable && ID_valid;

//EX
reg [31:0] EX_NextPC;
reg [31:0] EX_PC;
reg [4:0] EX_rs;
reg [4:0] EX_rt;
reg [4:0] EX_rd;
reg [3:0] EX_ALUControl;
reg [25:0] EX_InsIdx;
reg EX_MemWrite;
reg EX_MemToReg;
reg EX_RegDst;
reg EX_ALUSrc1;
reg EX_ALUSrc2;
reg EX_Jal;
reg [31:0] EX_SignExt_imm;
reg [31:0] EX_UnSignExt_imm;
reg [31:0] EX_rdata1;
reg [31:0] EX_rdata2;
reg EX_OldRegWrite;
wire EX_RegWrite;
wire [31:0] EX_TrueRdata1;
wire [31:0] EX_TrueRdata2;
wire [4:0] EX_WriteReg;
wire [31:0] EX_WriteData;
wire [31:0] EX_aluResult;
wire [31:0] EX_srcA;
wire [31:0] EX_srcB;

assign EX_ready_go = 1'b1;
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
        EX_SignExt_imm <= ID_SignExt_imm;
        EX_UnSignExt_imm <= ID_UnSignExt_imm;
        EX_rdata1 <= ID_rdata1;
        EX_rdata2 <= ID_rdata2;
    end
end

//数据相关的前递处理
assign EX_TrueRdata1 = ForwardA[0] ? WB_FinalData : (ForwardA[1] ? ME_FinalData : EX_rdata1); //写回级的寄存器前递优先级高
assign EX_TrueRdata2 = ForwardB[0] ? WB_FinalData : (ForwardB[1] ? ME_FinalData : EX_rdata2);        

assign EX_srcA = EX_ALUSrc1 ? EX_TrueRdata1 : EX_UnSignExt_imm;
assign EX_srcB = EX_ALUSrc2 ? EX_TrueRdata2 : EX_SignExt_imm;
assign EX_WriteData = EX_TrueRdata2;
assign EX_WriteReg = EX_Jal ? 5'd31 : (EX_RegDst ? EX_rd : EX_rt); //jal需要写31号寄存器
assign EX_RegWrite = (EX_WriteReg == 5'b0) ? 1'b0 : EX_OldRegWrite; //写寄存器0时 直接取消寄存器写使能 避免后续产生寄存器前递

alu calculation(.ALUControl(EX_ALUControl), .alu_src1(EX_srcA), .alu_src2(EX_srcB), .alu_result(EX_aluResult));

//ME
reg [31:0] ME_NextPC;
reg [31:0] ME_PC;
reg ME_MemToReg;
reg ME_Jal;
reg [31:0] ME_aluResult;
reg ME_MemWrite;
reg [31:0] ME_WriteData;

//wire [31:0] ME_FinalData; //寄存器写入值
//reg [4:0] ME_WriteReg; //写入寄存器号
//reg ME_RegWrite; //寄存器写使能

assign ME_ready_go = 1'b1;
assign ME_allowin = !ME_valid || ME_ready_go && WB_allowin;
assign ME_to_WB_valid = ME_valid && ME_ready_go;

assign data_sram_en = ME_valid; //同步RAM 上一拍输入， 下一拍得到结果
assign data_sram_wen = {4{ME_MemWrite}};
assign data_sram_wdata = ME_WriteData;
assign data_sram_addr = ME_aluResult;

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
    end
end

assign ME_FinalData = ME_Jal ? (ME_NextPC + 32'd4) : ME_aluResult; //jal特殊处理

//WB
reg [31:0] WB_NextPC;
reg [31:0] WB_PC;
reg WB_MemToReg;
reg WB_Jal;
reg [31:0] WB_aluResult;
reg [31:0] WB_OldFinalData;
//reg WB_RegWrite;
//reg [4:0] WB_WriteReg;
//wire [31:0] WB_FinalData;
//wire [31:0] WB_readData;

assign WB_ready_go = 1'b1;
assign WB_allowin = 1'b1;
assign WB_readData = data_sram_rdata; //上一周期、阶段输入， 当前阶段得到输出 同步处理

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
    end
end

//assign WB_FinalData = WB_Jal ? (WB_NextPC + 32'd4) : (WB_MemToReg ? WB_readData : WB_aluResult);
assign WB_FinalData = WB_MemToReg ? WB_readData : WB_OldFinalData;

assign debug_wb_pc = WB_PC;
assign debug_wb_rf_wen = {4{WB_RegWrite}};
assign debug_wb_rf_wnum = WB_WriteReg;
assign debug_wb_rf_wdata = WB_FinalData;

//冲突检测单元
//wire [1:0] ForwardA;
//wire [1:0] ForwardB;

assign ForwardA[0] = (EX_rs == WB_WriteReg) && WB_valid && WB_RegWrite; 
assign ForwardA[1] = (EX_rs == ME_WriteReg) && ME_valid && ME_RegWrite;
assign ForwardB[0] = (EX_rt == WB_WriteReg) && WB_valid && WB_RegWrite;
assign ForwardB[1] = (EX_rt == ME_WriteReg) && ME_valid && ME_RegWrite;

endmodule



