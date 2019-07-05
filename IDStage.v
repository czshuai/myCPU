`timescale 1ns / 1ps

module IDStage (
    input clk, resetn,

    input [31:0] next_PC, PC,
    input [31:0] ins_reg,

    input ID_Stall,
    input EX_allowin,
    input IF_to_ID_valid,

    input [31:0] ME_FinalData,

    input WB_RegWrite, WB_valid,
    input [4:0] WB_WriteReg,
    input [31:0] WB_FinalData,
    input WB_SpecialRegWri,
    input [1:0] WB_SpecialRegSel,
    input [31:0] WB_HIVal, WB_LOVal,

    input [2:0] ForwardA, ForwardB, 

    output reg ID_valid,
    output ID_allowin,
    output ID_to_EX_valid,

    output reg [31:0] ID_NextPC, ID_PC, 

    output [4:0] ID_rs, ID_rt, ID_rd,

    output [31:0] ID_Ext_imm150, ID_UnSignExt_imm106,

    output [31:0] ID_PCBranch,
    output ID_PCSrc,

    output [4:0] ID_ALUControl,
    output [25:0] ID_InsIdx,
    output ID_RegWrite,
    output ID_MemWrite,
    output ID_MemToReg,
    output ID_RegDst,
    output ID_ALUSrc1,
    output ID_ALUSrc2,
    output ID_WriReg31,
    output ID_WriPCPlus8,
    output [31:0] ID_rdata1,
    output [31:0] ID_rdata2,
    output ID_Div,
    output ID_DivSigned,
    output ID_Mul,
    output ID_MulSigned,
    output [31:0] ID_ReadHiReg,
    output [31:0] ID_ReadLoReg,
    output ID_SpecialRegWri,
    output ID_SpecialRegRead,
    output [1:0] ID_SpecialRegSel,
    output [2:0] ID_MemDataWidth,
    output [1:0] ID_MemDataCombine
    );

wire ID_ready_go;

reg [31:0] ID_ins;
wire [31:0] rdata1;
wire [31:0] rdata2;
wire [31:0] ID_SignExt_imm150;
wire [31:0] ID_UnSignExt_imm150;
wire ID_UnSignExt150; //ins[15:0]无符号扩�????
wire [63:0] ID_op_sel;
wire ID_Branch; //是否为分支跳转ins
wire [2:0] ID_BranchCond; //分支跳转条件 001 ID_Equal  010 ID_NoEqual  011 gz  100 g  101 le  110 l
wire ID_DirectBranch; //直接跳转 无cond
wire ID_Jr; //是否为jr跳转
wire ID_J; //是否为J跳转
wire ID_Jal;
wire ID_Jalr; //是否为Jalr跳转
wire [31:0] ID_NoDirectPCBranch; //非直接跳转的PC地址
wire [31:0] ID_DirectPCBranch; //直接跳转的PC地址
wire ID_Equal; //分支条件测试相等
wire ID_NoEqual; //不相�?
wire ID_GreatEqual; //大于等于
wire ID_Great; //大于
wire ID_LowEqual; //小于等于
wire ID_Low; //小于
wire ID_BranchEnable; //branch使能信号 跳转条件是否成立 

//特殊寄存�????
reg [31:0] LO, HI; //保存除法的结�????

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
assign ID_SignExt_imm150 = {{16{ID_ins[15]}}, ID_ins[15:0]}; //150有符号扩�????
assign ID_UnSignExt_imm150 = {16'b0, ID_ins[15:0]}; //150无符号扩�????
assign ID_UnSignExt_imm106 = {27'b0, ID_ins[10:6]}; //106无符号扩�????
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
assign ID_op_sel[38] = (ID_ins[31:26] == 6'b000010); //j
assign ID_op_sel[39] = (ID_ins[31:26] == 6'b000001) && (ID_ins[20:16] == 5'b1); //bgez
assign ID_op_sel[40] = (ID_ins[31:26] == 6'b000111) && (ID_ins[20:16] == 5'b0); //bgtz
assign ID_op_sel[41] = (ID_ins[31:26] == 6'b000110) && (ID_ins[20:16] == 5'b0); //blez
assign ID_op_sel[42] = (ID_ins[31:26] == 6'b000001) && (ID_ins[20:16] == 5'b0); //bltz
assign ID_op_sel[43] = (ID_ins[31:26] == 6'b000001) && (ID_ins[20:16] == 5'b10001); //bgezal
assign ID_op_sel[44] = (ID_ins[31:26] == 6'b000001) && (ID_ins[20:16] == 5'b10000); //bltzal
assign ID_op_sel[45] = (ID_ins[31:26] == 6'b0) && (ID_ins[5:0] == 6'b001001); //jalr
assign ID_op_sel[46] = (ID_ins[31:26] == 6'b100000); //lb
assign ID_op_sel[47] = (ID_ins[31:26] == 6'b100100); //lbu
assign ID_op_sel[48] = (ID_ins[31:26] == 6'b100001); //lh
assign ID_op_sel[49] = (ID_ins[31:26] == 6'b100101); //lhu
assign ID_op_sel[50] = (ID_ins[31:26] == 6'b100010); //lwl
assign ID_op_sel[51] = (ID_ins[31:26] == 6'b100110); //lwr
assign ID_op_sel[52] = (ID_ins[31:26] == 6'b101000); //sb
assign ID_op_sel[53] = (ID_ins[31:26] == 6'b101001); //sh
assign ID_op_sel[54] = (ID_ins[31:26] == 6'b101010); //swl
assign ID_op_sel[55] = (ID_ins[31:26] == 6'b101110); //swr

//有符号加减单独列出为了处理例�????
assign ID_ALUControl = ({5{ID_op_sel[3] || ID_op_sel[4]}} && 5'b0) //000 加法
                       | ({5{ID_op_sel[2]}} & 5'b1) //001 lui 高位加载
                       | ({5{ID_op_sel[5]}} & 5'b10) //010 减法
                       | ({5{ID_op_sel[6] || ID_op_sel[22]}} & 5'b11) //011 有符号比�????
                       | ({5{ID_op_sel[7] || ID_op_sel[23]}} & 5'b100) //100 无符号比�????
                       | ({5{ID_op_sel[8] || ID_op_sel[24]}} & 5'b101) //101 逻辑and
                       | ({5{ID_op_sel[9] || ID_op_sel[25]}} & 5'b110) //110 逻辑or
                       | ({5{ID_op_sel[10] || ID_op_sel[26]}} & 5'b111) //111 逻辑异或
                       | ({5{ID_op_sel[11]}} & 5'b1000) //1000 逻辑或非
                       | ({5{ID_op_sel[12] || ID_op_sel[27]}} & 5'b1001) //1001 逻辑左移
                       | ({5{ID_op_sel[13] || ID_op_sel[29]}} & 5'b1010) //1010 逻辑右移
                       | ({5{ID_op_sel[14] || ID_op_sel[28]}} & 5'b1011) //1011 算术右移
                       | ({5{ID_op_sel[19] || ID_op_sel[20]}} & 5'b1100) //1100 有符号加
                       | ({5{ID_op_sel[21]}} & 5'b1101) //1101 有符号减
                       | ({5{ID_op_sel[0] || ID_op_sel[1] || ID_op_sel[46] || ID_op_sel[47] || ID_op_sel[48] || ID_op_sel[49] || ID_op_sel[52] || ID_op_sel[53]}} & 5'b1110); //同样为加法运算，在地�?不对齐时产生例外

//控制信号设置
assign ID_RegWrite = ~(ID_op_sel[1] || ID_op_sel[15] || ID_op_sel[16] || ID_op_sel[18] || ID_op_sel[30] || ID_op_sel[31] ||
                      ID_op_sel[32] || ID_op_sel[33] || ID_op_sel[36] || ID_op_sel[37] || ID_op_sel[38] || ID_op_sel[39] ||
                      ID_op_sel[40] || ID_op_sel[41] || ID_op_sel[42] || ID_op_sel[52] || ID_op_sel[53] || ID_op_sel[54] || ID_op_sel[55]); //设置控制信号 是否写回寄存�????

assign ID_MemWrite = ID_op_sel[1] || ID_op_sel[52] || ID_op_sel[53] || ID_op_sel[54] || ID_op_sel[55]; //是否写Memory
assign ID_MemToReg = ID_op_sel[0] || ID_op_sel[46] || ID_op_sel[47] || ID_op_sel[48] || ID_op_sel[49] || ID_op_sel[50] || ID_op_sel[51]; // 1 选择 readData; 0 选择 aluResult
//assign ID_RegDst = ID_op_sel[3] || ID_op_sel[5] || ID_op_sel[6] || ID_op_sel[7] || ID_op_sel[8]; // 1 选择 rd; 0 选择 rt
//assign ID_ALUSrc2 = ID_op_sel[3] || ID_op_sel[5] || ID_op_sel[6] || ID_op_sel[7] || ID_op_sel[8]; // 1 选择 rdata2; 0 选择 SignExt_imm150
assign ID_RegDst = (ID_ins[31:26] == 6'b0);
assign ID_ALUSrc2 = (ID_ins[31:26] == 6'b0);
assign ID_ALUSrc1 = ~(ID_op_sel[12] || ID_op_sel[13] || ID_op_sel[14]); // 1 选择 rdata1; 0 选择 UnSignExt_imm106
assign ID_Branch = ID_op_sel[15] || ID_op_sel[16] || ID_op_sel[17] || ID_op_sel[18] || ID_op_sel[38] || ID_op_sel[39] ||
                   ID_op_sel[40] || ID_op_sel[41] || ID_op_sel[42] || ID_op_sel[43] || ID_op_sel[44] || ID_op_sel[45];

assign ID_BranchCond = ({3{ID_op_sel[16]}} & 3'b010) | ({3{ID_op_sel[15]}} & 3'b001) | 
                       ({3{ID_op_sel[39] || ID_op_sel[43]}} & 3'b011) | ({3{ID_op_sel[40]}} & 3'b100) |
                       ({3{ID_op_sel[41]}} & 3'b101) | ({3{ID_op_sel[42] || ID_op_sel[44]}} & 3'b110);  //001 ID_Equal  010 ID_NoEqual  011 gz  100 g  101 le  110 l

assign ID_Jal = ID_op_sel[17]; //1 采用jal跳转信号的特殊处�????
assign ID_WriReg31 = ID_op_sel[17] || ID_op_sel[43] || ID_op_sel[44];
assign ID_WriPCPlus8 = ID_op_sel[17] || ID_op_sel[43] || ID_op_sel[44] || ID_op_sel[45];
assign ID_J = ID_op_sel[38];
assign ID_Jalr = ID_op_sel[45];
assign ID_DirectBranch = ID_op_sel[17] || ID_op_sel[18] || ID_op_sel[38] || ID_op_sel[45]; //1 选择 DirectPCBranch; 0 选择 NoDirectPCBranch; 特殊信号 为jal设置
assign ID_Jr = ID_op_sel[18];
assign ID_UnSignExt150 = ID_op_sel[24] || ID_op_sel[25] || ID_op_sel[26]; //1 选择 UnSignExt_imm150
assign ID_Div = ID_op_sel[30] || ID_op_sel[31];
assign ID_DivSigned = ID_op_sel[30];
assign ID_Mul = ID_op_sel[32] || ID_op_sel[33];
assign ID_MulSigned = ID_op_sel[32];
assign ID_SpecialRegWri = ID_op_sel[30] || ID_op_sel[31] || ID_op_sel[32] || ID_op_sel[33] || ID_op_sel[36] || ID_op_sel[37];
assign ID_SpecialRegRead = ID_op_sel[34] || ID_op_sel[35];
assign ID_SpecialRegSel[0] = ID_op_sel[30] || ID_op_sel[31] || ID_op_sel[32] || ID_op_sel[33] || ID_op_sel[35] || ID_op_sel[37]; //选择lo特殊寄存�????
assign ID_SpecialRegSel[1] = ID_op_sel[30] || ID_op_sel[31] || ID_op_sel[32] || ID_op_sel[33] || ID_op_sel[34] || ID_op_sel[36]; //选择hi特殊寄存�????
assign ID_MemDataWidth = ({3{ID_op_sel[46] || ID_op_sel[52]}} & 3'b001) |
                         ({3{ID_op_sel[47]}} & 3'b010) | 
                         ({3{ID_op_sel[48] || ID_op_sel[53]}} & 3'b011) |
                         ({3{ID_op_sel[49]}} & 3'b100) |
                         ({3{ID_op_sel[0] || ID_op_sel[50] || ID_op_sel[51] || ID_op_sel[1] || ID_op_sel[54] || ID_op_sel[55]}} & 3'b101);
//选择内存数据宽度 001 选择 B  010 选择 BU 011 H  100 hu  101 w
//存储和加载并�? 上下两个数据
assign ID_MemDataCombine = ({2{ID_op_sel[50] || ID_op_sel[54]}} & 2'b01) | ({2{ID_op_sel[51] || ID_op_sel[55]}} & 2'b10);
//01 lwl  10 lwr

//分支处理
assign ID_NoDirectPCBranch = (ID_SignExt_imm150 << 2) + ID_NextPC;
assign ID_DirectPCBranch = {ID_NextPC[31:28], ID_InsIdx, 2'b0};
assign ID_PCBranch = (ID_Jr || ID_Jalr) ? ID_rdata1 : ((ID_Jal || ID_J) ? ID_DirectPCBranch : ID_NoDirectPCBranch); //选择jal、jr的特殊处理方�????
assign ID_Equal = (ID_rdata1 == ID_rdata2);//分支条件
assign ID_NoEqual = ~ID_Equal;
assign ID_GreatEqual = (ID_rdata1[31] == 1'b0);
assign ID_Great = (ID_rdata1[31] == 1'b0) && (ID_rdata1 != 32'b0);
assign ID_LowEqual = (ID_rdata1[31] == 1'b1) || (ID_rdata1 == 32'b0);
assign ID_Low = (ID_rdata1[31] == 1'b1);
assign ID_BranchEnable = (ID_NoEqual && (ID_BranchCond == 3'b010)) 
                         || (ID_Equal && (ID_BranchCond == 3'b001))
                         || (ID_GreatEqual && (ID_BranchCond == 3'b011))
                         || (ID_Great && (ID_BranchCond == 3'b100))
                         || (ID_LowEqual && (ID_BranchCond == 3'b101))
                         || (ID_Low && (ID_BranchCond == 3'b110))
                         || ID_DirectBranch;

assign ID_PCSrc = ID_Branch && ID_BranchEnable && ID_valid;   

//LO/HI特殊寄存器写
always @(posedge clk) begin
    if (WB_SpecialRegWri && WB_valid && WB_SpecialRegSel[0]) begin
        LO <= WB_LOVal;
    end

    if (WB_SpecialRegWri && WB_valid && WB_SpecialRegSel[1]) begin
        HI <= WB_HIVal;
    end
end

endmodule
