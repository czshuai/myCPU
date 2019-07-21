`timescale 1ns / 1ps
//x/y   //执行需要34个周期
module div(
    input div_clk, resetn,
    input div,
    input div_signed,
    input [31:0] x, y,
    output [31:0] s, r,
    output reg complete
    );

reg [32:0] UnsignS;
reg [32:0] UnsignR;
reg [32:0] tmp_r;
reg [7:0] count;
wire [32:0] tmp_d;
wire [32:0] result_r;
wire [32:0] UnsignX, UnsignY;

assign UnsignX = {1'b0, (div_signed ? (x[31] ? (~x + 1) : x) : x)}; //取绝对值并扩展至33位
assign UnsignY = {1'b0, (div_signed ? (y[31] ? (~y + 1) : y) : y)};

always @(posedge div_clk) begin  //33位除法计算
    if (~resetn || ~div) begin
        count <= 8'd32;     //计算33次
        complete <= 1'b0;
        tmp_r <= 33'b0;
    end
    else if (~(count[7])) begin
        if (tmp_d[32]) begin    //tmp_d为负数
            UnsignS <= {UnsignS[31:0], 1'b0};
            tmp_r <= result_r;
        end 
        else begin
            UnsignS <= {UnsignS[31:0], 1'b1};
            tmp_r <= tmp_d;
        end
        count <= count - 8'd1;
    end
    else begin
        complete <= 1'b1;
        UnsignR <= tmp_r;
    end

end

assign result_r = {tmp_r[31:0], UnsignX[count]};
assign tmp_d = result_r - UnsignY;

wire [32:0] TmpS, TmpR;
assign TmpS = (div_signed ? ((x[31] == y[31]) ? UnsignS : ~(UnsignS - 1)) : UnsignS); //去绝对值并截位
assign TmpR = (div_signed ? (x[31] ? ~(UnsignR - 1) : UnsignR) : UnsignR);

assign s = TmpS[31:0];
assign r = TmpR[31:0];

endmodule

//表达式的符号关系
//x[31]  y[31]  s[31]  r[31]
//  0      0      0      0
//  0      1      1      0
//  1      0      1      1
//  1      1      0      1