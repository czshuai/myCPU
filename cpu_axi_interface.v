`timescale 1ns / 1ps

module cpu_axi_interface(
    input clk,
    input resetn,

    //inst sram-like
    input inst_req,
    input inst_wr,
    input [1:0] inst_size,
    input [31:0] inst_addr,
    input [31:0] inst_wdata,
    output reg inst_addr_ok,
    output reg inst_data_ok,
    output reg [31:0] inst_rdata,

    //data sram-like
    input data_req,
    input data_wr,
    input [1:0] data_size,
    input [31:0] data_addr,
    input [31:0] data_wdata,
    output reg data_addr_ok,
    output reg data_data_ok,
    output reg [31:0] data_rdata,

    //axi
    //ar
    output [3:0] arid,
    output reg [31:0] araddr,
    output [7:0] arlen,
    output reg [2:0] arsize,
    output [1:0] arburst,
    output [1:0] arlock,
    output [3:0] arcache,
    output [2:0] arprot,
    output reg arvalid,
    input arready,

    //r
    input [3:0] rid,
    input [31:0] rdata,
    input [1:0] rresp,
    input rlast,
    input rvalid,
    output reg rready,

    //aw
    output [3:0] awid,
    output reg [31:0] awaddr,
    output [7:0] awlen,
    output reg [2:0] awsize,
    output [1:0] awburst,
    output [1:0] awlock,
    output [3:0] awcache,
    output [2:0] awprot,
    output reg awvalid,
    input awready,

    //w
    output [3:0] wid,
    output reg [31:0] wdata,
    output reg [3:0] wstrb,
    output wlast,
    output reg wvalid,
    input wready,

    //b
    input [3:0] bid,
    input [1:0] bresp,
    input bvalid,
    output reg bready
);

assign arid = 4'b0;
assign arlen = 8'b0;
assign arburst = 2'b01;
assign arlock = 2'b0;
assign arcache = 4'b0;
assign arprot = 3'b0;

assign awid = 4'b0;
assign awlen = 8'b0;
assign awburst = 2'b01;
assign awlock = 2'b0;
assign awcache = 4'b0;
assign awprot = 3'b0;

assign wid = 4'b0;
assign wlast = 1'b1;

//reg [1:0] tmp_inst_size;
//reg [31:0] tmp_inst_addr;

reg [5:0] state;
reg InstEnable;
reg DataEnable;

localparam S0 = 6'b000001;
localparam S1 = 6'b000010;
localparam S2 = 6'b000100;
localparam S3 = 6'b001000;
localparam S4 = 6'b010000;
localparam S5 = 6'b100000;

always @(posedge clk) begin
    if(~resetn) begin
        state <= S0;
        data_addr_ok <= 1'b0;
        inst_addr_ok <= 1'b0;
        arvalid <= 1'b0;
        awvalid <= 1'b0;
        rready <= 1'b0;
        wvalid <= 1'b0;
        bready <= 1'b0;
    end
    else begin
        case (state)
            S0: begin
                    if (data_req == 1'b1 && data_addr_ok == 1'b0) begin
                        DataEnable <= 1'b1;
                        data_addr_ok <= 1'b1;
                        state <= S1;
                    end
                    else if (inst_req == 1'b1 && inst_addr_ok == 1'b0) begin
                        InstEnable <= 1'b1;
                        inst_addr_ok <= 1'b1;
                        state <= S1;
                    end
                end
            S1: begin
                    if (data_wr == 1'b0 && data_addr_ok == 1'b1) begin
                        araddr <= data_addr;
                        arsize <= {1'b0, data_size};
                        arvalid <= 1'b1;
                        data_data_ok <= 1'b0;
                        data_addr_ok <= 1'b0;
                        state <= S2;
                    end
                    else if (data_wr == 1'b1 && data_addr_ok == 1'b1) begin
                        awaddr <= data_addr;
                        awsize <= {1'b0, data_size};
                        wdata <= data_wdata;
                        awvalid <= 1'b1;
                        data_data_ok <= 1'b0;
                        data_addr_ok <= 1'b0;
                        wstrb <= {4{(data_size == 2'b0 && data_addr[1:0] == 2'b0)}} & 4'b1 | 
                                 {4{(data_size == 2'b0 && data_addr[1:0] == 2'b1)}} & 4'b10 | 
                                 {4{(data_size == 2'b0 && data_addr[1:0] == 2'b10)}} & 4'b100 |
                                 {4{(data_size == 2'b0 && data_addr[1:0] == 2'b11)}} & 4'b1000 |
                                 {4{(data_size == 2'b1 && (data_addr[1:0] == 2'b0 || data_addr[1:0] == 2'b1))}} & 4'b11 |
                                 {4{(data_size == 2'b1 && data_addr[1:0] == 2'b10)}} & 4'b1100 |
                                 {4{(data_size == 2'b10 && (data_addr[1:0] == 2'b0 || data_addr[1:0] == 2'b11))}} & 4'b1111 |
                                 {4{(data_size == 2'b11 && data_addr[1:0] == 2'b10)}} & 4'b0111 |
                                 {4{(data_size == 2'b11 && data_addr[1:0] == 2'b1)}} & 4'b1110;
                        state <= S2;
                    end
                    else if (inst_wr == 1'b0 && inst_addr_ok == 1'b1) begin
                        araddr <= inst_addr;
                        arsize <= {1'b0, inst_size};
                        arvalid <= 1'b1;
                        inst_data_ok <= 1'b0;
                        inst_addr_ok <= 1'b0;
                        state <= S2;
                    end
                end
            S2: begin
                    if (arready == 1'b1 && arvalid == 1'b1) begin
                        arvalid <= 1'b0;
                        rready <= 1'b1;
                        state <= S3;
                    end
                    else if (awready == 1'b1 && awvalid == 1'b1) begin
                        awvalid <= 1'b0;
                        wvalid <= 1'b1;
                        bready <= 1'b1;
                        state <= S3;
                    end
                end
            S3: begin
                    if (rready == 1'b1 && rvalid == 1'b1 && InstEnable) begin
                        inst_rdata <= rdata;
                        rready <= 1'b0;
                        inst_data_ok <= 1'b1;
                        InstEnable <= 1'b0;
                        state <= S0;
                    end
                    else if (rready == 1'b1 && rvalid == 1'b1 && DataEnable) begin
                        data_rdata <= rdata;
                        rready <= 1'b0;
                        data_data_ok <= 1'b1;
                        DataEnable <= 1'b0;
                        state <= S0;
                    end
                    else if (wvalid == 1'b1 && wready == 1'b1) begin
                        wvalid <= 1'b0;
                        state <= S4;
                    end
                end
            S4: begin
                    if (bvalid == 1'b1 && bready == 1'b1) begin
                        bready <= 1'b0;
                        data_data_ok <= 1'b1;
                        state <= S0;
                    end
                end
            default: begin 
                        state <= S0;
                     end 
        endcase
    end
end 

endmodule
