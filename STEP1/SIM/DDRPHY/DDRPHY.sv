`include "TIME_SCALE.svh"
`include "SAL_DDR_PARAMS.svh"

module DDRPHY
(
    input   wire                    clk,
    input   wire                    rst_n,

    // DFI interface (interface with the controller)
    DFI_CTRL_IF.DST                 dfi_ctrl_if,
    DFI_WR_IF.DST                   dfi_wr_if,
    DFI_RD_IF.SRC                   dfi_rd_if,

    // command and address
    output  logic                   ck,
    output  logic                   ck_n,
    output  logic                   cke,
    output  logic   [`DRAM_CS_WIDTH-1:0]    cs_n,
    output  logic                   ras_n,
    output  logic                   cas_n,
    output  logic                   we_n,
    output  logic   [`DRAM_BA_WIDTH-1:0]    ba,
    output  logic   [`DRAM_ADDR_WIDTH-1:0]  addr,
    output  logic                   odt,

    //data
    inout   wire    [63:0]          dq,
    inout   wire    [7:0]           dqs,
    inout   wire    [7:0]           dqs_n,
    inout   wire    [7:0]           dm_rdqs,
    input   wire    [7:0]           rdqs_n
);

    assign  ck                      = clk;
    assign  ck_n                    = ~clk;

    // delay control signals by a half cycle to align the signals
    always_ff @(negedge clk) begin  // NEGedge
        if(!rst_n) begin
            cke                         <= 1'b0;
            cs_n                        <= {`DDR_CS_WIDTH{1'b1}};
            ras_n                       <= 1'b1;
            cas_n                       <= 1'b1;
            we_n                        <= 1'b1;
            ba                          <= 'h0;
            addr                        <= 'h0;
            odt                         <= 'h0;
        end
        else begin
            cke                         <= dfi_ctrl_if.cke;
            cs_n                        <= dfi_ctrl_if.cs_n;
            ras_n                       <= dfi_ctrl_if.ras_n;
            cas_n                       <= dfi_ctrl_if.cas_n;
            we_n                        <= dfi_ctrl_if.we_n;
            ba                          <= dfi_ctrl_if.ba;
            addr                        <= dfi_ctrl_if.addr;
            odt                         <= dfi_ctrl_if.odt;
        end
    end

    //----------------------------------------------------------
    // Write path
    //----------------------------------------------------------
    logic                           wren,       wren_d1;
    logic   [127:0]                 wdata;
    logic   [15:0]                  wmask;

    always_ff @(posedge clk) begin
        if(!rst_n) begin
            wren                        <= 'h0;
            wren_d1                     <= 'h0;
        end
        else begin
            wren                        <= dfi_wr_if.wrdata_en;
            wren_d1                     <= wren;
        end
    end
    
    logic                           wren_n,     wren_d1_n;
    logic   [127:0]                 wdata_d1;
    logic   [15:0]                  wmask_d1;
    
    always_ff @(negedge clk) begin
        if(!rst_n) begin
            wren_n                      <= 'h0;
            wren_d1_n                   <= 'h0;
            wdata                       <= 'h0;
            wmask                       <= 'h0;
        end
        else begin
            wren_n                      <= dfi_wr_if.wrdata_en;
            wren_d1_n                   <= wren_n;
            wdata                       <= dfi_wr_if.wrdata;
            wmask                       <= dfi_wr_if.wrdata_mask;
        end
    end
    
    always_ff @(negedge clk) begin
        
        if(!rst_n) begin
            wdata_d1                    <= 'h0;
            wmask_d1                    <= 'h0;
        end
        else begin
            wdata_d1                    <= wdata;
            wmask_d1                    <= wmask;
        end
    end

    //
    wire    [63:0]                  wdata_ddr;
    assign  wdata_ddr               = clk ? wdata_d1[127:64] : wdata_d1[63:0];
    assign  wmask_ddr               = clk ? wmask_d1[15:8] : wmask_d1[7:0];

    assign  dqs                     =  (wren & !wren_d1)
                                     ? 8'h00 // preamble
                                     : (wren_d1) ? {8{clk}}
                                              : 8'hz;
    assign  dqs_n                   =  (wren & !wren_d1)
                                     ? 8'hFF // preamble
                                     : (wren_d1) ? {8{~clk}}
                                              : 8'hz;
    assign  #(`CLK_PERIOD*1/4)  dq  = wren_d1_n ? wdata_ddr : 'hz;
    assign  #(`CLK_PERIOD*1/4)  dm_rdqs = wren_d1_n ? wmask_ddr : 'hz;

    //----------------------------------------------------------
    // Read path
    //----------------------------------------------------------

    //----------------------------------------------------------
    // DQS cleaning: DQS is bidirectional.
    // -> need to clean DQS to extract rDQS only
    logic                           rden_neg_d;
    always_ff @(negedge clk) begin  // NEGedge
        if(!rst_n)
            rden_neg_d                  <= 'h0;
        else
            rden_neg_d                  <= dfi_rd_if.rddata_en;
    end

    logic                           clean_rdqs;
    assign  #(`CLK_PERIOD*1/4)  clean_rdqs = dqs[0] & rden_neg_d;

    //----------------------------------------------------------
    // Capture DQ using clean DQS
    logic   [63:0]                  rdata_posedge,
                                    rdata_negedge;
    always_ff @(posedge clean_rdqs) begin
        if(!rst_n)
            rdata_posedge               <= 'h0;
        else
            rdata_posedge               <= dq;
    end
    always_ff @(negedge clean_rdqs) begin
        if(!rst_n)
            rdata_negedge               <= 'h0;
        else
            rdata_negedge               <= dq;
    end

    //----------------------------------------------------------
    // Provide read data to MC
    //----------------------------------------------------------
    logic   [1:0]                   rden_shift_reg;
    always_ff @(posedge clk)
        if (~rst_n)
            rden_shift_reg          <= 'd0;
        else
            rden_shift_reg          <= {rden_shift_reg[0], dfi_rd_if.rddata_en};

    always_ff @(posedge clk) begin
        if (~rst_n)
            dfi_rd_if.rddata       <= 'h0;
        else
            dfi_rd_if.rddata       <= {rdata_negedge, rdata_posedge};
    end
    
    assign  dfi_rd_if.rddata_valid  = rden_shift_reg[1];

endmodule
