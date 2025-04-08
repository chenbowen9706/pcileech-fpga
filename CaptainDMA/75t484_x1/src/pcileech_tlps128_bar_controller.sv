//
// PCILeech FPGA.
//
// PCIe BAR PIO controller.
//
// The PCILeech BAR PIO controller allows for easy user-implementation on top
// of the PCILeech AXIS128 PCIe TLP streaming interface.
// The controller consists of a read engine and a write engine and pluggable
// user-implemented PCIe BAR implementations (found at bottom of the file).
//
// Considerations:
// - The core handles 1 DWORD read + 1 DWORD write per CLK max. If a lot of
//   data is written / read from the TLP streaming interface the core may
//   drop packet silently.
// - The core reads 1 DWORD of data (without byte enable) per CLK.
// - The core writes 1 DWORD of data (with byte enable) per CLK.
// - All user-implemented cores must have the same latency in CLKs for the
//   returned read data or else undefined behavior will take place.
// - 32-bit addresses are passed for read/writes. Larger BARs than 4GB are
//   not supported due to addressing constraints. Lower bits (LSBs) are the
//   BAR offset, Higher bits (MSBs) are the 32-bit base address of the BAR.
// - DO NOT edit read/write engines.
// - DO edit pcileech_tlps128_bar_controller (to swap bar implementations).
// - DO edit the bar implementations (at bottom of the file, if neccessary).
//
// Example implementations exists below, swap out any of the example cores
// against a core of your use case, or modify existing cores.
// Following test cores exist (see below in this file):
// - pcileech_bar_impl_zerowrite4k = zero-initialized read/write BAR.
//     It's possible to modify contents by use of .coe file.
// - pcileech_bar_impl_loopaddr = test core that loops back the 32-bit
//     address of the current read. Does not support writes.
// - pcileech_bar_impl_none = core without any reply.
// 
// (c) Ulf Frisk, 2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_tlps128_bar_controller(
    input                   rst,
    input                   clk,
    input                   bar_en,
    input [15:0]            pcie_id,
    input [31:0]            base_address_register,//anpanman
    IfAXIS128.sink_lite     tlps_in,
    IfAXIS128.source        tlps_out
);
    
    // ------------------------------------------------------------------------
    // 1: TLP RECEIVE:
    // Receive incoming BAR requests from the TLP stream:
    // send them onwards to read and write FIFOs
    // ------------------------------------------------------------------------
    wire in_is_wr_ready;
    bit  in_is_wr_last;
    wire in_is_first    = tlps_in.tuser[0];
    wire in_is_bar      = bar_en && (tlps_in.tuser[8:2] != 0);
    wire in_is_rd       = (in_is_first && tlps_in.tlast && ((tlps_in.tdata[31:25] == 7'b0000000) || (tlps_in.tdata[31:25] == 7'b0010000) || (tlps_in.tdata[31:24] == 8'b00000010)));
    wire in_is_wr       = in_is_wr_last || (in_is_first && in_is_wr_ready && ((tlps_in.tdata[31:25] == 7'b0100000) || (tlps_in.tdata[31:25] == 7'b0110000) || (tlps_in.tdata[31:24] == 8'b01000010)));
    
    always @ ( posedge clk )
        if ( rst ) begin
            in_is_wr_last <= 0;
        end
        else if ( tlps_in.tvalid ) begin
            in_is_wr_last <= !tlps_in.tlast && in_is_wr;
        end
    
    wire [6:0]  wr_bar;
    wire [31:0] wr_addr;
    wire [3:0]  wr_be;
    wire [31:0] wr_data;
    wire        wr_valid;
    wire [87:0] rd_req_ctx;
    wire [6:0]  rd_req_bar;
    wire [31:0] rd_req_addr;
    wire        rd_req_valid;
    wire [87:0] rd_rsp_ctx;
    wire [31:0] rd_rsp_data;
    wire        rd_rsp_valid;
        
    pcileech_tlps128_bar_rdengine i_pcileech_tlps128_bar_rdengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .pcie_id        ( pcie_id                       ),
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_rd ),
        .tlps_out       ( tlps_out                      ),
        // BAR reads:
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_bar     ( rd_req_bar                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid                  ),
        .rd_rsp_ctx     ( rd_rsp_ctx                    ),
        .rd_rsp_data    ( rd_rsp_data                   ),
        .rd_rsp_valid   ( rd_rsp_valid                  )
    );

    pcileech_tlps128_bar_wrengine i_pcileech_tlps128_bar_wrengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_wr ),
        .tlps_in_ready  ( in_is_wr_ready                ),
        // outgoing BAR writes:
        .wr_bar         ( wr_bar                        ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid                      )
    );
    
    wire [87:0] bar_rsp_ctx[7];
    wire [31:0] bar_rsp_data[7];
    wire        bar_rsp_valid[7];
    
    assign rd_rsp_ctx = bar_rsp_valid[0] ? bar_rsp_ctx[0] :
                        bar_rsp_valid[1] ? bar_rsp_ctx[1] :
                        bar_rsp_valid[2] ? bar_rsp_ctx[2] :
                        bar_rsp_valid[3] ? bar_rsp_ctx[3] :
                        bar_rsp_valid[4] ? bar_rsp_ctx[4] :
                        bar_rsp_valid[5] ? bar_rsp_ctx[5] :
                        bar_rsp_valid[6] ? bar_rsp_ctx[6] : 0;
    assign rd_rsp_data = bar_rsp_valid[0] ? bar_rsp_data[0] :
                        bar_rsp_valid[1] ? bar_rsp_data[1] :
                        bar_rsp_valid[2] ? bar_rsp_data[2] :
                        bar_rsp_valid[3] ? bar_rsp_data[3] :
                        bar_rsp_valid[4] ? bar_rsp_data[4] :
                        bar_rsp_valid[5] ? bar_rsp_data[5] :
                        bar_rsp_valid[6] ? bar_rsp_data[6] : 0;
    assign rd_rsp_valid = bar_rsp_valid[0] || bar_rsp_valid[1] || bar_rsp_valid[2] || bar_rsp_valid[3] || bar_rsp_valid[4] || bar_rsp_valid[5] || bar_rsp_valid[6];
    
    pcileech_bar_impl_bar i_bar0(//anpanman
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[0]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[0] ),
        .base_address_register ( base_address_register         ),//anpanman
        .rd_rsp_ctx     ( bar_rsp_ctx[0]                ),
        .rd_rsp_data    ( bar_rsp_data[0]               ),
        .rd_rsp_valid   ( bar_rsp_valid[0]              )
    );
    
    pcileech_bar_impl_loopaddr i_bar1(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[1]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[1] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[1]                ),
        .rd_rsp_data    ( bar_rsp_data[1]               ),
        .rd_rsp_valid   ( bar_rsp_valid[1]              )
    );
    
    pcileech_bar_impl_none i_bar2(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[2]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[2] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[2]                ),
        .rd_rsp_data    ( bar_rsp_data[2]               ),
        .rd_rsp_valid   ( bar_rsp_valid[2]              )
    );
    
    pcileech_bar_impl_none i_bar3(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[3]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[3] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[3]                ),
        .rd_rsp_data    ( bar_rsp_data[3]               ),
        .rd_rsp_valid   ( bar_rsp_valid[3]              )
    );
    
    pcileech_bar_impl_none i_bar4(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[4]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[4] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[4]                ),
        .rd_rsp_data    ( bar_rsp_data[4]               ),
        .rd_rsp_valid   ( bar_rsp_valid[4]              )
    );
    
    pcileech_bar_impl_none i_bar5(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[5]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[5] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[5]                ),
        .rd_rsp_data    ( bar_rsp_data[5]               ),
        .rd_rsp_valid   ( bar_rsp_valid[5]              )
    );
    
    pcileech_bar_impl_none i_bar6_optrom(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[6]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[6] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[6]                ),
        .rd_rsp_data    ( bar_rsp_data[6]               ),
        .rd_rsp_valid   ( bar_rsp_valid[6]              )
    );


endmodule



// ------------------------------------------------------------------------
// BAR WRITE ENGINE:
// Receives BAR WRITE TLPs and output BAR WRITE requests.
// Holds a 2048-byte buffer.
// Input flow rate is 16bytes/CLK (max).
// Output flow rate is 4bytes/CLK.
// If write engine overflows incoming TLP is completely discarded silently.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_wrengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    output                  tlps_in_ready,
    // outgoing BAR writes:
    output bit [6:0]        wr_bar,
    output bit [31:0]       wr_addr,
    output bit [3:0]        wr_be,
    output bit [31:0]       wr_data,
    output bit              wr_valid
);

    wire            f_rd_en;
    wire [127:0]    f_tdata;
    wire [3:0]      f_tkeepdw;
    wire [8:0]      f_tuser;
    wire            f_tvalid;
    
    bit [127:0]     tdata;
    bit [3:0]       tkeepdw;
    bit             tlast;
    
    bit [3:0]       be_first;
    bit [3:0]       be_last;
    bit             first_dw;
    bit [31:0]      addr;

    fifo_141_141_clk1_bar_wr i_fifo_141_141_clk1_bar_wr(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( {tlps_in.tuser[8:0], tlps_in.tkeepdw, tlps_in.tdata} ),
        .full           (                               ),
        .prog_empty     ( tlps_in_ready                 ),
        .rd_en          ( f_rd_en                       ),
        .dout           ( {f_tuser, f_tkeepdw, f_tdata} ),    
        .empty          (                               ),
        .valid          ( f_tvalid                      )
    );
    
    // STATE MACHINE:
    `define S_ENGINE_IDLE        3'h0
    `define S_ENGINE_FIRST       3'h1
    `define S_ENGINE_4DW_REQDATA 3'h2
    `define S_ENGINE_TX0         3'h4
    `define S_ENGINE_TX1         3'h5
    `define S_ENGINE_TX2         3'h6
    `define S_ENGINE_TX3         3'h7
    (* KEEP = "TRUE" *) bit [3:0] state = `S_ENGINE_IDLE;
    
    assign f_rd_en = (state == `S_ENGINE_IDLE) ||
                     (state == `S_ENGINE_4DW_REQDATA) ||
                     (state == `S_ENGINE_TX3) ||
                     ((state == `S_ENGINE_TX2 && !tkeepdw[3])) ||
                     ((state == `S_ENGINE_TX1 && !tkeepdw[2])) ||
                     ((state == `S_ENGINE_TX0 && !f_tkeepdw[1]));

    always @ ( posedge clk ) begin
        wr_addr     <= addr;
        wr_valid    <= ((state == `S_ENGINE_TX0) && f_tvalid) || (state == `S_ENGINE_TX1) || (state == `S_ENGINE_TX2) || (state == `S_ENGINE_TX3);
        
    end

    always @ ( posedge clk )
        if ( rst ) begin
            state <= `S_ENGINE_IDLE;
        end
        else case ( state )
            `S_ENGINE_IDLE: begin
                state   <= `S_ENGINE_FIRST;
            end
            `S_ENGINE_FIRST: begin
                if ( f_tvalid && f_tuser[0] ) begin
                    wr_bar      <= f_tuser[8:2];
                    tdata       <= f_tdata;
                    tkeepdw     <= f_tkeepdw;
                    tlast       <= f_tuser[1];
                    first_dw    <= 1;
                    be_first    <= f_tdata[35:32];
                    be_last     <= f_tdata[39:36];
                    if ( f_tdata[31:29] == 8'b010 ) begin       // 3 DW header, with data
                        addr    <= { f_tdata[95:66], 2'b00 };
                        state   <= `S_ENGINE_TX3;
                    end
                    else if ( f_tdata[31:29] == 8'b011 ) begin  // 4 DW header, with data
                        addr    <= { f_tdata[127:98], 2'b00 };
                        state   <= `S_ENGINE_4DW_REQDATA;
                    end 
                end
                else begin
                    state   <= `S_ENGINE_IDLE;
                end
            end 
            `S_ENGINE_4DW_REQDATA: begin
                state   <= `S_ENGINE_TX0;
            end
            `S_ENGINE_TX0: begin
                tdata       <= f_tdata;
                tkeepdw     <= f_tkeepdw;
                tlast       <= f_tuser[1];
                addr        <= addr + 4;
                wr_data     <= { f_tdata[0+00+:8], f_tdata[0+08+:8], f_tdata[0+16+:8], f_tdata[0+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (f_tkeepdw[1] ? 4'hf : be_last);
                state       <= f_tvalid ? (f_tkeepdw[1] ? `S_ENGINE_TX1 : `S_ENGINE_FIRST) : `S_ENGINE_IDLE;
            end
            `S_ENGINE_TX1: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[32+00+:8], tdata[32+08+:8], tdata[32+16+:8], tdata[32+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[2] ? 4'hf : be_last);
                state       <= tkeepdw[2] ? `S_ENGINE_TX2 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX2: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[64+00+:8], tdata[64+08+:8], tdata[64+16+:8], tdata[64+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[3] ? 4'hf : be_last);
                state       <= tkeepdw[3] ? `S_ENGINE_TX3 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX3: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[96+00+:8], tdata[96+08+:8], tdata[96+16+:8], tdata[96+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (!tlast ? 4'hf : be_last);
                state       <= !tlast ? `S_ENGINE_TX0 : `S_ENGINE_FIRST;
            end
        endcase

endmodule



// ------------------------------------------------------------------------
// BAR READ ENGINE:
// Receives BAR READ TLPs and output BAR READ requests.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_rdengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    input [15:0]            pcie_id,
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    IfAXIS128.source        tlps_out,
    // BAR reads:
    output [87:0]           rd_req_ctx,
    output [6:0]            rd_req_bar,
    output [31:0]           rd_req_addr,
    output                  rd_req_valid,
    input  [87:0]           rd_rsp_ctx,
    input  [31:0]           rd_rsp_data,
    input                   rd_rsp_valid
);

    // ------------------------------------------------------------------------
    // 1: PROCESS AND QUEUE INCOMING READ TLPs:
    // ------------------------------------------------------------------------
    wire [10:0] rd1_in_dwlen    = (tlps_in.tdata[9:0] == 0) ? 11'd1024 : {1'b0, tlps_in.tdata[9:0]};
    wire [6:0]  rd1_in_bar      = tlps_in.tuser[8:2];
    wire [15:0] rd1_in_reqid    = tlps_in.tdata[63:48];
    wire [7:0]  rd1_in_tag      = tlps_in.tdata[47:40];
    wire [31:0] rd1_in_addr     = { ((tlps_in.tdata[31:29] == 3'b000) ? tlps_in.tdata[95:66] : tlps_in.tdata[127:98]), 2'b00 };
    wire [73:0] rd1_in_data;
    assign rd1_in_data[73:63]   = rd1_in_dwlen;
    assign rd1_in_data[62:56]   = rd1_in_bar;   
    assign rd1_in_data[55:48]   = rd1_in_tag;
    assign rd1_in_data[47:32]   = rd1_in_reqid;
    assign rd1_in_data[31:0]    = rd1_in_addr;
    
    wire        rd1_out_rden;
    wire [73:0] rd1_out_data;
    wire        rd1_out_valid;
    
    fifo_74_74_clk1_bar_rd1 i_fifo_74_74_clk1_bar_rd1(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( rd1_in_data                   ),
        .full           (                               ),
        .rd_en          ( rd1_out_rden                  ),
        .dout           ( rd1_out_data                  ),    
        .empty          (                               ),
        .valid          ( rd1_out_valid                 )
    );
    
    // ------------------------------------------------------------------------
    // 2: PROCESS AND SPLIT READ TLPs INTO RESPONSE TLP READ REQUESTS AND QUEUE:
    //    (READ REQUESTS LARGER THAN 128-BYTES WILL BE SPLIT INTO MULTIPLE).
    // ------------------------------------------------------------------------
    
    wire [10:0] rd1_out_dwlen       = rd1_out_data[73:63];
    wire [4:0]  rd1_out_dwlen5      = rd1_out_data[67:63];
    wire [4:0]  rd1_out_addr5       = rd1_out_data[6:2];
    
    // 1st "instant" packet:
    wire [4:0]  rd2_pkt1_dwlen_pre  = ((rd1_out_addr5 + rd1_out_dwlen5 > 6'h20) || ((rd1_out_addr5 != 0) && (rd1_out_dwlen5 == 0))) ? (6'h20 - rd1_out_addr5) : rd1_out_dwlen5;
    wire [5:0]  rd2_pkt1_dwlen      = (rd2_pkt1_dwlen_pre == 0) ? 6'h20 : rd2_pkt1_dwlen_pre;
    wire [10:0] rd2_pkt1_dwlen_next = rd1_out_dwlen - rd2_pkt1_dwlen;
    wire        rd2_pkt1_large      = (rd1_out_dwlen > 32) || (rd1_out_dwlen != rd2_pkt1_dwlen);
    wire        rd2_pkt1_tiny       = (rd1_out_dwlen == 1);
    wire [11:0] rd2_pkt1_bc         = rd1_out_dwlen << 2;
    wire [85:0] rd2_pkt1;
    assign      rd2_pkt1[85:74]     = rd2_pkt1_bc;
    assign      rd2_pkt1[73:63]     = rd2_pkt1_dwlen;
    assign      rd2_pkt1[62:0]      = rd1_out_data[62:0];
    
    // Nth packet (if split should take place):
    bit  [10:0] rd2_total_dwlen;
    wire [10:0] rd2_total_dwlen_next = rd2_total_dwlen - 11'h20;
    
    bit  [85:0] rd2_pkt2;
    wire [10:0] rd2_pkt2_dwlen = rd2_pkt2[73:63];
    wire        rd2_pkt2_large = (rd2_total_dwlen > 11'h20);
    
    wire        rd2_out_rden;
    
    // STATE MACHINE:
    `define S2_ENGINE_REQDATA     1'h0
    `define S2_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state2 = `S2_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            state2 <= `S2_ENGINE_REQDATA;
        end
        else case ( state2 )
            `S2_ENGINE_REQDATA: begin
                if ( rd1_out_valid && rd2_pkt1_large ) begin
                    rd2_total_dwlen <= rd2_pkt1_dwlen_next;                             // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_pkt1_dwlen_next << 2;                        // byte-count
                    rd2_pkt2[73:63] <= (rd2_pkt1_dwlen_next > 11'h20) ? 11'h20 : rd2_pkt1_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd1_out_data[62:12];                             // various data
                    rd2_pkt2[11:0]  <= rd1_out_data[11:0] + (rd2_pkt1_dwlen << 2);      // base address (within 4k page)
                    state2 <= `S2_ENGINE_PROCESSING;
                end
            end
            `S2_ENGINE_PROCESSING: begin
                if ( rd2_out_rden ) begin
                    rd2_total_dwlen <= rd2_total_dwlen_next;                                // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_total_dwlen_next << 2;                           // byte-count
                    rd2_pkt2[73:63] <= (rd2_total_dwlen_next > 11'h20) ? 11'h20 : rd2_total_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd2_pkt2[62:12];                                     // various data
                    rd2_pkt2[11:0]  <= rd2_pkt2[11:0] + (rd2_pkt2_dwlen << 2);              // base address (within 4k page)
                    if ( !rd2_pkt2_large ) begin
                        state2 <= `S2_ENGINE_REQDATA;
                    end
                end
            end
        endcase
    
    assign rd1_out_rden = rd2_out_rden && (((state2 == `S2_ENGINE_REQDATA) && (!rd1_out_valid || rd2_pkt1_tiny)) || ((state2 == `S2_ENGINE_PROCESSING) && !rd2_pkt2_large));

    wire [85:0] rd2_in_data  = (state2 == `S2_ENGINE_REQDATA) ? rd2_pkt1 : rd2_pkt2;
    wire        rd2_in_valid = rd1_out_valid || ((state2 == `S2_ENGINE_PROCESSING) && rd2_out_rden);

    bit  [85:0] rd2_out_data;
    bit         rd2_out_valid;
    always @ ( posedge clk ) begin
        rd2_out_data    <= rd2_in_valid ? rd2_in_data : rd2_out_data;
        rd2_out_valid   <= rd2_in_valid && !rst;
    end

    // ------------------------------------------------------------------------
    // 3: PROCESS EACH READ REQUEST PACKAGE PER INDIVIDUAL 32-bit READ DWORDS:
    // ------------------------------------------------------------------------

    wire [4:0]  rd2_out_dwlen   = rd2_out_data[67:63];
    wire        rd2_out_last    = (rd2_out_dwlen == 1);
    wire [9:0]  rd2_out_dwaddr  = rd2_out_data[11:2];
    
    wire        rd3_enable;
    
    bit         rd3_process_valid;
    bit         rd3_process_first;
    bit         rd3_process_last;
    bit [4:0]   rd3_process_dwlen;
    bit [9:0]   rd3_process_dwaddr;
    bit [85:0]  rd3_process_data;
    wire        rd3_process_next_last = (rd3_process_dwlen == 2);
    wire        rd3_process_nextnext_last = (rd3_process_dwlen <= 3);
    
    assign rd_req_ctx   = { rd3_process_first, rd3_process_last, rd3_process_data };
    assign rd_req_bar   = rd3_process_data[62:56];
    assign rd_req_addr  = { rd3_process_data[31:12], rd3_process_dwaddr, 2'b00 };
    assign rd_req_valid = rd3_process_valid;
    
    // STATE MACHINE:
    `define S3_ENGINE_REQDATA     1'h0
    `define S3_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state3 = `S3_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            rd3_process_valid   <= 1'b0;
            state3              <= `S3_ENGINE_REQDATA;
        end
        else case ( state3 )
            `S3_ENGINE_REQDATA: begin
                if ( rd2_out_valid ) begin
                    rd3_process_valid       <= 1'b1;
                    rd3_process_first       <= 1'b1;                    // FIRST
                    rd3_process_last        <= rd2_out_last;            // LAST (low 5 bits of dwlen == 1, [max pktlen = 0x20))
                    rd3_process_dwlen       <= rd2_out_dwlen;           // PKT LENGTH IN DW
                    rd3_process_dwaddr      <= rd2_out_dwaddr;          // DWADDR OF THIS DWORD
                    rd3_process_data[85:0]  <= rd2_out_data[85:0];      // FORWARD / SAVE DATA
                    if ( !rd2_out_last ) begin
                        state3 <= `S3_ENGINE_PROCESSING;
                    end
                end
                else begin
                    rd3_process_valid       <= 1'b0;
                end
            end
            `S3_ENGINE_PROCESSING: begin
                rd3_process_first           <= 1'b0;                    // FIRST
                rd3_process_last            <= rd3_process_next_last;   // LAST
                rd3_process_dwlen           <= rd3_process_dwlen - 1;   // LEN DEC
                rd3_process_dwaddr          <= rd3_process_dwaddr + 1;  // ADDR INC
                if ( rd3_process_next_last ) begin
                    state3 <= `S3_ENGINE_REQDATA;
                end
            end
        endcase

    assign rd2_out_rden = rd3_enable && (
        ((state3 == `S3_ENGINE_REQDATA) && (!rd2_out_valid || rd2_out_last)) ||
        ((state3 == `S3_ENGINE_PROCESSING) && rd3_process_nextnext_last));
    
    // ------------------------------------------------------------------------
    // 4: PROCESS RESPONSES:
    // ------------------------------------------------------------------------
    
    wire        rd_rsp_first    = rd_rsp_ctx[87];
    wire        rd_rsp_last     = rd_rsp_ctx[86];
    
    wire [9:0]  rd_rsp_dwlen    = rd_rsp_ctx[72:63];
    wire [11:0] rd_rsp_bc       = rd_rsp_ctx[85:74];
    wire [15:0] rd_rsp_reqid    = rd_rsp_ctx[47:32];
    wire [7:0]  rd_rsp_tag      = rd_rsp_ctx[55:48];
    wire [6:0]  rd_rsp_lowaddr  = rd_rsp_ctx[6:0];
    wire [31:0] rd_rsp_addr     = rd_rsp_ctx[31:0];
    wire [31:0] rd_rsp_data_bs  = { rd_rsp_data[7:0], rd_rsp_data[15:8], rd_rsp_data[23:16], rd_rsp_data[31:24] };
    
    // 1: 32-bit -> 128-bit state machine:
    bit [127:0] tdata;
    bit [3:0]   tkeepdw = 0;
    bit         tlast;
    bit         first   = 1;
    wire        tvalid  = tlast || tkeepdw[3];
    
    always @ ( posedge clk )
        if ( rst ) begin
            tkeepdw <= 0;
            tlast   <= 0;
            first   <= 0;
        end
        else if ( rd_rsp_valid && rd_rsp_first ) begin
            tkeepdw         <= 4'b1111;
            tlast           <= rd_rsp_last;
            first           <= 1'b1;
            tdata[31:0]     <= { 22'b0100101000000000000000, rd_rsp_dwlen };            // format, type, length
            tdata[63:32]    <= { pcie_id[7:0], pcie_id[15:8], 4'b0, rd_rsp_bc };        // pcie_id, byte_count
            tdata[95:64]    <= { rd_rsp_reqid, rd_rsp_tag, 1'b0, rd_rsp_lowaddr };      // req_id, tag, lower_addr
            tdata[127:96]   <= rd_rsp_data_bs;
        end
        else begin
            tlast   <= rd_rsp_valid && rd_rsp_last;
            tkeepdw <= tvalid ? (rd_rsp_valid ? 4'b0001 : 4'b0000) : (rd_rsp_valid ? ((tkeepdw << 1) | 1'b1) : tkeepdw);
            first   <= 0;
            if ( rd_rsp_valid ) begin
                if ( tvalid || !tkeepdw[0] )
                    tdata[31:0]   <= rd_rsp_data_bs;
                if ( !tkeepdw[1] )
                    tdata[63:32]  <= rd_rsp_data_bs;
                if ( !tkeepdw[2] )
                    tdata[95:64]  <= rd_rsp_data_bs;
                if ( !tkeepdw[3] )
                    tdata[127:96] <= rd_rsp_data_bs;   
            end
        end
    
    // 2.1 - submit to output fifo - will feed into mux/pcie core.
    fifo_134_134_clk1_bar_rdrsp i_fifo_134_134_clk1_bar_rdrsp(
        .srst           ( rst                       ),
        .clk            ( clk                       ),
        .din            ( { first, tlast, tkeepdw, tdata } ),
        .wr_en          ( tvalid                    ),
        .rd_en          ( tlps_out.tready           ),
        .dout           ( { tlps_out.tuser[0], tlps_out.tlast, tlps_out.tkeepdw, tlps_out.tdata } ),
        .full           (                           ),
        .empty          (                           ),
        .prog_empty     ( rd3_enable                ),
        .valid          ( tlps_out.tvalid           )
    );
    
    assign tlps_out.tuser[1] = tlps_out.tlast;
    assign tlps_out.tuser[8:2] = 0;
    
    // 2.2 - packet count:
    bit [10:0]  pkt_count       = 0;
    wire        pkt_count_dec   = tlps_out.tvalid && tlps_out.tlast;
    wire        pkt_count_inc   = tvalid && tlast;
    wire [10:0] pkt_count_next  = pkt_count + pkt_count_inc - pkt_count_dec;
    assign tlps_out.has_data    = (pkt_count_next > 0);
    
    always @ ( posedge clk ) begin
        pkt_count <= rst ? 0 : pkt_count_next;
    end

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation that does nothing but drop any read/writes
// silently without generating a response.
// This is only recommended for placeholder designs.
// Latency = N/A.
// ------------------------------------------------------------------------
module pcileech_bar_impl_none(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    initial rd_rsp_ctx = 0;
    initial rd_rsp_data = 0;
    initial rd_rsp_valid = 0;

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of "address loopback" which can be useful
// for testing. Any read to a specific BAR address will result in the
// address as response.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_loopaddr(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input [87:0]        rd_req_ctx,
    input [31:0]        rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]      rd_req_ctx_1;
    bit [31:0]      rd_req_addr_1;
    bit             rd_req_valid_1;
    
    always @ ( posedge clk ) begin
        rd_req_ctx_1    <= rd_req_ctx;
        rd_req_addr_1   <= rd_req_addr;
        rd_req_valid_1  <= rd_req_valid;
        rd_rsp_ctx      <= rd_req_ctx_1;
        rd_rsp_data     <= rd_req_addr_1;
        rd_rsp_valid    <= rd_req_valid_1;
    end    

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of a 4kB writable initial-zero BAR.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_zerowrite4k(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]  drd_req_ctx;
    bit         drd_req_valid;
    wire [31:0] doutb;
    
    always @ ( posedge clk ) begin
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        rd_rsp_data     <= doutb; 
    end
    
    bram_bar_zero4k i_bram_bar_zero4k(
        // Port A - write:
        .addra  ( wr_addr[11:2]     ),
        .clka   ( clk               ),
        .dina   ( wr_data           ),
        .ena    ( wr_valid          ),
        .wea    ( wr_be             ),
        // Port A - read (2 CLK latency):
        .addrb  ( rd_req_addr[11:2] ),
        .clkb   ( clk               ),
        .doutb  ( doutb             ),
        .enb    ( rd_req_valid      )
    );

endmodule

/*anpanman*/
module pcileech_bar_impl_bar(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    // outgoing BAR read replies:
    output reg [87:0]   rd_rsp_ctx,
    output reg [31:0]   rd_rsp_data,
    output reg          rd_rsp_valid
);
                     
    reg [87:0]      drd_req_ctx;
    reg [31:0]      drd_req_addr;
    reg             drd_req_valid;
                  
    reg [31:0]      dwr_addr;
    reg [31:0]      dwr_data;
    reg             dwr_valid;
               
    reg [31:0]      data_32;
              
    time number = 0;
                  
    always @ (posedge clk) begin
        if (rst)
            number <= 0;
               
        number          <= number + 1;
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'h00FF)
        16'h0010 : rd_rsp_data <= 32'h8C8CE578;
        16'h0014 : rd_rsp_data <= 32'h4F1C8A3D;
        16'h0018 : rd_rsp_data <= 32'h61893599;
        16'h001C : rd_rsp_data <= 32'hD32DC385;
        16'h0020 : rd_rsp_data <= 32'h00040000;
        16'h0028 : rd_rsp_data <= 32'h4856465F;
        16'h002C : rd_rsp_data <= 32'h0004FEFF;
        16'h0030 : rd_rsp_data <= 32'hE62B0048;
        16'h0034 : rd_rsp_data <= 32'h02000060;
        16'h0038 : rd_rsp_data <= 32'h00000040;
        16'h003C : rd_rsp_data <= 32'h00001000;
        16'h0048 : rd_rsp_data <= 32'hFFFFFFFF;
        16'h004C : rd_rsp_data <= 32'hFFFFFFFF;
        16'h0050 : rd_rsp_data <= 32'hFFFFFFFF;
        16'h0054 : rd_rsp_data <= 32'hFFFFFFFF;
        16'h0058 : rd_rsp_data <= 32'h00F0AAF4;
        16'h005C : rd_rsp_data <= 32'hF800002C;
        16'h0060 : rd_rsp_data <= 32'h289EFB98;
        16'h0064 : rd_rsp_data <= 32'h47F59F4C;
        16'h0068 : rd_rsp_data <= 32'hA435C49A;
        16'h006C : rd_rsp_data <= 32'hA9B047D8;
        16'h0070 : rd_rsp_data <= 32'h00000014;
        16'h0074 : rd_rsp_data <= 32'hFFFFFFFF;
        16'h0078 : rd_rsp_data <= 32'h1B45CC0A;
        16'h007C : rd_rsp_data <= 32'h428A156A;
        16'h0080 : rd_rsp_data <= 32'h864962AF;
        16'h0084 : rd_rsp_data <= 32'hE6E6A04D;
        16'h0088 : rd_rsp_data <= 32'h0002AA68;
        16'h008C : rd_rsp_data <= 32'hF800007C;
        16'h0090 : rd_rsp_data <= 32'h19000064;
        16'h0094 : rd_rsp_data <= 32'hE9DD7F62;
        16'h0098 : rd_rsp_data <= 32'h4F9D25EC;
        16'h009C : rd_rsp_data <= 32'hD2AAABA4;
        16'h00A0 : rd_rsp_data <= 32'h109AF50B;
        16'h00A4 : rd_rsp_data <= 32'h6B8D7F26;
        16'h00A8 : rd_rsp_data <= 32'h4470E16E;
        16'h00AC : rd_rsp_data <= 32'h7BE67B9A;
        16'h00B0 : rd_rsp_data <= 32'h8BECD1B4;
        16'h00B4 : rd_rsp_data <= 32'h9B3ADA4F;
        16'h00B8 : rd_rsp_data <= 32'h4C24AE56;
        16'h00BC : rd_rsp_data <= 32'h3BF0EA8D;
        16'h00C0 : rd_rsp_data <= 32'h50AE5875;
        16'h00C4 : rd_rsp_data <= 32'h6141E486;
        16'h00C8 : rd_rsp_data <= 32'h4F1A7543;
        16'h00CC : rd_rsp_data <= 32'h53FF79A5;
        16'h00D0 : rd_rsp_data <= 32'h758ED72E;
        16'h00D4 : rd_rsp_data <= 32'h28BF2E78;
        16'h00D8 : rd_rsp_data <= 32'h4616AD2D;
        16'h00DC : rd_rsp_data <= 32'h4476ADAB;
        16'h00E0 : rd_rsp_data <= 32'hC8477ECD;
        16'h00E4 : rd_rsp_data <= 32'h7EB7126D;
        16'h00E8 : rd_rsp_data <= 32'h4BD0C45E;
        16'h00EC : rd_rsp_data <= 32'h507F5793;
        16'h00F0 : rd_rsp_data <= 32'hF99C5C7C;
        16'h00F4 : rd_rsp_data <= 32'hFFFFFFFF;
        16'h00F8 : rd_rsp_data <= 32'h588ACA73;
        16'h00FC : rd_rsp_data <= 32'h4F762ED8;
        16'h0100 : rd_rsp_data <= 32'h4D6F1093;
        16'h0104 : rd_rsp_data <= 32'h4292440B;
        16'h0108 : rd_rsp_data <= 32'h0006AA0E;
        16'h010C : rd_rsp_data <= 32'hF800027E;
        16'h0110 : rd_rsp_data <= 32'h1B000016;
        16'h0114 : rd_rsp_data <= 32'hF34D2502;
        16'h0118 : rd_rsp_data <= 32'hAD4DE201;
        16'h011C : rd_rsp_data <= 32'h36F33F23;
        16'h0120 : rd_rsp_data <= 32'h23F33F35;
        16'h0124 : rd_rsp_data <= 32'h000008F1;
        16'h0128 : rd_rsp_data <= 32'h12000224;
        16'h012C : rd_rsp_data <= 32'h014C5A56;
        16'h0130 : rd_rsp_data <= 32'h01A80B03;
        16'h0134 : rd_rsp_data <= 32'h00000220;
        16'h0138 : rd_rsp_data <= 32'h00000220;
        16'h013C : rd_rsp_data <= 32'hFFFBFFAC;
        16'h0144 : rd_rsp_data <= 32'h00000380;
        16'h0148 : rd_rsp_data <= 32'h0000000C;
        16'h014C : rd_rsp_data <= 32'h00000300;
        16'h0150 : rd_rsp_data <= 32'h0000001C;
        16'h0154 : rd_rsp_data <= 32'h7865742E;
        16'h0158 : rd_rsp_data <= 32'h00000074;
        16'h015C : rd_rsp_data <= 32'h0000000A;
        16'h0160 : rd_rsp_data <= 32'h00000220;
        16'h0164 : rd_rsp_data <= 32'h00000020;
        16'h0168 : rd_rsp_data <= 32'h00000220;
        16'h0178 : rd_rsp_data <= 32'h68000020;
        16'h017C : rd_rsp_data <= 32'h7461642E;
        16'h0180 : rd_rsp_data <= 32'h00000061;
        16'h0184 : rd_rsp_data <= 32'h0000012C;
        16'h0188 : rd_rsp_data <= 32'h00000240;
        16'h018C : rd_rsp_data <= 32'h00000140;
        16'h0190 : rd_rsp_data <= 32'h00000240;
        16'h01A0 : rd_rsp_data <= 32'hC8000040;
        16'h01A4 : rd_rsp_data <= 32'h6C65722E;
        16'h01A8 : rd_rsp_data <= 32'h0000636F;
        16'h01AC : rd_rsp_data <= 32'h0000000C;
        16'h01B0 : rd_rsp_data <= 32'h00000380;
        16'h01B4 : rd_rsp_data <= 32'h00000020;
        16'h01B8 : rd_rsp_data <= 32'h00000380;
        16'h01C8 : rd_rsp_data <= 32'h42000040;
        16'h01CC : rd_rsp_data <= 32'h01F505C6;
        16'h01D0 : rd_rsp_data <= 32'h3306FFFC;
        16'h01D4 : rd_rsp_data <= 32'h0000C3C0;
        16'h01EC : rd_rsp_data <= 32'h4F4F4224;
        16'h01F0 : rd_rsp_data <= 32'h49464554;
        16'h01F4 : rd_rsp_data <= 32'h00000624;
        16'h01F8 : rd_rsp_data <= 32'h36471C03;
        16'h01FC : rd_rsp_data <= 32'h564A3431;
        16'h0218 : rd_rsp_data <= 32'h34313647;
        16'h021C : rd_rsp_data <= 32'h0000564A;
        16'h0234 : rd_rsp_data <= 32'h53410000;
        16'h0238 : rd_rsp_data <= 32'h00005355;
        16'h0248 : rd_rsp_data <= 32'h30000000;
        16'h024C : rd_rsp_data <= 32'h39312F34;
        16'h0250 : rd_rsp_data <= 32'h3230322F;
        16'h0254 : rd_rsp_data <= 32'h00000034;
        16'h0264 : rd_rsp_data <= 32'h0E800002;
        16'h0268 : rd_rsp_data <= 32'h6B8000FF;
        16'h026C : rd_rsp_data <= 32'hD1000000;
        16'h0270 : rd_rsp_data <= 32'h080000FF;
        16'h0278 : rd_rsp_data <= 32'h030000FF;
        16'h027C : rd_rsp_data <= 32'h59413100;
        16'h0280 : rd_rsp_data <= 32'h32305452;
        16'h0284 : rd_rsp_data <= 32'h6F722E36;
        16'h0288 : rd_rsp_data <= 32'h0300B26D;
        16'h0290 : rd_rsp_data <= 32'h00FF0E00;
        16'h0294 : rd_rsp_data <= 32'h00000080;
        16'h0298 : rd_rsp_data <= 32'h00FF0300;
        16'h029C : rd_rsp_data <= 32'h00000300;
        16'h02A0 : rd_rsp_data <= 32'h00FE0000;
        16'h02A4 : rd_rsp_data <= 32'h00010000;
        16'h0330 : rd_rsp_data <= 32'h0000000C;
        16'h0334 : rd_rsp_data <= 32'h00003222;
        16'h034C : rd_rsp_data <= 32'h1500001A;
        16'h0350 : rd_rsp_data <= 32'h00730041;
        16'h0354 : rd_rsp_data <= 32'h00730075;
        16'h0358 : rd_rsp_data <= 32'h00650048;
        16'h035C : rd_rsp_data <= 32'h00640061;
        16'h0360 : rd_rsp_data <= 32'h00720065;
        16'h0368 : rd_rsp_data <= 32'h1400000E;
        16'h036C : rd_rsp_data <= 32'h00310000;
        16'h0370 : rd_rsp_data <= 32'h0030002E;
        16'h0374 : rd_rsp_data <= 32'hFFFF0000;
        16'h0378 : rd_rsp_data <= 32'h52C05B14;
        16'h037C : rd_rsp_data <= 32'h496C0B98;
        16'h0380 : rd_rsp_data <= 32'hB5043BBC;
        16'h0384 : rd_rsp_data <= 32'h80D61102;
        16'h0388 : rd_rsp_data <= 32'h0004AA27;
        16'h038C : rd_rsp_data <= 32'hF800657E;
        16'h0390 : rd_rsp_data <= 32'h12006544;
        16'h0394 : rd_rsp_data <= 32'h014C5A56;
        16'h0398 : rd_rsp_data <= 32'h01A80B03;
        16'h039C : rd_rsp_data <= 32'h000003BD;
        16'h03A0 : rd_rsp_data <= 32'h00000220;
        16'h03A4 : rd_rsp_data <= 32'hFFFC0214;
        16'h03AC : rd_rsp_data <= 32'h00006540;
        16'h03B0 : rd_rsp_data <= 32'h0000016C;
        16'h03B4 : rd_rsp_data <= 32'h00006490;
        16'h03B8 : rd_rsp_data <= 32'h0000001C;
        16'h03BC : rd_rsp_data <= 32'h7865742E;
        16'h03C0 : rd_rsp_data <= 32'h00000074;
        16'h03C4 : rd_rsp_data <= 32'h00005EF0;
        16'h03C8 : rd_rsp_data <= 32'h00000220;
        16'h03CC : rd_rsp_data <= 32'h00005F00;
        16'h03D0 : rd_rsp_data <= 32'h00000220;
        16'h03E0 : rd_rsp_data <= 32'h68000020;
        16'h03E4 : rd_rsp_data <= 32'h7461642E;
        16'h03E8 : rd_rsp_data <= 32'h00000061;
        16'h03EC : rd_rsp_data <= 32'h00000408;
        16'h03F0 : rd_rsp_data <= 32'h00006120;
        16'h03F4 : rd_rsp_data <= 32'h00000420;
        16'h03F8 : rd_rsp_data <= 32'h00006120;
        16'h0408 : rd_rsp_data <= 32'hC8000040;
        16'h040C : rd_rsp_data <= 32'h6C65722E;
        16'h0410 : rd_rsp_data <= 32'h0000636F;
        16'h0414 : rd_rsp_data <= 32'h0000016C;
        16'h0418 : rd_rsp_data <= 32'h00006540;
        16'h041C : rd_rsp_data <= 32'h00000180;
        16'h0420 : rd_rsp_data <= 32'h00006540;
        16'h0430 : rd_rsp_data <= 32'h42000040;
        16'h0434 : rd_rsp_data <= 32'hFC6684A1;
        16'h0438 : rd_rsp_data <= 32'h74C085FF;
        16'h043C : rd_rsp_data <= 32'hE0200F1F;
        16'h0440 : rd_rsp_data <= 32'h17E0BA0F;
        16'h0444 : rd_rsp_data <= 32'h548B1673;
        16'h0448 : rd_rsp_data <= 32'h528B0424;
        16'h044C : rd_rsp_data <= 32'h1E0FF318;
        16'h0450 : rd_rsp_data <= 32'h89C229C8;
        16'h0454 : rd_rsp_data <= 32'h02E8C1D0;
        16'h0458 : rd_rsp_data <= 32'hE8AE0FF3;
        16'h045C : rd_rsp_data <= 32'h8B585A58;
        16'h0460 : rd_rsp_data <= 32'h04728B1A;
        16'h0464 : rd_rsp_data <= 32'h8B087A8B;
        16'h0468 : rd_rsp_data <= 32'h628B0C6A;
        16'h046C : rd_rsp_data <= 32'h1462FF10;
        16'h0470 : rd_rsp_data <= 32'hCCCCCCCC;
        16'h0474 : rd_rsp_data <= 32'h748B5756;
        16'h0478 : rd_rsp_data <= 32'h7C8B1024;
        16'h047C : rd_rsp_data <= 32'h548B0C24;
        16'h0480 : rd_rsp_data <= 32'h448D1424;
        16'h0484 : rd_rsp_data <= 32'hFE39FF16;
        16'h0488 : rd_rsp_data <= 32'hF8390473;
        16'h048C : rd_rsp_data <= 32'hD1890C73;
        16'h0490 : rd_rsp_data <= 32'hC103E283;
        16'h0494 : rd_rsp_data <= 32'hA5F302E9;
        16'h0498 : rd_rsp_data <= 32'hC68907EB;
        16'h049C : rd_rsp_data <= 32'hFF177C8D;
        16'h04A0 : rd_rsp_data <= 32'hF3D189FD;
        16'h04A4 : rd_rsp_data <= 32'h448BFCA4;
        16'h04A8 : rd_rsp_data <= 32'h5E5F0C24;
        16'h04AC : rd_rsp_data <= 32'hCCCCCCC3;
        16'h04B0 : rd_rsp_data <= 32'hCCCCCCCC;
        16'h04B4 : rd_rsp_data <= 32'h8BC03157;
        16'h04B8 : rd_rsp_data <= 32'h8B08247C;
        16'h04BC : rd_rsp_data <= 32'h890C244C;
        16'h04C0 : rd_rsp_data <= 32'h02E9C1CA;
        16'h04C4 : rd_rsp_data <= 32'h5703E283;
        16'h04C8 : rd_rsp_data <= 32'hD189ABF3;
        16'h04CC : rd_rsp_data <= 32'h5F58AAF3;
        16'h04D0 : rd_rsp_data <= 32'hCCCCCCC3;
        16'h04D4 : rd_rsp_data <= 32'h24448B57;
        16'h04D8 : rd_rsp_data <= 32'h247C8B10;
        16'h04DC : rd_rsp_data <= 32'h244C8B08;
        16'h04E0 : rd_rsp_data <= 32'h8BAAF30C;
        16'h04E4 : rd_rsp_data <= 32'h5F082444;
        16'h04E8 : rd_rsp_data <= 32'hCCCCCCC3;
        16'h04EC : rd_rsp_data <= 32'hCCCCCCCC;
        16'h04F0 : rd_rsp_data <= 32'hCCCCCCCC;
        16'h04F4 : rd_rsp_data <= 32'h244C8B57;
        16'h04F8 : rd_rsp_data <= 32'h24448B0C;
        16'h04FC : rd_rsp_data <= 32'h24548B10;
        16'h0500 : rd_rsp_data <= 32'h247C8B14;
        16'h0504 : rd_rsp_data <= 32'hCF448908;
        16'h0508 : rd_rsp_data <= 32'hCF5489F8;
        16'h050C : rd_rsp_data <= 32'h89F6E2FC;
        16'h0510 : rd_rsp_data <= 32'hCCC35FF8;
        16'h0514 : rd_rsp_data <= 32'h24448B57;
        16'h0518 : rd_rsp_data <= 32'h247C8B10;
        16'h051C : rd_rsp_data <= 32'h244C8B08;
        16'h0520 : rd_rsp_data <= 32'h8BABF30C;
        16'h0524 : rd_rsp_data <= 32'h5F082444;
        16'h0528 : rd_rsp_data <= 32'hCCCCCCC3;
        16'h052C : rd_rsp_data <= 32'hCCCCCCCC;
        16'h0530 : rd_rsp_data <= 32'hCCCCCCCC;
        16'h0534 : rd_rsp_data <= 32'h10244C8B;
        16'h0538 : rd_rsp_data <= 32'h1375C985;
        16'h053C : rd_rsp_data <= 32'h14244C8B;
        16'h0540 : rd_rsp_data <= 32'h618308E3;
        16'h0544 : rd_rsp_data <= 32'h4C890004;
        16'h0548 : rd_rsp_data <= 32'h5DE91024;
        16'h054C : rd_rsp_data <= 32'h53000045;
        16'h0550 : rd_rsp_data <= 32'h548B5756;
        16'h0554 : rd_rsp_data <= 32'h448B1424;
        16'h0558 : rd_rsp_data <= 32'hD7891024;
        16'h055C : rd_rsp_data <= 32'h5C8BC689;
        16'h0560 : rd_rsp_data <= 32'hEAD11824;
        16'h0564 : rd_rsp_data <= 32'hAC0FD8D1;
        16'h0568 : rd_rsp_data <= 32'hE9D101CB;
        16'h056C : rd_rsp_data <= 32'hF3F7F475;
        16'h0570 : rd_rsp_data <= 32'h4C8BC389;
        16'h0574 : rd_rsp_data <= 32'h64F71C24;
        16'h0578 : rd_rsp_data <= 32'hAF0F1824;
        16'h057C : rd_rsp_data <= 32'h8BCA01CB;
        16'h0580 : rd_rsp_data <= 32'h7220244C;
        16'h0584 : rd_rsp_data <= 32'h77D7390A;
        16'h0588 : rd_rsp_data <= 32'h39047211;
        16'h058C : rd_rsp_data <= 32'h4B0B73C6;
        16'h0590 : rd_rsp_data <= 32'h442B13E3;
        16'h0594 : rd_rsp_data <= 32'h541B1824;
        16'h0598 : rd_rsp_data <= 32'h09E31C24;
        16'h059C : rd_rsp_data <= 32'hD719C629;
        16'h05A0 : rd_rsp_data <= 32'h79893189;
        16'h05A4 : rd_rsp_data <= 32'h31D88904;
        16'h05A8 : rd_rsp_data <= 32'h5B5E5FD2;
        16'h05AC : rd_rsp_data <= 32'hCCCCCCC3;
        16'h05B0 : rd_rsp_data <= 32'hCCCCCCCC;
        16'h05B4 : rd_rsp_data <= 32'h748B5756;
        16'h05B8 : rd_rsp_data <= 32'h7C8B0C24;
        16'h05BC : rd_rsp_data <= 32'h4C8B1024;
        16'h05C0 : rd_rsp_data <= 32'hA6F31424;
        16'h05C4 : rd_rsp_data <= 32'hFF46B60F;
        16'h05C8 : rd_rsp_data <= 32'hFF57B60F;
        16'h05CC : rd_rsp_data <= 32'h5E5FD029;
        16'h05D0 : rd_rsp_data <= 32'hEC8B55C3;
        16'h05D4 : rd_rsp_data <= 32'h6AF8E483;
        16'h05D8 : rd_rsp_data <= 32'h0C75FF00;
        16'h05DC : rd_rsp_data <= 32'hE80875FF;
        16'h05E0 : rd_rsp_data <= 32'h000000CC;
        16'h05E4 : rd_rsp_data <= 32'hE80CC483;
        16'h05E8 : rd_rsp_data <= 32'h00004475;
        16'h05EC : rd_rsp_data <= 32'hC35DE58B;
        16'h05F0 : rd_rsp_data <= 32'h5ADEE853;
        16'h05F4 : rd_rsp_data <= 32'hD88A0000;
        16'h05F8 : rd_rsp_data <= 32'h7402C3F6;
        16'h05FC : rd_rsp_data <= 32'h59CDE805;
        16'h0600 : rd_rsp_data <= 32'hC3F60000;
        16'h0604 : rd_rsp_data <= 32'h830B7410;
        16'h0608 : rd_rsp_data <= 32'h76E80CEC;
        16'h060C : rd_rsp_data <= 32'h8300005B;
        16'h0610 : rd_rsp_data <= 32'hC35B0CC4;
        16'h0614 : rd_rsp_data <= 32'h83EC8B55;
        16'h0618 : rd_rsp_data <= 32'h575620EC;
        16'h061C : rd_rsp_data <= 32'h33F8458D;
        16'h0620 : rd_rsp_data <= 32'hF98B50F6;
        16'h0624 : rd_rsp_data <= 32'h56FC7589;
        16'h0628 : rd_rsp_data <= 32'hE4B9D233;
        16'h062C : rd_rsp_data <= 32'hE8FFFC63;
        16'h0630 : rd_rsp_data <= 32'h00004650;
        16'h0634 : rd_rsp_data <= 32'hC0855959;
        16'h0638 : rd_rsp_data <= 32'h458B2778;
        16'h063C : rd_rsp_data <= 32'h85108BF8;
        16'h0640 : rd_rsp_data <= 32'h8B1E74D2;
        16'h0644 : rd_rsp_data <= 32'h344F39CE;
        16'h0648 : rd_rsp_data <= 32'h478B1776;
        16'h064C : rd_rsp_data <= 32'h08C0833C;
        16'h0650 : rd_rsp_data <= 32'h0B741039;
        16'h0654 : rd_rsp_data <= 32'h20C08341;
        16'h0658 : rd_rsp_data <= 32'h72344F3B;
        16'h065C : rd_rsp_data <= 32'h8B02EBF3;
        16'h0660 : rd_rsp_data <= 32'h3C478BF1;
        16'h0664 : rd_rsp_data <= 32'hC1FC558D;
        16'h0668 : rd_rsp_data <= 32'hFF5205E6;
        16'h066C : rd_rsp_data <= 32'h8B080674;
        16'h0670 : rd_rsp_data <= 32'h6A04064C;
        16'h0674 : rd_rsp_data <= 32'h51FF5104;
        16'h0678 : rd_rsp_data <= 32'hE8458D04;
        16'h067C : rd_rsp_data <= 32'h8910C483;
        16'h0680 : rd_rsp_data <= 32'h458BF445;
        16'h0684 : rd_rsp_data <= 32'h08010FF4;
        16'h0688 : rd_rsp_data <= 32'h8DFC558B;
        16'h068C : rd_rsp_data <= 32'h8D50F045;
        16'h0690 : rd_rsp_data <= 32'h6A50E045;
        16'h0694 : rd_rsp_data <= 32'h4045E802;
        16'h0698 : rd_rsp_data <= 32'hC4830000;
        16'h069C : rd_rsp_data <= 32'h06B0B80C;
        16'h06A0 : rd_rsp_data <= 32'hD12DFFFC;
        16'h06A4 : rd_rsp_data <= 32'h03FFFC05;
        16'h06A8 : rd_rsp_data <= 32'h5E5FE045;
        16'h06AC : rd_rsp_data <= 32'hC35DE58B;
        16'h06B0 : rd_rsp_data <= 32'h83EC8B55;
        16'h06B4 : rd_rsp_data <= 32'hEC81F8E4;
        16'h06B8 : rd_rsp_data <= 32'h000002FC;
        16'h06BC : rd_rsp_data <= 32'h105D8B53;
        16'h06C0 : rd_rsp_data <= 32'hDB855756;
        16'h06C4 : rd_rsp_data <= 32'h448D3675;
        16'h06C8 : rd_rsp_data <= 32'hC0684824;
        16'h06CC : rd_rsp_data <= 32'h50000002;
        16'h06D0 : rd_rsp_data <= 32'hFFFDDFE8;
        16'h06D4 : rd_rsp_data <= 32'h008868FF;
        16'h06D8 : rd_rsp_data <= 32'h848D0000;
        16'h06DC : rd_rsp_data <= 32'h0000CC24;
        16'h06E0 : rd_rsp_data <= 32'h2444C700;
        16'h06E4 : rd_rsp_data <= 32'h69655054;
        16'h06E8 : rd_rsp_data <= 32'h64B46843;
        16'h06EC : rd_rsp_data <= 32'hE850FFFC;
        16'h06F0 : rd_rsp_data <= 32'h0000482B;
        16'h06F4 : rd_rsp_data <= 32'hE914C483;
        16'h06F8 : rd_rsp_data <= 32'h000002EE;
        16'h06FC : rd_rsp_data <= 32'h0148BB83;
        16'h0700 : rd_rsp_data <= 32'h0F000000;
        16'h0704 : rd_rsp_data <= 32'h00027E85;
        16'h0708 : rd_rsp_data <= 32'h24BB8000;
        16'h070C : rd_rsp_data <= 32'h00000001;
        16'h0710 : rd_rsp_data <= 32'h8D78438D;
        16'h0714 : rd_rsp_data <= 32'h06890473;
        16'h0718 : rd_rsp_data <= 32'h00D8838D;
        16'h071C : rd_rsp_data <= 32'h43890000;
        16'h0720 : rd_rsp_data <= 32'hA2840F6C;
        16'h0724 : rd_rsp_data <= 32'h8B000000;
        16'h0728 : rd_rsp_data <= 32'h0001208B;
        16'h072C : rd_rsp_data <= 32'h644B0100;
        16'h0730 : rd_rsp_data <= 32'h8540438B;
        16'h0734 : rd_rsp_data <= 32'h030574C0;
        16'h0738 : rd_rsp_data <= 32'h404389C1;
        16'h073C : rd_rsp_data <= 32'h854C438B;
        16'h0740 : rd_rsp_data <= 32'h030574C0;
        16'h0744 : rd_rsp_data <= 32'h4C4389C1;
        16'h0748 : rd_rsp_data <= 32'h8514438B;
        16'h074C : rd_rsp_data <= 32'h030574C0;
        16'h0750 : rd_rsp_data <= 32'h144389C1;
        16'h0754 : rd_rsp_data <= 32'h8520438B;
        16'h0758 : rd_rsp_data <= 32'h030574C0;
        16'h075C : rd_rsp_data <= 32'h204389C1;
        16'h0760 : rd_rsp_data <= 32'h8530438B;
        16'h0764 : rd_rsp_data <= 32'h030574C0;
        16'h0768 : rd_rsp_data <= 32'h304389C1;
        16'h076C : rd_rsp_data <= 32'h333C4B01;
        16'h0770 : rd_rsp_data <= 32'h345339D2;
        16'h0774 : rd_rsp_data <= 32'hC9333C76;
        16'h0778 : rd_rsp_data <= 32'h8B3C738B;
        16'h077C : rd_rsp_data <= 32'h8510317C;
        16'h0780 : rd_rsp_data <= 32'h8B0C74FF;
        16'h0784 : rd_rsp_data <= 32'h00012083;
        16'h0788 : rd_rsp_data <= 32'h89C70300;
        16'h078C : rd_rsp_data <= 32'h8B103144;
        16'h0790 : rd_rsp_data <= 32'h7C8B3C73;
        16'h0794 : rd_rsp_data <= 32'hFF851431;
        16'h0798 : rd_rsp_data <= 32'h838B0C74;
        16'h079C : rd_rsp_data <= 32'h00000120;
        16'h07A0 : rd_rsp_data <= 32'h4489C703;
        16'h07A4 : rd_rsp_data <= 32'h83421431;
        16'h07A8 : rd_rsp_data <= 32'h533B20C1;
        16'h07AC : rd_rsp_data <= 32'h8DC97234;
        16'h07B0 : rd_rsp_data <= 32'h838B0473;
        16'h07B4 : rd_rsp_data <= 32'h00000120;
        16'h07B8 : rd_rsp_data <= 32'h02708301;
        16'h07BC : rd_rsp_data <= 32'h83010000;
        16'h07C0 : rd_rsp_data <= 32'h0000026C;
        16'h07C4 : rd_rsp_data <= 32'h000099E9;
        16'h07C8 : rd_rsp_data <= 32'h20838B00;
        16'h07CC : rd_rsp_data <= 32'h29000001;
        16'h07D0 : rd_rsp_data <= 32'h4B8B6443;
        16'h07D4 : rd_rsp_data <= 32'h74C98540;
        16'h07D8 : rd_rsp_data <= 32'h89C82B05;
        16'h07DC : rd_rsp_data <= 32'h4B8B404B;
        16'h07E0 : rd_rsp_data <= 32'h74C9854C;
        16'h07E4 : rd_rsp_data <= 32'h89C82B05;
        16'h07E8 : rd_rsp_data <= 32'h4B8B4C4B;
        16'h07EC : rd_rsp_data <= 32'h74C98514;
        16'h07F0 : rd_rsp_data <= 32'h89C82B05;
        16'h07F4 : rd_rsp_data <= 32'h4B8B144B;
        16'h07F8 : rd_rsp_data <= 32'h74C98520;
        16'h07FC : rd_rsp_data <= 32'h89C82B05;
        16'h0800 : rd_rsp_data <= 32'h4B8B204B;
        16'h0804 : rd_rsp_data <= 32'h74C98530;
        16'h0808 : rd_rsp_data <= 32'h89C82B05;
        16'h080C : rd_rsp_data <= 32'h4329304B;
        16'h0810 : rd_rsp_data <= 32'h39D2333C;
        16'h0814 : rd_rsp_data <= 32'h38763453;
        16'h0818 : rd_rsp_data <= 32'h738BC033;
        16'h081C : rd_rsp_data <= 32'h304C8B3C;
        16'h0820 : rd_rsp_data <= 32'h74C98510;
        16'h0824 : rd_rsp_data <= 32'h208B2B0A;
        16'h0828 : rd_rsp_data <= 32'h89000001;
        16'h082C : rd_rsp_data <= 32'h8B10304C;
        16'h0830 : rd_rsp_data <= 32'h4C8B3C73;
        16'h0834 : rd_rsp_data <= 32'hC9851430;
        16'h0838 : rd_rsp_data <= 32'h8B2B0A74;
        16'h083C : rd_rsp_data <= 32'h00000120;
        16'h0840 : rd_rsp_data <= 32'h14304C89;
        16'h0844 : rd_rsp_data <= 32'h20C08342;
        16'h0848 : rd_rsp_data <= 32'h7234533B;
        16'h084C : rd_rsp_data <= 32'h04738DCD;
        16'h0850 : rd_rsp_data <= 32'h0120838B;
        16'h0854 : rd_rsp_data <= 32'h83290000;
        16'h0858 : rd_rsp_data <= 32'h00000270;
        16'h085C : rd_rsp_data <= 32'h026C8329;
        16'h0860 : rd_rsp_data <= 32'h448D0000;
        16'h0864 : rd_rsp_data <= 32'h44891824;
        16'h0868 : rd_rsp_data <= 32'h448B1024;
        16'h086C : rd_rsp_data <= 32'h010F1024;
        16'h0870 : rd_rsp_data <= 32'h24448B08;
        16'h0874 : rd_rsp_data <= 32'hFC70891A;
        16'h0878 : rd_rsp_data <= 32'hFFFD73E8;
        16'h087C : rd_rsp_data <= 32'h24BB80FF;
        16'h0880 : rd_rsp_data <= 32'h00000001;
        16'h0884 : rd_rsp_data <= 32'h8B64538B;
        16'h0888 : rd_rsp_data <= 32'h00012083;
        16'h088C : rd_rsp_data <= 32'h01097400;
        16'h0890 : rd_rsp_data <= 32'h52833042;
        16'h0894 : rd_rsp_data <= 32'h07EB0034;
        16'h0898 : rd_rsp_data <= 32'h83304229;
        16'h089C : rd_rsp_data <= 32'h8B00345A;
        16'h08A0 : rd_rsp_data <= 32'h0001108B;
        16'h08A4 : rd_rsp_data <= 32'h088B0300;
        16'h08A8 : rd_rsp_data <= 32'h8B000001;
        16'h08AC : rd_rsp_data <= 32'h00011483;
        16'h08B0 : rd_rsp_data <= 32'h0C831300;
        16'h08B4 : rd_rsp_data <= 32'h89000001;
        16'h08B8 : rd_rsp_data <= 32'h4A8B104A;
        16'h08BC : rd_rsp_data <= 32'h14428930;
        16'h08C0 : rd_rsp_data <= 32'h8B08C183;
        16'h08C4 : rd_rsp_data <= 32'h00010883;
        16'h08C8 : rd_rsp_data <= 32'h18428900;
        16'h08CC : rd_rsp_data <= 32'h010C838B;
        16'h08D0 : rd_rsp_data <= 32'h42890000;
        16'h08D4 : rd_rsp_data <= 32'h18838B1C;
        16'h08D8 : rd_rsp_data <= 32'h89000001;
        16'h08DC : rd_rsp_data <= 32'h838B2042;
        16'h08E0 : rd_rsp_data <= 32'h0000011C;
        16'h08E4 : rd_rsp_data <= 32'h8B244289;
        16'h08E8 : rd_rsp_data <= 32'hD0833442;
        16'h08EC : rd_rsp_data <= 32'h284A8900;
        16'h08F0 : rd_rsp_data <= 32'h4289CB8B;
        16'h08F4 : rd_rsp_data <= 32'h1BB9E82C;
        16'h08F8 : rd_rsp_data <= 32'h758B0000;
        16'h08FC : rd_rsp_data <= 32'h39FF3308;
        16'h0900 : rd_rsp_data <= 32'h1776087B;
        16'h0904 : rd_rsp_data <= 32'h8B14438B;
        16'h0908 : rd_rsp_data <= 32'h8DCE8BD3;
        16'h090C : rd_rsp_data <= 32'hE850B804;
        16'h0910 : rd_rsp_data <= 32'h00003273;
        16'h0914 : rd_rsp_data <= 32'h7B3B5947;
        16'h0918 : rd_rsp_data <= 32'h33E97208;
        16'h091C : rd_rsp_data <= 32'h187B39FF;
        16'h0920 : rd_rsp_data <= 32'h438B1776;
        16'h0924 : rd_rsp_data <= 32'h8BD38B20;
        16'h0928 : rd_rsp_data <= 32'hB8048DCE;
        16'h092C : rd_rsp_data <= 32'h3255E850;
        16'h0930 : rd_rsp_data <= 32'h59470000;
        16'h0934 : rd_rsp_data <= 32'h72187B3B;
        16'h0938 : rd_rsp_data <= 32'h39FF33E9;
        16'h093C : rd_rsp_data <= 32'h1776247B;
        16'h0940 : rd_rsp_data <= 32'h8B30438B;
        16'h0944 : rd_rsp_data <= 32'h8DCE8BD3;
        16'h0948 : rd_rsp_data <= 32'hE850B804;
        16'h094C : rd_rsp_data <= 32'h00003237;
        16'h0950 : rd_rsp_data <= 32'h7B3B5947;
        16'h0954 : rd_rsp_data <= 32'h8BE97224;
        16'h0958 : rd_rsp_data <= 32'h6943C6CB;
        16'h095C : rd_rsp_data <= 32'h6243C601;
        16'h0960 : rd_rsp_data <= 32'h4883C701;
        16'h0964 : rd_rsp_data <= 32'hB0000001;
        16'h0968 : rd_rsp_data <= 32'hE8FFFC06;
        16'h096C : rd_rsp_data <= 32'hFFFFFCA4;
        16'h0970 : rd_rsp_data <= 32'h0C75FF53;
        16'h0974 : rd_rsp_data <= 32'h01488389;
        16'h0978 : rd_rsp_data <= 32'hFF560000;
        16'h097C : rd_rsp_data <= 32'h0CC483D0;
        16'h0980 : rd_rsp_data <= 32'h0040DCE8;
        16'h0984 : rd_rsp_data <= 32'h8B03EB00;
        16'h0988 : rd_rsp_data <= 32'h246A0875;
        16'h098C : rd_rsp_data <= 32'h2824448D;
        16'h0990 : rd_rsp_data <= 32'h88E85056;
        16'h0994 : rd_rsp_data <= 32'h83000045;
        16'h0998 : rd_rsp_data <= 32'h448D0CC4;
        16'h099C : rd_rsp_data <= 32'h45892424;
        16'h09A0 : rd_rsp_data <= 32'h24448D08;
        16'h09A4 : rd_rsp_data <= 32'h02C06848;
        16'h09A8 : rd_rsp_data <= 32'h50530000;
        16'h09AC : rd_rsp_data <= 32'h00456EE8;
        16'h09B0 : rd_rsp_data <= 32'h24B48B00;
        16'h09B4 : rd_rsp_data <= 32'h0000012C;
        16'h09B8 : rd_rsp_data <= 32'hCC24848D;
        16'h09BC : rd_rsp_data <= 32'h8B000000;
        16'h09C0 : rd_rsp_data <= 32'h013024BC;
        16'h09C4 : rd_rsp_data <= 32'hC4830000;
        16'h09C8 : rd_rsp_data <= 32'h0088680C;
        16'h09CC : rd_rsp_data <= 32'hB4680000;
        16'h09D0 : rd_rsp_data <= 32'h50FFFC64;
        16'h09D4 : rd_rsp_data <= 32'h004546E8;
        16'h09D8 : rd_rsp_data <= 32'h0CC48300;
        16'h09DC : rd_rsp_data <= 32'h2024B489;
        16'h09E0 : rd_rsp_data <= 32'h89000001;
        16'h09E4 : rd_rsp_data <= 32'h012424BC;
        16'h09E8 : rd_rsp_data <= 32'h848D0000;
        16'h09EC : rd_rsp_data <= 32'h0000C024;
        16'h09F0 : rd_rsp_data <= 32'h24448900;
        16'h09F4 : rd_rsp_data <= 32'h24448D4C;
        16'h09F8 : rd_rsp_data <= 32'h24448918;
        16'h09FC : rd_rsp_data <= 32'h24448B10;
        16'h0A00 : rd_rsp_data <= 32'h08010F10;
        16'h0A04 : rd_rsp_data <= 32'h1A24448B;
        16'h0A08 : rd_rsp_data <= 32'h4C244C8D;
        16'h0A0C : rd_rsp_data <= 32'hE8FC4889;
        16'h0A10 : rd_rsp_data <= 32'hFFFFFBDC;
        16'h0A14 : rd_rsp_data <= 32'hB02484C6;
        16'h0A1C : rd_rsp_data <= 32'h48244C8D;
        16'h0A20 : rd_rsp_data <= 32'h850FDB85;
        16'h0A24 : rd_rsp_data <= 32'h00000080;
        16'h0A28 : rd_rsp_data <= 32'h8B087D8B;
        16'h0A2C : rd_rsp_data <= 32'h17F5E8D7;
        16'h0A30 : rd_rsp_data <= 32'hCCB90000;
        16'h0A34 : rd_rsp_data <= 32'hE8FFFC65;
        16'h0A38 : rd_rsp_data <= 32'h00004277;
        16'h0A3C : rd_rsp_data <= 32'h9C88D78B;
        16'h0A40 : rd_rsp_data <= 32'h0000AA24;
        16'h0A44 : rd_rsp_data <= 32'h244C8D00;
        16'h0A48 : rd_rsp_data <= 32'h050CE848;
        16'h0A4C : rd_rsp_data <= 32'h3CB90000;
        16'h0A50 : rd_rsp_data <= 32'h89FFFC66;
        16'h0A54 : rd_rsp_data <= 32'h0148248C;
        16'h0A58 : rd_rsp_data <= 32'hD7E80000;
        16'h0A5C : rd_rsp_data <= 32'h53000041;
        16'h0A60 : rd_rsp_data <= 32'hFC648468;
        16'h0A64 : rd_rsp_data <= 32'hC93351FF;
        16'h0A68 : rd_rsp_data <= 32'h020000BA;
        16'h0A6C : rd_rsp_data <= 32'hE6E84103;
        16'h0A70 : rd_rsp_data <= 32'h83000043;
        16'h0A74 : rd_rsp_data <= 32'h5D390CC4;
        16'h0A78 : rd_rsp_data <= 32'h8B0C740C;
        16'h0A7C : rd_rsp_data <= 32'h4C8D0C55;
        16'h0A80 : rd_rsp_data <= 32'h2CE84C24;
        16'h0A84 : rd_rsp_data <= 32'hA1000037;
        16'h0A88 : rd_rsp_data <= 32'hFFFC66C4;
        16'h0A8C : rd_rsp_data <= 32'h12EBF38B;
        16'h0A90 : rd_rsp_data <= 32'h4C244C8D;
        16'h0A94 : rd_rsp_data <= 32'hD0FF5351;
        16'h0A98 : rd_rsp_data <= 32'hC8B5048B;
        16'h0A9C : rd_rsp_data <= 32'h46FFFC66;
        16'h0AA0 : rd_rsp_data <= 32'hC0855959;
        16'h0AA4 : rd_rsp_data <= 32'h76EBEA75;
        16'h0AA8 : rd_rsp_data <= 32'h0013F5E8;
        16'h0AAC : rd_rsp_data <= 32'h248C8B00;
        16'h0AB0 : rd_rsp_data <= 32'h00000148;
        16'h0AB4 : rd_rsp_data <= 32'hFC663CBA;
        16'h0AB8 : rd_rsp_data <= 32'h419DE8FF;
        16'h0ABC : rd_rsp_data <= 32'h7D8B0000;
        16'h0AC0 : rd_rsp_data <= 32'h244C8D08;
        16'h0AC4 : rd_rsp_data <= 32'hE8D78B48;
        16'h0AC8 : rd_rsp_data <= 32'h00002871;
        16'h0ACC : rd_rsp_data <= 32'h2024448D;
        16'h0AD0 : rd_rsp_data <= 32'h5350DB33;
        16'h0AD4 : rd_rsp_data <= 32'hB4B9D233;
        16'h0AD8 : rd_rsp_data <= 32'hE8FFFC63;
        16'h0ADC : rd_rsp_data <= 32'h000041A4;
        16'h0AE0 : rd_rsp_data <= 32'hC0855959;
        16'h0AE4 : rd_rsp_data <= 32'h448B0678;
        16'h0AE8 : rd_rsp_data <= 32'h10FF2024;
        16'h0AEC : rd_rsp_data <= 32'hFC64A4B9;
        16'h0AF0 : rd_rsp_data <= 32'h4140E8FF;
        16'h0AF4 : rd_rsp_data <= 32'h4C8D0000;
        16'h0AF8 : rd_rsp_data <= 32'hA4E84824;
        16'h0AFC : rd_rsp_data <= 32'hA1000035;
        16'h0B00 : rd_rsp_data <= 32'hFFFC66C8;
        16'h0B04 : rd_rsp_data <= 32'h12EBF38B;
        16'h0B08 : rd_rsp_data <= 32'h4C244C8D;
        16'h0B0C : rd_rsp_data <= 32'hD0FF5351;
        16'h0B10 : rd_rsp_data <= 32'hCCB5048B;
        16'h0B14 : rd_rsp_data <= 32'h46FFFC66;
        16'h0B18 : rd_rsp_data <= 32'hC0855959;
        16'h0B1C : rd_rsp_data <= 32'h548DEA75;
        16'h0B20 : rd_rsp_data <= 32'hCF8B4824;
        16'h0B24 : rd_rsp_data <= 32'h002B5CE8;
        16'h0B28 : rd_rsp_data <= 32'h24848B00;
        16'h0B2C : rd_rsp_data <= 32'h000000AC;
        16'h0B30 : rd_rsp_data <= 32'h000002BE;
        16'h0B34 : rd_rsp_data <= 32'h0C788380;
        16'h0B38 : rd_rsp_data <= 32'h80207411;
        16'h0B3C : rd_rsp_data <= 32'h00B124BC;
        16'h0B40 : rd_rsp_data <= 32'h75000000;
        16'h0B44 : rd_rsp_data <= 32'h84685316;
        16'h0B48 : rd_rsp_data <= 32'h51FFFC64;
        16'h0B4C : rd_rsp_data <= 32'h021002BA;
        16'h0B50 : rd_rsp_data <= 32'hE8CE8B03;
        16'h0B54 : rd_rsp_data <= 32'h00004301;
        16'h0B58 : rd_rsp_data <= 32'h8D0CC483;
        16'h0B5C : rd_rsp_data <= 32'h33142444;
        16'h0B60 : rd_rsp_data <= 32'hB95350D2;
        16'h0B64 : rd_rsp_data <= 32'hFFFC63D4;
        16'h0B68 : rd_rsp_data <= 32'h004117E8;
        16'h0B6C : rd_rsp_data <= 32'h85595900;
        16'h0B70 : rd_rsp_data <= 32'h531B79C0;
        16'h0B74 : rd_rsp_data <= 32'hFC648468;
        16'h0B78 : rd_rsp_data <= 32'h01BA51FF;
        16'h0B7C : rd_rsp_data <= 32'h8B030210;
        16'h0B80 : rd_rsp_data <= 32'h42D3E8CE;
        16'h0B84 : rd_rsp_data <= 32'hC4830000;
        16'h0B88 : rd_rsp_data <= 32'h3ED3E80C;
        16'h0B8C : rd_rsp_data <= 32'hB4FF0000;
        16'h0B90 : rd_rsp_data <= 32'h0000AC24;
        16'h0B94 : rd_rsp_data <= 32'h24448D00;
        16'h0B98 : rd_rsp_data <= 32'h448B5050;
        16'h0B9C : rd_rsp_data <= 32'hFF501C24;
        16'h0BA0 : rd_rsp_data <= 32'h0CC48310;
        16'h0BA4 : rd_rsp_data <= 32'h4824548D;
        16'h0BA8 : rd_rsp_data <= 32'hD6E8CF8B;
        16'h0BAC : rd_rsp_data <= 32'hFF00002A;
        16'h0BB0 : rd_rsp_data <= 32'h00AC24B4;
        16'h0BB4 : rd_rsp_data <= 32'h448D0000;
        16'h0BB8 : rd_rsp_data <= 32'h8B505024;
        16'h0BBC : rd_rsp_data <= 32'h501C2444;
        16'h0BC0 : rd_rsp_data <= 32'hC48310FF;
        16'h0BC4 : rd_rsp_data <= 32'h3E97E80C;
        16'h0BC8 : rd_rsp_data <= 32'h5E5F0000;
        16'h0BCC : rd_rsp_data <= 32'h5DE58B5B;
        16'h0BD0 : rd_rsp_data <= 32'h20EC83C3;
        16'h0BD4 : rd_rsp_data <= 32'h448D5653;
        16'h0BD8 : rd_rsp_data <= 32'h4C892024;
        16'h0BDC : rd_rsp_data <= 32'h89570824;
        16'h0BE0 : rd_rsp_data <= 32'h8B142444;
        16'h0BE4 : rd_rsp_data <= 32'h0F142444;
        16'h0BE8 : rd_rsp_data <= 32'h448B0801;
        16'h0BEC : rd_rsp_data <= 32'hFF332624;
        16'h0BF0 : rd_rsp_data <= 32'h5C89DF8B;
        16'h0BF4 : rd_rsp_data <= 32'h708B1424;
        16'h0BF8 : rd_rsp_data <= 32'h04EE83FC;
        16'h0BFC : rd_rsp_data <= 32'h20247489;
        16'h0C00 : rd_rsp_data <= 32'h8B34568B;
        16'h0C04 : rd_rsp_data <= 32'h245489CA;
        16'h0C08 : rd_rsp_data <= 32'h74D2851C;
        16'h0C0C : rd_rsp_data <= 32'h3C468B78;
        16'h0C10 : rd_rsp_data <= 32'h0C24748B;
        16'h0C14 : rd_rsp_data <= 32'h89E98B55;
        16'h0C18 : rd_rsp_data <= 32'hC1142444;
        16'h0C1C : rd_rsp_data <= 32'h048B05E5;
        16'h0C20 : rd_rsp_data <= 32'h76F03B07;
        16'h0C24 : rd_rsp_data <= 32'h245C8B4B;
        16'h0C28 : rd_rsp_data <= 32'h8BC03314;
        16'h0C2C : rd_rsp_data <= 32'hD68B1F34;
        16'h0C30 : rd_rsp_data <= 32'h18245C8B;
        16'h0C34 : rd_rsp_data <= 32'h89205603;
        16'h0C38 : rd_rsp_data <= 32'h131C2454;
        16'h0C3C : rd_rsp_data <= 32'hF28B2446;
        16'h0C40 : rd_rsp_data <= 32'h83FFC683;
        16'h0C44 : rd_rsp_data <= 32'hD233FFD0;
        16'h0C48 : rd_rsp_data <= 32'h548BD03B;
        16'h0C4C : rd_rsp_data <= 32'h1C772024;
        16'h0C50 : rd_rsp_data <= 32'h74390672;
        16'h0C54 : rd_rsp_data <= 32'h14771024;
        16'h0C58 : rd_rsp_data <= 32'h0C74CA3B;
        16'h0C5C : rd_rsp_data <= 32'h1424748B;
        16'h0C60 : rd_rsp_data <= 32'h3937048B;
        16'h0C64 : rd_rsp_data <= 32'h04732E04;
        16'h0C68 : rd_rsp_data <= 32'hCB8BEF8B;
        16'h0C6C : rd_rsp_data <= 32'h1024748B;
        16'h0C70 : rd_rsp_data <= 32'h1424448B;
        16'h0C74 : rd_rsp_data <= 32'h20C78343;
        16'h0C78 : rd_rsp_data <= 32'h18245C89;
        16'h0C7C : rd_rsp_data <= 32'h9E72DA3B;
        16'h0C80 : rd_rsp_data <= 32'h2424748B;
        16'h0C84 : rd_rsp_data <= 32'h73CA3B5D;
        16'h0C88 : rd_rsp_data <= 32'h05E1C10A;
        16'h0C8C : rd_rsp_data <= 32'h8B3C4E03;
        16'h0C90 : rd_rsp_data <= 32'h3302EBC1;
        16'h0C94 : rd_rsp_data <= 32'h5B5E5FC0;
        16'h0C98 : rd_rsp_data <= 32'hC320C483;
        16'h0C9C : rd_rsp_data <= 32'h83EC8B55;
        16'h0CA0 : rd_rsp_data <= 32'hEC83F8E4;
        16'h0CA4 : rd_rsp_data <= 32'h5756533C;
        16'h0CA8 : rd_rsp_data <= 32'hF28BF98B;
        16'h0CAC : rd_rsp_data <= 32'hFC6384BA;
        16'h0CB0 : rd_rsp_data <= 32'h247489FF;
        16'h0CB4 : rd_rsp_data <= 32'h104F8D24;
        16'h0CB8 : rd_rsp_data <= 32'h0042C7E8;
        16'h0CBC : rd_rsp_data <= 32'h0C5D8B00;
        16'h0CC0 : rd_rsp_data <= 32'h88204F8B;
        16'h0CC4 : rd_rsp_data <= 32'h8B132444;
        16'h0CC8 : rd_rsp_data <= 32'h138B2447;
        16'h0CCC : rd_rsp_data <= 32'h18244489;
        16'h0CD0 : rd_rsp_data <= 32'h252C478B;
        16'h0CD4 : rd_rsp_data <= 32'h00000800;
        16'h0CD8 : rd_rsp_data <= 32'h1C244C89;
        16'h0CDC : rd_rsp_data <= 32'h20244489;
        16'h0CE0 : rd_rsp_data <= 32'h3674D285;
        16'h0CE4 : rd_rsp_data <= 32'h3275F685;
        16'h0CE8 : rd_rsp_data <= 32'h011342F6;
        16'h0CEC : rd_rsp_data <= 32'h4A8B0574;
        16'h0CF0 : rd_rsp_data <= 32'h0F16EB18;
        16'h0CF4 : rd_rsp_data <= 32'h0F164AB6;
        16'h0CF8 : rd_rsp_data <= 32'hC11542B6;
        16'h0CFC : rd_rsp_data <= 32'hC80B08E1;
        16'h0D00 : rd_rsp_data <= 32'h1442B60F;
        16'h0D04 : rd_rsp_data <= 32'h0B08E1C1;
        16'h0D08 : rd_rsp_data <= 32'hF7F18BC8;
        16'h0D0C : rd_rsp_data <= 32'h07E683DE;
        16'h0D10 : rd_rsp_data <= 32'hF103F203;
        16'h0D14 : rd_rsp_data <= 32'h1C244C8B;
        16'h0D18 : rd_rsp_data <= 32'hB70F21EB;
        16'h0D1C : rd_rsp_data <= 32'h85663447;
        16'h0D20 : rd_rsp_data <= 32'h030974C0;
        16'h0D24 : rd_rsp_data <= 32'h10708BC7;
        16'h0D28 : rd_rsp_data <= 32'h06EBF003;
        16'h0D2C : rd_rsp_data <= 32'h3077B70F;
        16'h0D30 : rd_rsp_data <= 32'hC68BF703;
        16'h0D34 : rd_rsp_data <= 32'hE083D8F7;
        16'h0D38 : rd_rsp_data <= 32'h8BF00307;
        16'h0D3C : rd_rsp_data <= 32'h8B182444;
        16'h0D40 : rd_rsp_data <= 32'h83D72BD6;
        16'h0D44 : rd_rsp_data <= 32'h5489E8C1;
        16'h0D48 : rd_rsp_data <= 32'hD0831424;
        16'h0D4C : rd_rsp_data <= 32'h244C89FF;
        16'h0D50 : rd_rsp_data <= 32'h89FF331C;
        16'h0D54 : rd_rsp_data <= 32'h3B182444;
        16'h0D58 : rd_rsp_data <= 32'hD6870FF8;
        16'h0D5C : rd_rsp_data <= 32'h72000001;
        16'h0D60 : rd_rsp_data <= 32'h0FD13B08;
        16'h0D64 : rd_rsp_data <= 32'h0001CC83;
        16'h0D68 : rd_rsp_data <= 32'h087D8A00;
        16'h0D6C : rd_rsp_data <= 32'h20247C83;
        16'h0D70 : rd_rsp_data <= 32'h174E8A00;
        16'h0D74 : rd_rsp_data <= 32'hD1F60274;
        16'h0D78 : rd_rsp_data <= 32'hC18480B0;
        16'h0D7C : rd_rsp_data <= 32'hE8D00475;
        16'h0D80 : rd_rsp_data <= 32'hB60FF875;
        16'h0D84 : rd_rsp_data <= 32'h01E883C0;
        16'h0D88 : rd_rsp_data <= 32'hE8831874;
        16'h0D8C : rd_rsp_data <= 32'h83577403;
        16'h0D90 : rd_rsp_data <= 32'h527404E8;
        16'h0D94 : rd_rsp_data <= 32'h7408E883;
        16'h0D98 : rd_rsp_data <= 32'h10E8831C;
        16'h0D9C : rd_rsp_data <= 32'h01A1850F;
        16'h0DA0 : rd_rsp_data <= 32'h46F60000;
        16'h0DA4 : rd_rsp_data <= 32'h840F0113;
        16'h0DA8 : rd_rsp_data <= 32'h00000161;
        16'h0DAC : rd_rsp_data <= 32'h5058206A;
        16'h0DB0 : rd_rsp_data <= 32'h00015CE9;
        16'h0DB4 : rd_rsp_data <= 32'h1346F600;
        16'h0DB8 : rd_rsp_data <= 32'h8B057401;
        16'h0DBC : rd_rsp_data <= 32'h16EB184E;
        16'h0DC0 : rd_rsp_data <= 32'h164EB60F;
        16'h0DC4 : rd_rsp_data <= 32'h1546B60F;
        16'h0DC8 : rd_rsp_data <= 32'h0B08E1C1;
        16'h0DCC : rd_rsp_data <= 32'h46B60FC8;
        16'h0DD0 : rd_rsp_data <= 32'h08E1C114;
        16'h0DD4 : rd_rsp_data <= 32'hC18BC80B;
        16'h0DD8 : rd_rsp_data <= 32'hE083D8F7;
        16'h0DDC : rd_rsp_data <= 32'h8BC10307;
        16'h0DE0 : rd_rsp_data <= 32'h012CE9F8;
        16'h0DE4 : rd_rsp_data <= 32'h46F60000;
        16'h0DE8 : rd_rsp_data <= 32'h448D0113;
        16'h0DEC : rd_rsp_data <= 32'h26742824;
        16'h0DF0 : rd_rsp_data <= 32'h5056206A;
        16'h0DF4 : rd_rsp_data <= 32'h004126E8;
        16'h0DF8 : rd_rsp_data <= 32'h0CC48300;
        16'h0DFC : rd_rsp_data <= 32'h5488D233;
        16'h0E00 : rd_rsp_data <= 32'hC28A3F24;
        16'h0E04 : rd_rsp_data <= 32'h39245488;
        16'h0E08 : rd_rsp_data <= 32'h4402CA8B;
        16'h0E0C : rd_rsp_data <= 32'h8341280C;
        16'h0E10 : rd_rsp_data <= 32'hF67220F9;
        16'h0E14 : rd_rsp_data <= 32'h186A24EB;
        16'h0E18 : rd_rsp_data <= 32'h00E85056;
        16'h0E1C : rd_rsp_data <= 32'h83000041;
        16'h0E20 : rd_rsp_data <= 32'hD2330CC4;
        16'h0E24 : rd_rsp_data <= 32'h3F245488;
        16'h0E28 : rd_rsp_data <= 32'h5488C28A;
        16'h0E2C : rd_rsp_data <= 32'hCA8B3924;
        16'h0E30 : rd_rsp_data <= 32'h280C4402;
        16'h0E34 : rd_rsp_data <= 32'h18F98341;
        16'h0E38 : rd_rsp_data <= 32'hC084F672;
        16'h0E3C : rd_rsp_data <= 32'h0111850F;
        16'h0E40 : rd_rsp_data <= 32'h5E8A0000;
        16'h0E44 : rd_rsp_data <= 32'h80D38A13;
        16'h0E48 : rd_rsp_data <= 32'h1D7401E2;
        16'h0E4C : rd_rsp_data <= 32'h8B184E8B;
        16'h0E50 : rd_rsp_data <= 32'h83DFF7F9;
        16'h0E54 : rd_rsp_data <= 32'hF90307E7;
        16'h0E58 : rd_rsp_data <= 32'h13244438;
        16'h0E5C : rd_rsp_data <= 32'h548B2A75;
        16'h0E60 : rd_rsp_data <= 32'hC78B1424;
        16'h0E64 : rd_rsp_data <= 32'h0000A9E9;
        16'h0E68 : rd_rsp_data <= 32'h4EB60F00;
        16'h0E6C : rd_rsp_data <= 32'h46B60F16;
        16'h0E70 : rd_rsp_data <= 32'h08E1C115;
        16'h0E74 : rd_rsp_data <= 32'hB60FC80B;
        16'h0E78 : rd_rsp_data <= 32'hE1C11446;
        16'h0E7C : rd_rsp_data <= 32'h8BC80B08;
        16'h0E80 : rd_rsp_data <= 32'h83DFF7F9;
        16'h0E84 : rd_rsp_data <= 32'hF90307E7;
        16'h0E88 : rd_rsp_data <= 32'hC3F6AAB0;
        16'h0E8C : rd_rsp_data <= 32'h332E7440;
        16'h0E90 : rd_rsp_data <= 32'h74D284C0;
        16'h0E94 : rd_rsp_data <= 32'hE0518D14;
        16'h0E98 : rd_rsp_data <= 32'hD285C933;
        16'h0E9C : rd_rsp_data <= 32'h44021D74;
        16'h0EA0 : rd_rsp_data <= 32'h3B41200E;
        16'h0EA4 : rd_rsp_data <= 32'hEBF772CA;
        16'h0EA8 : rd_rsp_data <= 32'hE8518D12;
        16'h0EAC : rd_rsp_data <= 32'hD285C933;
        16'h0EB0 : rd_rsp_data <= 32'h44020974;
        16'h0EB4 : rd_rsp_data <= 32'h3B41180E;
        16'h0EB8 : rd_rsp_data <= 32'hF6F772CA;
        16'h0EBC : rd_rsp_data <= 32'h114638D8;
        16'h0EC0 : rd_rsp_data <= 32'h0084850F;
        16'h0EC4 : rd_rsp_data <= 32'h448B0000;
        16'h0EC8 : rd_rsp_data <= 32'hC0852424;
        16'h0ECC : rd_rsp_data <= 32'hD08B1674;
        16'h0ED0 : rd_rsp_data <= 32'hADE8CE8B;
        16'h0ED4 : rd_rsp_data <= 32'h84000040;
        16'h0ED8 : rd_rsp_data <= 32'h8B8374C0;
        16'h0EDC : rd_rsp_data <= 32'h30890C45;
        16'h0EE0 : rd_rsp_data <= 32'h58EBC033;
        16'h0EE4 : rd_rsp_data <= 32'h8012468A;
        16'h0EE8 : rd_rsp_data <= 32'h1175FFFF;
        16'h0EEC : rd_rsp_data <= 32'hEB74063C;
        16'h0EF0 : rd_rsp_data <= 32'hE774083C;
        16'h0EF4 : rd_rsp_data <= 32'hE3740B3C;
        16'h0EF8 : rd_rsp_data <= 32'hFFFF61E9;
        16'h0EFC : rd_rsp_data <= 32'h74F83AFF;
        16'h0F00 : rd_rsp_data <= 32'h0FFF8408;
        16'h0F04 : rd_rsp_data <= 32'hFFFF5585;
        16'h0F08 : rd_rsp_data <= 32'hEBF03CFF;
        16'h0F0C : rd_rsp_data <= 32'h58186ACC;
        16'h0F10 : rd_rsp_data <= 32'hD7035F50;
        16'h0F14 : rd_rsp_data <= 32'hFF33F003;
        16'h0F18 : rd_rsp_data <= 32'h14245489;
        16'h0F1C : rd_rsp_data <= 32'h18247C3B;
        16'h0F20 : rd_rsp_data <= 32'h820F1077;
        16'h0F24 : rd_rsp_data <= 32'hFFFFFE44;
        16'h0F28 : rd_rsp_data <= 32'h1C24543B;
        16'h0F2C : rd_rsp_data <= 32'hFE3A820F;
        16'h0F30 : rd_rsp_data <= 32'h5D8BFFFF;
        16'h0F34 : rd_rsp_data <= 32'hB83B890C;
        16'h0F38 : rd_rsp_data <= 32'h8000000E;
        16'h0F3C : rd_rsp_data <= 32'h8B5B5E5F;
        16'h0F40 : rd_rsp_data <= 32'h8BC35DE5;
        16'h0F44 : rd_rsp_data <= 32'h38890C45;
        16'h0F48 : rd_rsp_data <= 32'h458BEDEB;
        16'h0F4C : rd_rsp_data <= 32'h89C9330C;
        16'h0F50 : rd_rsp_data <= 32'h8BE4EB08;
        16'h0F54 : rd_rsp_data <= 32'h10890C45;
        16'h0F58 : rd_rsp_data <= 32'h8B55DDEB;
        16'h0F5C : rd_rsp_data <= 32'h535151EC;
        16'h0F60 : rd_rsp_data <= 32'h8BD98B56;
        16'h0F64 : rd_rsp_data <= 32'hC0B957F2;
        16'h0F68 : rd_rsp_data <= 32'hE8FFFC65;
        16'h0F6C : rd_rsp_data <= 32'h00003CC6;
        16'h0F70 : rd_rsp_data <= 32'hFC653CB9;
        16'h0F74 : rd_rsp_data <= 32'h3CBCE8FF;
        16'h0F78 : rd_rsp_data <= 32'h7E8B0000;
        16'h0F7C : rd_rsp_data <= 32'hFC458D04;
        16'h0F80 : rd_rsp_data <= 32'h33006A50;
        16'h0F84 : rd_rsp_data <= 32'h104F8DD2;
        16'h0F88 : rd_rsp_data <= 32'h003CF7E8;
        16'h0F8C : rd_rsp_data <= 32'hF8458D00;
        16'h0F90 : rd_rsp_data <= 32'h2077FF50;
        16'h0F94 : rd_rsp_data <= 32'hFFFC458B;
        16'h0F98 : rd_rsp_data <= 32'hFF500476;
        16'h0F9C : rd_rsp_data <= 32'h18C48310;
        16'h0FA0 : rd_rsp_data <= 32'h000100B9;
        16'h0FA4 : rd_rsp_data <= 32'h3A9FE800;
        16'h0FA8 : rd_rsp_data <= 32'h4B8B0000;
        16'h0FAC : rd_rsp_data <= 32'h05E1C134;
        16'h0FB0 : rd_rsp_data <= 32'hC73C4389;
        16'h0FB4 : rd_rsp_data <= 32'h00083843;
        16'h0FB8 : rd_rsp_data <= 32'h3C890000;
        16'h0FBC : rd_rsp_data <= 32'h34538B01;
        16'h0FC0 : rd_rsp_data <= 32'h8B3C4B8B;
        16'h0FC4 : rd_rsp_data <= 32'hE2C1FC45;
        16'h0FC8 : rd_rsp_data <= 32'h0A448905;
        16'h0FCC : rd_rsp_data <= 32'h34538B04;
        16'h0FD0 : rd_rsp_data <= 32'h8B3C4B8B;
        16'h0FD4 : rd_rsp_data <= 32'hE2C1F845;
        16'h0FD8 : rd_rsp_data <= 32'h0A448905;
        16'h0FDC : rd_rsp_data <= 32'h344B8B08;
        16'h0FE0 : rd_rsp_data <= 32'hC13C438B;
        16'h0FE4 : rd_rsp_data <= 32'h648305E1;
        16'h0FE8 : rd_rsp_data <= 32'hB9001C01;
        16'h0FEC : rd_rsp_data <= 32'hFFFC6578;
        16'h0FF0 : rd_rsp_data <= 32'hE83443FF;
        16'h0FF4 : rd_rsp_data <= 32'h00003CBB;
        16'h0FF8 : rd_rsp_data <= 32'h8B5B5E5F;
        16'h0FFC : rd_rsp_data <= 32'h55C35DE5;
        default: rd_rsp_data <= 32'h00000000;
    endcase
        end else if (dwr_valid) begin
            case (({dwr_addr[31:24], dwr_addr[23:16], dwr_addr[15:08], dwr_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'h00FF)
                //Dont be scared
            endcase
        end else begin
            rd_rsp_data <= 32'h00000000;
        end
    end
            
endmodule
