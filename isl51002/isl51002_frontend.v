//
// Copyright (C) 2019-2023  Markus Hiienkari <mhiienka@niksula.hut.fi>
//
// This file is part of Open Source Scan Converter project.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

module isl51002_frontend (
    input PCLK_i,
    input CLK_MEAS_i,
    input reset_n,
    input [7:0] R_i,
    input [7:0] G_i,
    input [7:0] B_i,
    input HS_i,
    input VS_i,
    input HSYNC_i,
    input VSYNC_i,
    input DE_i,
    input FID_i,
    input hsync_i_polarity,
    input vsync_i_polarity,
    input vsync_i_type,
    input csc_enable,
    input csc_cs,
    input [31:0] hv_in_config,
    input [31:0] hv_in_config2,
    input [31:0] hv_in_config3,
    input [31:0] misc_config,
    input [31:0] misc_config2,
    output [7:0] R_o,
    output [7:0] G_o,
    output [7:0] B_o,
    output HSYNC_o,
    output VSYNC_o,
    output DE_o,
    output FID_o,
    output reg interlace_flag,
    output datavalid_o,
    output [10:0] xpos_o,
    output [10:0] ypos_o,
    output reg [10:0] vtotal,
    output reg frame_change,
    output reg sof_scaler,
    output reg [19:0] pcnt_field
);

localparam FID_EVEN = 1'b0;
localparam FID_ODD = 1'b1;

localparam VSYNC_SEPARATED = 1'b0;
localparam VSYNC_RAW = 1'b1;

localparam CSC_YCBCR601_RGB = 1'b0;
localparam CSC_YCBCR709_RGB = 1'b1;

localparam PP_PL_START      = 1;
localparam PP_DE_POS_START  = PP_PL_START;
localparam PP_DE_POS_LENGTH = 1;
localparam PP_DE_POS_END    = PP_DE_POS_START + PP_DE_POS_LENGTH;
localparam PP_CSC_START     = 0;
localparam PP_CSC_LENGTH    = 5;
localparam PP_CSC_END       = PP_CSC_START + PP_CSC_LENGTH;
localparam PP_RLPF_START    = PP_CSC_END;
localparam PP_RLPF_LENGTH   = 3;
localparam PP_RLPF_END      = PP_RLPF_START + PP_RLPF_LENGTH;
localparam PP_PL_END        = PP_RLPF_END;

reg [11:0] h_cnt;
reg [10:0] v_cnt;
reg [10:0] vmax_cnt;
reg HS_i_prev, VS_i_np_prev;
reg HSYNC_i_np_prev, VSYNC_i_np_prev;
reg [1:0] fid_next_ctr;
reg fid_next;
reg [3:0] h_ctr;

reg [7:0] R_pp[PP_PL_START:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg [7:0] G_pp[PP_PL_START:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg [7:0] B_pp[PP_PL_START:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg HSYNC_pp[PP_PL_START:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg VSYNC_pp[PP_PL_START:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg FID_pp[PP_PL_START:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg DE_pp[PP_DE_POS_END:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg datavalid_pp[PP_DE_POS_END:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg [10:0] xpos_pp[PP_DE_POS_END:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg [10:0] ypos_pp[PP_DE_POS_END:PP_PL_END] /* synthesis ramstyle = "logic" */;

// Reverse LPF
wire rlpf_trigger_act;
reg signed [14:0] R_diff_s15_pre, G_diff_s15_pre, B_diff_s15_pre, R_diff_s15, G_diff_s15, B_diff_s15;
reg [7:0] R_pp_prev_rlpf, G_pp_prev_rlpf, B_pp_prev_rlpf;

// Lumacode
reg [1:0] lc_code[1:4];
reg [2:0] lc_ctr;
reg [2:0] lc_cnt;
reg [2:0] lc_emp_nes;

// Measurement registers
reg [20:0] pcnt_frame_ctr;
reg [11:0] pcnt_line, pcnt_line_ctr, meas_h_cnt;
reg pcnt_line_stored;
reg [10:0] meas_v_cnt;
reg meas_vsync_region, meas_hl_det, meas_fid;

wire [11:0] H_TOTAL = hv_in_config[11:0];
wire [11:0] H_ACTIVE = hv_in_config[23:12];
wire [7:0] H_SYNCLEN = hv_in_config[31:24];
wire [8:0] H_BACKPORCH = hv_in_config2[8:0];

wire [10:0] V_ACTIVE = hv_in_config3[11:0];
wire [3:0] V_SYNCLEN = hv_in_config3[15:12];
wire [8:0] V_BACKPORCH = hv_in_config2[29:21];

wire [5:0] MISC_REV_LPF_STR = (misc_config[11:7] + 6'd16);
wire MISC_REV_LPF_ENABLE = (misc_config[11:7] != 5'h0);
wire [2:0] MISC_LUMACODE_MODE = misc_config2[2:0];

wire [11:0] even_min_thold_hv = (H_TOTAL / 12'd4);
wire [11:0] even_max_thold_hv = (H_TOTAL / 12'd2) + (H_TOTAL / 12'd4);
wire [11:0] even_min_thold_ss = (H_TOTAL / 12'd2);
wire [11:0] even_max_thold_ss = H_TOTAL;
wire [11:0] even_min_thold = (vsync_i_type == VSYNC_SEPARATED) ? even_min_thold_ss : even_min_thold_hv;
wire [11:0] even_max_thold = (vsync_i_type == VSYNC_SEPARATED) ? even_max_thold_ss : even_max_thold_hv;

wire [11:0] meas_even_min_thold = (vsync_i_type == VSYNC_SEPARATED) ? (pcnt_line / 12'd2) : (pcnt_line / 12'd4);
wire [11:0] meas_even_max_thold = (vsync_i_type == VSYNC_SEPARATED) ? pcnt_line : (pcnt_line / 12'd2) + (pcnt_line / 12'd4);

wire meas_vblank_region = (pcnt_frame_ctr < 8*pcnt_line) | (pcnt_frame_ctr > (({1'b0, pcnt_field}<<interlace_flag) - 4*pcnt_line)) |
                          (interlace_flag & (pcnt_frame_ctr < (pcnt_field+8*pcnt_line)) & (pcnt_frame_ctr > (pcnt_field - 4*pcnt_line)));
wire [11:0] glitch_filt_thold = meas_vblank_region ? (pcnt_line/4) : (pcnt_line/8);

// TODO: calculate H/V polarity independently
wire VS_i_np = (VS_i ^ ~vsync_i_polarity);
wire VSYNC_i_np = (VSYNC_i ^ ~vsync_i_polarity);
wire HSYNC_i_np = (HSYNC_i ^ ~hsync_i_polarity);

// Sample skip for low-res optimized modes
wire [3:0] H_SKIP = hv_in_config2[12:9];
wire [3:0] H_SAMPLE_SEL = hv_in_config2[17:14];

// Lumacode uses 2 samples for {C64, C128, VIC20, Spectrum, TMS99xxA}, 3 samples for NES, 4 samples for VCS and 6 samples for Atari 8bit
wire [2:0] LC_SAMPLES = (MISC_LUMACODE_MODE <= 3) ? 2 : ((MISC_LUMACODE_MODE <= 4) ? 3 : ((MISC_LUMACODE_MODE <= 5) ? 4 : 6));
wire [2:0] LC_H_SKIP = ((H_SKIP+1) / LC_SAMPLES) - 1;

// Lumacode palettes for 2-sample index-based sources (C64, Spectrum, Coleco/MSX)
wire [23:0] lumacode_data_2s[0:2][0:15] = '{'{ 24'h000000,24'h2a1b9d,24'h7d202c,24'h84258c,24'h4c2e00,24'h3c3c3c,24'h646464,24'h4fb3a5,24'h7f410d,24'h6351db,24'h939393,24'hbfd04a,24'h339840,24'hb44f5c,24'h7ce587,24'hffffff},
                                            '{ 24'h000000,24'h000000,24'h0200FD,24'hCF01CE,24'h0100CE,24'hCF0100,24'hFF02FD,24'h01CFCF,24'hFF0201,24'h00CF15,24'h02FFFF,24'hFFFF1D,24'h00FF1C,24'hCFCF15,24'hCFCFCF,24'hFFFFFF},
                                            '{ 24'h000000,24'h5455ed,24'hfc5554,24'hff7978,24'h000000,24'hd4524d,24'h7d76fc,24'h42ebf5,24'h21b03b,24'h21c842,24'hff7978,24'hcccccc,24'hc95bba,24'hd4c154,24'he6ce80,24'hffffff}};

// Lumacode palette for NES
wire [23:0] lumacode_data_3s[0:63] = '{ 24'h000000, 24'h000000, 24'h000000, 24'h000000, 24'h000000, 24'h000000, 24'h000000, 24'h000000,
                                        24'h626262, 24'h001fb2, 24'h2404c8, 24'h5200b2, 24'h730076, 24'h800024, 24'h730b00, 24'h522800, 24'h244400, 24'h005700, 24'h005c00, 24'h005324, 24'h003c76, 24'h000000,
                                        24'hababab, 24'h0d57ff, 24'h4b30ff, 24'h8a13ff, 24'hbc08d6, 24'hd21269, 24'hc72e00, 24'h9d5400, 24'h607b00, 24'h209800, 24'h00a300, 24'h009942, 24'h007db4, 24'h000000,
                                        24'hffffff, 24'h53aeff, 24'h9085ff, 24'hd365ff, 24'hff57ff, 24'hff5dcf, 24'hff7757, 24'hfa9e00, 24'hbdc700, 24'h7ae700, 24'h43f611, 24'h26ef7e, 24'h2cd5f6, 24'h4e4e4e,
                                        24'hffffff, 24'hb6e1ff, 24'hced1ff, 24'he9c3ff, 24'hffbcff, 24'hffbdf4, 24'hffc6c3, 24'hffd59a, 24'he9e681, 24'hcef481, 24'hb6fb9a, 24'ha9fac3, 24'ha9f0f4, 24'hb8b8b8};
wire [7:0] lumacode_data_3s_R = lumacode_data_3s[{lc_code[1], lc_code[2], lc_code[3]}][23:16];
wire [7:0] lumacode_data_3s_G = lumacode_data_3s[{lc_code[1], lc_code[2], lc_code[3]}][15:8];
wire [7:0] lumacode_data_3s_B = lumacode_data_3s[{lc_code[1], lc_code[2], lc_code[3]}][7:0];

// SOF position for scaler
wire [10:0] V_SOF_LINE = hv_in_config3[27:16];

// CSC_registers
reg [10:0] Y;
reg [10:0] Cb;
reg [10:0] Cr;
reg [17:0] R_Cr_coeff;
reg [17:0] G_Cb_coeff;
reg [17:0] G_Cr_coeff;
reg [17:0] B_Cb_coeff;
reg [10:0] Y_q, Y_qq, R_Cr, G_Cb, G_Cr, B_Cb;
reg [7:0] R_csc, G_csc, B_csc;

wire [35:0] R_Cr_pre, G_Cb_pre, G_Cr_pre, B_Cb_pre;
wire [10:0] R_csc_sum, G_csc_sum, B_csc_sum;

function [7:0] apply_reverse_lpf;
    input [7:0] data_prev;
    input signed [14:0] diff;
    reg signed [10:0] result;

    begin
        result = {3'b0,data_prev} + ~diff[14:4]; // allow for a small error to reduce adder length
        apply_reverse_lpf = result[10] ? 8'h00 : |result[9:8] ? 8'hFF : result[7:0];
    end
endfunction


// Pipeline stage 1
always @(posedge PCLK_i) begin
    R_pp[1] <= R_i;
    G_pp[1] <= G_i;
    B_pp[1] <= B_i;

    HS_i_prev <= HS_i;
    VS_i_np_prev <= VS_i_np;

    if (HS_i_prev & ~HS_i) begin
        h_cnt <= 0;
        h_ctr <= 0;
        HSYNC_pp[1] <= 1'b0;

        if (fid_next_ctr > 0)
            fid_next_ctr <= fid_next_ctr - 1'b1;

        if (fid_next_ctr == 2'h1) begin
            // regenerated output timings start lagging by one scanline due to vsync detection,
            // compensate by starting v_cnt from 1 (effectively reduces V_SYNCLEN by 1)
            v_cnt <= 1;
            if (~(interlace_flag & (fid_next == FID_EVEN))) begin
                vmax_cnt <= 0;
                //vtotal <= vmax_cnt + 1'b1;
                frame_change <= 1'b1;
            end else begin
                vmax_cnt <= vmax_cnt + 1'b1;
            end
        end else begin
            v_cnt <= v_cnt + 1'b1;
            vmax_cnt <= vmax_cnt + 1'b1;
            frame_change <= 1'b0;
        end

        sof_scaler <= (vmax_cnt == V_SOF_LINE);
    end else begin
        if (h_ctr == H_SKIP) begin
            h_cnt <= h_cnt + 1'b1;
            h_ctr <= 0;
            if (h_cnt == H_SYNCLEN-1)
                HSYNC_pp[1] <= 1'b1;
        end else begin
            h_ctr <= h_ctr + 1'b1;
        end
    end

    // vsync leading edge processing per quadrant
    if (VS_i_np_prev & ~VS_i_np) begin
        if ((HS_i_prev & ~HS_i) | (h_cnt < even_min_thold)) begin
            fid_next <= FID_ODD;
            fid_next_ctr <= 2'h1;
        end else if ((h_cnt > even_max_thold) | ~interlace_flag) begin
            fid_next <= FID_ODD;
            fid_next_ctr <= 2'h2;
        end else begin
            fid_next <= FID_EVEN;
            fid_next_ctr <= 2'h2;
        end
    end

    if (((fid_next == FID_ODD) & (HS_i_prev & ~HS_i)) | ((fid_next == FID_EVEN) & (h_cnt == (H_TOTAL/2)-1'b1))) begin
        if (fid_next_ctr == 2'h1) begin
            VSYNC_pp[1] <= 1'b0;
            FID_pp[1] <= fid_next;
            //interlace_flag <= FID_pp[1] ^ fid_next;
        end else begin
            if (v_cnt == V_SYNCLEN-1)
                VSYNC_pp[1] <= 1'b1;
        end
    end
end

// Pipeline stage 2
always @(posedge PCLK_i) begin
    // Lumacode sample aggregation
    if (h_ctr == H_SAMPLE_SEL) begin
        lc_code[1] <= G_pp[1][7:6];
        lc_cnt <= 0;
        lc_ctr <= 0;
    end else if (lc_ctr == LC_H_SKIP) begin
        lc_code[2+lc_cnt] <= G_pp[1][7:6];
        lc_cnt <= lc_cnt + 1;
        lc_ctr <= 0;
    end else begin
        lc_ctr <= lc_ctr + 1;
    end

    // Standard output
    if (MISC_LUMACODE_MODE == '0) begin
        {R_pp[2], G_pp[2], B_pp[2]} <= {R_pp[1], G_pp[1], B_pp[1]};
    // Lumacode C64, C128, VIC20, Spectrum, TMS99xxA
    end else if (MISC_LUMACODE_MODE <= 3) begin
        {R_pp[2], G_pp[2], B_pp[2]} <= lumacode_data_2s[MISC_LUMACODE_MODE-1'b1][{lc_code[1], lc_code[2]}];
    // Lumacode NES
    end else if (MISC_LUMACODE_MODE == 4) begin
        if (lc_emp_nes[1] & lc_emp_nes[0])
            R_pp[2] <= lumacode_data_3s_R/2;
        else if (lc_emp_nes[1] | lc_emp_nes[0])
            R_pp[2] <= lumacode_data_3s_R - lumacode_data_3s_R/4;
        else
            R_pp[2] <= lumacode_data_3s_R;

        if (lc_emp_nes[2] & lc_emp_nes[0])
            G_pp[2] <= lumacode_data_3s_G/2;
        else if (lc_emp_nes[2] | lc_emp_nes[0])
            G_pp[2] <= lumacode_data_3s_G - lumacode_data_3s_G/4;
        else
            G_pp[2] <= lumacode_data_3s_G;

        if (lc_emp_nes[2] & lc_emp_nes[1])
            B_pp[2] <= lumacode_data_3s_B/2;
        else if (lc_emp_nes[2] | lc_emp_nes[1])
            B_pp[2] <= lumacode_data_3s_B - lumacode_data_3s_B/4;
        else
            B_pp[2] <= lumacode_data_3s_B;

        if ((h_ctr == H_SAMPLE_SEL) & ({lc_code[1], lc_code[2], lc_code[3]} < 8))
            lc_emp_nes <= {lc_code[2][0], lc_code[3]};
    // TODO: Lumacode VCS
    end else if (MISC_LUMACODE_MODE == 5) begin
        {R_pp[2], G_pp[2], B_pp[2]} <= '0;
    // TODO: Lumacode Atari 8-bit
    end else begin
        {R_pp[2], G_pp[2], B_pp[2]} <= '0;
    end

    HSYNC_pp[2] <= HSYNC_pp[1];
    VSYNC_pp[2] <= VSYNC_pp[1];
    FID_pp[2] <= FID_pp[1];
    DE_pp[2] <= (h_cnt >= H_SYNCLEN+H_BACKPORCH) & (h_cnt < H_SYNCLEN+H_BACKPORCH+H_ACTIVE) & (v_cnt >= V_SYNCLEN+V_BACKPORCH) & (v_cnt < V_SYNCLEN+V_BACKPORCH+V_ACTIVE);
    datavalid_pp[2] <= (h_ctr == H_SAMPLE_SEL);
    xpos_pp[2] <= (h_cnt-H_SYNCLEN-H_BACKPORCH);
    ypos_pp[2] <= (v_cnt-V_SYNCLEN-V_BACKPORCH);
end

// Pipeline stages 3-
integer pp_idx;
always @(posedge PCLK_i) begin
    for(pp_idx = PP_DE_POS_END+1; pp_idx <= PP_PL_END; pp_idx = pp_idx+1) begin
        R_pp[pp_idx] <= R_pp[pp_idx-1];
        G_pp[pp_idx] <= G_pp[pp_idx-1];
        B_pp[pp_idx] <= B_pp[pp_idx-1];
        HSYNC_pp[pp_idx] <= HSYNC_pp[pp_idx-1];
        VSYNC_pp[pp_idx] <= VSYNC_pp[pp_idx-1];
        FID_pp[pp_idx] <= FID_pp[pp_idx-1];
        DE_pp[pp_idx] <= DE_pp[pp_idx-1];
        datavalid_pp[pp_idx] <= datavalid_pp[pp_idx-1];
        xpos_pp[pp_idx] <= xpos_pp[pp_idx-1];
        ypos_pp[pp_idx] <= ypos_pp[pp_idx-1];
    end

    /* ---------- CSC (5 cycles) ---------- */
    R_pp[PP_CSC_END] <= csc_enable ? R_csc : R_pp[PP_CSC_END-1];
    G_pp[PP_CSC_END] <= csc_enable ? G_csc : G_pp[PP_CSC_END-1];
    B_pp[PP_CSC_END] <= csc_enable ? B_csc : B_pp[PP_CSC_END-1];

    /* ---------- Reverse LPF (3 cycles) ---------- */
    // Store a copy of valid sample data
    if (datavalid_pp[PP_RLPF_START]) begin
        R_pp_prev_rlpf <= R_pp[PP_RLPF_START];
        G_pp_prev_rlpf <= G_pp[PP_RLPF_START];
        B_pp_prev_rlpf <= B_pp[PP_RLPF_START];
    end
    // Push previous valid data into pipeline when RLPF enabled
    if (MISC_REV_LPF_ENABLE) begin
        R_pp[PP_RLPF_START+1] <= R_pp_prev_rlpf;
        G_pp[PP_RLPF_START+1] <= G_pp_prev_rlpf;
        B_pp[PP_RLPF_START+1] <= B_pp_prev_rlpf;
    end
    // Calculate diff to previous valid data
    R_diff_s15_pre <= (R_pp_prev_rlpf - R_pp[PP_RLPF_START]);
    G_diff_s15_pre <= (G_pp_prev_rlpf - G_pp[PP_RLPF_START]);
    B_diff_s15_pre <= (B_pp_prev_rlpf - B_pp[PP_RLPF_START]);

    // Cycle 2
    R_diff_s15 <= (R_diff_s15_pre * MISC_REV_LPF_STR);
    G_diff_s15 <= (G_diff_s15_pre * MISC_REV_LPF_STR);
    B_diff_s15 <= (B_diff_s15_pre * MISC_REV_LPF_STR);

    // Cycle 3
    if (MISC_REV_LPF_ENABLE) begin
        R_pp[PP_RLPF_END] <= apply_reverse_lpf(R_pp[PP_RLPF_START+2], R_diff_s15);
        G_pp[PP_RLPF_END] <= apply_reverse_lpf(G_pp[PP_RLPF_START+2], G_diff_s15);
        B_pp[PP_RLPF_END] <= apply_reverse_lpf(B_pp[PP_RLPF_START+2], B_diff_s15);
    end
end

// Output
assign R_o = R_pp[PP_PL_END];
assign G_o = G_pp[PP_PL_END];
assign B_o = B_pp[PP_PL_END];
assign HSYNC_o = HSYNC_pp[PP_PL_END];
assign VSYNC_o = VSYNC_pp[PP_PL_END];
assign FID_o = FID_pp[PP_PL_END];
assign DE_o = DE_pp[PP_PL_END];
assign datavalid_o = datavalid_pp[PP_PL_END];
assign xpos_o = xpos_pp[PP_PL_END];
assign ypos_o = ypos_pp[PP_PL_END];


// Calculate horizontal and vertical counts
always @(posedge CLK_MEAS_i) begin
    if ((VSYNC_i_np_prev & ~VSYNC_i_np) & (~interlace_flag | (meas_fid == FID_EVEN))) begin
        pcnt_frame_ctr <= 1;
        pcnt_line_stored <= 1'b0;
        pcnt_field <= interlace_flag ? (pcnt_frame_ctr>>1) : pcnt_frame_ctr[19:0];
    end else if (pcnt_frame_ctr < 21'h1fffff) begin
        pcnt_frame_ctr <= pcnt_frame_ctr + 1'b1;
    end

    if (HSYNC_i_np_prev & ~HSYNC_i_np) begin
        pcnt_line_ctr <= 1;

        // store count 1ms after vsync
        if (~pcnt_line_stored & (pcnt_frame_ctr > 21'd27000)) begin
            pcnt_line <= pcnt_line_ctr;
            pcnt_line_stored <= 1'b1;
        end
    end else begin
        pcnt_line_ctr <= pcnt_line_ctr + 1'b1;
    end

    HSYNC_i_np_prev <= HSYNC_i_np;
    VSYNC_i_np_prev <= VSYNC_i_np;
end

// Detect interlace and line count
always @(posedge CLK_MEAS_i) begin
    if ((HSYNC_i_np_prev & ~HSYNC_i_np) & (meas_h_cnt > glitch_filt_thold)) begin
        // detect half-line equalization pulses
        if ((meas_h_cnt > ((pcnt_line/2) - (pcnt_line/4))) && (meas_h_cnt < ((pcnt_line/2) + (pcnt_line/4)))) begin
            if (meas_hl_det) begin
                meas_hl_det <= 1'b0;
                meas_h_cnt <= 0;
                meas_v_cnt <= meas_v_cnt + 1'b1;
            end else begin
                meas_hl_det <= 1'b1;
                meas_h_cnt <= meas_h_cnt + 1'b1;
            end
        end else begin
            meas_hl_det <= 1'b0;
            meas_h_cnt <= 0;
            meas_v_cnt <= meas_v_cnt + 1'b1;
            if (VSYNC_i_np)
                meas_vsync_region <= 1'b0;
        end
    end else if (meas_vblank_region & (meas_h_cnt >= pcnt_line+4)) begin
        // hsync may be missing or irregular during vblank, force line change detect if pcnt_line is exceeded
        meas_hl_det <= 1'b0;
        meas_h_cnt <= 0;
        meas_v_cnt <= meas_v_cnt + 1'b1;
        if (VSYNC_i_np)
            meas_vsync_region <= 1'b0;
    end else begin
        meas_h_cnt <= meas_h_cnt + 1'b1;
    end

    if (VSYNC_i_np_prev & ~VSYNC_i_np) begin
        if ((meas_h_cnt < meas_even_min_thold) | (meas_h_cnt > meas_even_max_thold)) begin
            meas_fid <= FID_ODD;
            interlace_flag <= (meas_fid == FID_EVEN);

            if (vsync_i_type == VSYNC_RAW) begin
                // vsync leading edge may occur at hsync edge or either side of it
                if (HSYNC_i_np_prev & ~HSYNC_i_np) begin
                    meas_v_cnt <= 1;
                    vtotal <= meas_v_cnt;
                end else if (meas_h_cnt < meas_even_min_thold) begin
                    meas_v_cnt <= 1;
                    vtotal <= meas_v_cnt - 1'b1;
                end else begin
                    meas_v_cnt <= 0;
                    vtotal <= meas_v_cnt;
                end
            end else begin
                meas_v_cnt <= 0;
                vtotal <= meas_v_cnt;
            end
        end else begin
            meas_fid <= FID_EVEN;
            interlace_flag <= (meas_fid == FID_ODD);
            if (meas_fid == FID_EVEN) begin
                meas_v_cnt <= 0;
                vtotal <= meas_v_cnt;
            end
        end

        meas_vsync_region <= 1'b1;
    end
end

// CSC
always @(posedge PCLK_i) begin
    if (csc_enable) begin
        Y <= {3'b000, G_i};
        Cb <= {3'b000, B_i} - 11'd128;
        Cr <= {3'b000, R_i} - 11'd128;

        Y_q <= Y;
        Y_qq <= Y_q;
        R_Cr <= R_Cr_pre[25:15];
        G_Cb <= G_Cb_pre[25:15];
        G_Cr <= G_Cr_pre[25:15];
        B_Cb <= B_Cb_pre[25:15];

        if (R_csc_sum[10] == 1'b1)
            R_csc <= 8'h00;
        else if ((R_csc_sum[9] | R_csc_sum[8]) == 1'b1)
            R_csc <= 8'hFF;
        else
            R_csc <= R_csc_sum[7:0];

        if (G_csc_sum[10] == 1'b1)
            G_csc <= 8'h00;
        else if ((G_csc_sum[9] | G_csc_sum[8]) == 1'b1)
            G_csc <= 8'hFF;
        else
            G_csc <= G_csc_sum[7:0];

        if (B_csc_sum[10] == 1'b1)
            B_csc <= 8'h00;
        else if ((B_csc_sum[9] | B_csc_sum[8]) == 1'b1)
            B_csc <= 8'hFF;
        else
            B_csc <= B_csc_sum[7:0];
    end

    if (csc_cs == CSC_YCBCR601_RGB) begin
        R_Cr_coeff <= 18'h0B395;
        G_Cb_coeff <= 18'h02C08;
        G_Cr_coeff <= 18'h05B64;
        B_Cb_coeff <= 18'h0E2F1;
    end else begin
        R_Cr_coeff <= 18'h0C8F9;
        G_Cb_coeff <= 18'h017EF;
        G_Cr_coeff <= 18'h03BB2;
        B_Cb_coeff <= 18'h0ED84;
    end
end

assign R_csc_sum = Y_qq + R_Cr;
assign G_csc_sum = Y_qq - G_Cr - G_Cb;
assign B_csc_sum = Y_qq + B_Cb;

lpm_mult csc_mult_component_0 (
    // Inputs
    .dataa  ({{7{Cr[10]}}, Cr}),
    .datab  (R_Cr_coeff),
    .aclr   (1'b0),
    .clken  (csc_enable),
    .clock  (PCLK_i),

    // Outputs
    .result (R_Cr_pre),
    .sum    (1'b0)
);
defparam
    csc_mult_component_0.lpm_widtha             = 18,
    csc_mult_component_0.lpm_widthb             = 18,
    csc_mult_component_0.lpm_widthp             = 36,
    csc_mult_component_0.lpm_widths             = 1,
    csc_mult_component_0.lpm_type               = "LPM_MULT",
    csc_mult_component_0.lpm_representation     = "SIGNED",
    csc_mult_component_0.lpm_hint               = "LPM_PIPELINE=1,MAXIMIZE_SPEED=5";

lpm_mult csc_mult_component_1 (
    // Inputs
    .dataa  ({{7{Cb[10]}}, Cb}),
    .datab  (G_Cb_coeff),
    .aclr   (1'b0),
    .clken  (csc_enable),
    .clock  (PCLK_i),

    // Outputs
    .result (G_Cb_pre),
    .sum    (1'b0)
);
defparam
    csc_mult_component_1.lpm_widtha             = 18,
    csc_mult_component_1.lpm_widthb             = 18,
    csc_mult_component_1.lpm_widthp             = 36,
    csc_mult_component_1.lpm_widths             = 1,
    csc_mult_component_1.lpm_type               = "LPM_MULT",
    csc_mult_component_1.lpm_representation     = "SIGNED",
    csc_mult_component_1.lpm_hint               = "LPM_PIPELINE=1,MAXIMIZE_SPEED=5";

lpm_mult csc_mult_component_2 (
    // Inputs
    .dataa  ({{7{Cr[10]}}, Cr}),
    .datab  (G_Cr_coeff),
    .aclr   (1'b0),
    .clken  (csc_enable),
    .clock  (PCLK_i),

    // Outputs
    .result (G_Cr_pre),
    .sum    (1'b0)
);
defparam
    csc_mult_component_2.lpm_widtha             = 18,
    csc_mult_component_2.lpm_widthb             = 18,
    csc_mult_component_2.lpm_widthp             = 36,
    csc_mult_component_2.lpm_widths             = 1,
    csc_mult_component_2.lpm_type               = "LPM_MULT",
    csc_mult_component_2.lpm_representation     = "SIGNED",
    csc_mult_component_2.lpm_hint               = "LPM_PIPELINE=1,MAXIMIZE_SPEED=5";

lpm_mult csc_mult_component_3 (
    // Inputs
    .dataa  ({{7{Cb[10]}}, Cb}),
    .datab  (B_Cb_coeff),
    .aclr   (1'b0),
    .clken  (csc_enable),
    .clock  (PCLK_i),

    // Outputs
    .result (B_Cb_pre),
    .sum    (1'b0)
);
defparam
    csc_mult_component_3.lpm_widtha             = 18,
    csc_mult_component_3.lpm_widthb             = 18,
    csc_mult_component_3.lpm_widthp             = 36,
    csc_mult_component_3.lpm_widths             = 1,
    csc_mult_component_3.lpm_type               = "LPM_MULT",
    csc_mult_component_3.lpm_representation     = "SIGNED",
    csc_mult_component_3.lpm_hint               = "LPM_PIPELINE=1,MAXIMIZE_SPEED=5";

endmodule
