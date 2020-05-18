//
// Copyright (C) 2019-2020  Markus Hiienkari <mhiienka@niksula.hut.fi>
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
    input HSYNC_i,
    input VSYNC_i,
    input DE_i,
    input FID_i,
    input vs_type,
    input vs_polarity,
    input csc_enable,
    input csc_cs,
    input [31:0] hv_in_config,
    input [31:0] hv_in_config2,
    input [31:0] hv_in_config3,
    output reg [7:0] R_o,
    output reg [7:0] G_o,
    output reg [7:0] B_o,
    output reg HSYNC_o,
    output reg VSYNC_o,
    output reg DE_o,
    output reg FID_o,
    output reg interlace_flag,
    output reg [10:0] xpos,
    output reg [10:0] ypos,
    output reg [10:0] vtotal,
    output reg frame_change,
    output reg [19:0] pcnt_frame
);

localparam FID_EVEN = 1'b0;
localparam FID_ODD = 1'b1;

localparam VSYNC_SEPARATED = 1'b0;
localparam VSYNC_RAW = 1'b1;

localparam CSC_YCBCR601_RGB = 1'b0;
localparam CSC_YCBCR709_RGB = 1'b1;

reg [11:0] h_ctr;
reg [10:0] v_ctr;
reg [10:0] vmax_ctr;
reg HS_i_prev;
reg VSYNC_i_np_prev;
reg [1:0] fid_next_ctr;
reg fid_next;

reg [7:0] R, G, B;
reg HSYNC, VSYNC, FID;

// Measurement registers
reg [19:0] pcnt_ctr;
reg frame_change_meas_sync1_reg, frame_change_meas_sync2_reg, frame_change_meas_prev;
wire frame_change_meas = frame_change_meas_sync2_reg;

wire [11:0] H_TOTAL = hv_in_config[11:0];
wire [10:0] H_ACTIVE = hv_in_config[22:12];
wire [8:0] H_BACKPORCH = hv_in_config[31:23];
wire [8:0] H_SYNCLEN = hv_in_config2[8:0];

wire [10:0] V_ACTIVE = hv_in_config2[30:20];
wire [8:0] V_BACKPORCH = hv_in_config3[8:0];
wire [4:0] V_SYNCLEN = hv_in_config3[13:9];

wire [11:0] even_min_thold_hv = (H_TOTAL / 12'd4);
wire [11:0] even_max_thold_hv = (H_TOTAL / 12'd2) + (H_TOTAL / 12'd4);
wire [11:0] even_min_thold_ss = (H_TOTAL / 12'd2);
wire [11:0] even_max_thold_ss = H_TOTAL;
wire [11:0] even_min_thold = (vs_type == VSYNC_SEPARATED) ? even_min_thold_ss : even_min_thold_hv;
wire [11:0] even_max_thold = (vs_type == VSYNC_SEPARATED) ? even_max_thold_ss : even_max_thold_hv;

// TODO: calculate V polarity independently
wire VSYNC_i_np = (VSYNC_i ^ ~vs_polarity);

// CSC_registers
reg [10:0] Y;
reg [10:0] Cb;
reg [10:0] Cr;
reg [17:0] R_Cr_coeff;
reg [17:0] G_Cb_coeff;
reg [17:0] G_Cr_coeff;
reg [17:0] B_Cb_coeff;
reg [10:0] Y_1, R_Cr, G_Cb, G_Cr, B_Cb;
reg [7:0] R_csc, G_csc, B_csc;

wire [35:0] R_Cr_pre, G_Cb_pre, G_Cr_pre, B_Cb_pre;
wire [10:0] R_csc_sum, G_csc_sum, B_csc_sum;


always @(posedge PCLK_i) begin
    R <= R_i;
    G <= G_i;
    B <= B_i;

    HS_i_prev <= HS_i;
    VSYNC_i_np_prev <= VSYNC_i_np;

    if (HS_i_prev & ~HS_i) begin
        h_ctr <= 0;
        HSYNC <= 1'b0;

        if (fid_next_ctr > 0)
            fid_next_ctr <= fid_next_ctr - 1'b1;

        if (fid_next_ctr == 2'h1) begin
            v_ctr <= 0;
            if (~(interlace_flag & (fid_next == FID_EVEN))) begin
                vmax_ctr <= 0;
                vtotal <= vmax_ctr + 1'b1;
                frame_change <= 1'b1;
            end else begin
                vmax_ctr <= vmax_ctr + 1'b1;
            end
        end else begin
            v_ctr <= v_ctr + 1'b1;
            vmax_ctr <= vmax_ctr + 1'b1;
            frame_change <= 1'b0;
        end
    end else begin
        h_ctr <= h_ctr + 1'b1;
        if (h_ctr == H_SYNCLEN-1)
            HSYNC <= 1'b1;
    end

    // vsync leading edge processing per quadrant
    if (VSYNC_i_np_prev & ~VSYNC_i_np) begin
        if (h_ctr < even_min_thold) begin
            fid_next <= FID_ODD;
            fid_next_ctr <= 2'h1;
        end else if (h_ctr > even_max_thold) begin
            fid_next <= FID_ODD;
            fid_next_ctr <= 2'h2;
        end else begin
            fid_next <= FID_EVEN;
            fid_next_ctr <= 2'h2;
        end
    end

    if (((fid_next == FID_ODD) & (HS_i_prev & ~HS_i)) | ((fid_next == FID_EVEN) & (h_ctr == (H_TOTAL/2)-1'b1))) begin
        if (fid_next_ctr == 2'h1) begin
            VSYNC <= 1'b0;
            FID <= fid_next;
            interlace_flag <= FID ^ fid_next;
        end else begin
            if (v_ctr == V_SYNCLEN-1)
                VSYNC <= 1'b1;
        end
    end
end

always @(posedge PCLK_i) begin
    R_o <= csc_enable ? R_csc : R;
    G_o <= csc_enable ? G_csc : G;
    B_o <= csc_enable ? B_csc : B;
    HSYNC_o <= HSYNC;
    VSYNC_o <= VSYNC;
    FID_o <= FID;

    DE_o <= (h_ctr >= H_SYNCLEN+H_BACKPORCH) & (h_ctr < H_SYNCLEN+H_BACKPORCH+H_ACTIVE) & (v_ctr >= V_SYNCLEN+V_BACKPORCH) & (v_ctr < V_SYNCLEN+V_BACKPORCH+V_ACTIVE);
    xpos <= (h_ctr-H_SYNCLEN-H_BACKPORCH);
    ypos <= (v_ctr-V_SYNCLEN-V_BACKPORCH);
end

//Calculate exact vertical frequency
always @(posedge CLK_MEAS_i) begin
    if (~frame_change_meas_prev & frame_change_meas) begin
        pcnt_ctr <= 1;
        pcnt_frame <= pcnt_ctr;
    end else if (pcnt_ctr < 20'hfffff) begin
        pcnt_ctr <= pcnt_ctr + 1'b1;
    end

    frame_change_meas_sync1_reg <= frame_change;
    frame_change_meas_sync2_reg <= frame_change_meas_sync1_reg;
    frame_change_meas_prev <= frame_change_meas;
end

// CSC
always @(posedge PCLK_i) begin
    if (csc_enable) begin
        Y <= {3'b000, G_i};
        Cb <= {3'b000, B_i} - 11'd128;
        Cr <= {3'b000, R_i} - 11'd128;

        Y_1 <= Y;
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

assign R_csc_sum = Y_1 + R_Cr;
assign G_csc_sum = Y_1 - G_Cr - G_Cb;
assign B_csc_sum = Y_1 + B_Cb;

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
