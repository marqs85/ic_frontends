//
// Copyright (C) 2024  Markus Hiienkari <mhiienka@niksula.hut.fi>
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

module adv7280a_frontend (
    input PCLK_i,
    input CLK_MEAS_i,
    input reset_n,
    input [7:0] P_DATA_i,
    input HS_i,
    input VS_i, // unreliable, 1-2px random offset
    input [31:0] hv_in_config,
    input [31:0] hv_in_config2,
    input [31:0] hv_in_config3,
    output [7:0] R_o,
    output [7:0] G_o,
    output [7:0] B_o,
    output reg HSYNC_o,
    output reg VSYNC_o,
    output DE_o,
    output FID_o,
    output reg interlace_flag,
    output reg datavalid_o,
    output reg [10:0] xpos_o,
    output reg [10:0] ypos_o,
    output reg [10:0] vtotal,
    output reg frame_change,
    output reg sof_scaler,
    output reg [19:0] pcnt_frame
);

localparam FID_EVEN = 1'b0;
localparam FID_ODD = 1'b1;

localparam PP_PL_START      = 1;
localparam PP_PL_END        = 6;

reg HS_i_prev, VS_i_prev;
reg [7:0] EAV, EAV_prev;

reg [11:0] h_cnt;
reg [10:0] v_cnt;
reg [10:0] vmax_cnt;
reg [1:0] h_ctr;
reg frame_change_raw;

// Meas regs
reg frame_change_meas_sync1_reg, frame_change_meas_sync2_reg, frame_change_meas_sync3_reg;
reg [20:0] pcnt_frame_ctr;

reg HSYNC_pp[PP_PL_START+1:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg VSYNC_pp[PP_PL_START+1:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg FID_pp[PP_PL_START:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg DE_pp[PP_PL_START+1:PP_PL_END] /* synthesis ramstyle = "logic" */;
reg [7:0] Cb_pp[PP_PL_START:3] /* synthesis ramstyle = "logic" */;
reg [7:0] Y_pp[PP_PL_START+1:5] /* synthesis ramstyle = "logic" */;
reg [7:0] Cr_pp[PP_PL_START+2:3] /* synthesis ramstyle = "logic" */;

reg [10:0] R_Cr, G_Cb, G_Cr, B_Cb;
reg [7:0] R_csc, G_csc, B_csc;

wire [11:0] H_TOTAL = hv_in_config[11:0];
wire [11:0] H_ACTIVE = hv_in_config[23:12];
wire [7:0] H_SYNCLEN = hv_in_config[31:24];
wire [8:0] H_BACKPORCH = hv_in_config2[8:0];

wire [10:0] V_ACTIVE = hv_in_config3[11:0];
wire [3:0] V_SYNCLEN = hv_in_config3[15:12];
wire [8:0] V_BACKPORCH = hv_in_config2[29:21];

// SOF position for scaler
wire [10:0] V_SOF_LINE = hv_in_config3[27:16];

wire [25:0] R_Cr_pre, G_Cb_pre, G_Cr_pre, B_Cb_pre;
reg [10:0] R_csc_sum, G_csc_sum, B_csc_sum;

always @(posedge PCLK_i) begin
    if (~HS_i_prev & HS_i) begin
        h_cnt <= 0;
        h_ctr <= 0;
        EAV_prev <= EAV;

        // Detect field change when V bit goes from 0 to 1 (where F-bit indicated previously rendered field)
        if (~EAV_prev[5] & EAV[5]) begin
            FID_pp[1] <= EAV[6];
            interlace_flag <= (EAV[6] != FID_pp[1]);

            if (~interlace_flag | (EAV[6] == 1'b1)) begin
                frame_change_raw <= 1'b1;
                v_cnt <= 0;
                vmax_cnt <= 0;
                vtotal <= vmax_cnt + 1'b1;
            end else begin
                v_cnt <= -1;
                vmax_cnt <= vmax_cnt + 1'b1;
            end
        end else begin
            frame_change <= frame_change_raw;
            frame_change_raw <= 1'b0;
            v_cnt <= v_cnt + 1'b1;
            vmax_cnt <= vmax_cnt + 1'b1;
            sof_scaler <= (vmax_cnt == V_SOF_LINE);
        end
    end else begin
        h_ctr <= h_ctr + 1'b1;
        if (h_ctr[0] == 1'b1)
            h_cnt <= h_cnt + 1'b1;
    end

    HSYNC_pp[2] <= (h_cnt < H_SYNCLEN) ? 1'b0 : 1'b1;
    VSYNC_pp[2] <= (v_cnt < V_SYNCLEN) ? 1'b0 : 1'b1;
    FID_pp[2] <= interlace_flag ? FID_pp[1] : FID_ODD;
    DE_pp[2] <= (h_cnt >= H_SYNCLEN+H_BACKPORCH) & (h_cnt < H_SYNCLEN+H_BACKPORCH+H_ACTIVE) & (v_cnt >= V_SYNCLEN+V_BACKPORCH) & (v_cnt < V_SYNCLEN+V_BACKPORCH+V_ACTIVE);

    HS_i_prev <= HS_i;
    VS_i_prev <= VS_i;
    EAV <= P_DATA_i;

    // Merge components (Cb, Y, Cr, Y)
    if (h_ctr == 3) begin
        Cb_pp[1] <= P_DATA_i;
    end
    Cb_pp[2] <= Cb_pp[1];
    Cb_pp[3] <= Cb_pp[2];

    if ((h_ctr == 0) || (h_ctr == 2)) begin
        Y_pp[2] <= P_DATA_i;
    end
    Y_pp[3] <= Y_pp[2];

    if (h_ctr == 1) begin
        Cr_pp[3] <= P_DATA_i;
    end
end

// Pipeline stages 3-
integer pp_idx;
always @(posedge PCLK_i) begin
    for(pp_idx = 3; pp_idx <= PP_PL_END; pp_idx = pp_idx+1) begin
        HSYNC_pp[pp_idx] <= HSYNC_pp[pp_idx-1];
        VSYNC_pp[pp_idx] <= VSYNC_pp[pp_idx-1];
        FID_pp[pp_idx] <= FID_pp[pp_idx-1];
        DE_pp[pp_idx] <= DE_pp[pp_idx-1];
    end

    Y_pp[4] <= Y_pp[3];
    Y_pp[5] <= Y_pp[4];

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

    xpos_o <= (h_cnt-H_SYNCLEN-H_BACKPORCH-2);
    ypos_o <= (v_cnt-V_SYNCLEN-V_BACKPORCH);
    datavalid_o <= ~h_ctr[0];
end

// Detect number of clocks per frame
always @(posedge CLK_MEAS_i) begin
    frame_change_meas_sync1_reg <= frame_change_raw;
    frame_change_meas_sync2_reg <= frame_change_meas_sync1_reg;
    frame_change_meas_sync3_reg <= frame_change_meas_sync2_reg;

    if (~frame_change_meas_sync3_reg & frame_change_meas_sync2_reg) begin
        pcnt_frame_ctr <= 1;
        pcnt_frame <= interlace_flag ? (pcnt_frame_ctr>>1) : pcnt_frame_ctr[19:0];
    end else if (pcnt_frame_ctr < 21'h1fffff) begin
        pcnt_frame_ctr <= pcnt_frame_ctr + 1'b1;
    end
end

assign R_csc_sum = Y_pp[5] + R_Cr;
assign G_csc_sum = Y_pp[5] - G_Cr - G_Cb;
assign B_csc_sum = Y_pp[5] + B_Cb;

// Output
assign R_o = R_csc;
assign G_o = G_csc;
assign B_o = B_csc;
assign HSYNC_o = HSYNC_pp[PP_PL_END];
assign VSYNC_o = VSYNC_pp[PP_PL_END];
assign FID_o = FID_pp[PP_PL_END];
assign DE_o = DE_pp[PP_PL_END];

wire signed [17:0] Cr_in = Cr_pp[3] - 8'sd128;
wire signed [17:0] Cb_in = Cb_pp[3] - 8'sd128;

lpm_mult csc_mult_component_0 (
    // Inputs
    .dataa  (Cr_in),
    .datab  (18'h0B395),
    .aclr   (1'b0),
    .clken  (1'b1),
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
    .dataa  (Cb_in),
    .datab  (18'h02C08),
    .aclr   (1'b0),
    .clken  (1'b1),
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
    .dataa  (Cr_in),
    .datab  (18'h05B64),
    .aclr   (1'b0),
    .clken  (1'b1),
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
    .dataa  (Cb_in),
    .datab  (18'h0E2F1),
    .aclr   (1'b0),
    .clken  (1'b1),
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
