//
// Copyright (C) 2019  Markus Hiienkari <mhiienka@niksula.hut.fi>
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

module adv7611_frontend (
    input PCLK_i,
    input reset_n,
    input [7:0] R_i,
    input [7:0] G_i,
    input [7:0] B_i,
    input HSYNC_i,
    input VSYNC_i,
    input DE_i,
    input [31:0] hv_in_config,
    input [31:0] hv_in_config2,
    input [31:0] hv_in_config3,
    input sync_passthru,
    output reg [7:0] R_o,
    output reg [7:0] G_o,
    output reg [7:0] B_o,
    output reg HSYNC_o,
    output reg VSYNC_o,
    output reg DE_o,
    output reg FID_o,
    output reg interlace_flag,
    output reg datavalid_o,
    output reg [10:0] xpos_o,
    output reg [10:0] ypos_o,
    output reg frame_change,
    output reg sof_scaler
);

localparam FID_EVEN = 1'b0;
localparam FID_ODD = 1'b1;

reg HSYNC_i_prev, VSYNC_i_prev, DE_i_prev;
reg FID_prev;

reg [11:0] h_cnt;
reg [10:0] v_cnt;
reg [10:0] vmax_cnt;
reg [4:0] h_ctr;
reg frame_change_raw;

reg [7:0] R_i_prev, G_i_prev, B_i_prev;

wire [11:0] H_ACTIVE = hv_in_config[23:12];
wire [7:0] H_SYNCLEN = hv_in_config[31:24];
wire [8:0] H_BACKPORCH = hv_in_config2[8:0];

wire [10:0] V_ACTIVE = hv_in_config3[11:0];
wire [3:0] V_SYNCLEN = hv_in_config3[15:12];
wire [8:0] V_BACKPORCH = hv_in_config2[29:21];

// Sample skip for low-res modes
wire [4:0] H_SKIP = hv_in_config2[13:9];
wire [4:0] H_SAMPLE_SEL = '0;

// SOF position for scaler
wire [10:0] V_SOF_LINE = hv_in_config3[27:16];

always @(posedge PCLK_i) begin
    if (VSYNC_i_prev & ~VSYNC_i) begin
        if (HSYNC_i_prev & ~HSYNC_i) begin
            FID_o <= FID_ODD;
            interlace_flag <= (FID_o == FID_EVEN);
            frame_change_raw <= 1'b1;
            h_cnt <= 0;
            h_ctr <= 0;
            v_cnt <= 0;
            vmax_cnt <= 0;
        end else begin
            FID_o <= FID_EVEN;
            interlace_flag <= (FID_o == FID_ODD);
            frame_change_raw <= ~interlace_flag;
            if (h_ctr == H_SKIP) begin
                h_cnt <= h_cnt + 1'b1;
                h_ctr <= 0;
            end else begin
                h_ctr <= h_ctr + 1'b1;
            end
            v_cnt <= -1;
        end

        xpos_o <= 0;
        ypos_o <= 0;
    end else begin
        if (HSYNC_i_prev & ~HSYNC_i) begin
            frame_change <= frame_change_raw;
            frame_change_raw <= 1'b0;
            h_cnt <= 0;
            h_ctr <= 0;
            v_cnt <= v_cnt + 1'b1;
            vmax_cnt <= vmax_cnt + 1'b1;
            sof_scaler <= (vmax_cnt == V_SOF_LINE);
        end else begin
            if (h_ctr == H_SKIP) begin
                h_cnt <= h_cnt + 1'b1;
                h_ctr <= 0;
            end else begin
                h_ctr <= h_ctr + 1'b1;
            end
        end

        if (DE_i_prev & ~DE_i) begin
            xpos_o <= 0;
            ypos_o <= ypos_o + 1'b1;
        end else if (DE_i_prev & DE_i) begin
            xpos_o <= xpos_o + 1'b1;
        end
    end

    if (sync_passthru) begin
        R_o <= R_i;
        G_o <= G_i;
        B_o <= B_i;
        HSYNC_o <= HSYNC_i;
        VSYNC_o <= VSYNC_i;
        DE_o <= DE_i;
        datavalid_o <= 1'b1;
    end else begin
        R_o <= R_i_prev;
        G_o <= G_i_prev;
        B_o <= B_i_prev;
        HSYNC_o <= (h_cnt < H_SYNCLEN) ? 1'b0 : 1'b1;
        VSYNC_o <= (v_cnt < V_SYNCLEN) ? 1'b0 : 1'b1;
        DE_o <= (h_cnt >= H_SYNCLEN+H_BACKPORCH) & (h_cnt < H_SYNCLEN+H_BACKPORCH+H_ACTIVE) & (v_cnt >= V_SYNCLEN+V_BACKPORCH) & (v_cnt < V_SYNCLEN+V_BACKPORCH+V_ACTIVE);
        xpos_o <= (h_cnt-H_SYNCLEN-H_BACKPORCH);
        ypos_o <= (v_cnt-V_SYNCLEN-V_BACKPORCH);
        datavalid_o <= (h_ctr == H_SAMPLE_SEL);
    end

    R_i_prev <= R_i;
    G_i_prev <= G_i;
    B_i_prev <= B_i;
    HSYNC_i_prev <= HSYNC_i;
    VSYNC_i_prev <= VSYNC_i;
    DE_i_prev <= DE_i;
end

endmodule
