`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// hdmi_bars.v
//
// 1280 x 720 @ 60 Hz video timing generator with a built-in test pattern.
// Step 1 of the HDMI waterfall build: it proves the clocks, the pins, the
// cable and the monitor before any waterfall logic exists.
//
//   pix_clk    pixel clock from the Clocking Wizard (74.21875 MHz)
//   locked     Clocking Wizard "locked" output (1 = clocks are good)
//   vid_out    video to Digilent rgb2dvi (connect to its RGB interface)
//   led_lock   on while the Clocking Wizard is locked
//   led_frame  toggles every 30 frames, so it blinks once a second
//
// vid_pData is packed {R, B, G}: red in [23:16], blue in [15:8], green in
// [7:0]. That is the order rgb2dvi expects on vid_pData.
//
// Timing is CEA-861 1280x720p60: 1650 x 750 total, both syncs active high,
// vsync edges lined up with the leading edge of hsync.
//////////////////////////////////////////////////////////////////////////////
module hdmi_bars (
    input  wire        pix_clk,
    input  wire        locked,

    (* X_INTERFACE_INFO = "xilinx.com:interface:vid_io:1.0 vid_out DATA" *)
    output reg  [23:0] vid_pData,
    (* X_INTERFACE_INFO = "xilinx.com:interface:vid_io:1.0 vid_out HSYNC" *)
    output reg         vid_pHSync,
    (* X_INTERFACE_INFO = "xilinx.com:interface:vid_io:1.0 vid_out VSYNC" *)
    output reg         vid_pVSync,
    (* X_INTERFACE_INFO = "xilinx.com:interface:vid_io:1.0 vid_out ACTIVE_VIDEO" *)
    output reg         vid_pVDE,

    output wire        led_lock,
    output reg         led_frame
);

    // ---------------------------------------------------------------------
    // 1280x720p60 timing, in pixels and lines
    // ---------------------------------------------------------------------
    localparam H_ACTIVE = 1280;
    localparam H_FP     = 110;                               // front porch
    localparam H_SYNC   = 40;
    localparam H_BP     = 220;                               // back porch
    localparam H_TOTAL  = H_ACTIVE + H_FP + H_SYNC + H_BP;   // 1650

    localparam V_ACTIVE = 720;
    localparam V_FP     = 5;
    localparam V_SYNC   = 5;
    localparam V_BP     = 20;
    localparam V_TOTAL  = V_ACTIVE + V_FP + V_SYNC + V_BP;   // 750

    localparam HS_START = H_ACTIVE + H_FP;                   // 1390
    localparam HS_END   = HS_START + H_SYNC;                 // 1430
    localparam VS_START = V_ACTIVE + V_FP - 1;               // 724
    localparam VS_END   = VS_START + V_SYNC;                 // 729

    // ---------------------------------------------------------------------
    // Reset: held while the Clocking Wizard is not locked, released in step
    // with pix_clk.
    // ---------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] lock_ff = 2'b00;

    always @(posedge pix_clk or negedge locked) begin
        if (!locked) lock_ff <= 2'b00;
        else         lock_ff <= {lock_ff[0], 1'b1};
    end

    wire rst = ~lock_ff[1];

    // ---------------------------------------------------------------------
    // Position counters. (hcnt, vcnt) = (0, 0) is the top-left pixel.
    // ---------------------------------------------------------------------
    reg [10:0] hcnt = 11'd0;                                 // 0 .. 1649
    reg [9:0]  vcnt = 10'd0;                                 // 0 .. 749

    wire h_last = (hcnt == H_TOTAL - 1);
    wire v_last = (vcnt == V_TOTAL - 1);

    always @(posedge pix_clk) begin
        if (rst) begin
            hcnt <= 11'd0;
            vcnt <= 10'd0;
        end else if (h_last) begin
            hcnt <= 11'd0;
            vcnt <= v_last ? 10'd0 : vcnt + 10'd1;
        end else begin
            hcnt <= hcnt + 11'd1;
        end
    end

    // ---------------------------------------------------------------------
    // Sync and data-enable, decoded from the counters.
    // vsync rises and falls where hsync rises, as CEA-861 asks.
    // ---------------------------------------------------------------------
    wire active = (hcnt < H_ACTIVE) && (vcnt < V_ACTIVE);
    wire hs     = (hcnt >= HS_START) && (hcnt < HS_END);
    wire vs     = ((vcnt == VS_START) && (hcnt >= HS_START)) ||
                  ((vcnt >  VS_START) && (vcnt <  VS_END))   ||
                  ((vcnt == VS_END)   && (hcnt <  HS_START));

    // ---------------------------------------------------------------------
    // Test pattern
    //   rows   0..479  eight colour bars, 160 pixels each
    //   rows 480..539  grey ramp   (0..255, repeated five times)
    //   rows 540..599  red ramp
    //   rows 600..659  green ramp
    //   rows 660..719  blue ramp
    //   one-pixel white border around the whole picture
    // ---------------------------------------------------------------------
    wire border = (hcnt == 0) || (hcnt == H_ACTIVE - 1) ||
                  (vcnt == 0) || (vcnt == V_ACTIVE - 1);

    wire [2:0] bar = (hcnt <  160) ? 3'd0 :
                     (hcnt <  320) ? 3'd1 :
                     (hcnt <  480) ? 3'd2 :
                     (hcnt <  640) ? 3'd3 :
                     (hcnt <  800) ? 3'd4 :
                     (hcnt <  960) ? 3'd5 :
                     (hcnt < 1120) ? 3'd6 : 3'd7;

    wire [7:0] ramp = hcnt[7:0];

    reg [7:0] r, g, b;

    always @* begin
        if (border) begin                  // white frame
            r = 8'hFF;  g = 8'hFF;  b = 8'hFF;
        end else if (vcnt < 480) begin     // white yellow cyan green magenta red blue black
            r = {8{~bar[1]}};
            g = {8{~bar[2]}};
            b = {8{~bar[0]}};
        end else if (vcnt < 540) begin     // grey
            r = ramp;   g = ramp;   b = ramp;
        end else if (vcnt < 600) begin     // red
            r = ramp;   g = 8'h00;  b = 8'h00;
        end else if (vcnt < 660) begin     // green
            r = 8'h00;  g = ramp;   b = 8'h00;
        end else begin                     // blue
            r = 8'h00;  g = 8'h00;  b = ramp;
        end
    end

    // ---------------------------------------------------------------------
    // Output registers: data, syncs and data-enable leave on the same edge.
    // ---------------------------------------------------------------------
    always @(posedge pix_clk) begin
        if (rst) begin
            vid_pData  <= 24'd0;
            vid_pHSync <= 1'b0;
            vid_pVSync <= 1'b0;
            vid_pVDE   <= 1'b0;
        end else begin
            vid_pData  <= active ? {r, b, g} : 24'd0;        // {R, B, G}
            vid_pHSync <= hs;
            vid_pVSync <= vs;
            vid_pVDE   <= active;
        end
    end

    // ---------------------------------------------------------------------
    // Bring-up LEDs
    // ---------------------------------------------------------------------
    reg [4:0] fcnt = 5'd0;

    always @(posedge pix_clk) begin
        if (rst) begin
            fcnt      <= 5'd0;
            led_frame <= 1'b0;
        end else if (h_last && v_last) begin                 // once per frame
            if (fcnt == 5'd29) begin
                fcnt      <= 5'd0;
                led_frame <= ~led_frame;
            end else begin
                fcnt <= fcnt + 5'd1;
            end
        end
    end

    assign led_lock = locked;

endmodule
