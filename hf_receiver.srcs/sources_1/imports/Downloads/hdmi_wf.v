`timescale 1ns / 1ps
// hdmi_wf.v - spectrum waterfall for a 1280 x 720 @ 60 Hz monitor (replaces hdmi_bars).
//
//   aclk side     Takes finished spectrum lines from spec_avg (wf_i), converts each bin's
//                 power to a log-scale colour number, merges 2048 bins into 1024 pixels and
//                 stores the line in a ring of block RAM. Nothing is ever copied: a new
//                 line overwrites the oldest one and a pointer moves on.
//   pix_clk side  Scans the ring out with the newest line on top, and draws a spectrum
//                 trace, a frequency ruler, a tuning marker and the receiver's settings.
//
//   Screen layout (x = 0..1279, y = 0..719); the spectrum pane is x = 128..1151:
//     y =   8..135  spectrum trace of the newest line; to its left the tuned frequency,
//                   the mode and the filter width
//     y = 136..167  ruler and tuning marker
//     y = 168..679  waterfall, newest line on top
//     y = 692..703  colour key, weakest on the left
//
//   Two views, told apart by the tag that spec_avg puts on every line (wf_i[77]):
//     wide   0 to 31.25 MHz.  Ruler in MHz, marker at the tuned frequency.
//     local  122 kHz around the tuned frequency.  The ruler shows real frequencies in kHz:
//            a tick on every whole 10 kHz and a number on every whole 20 kHz, and they
//            slide as the receiver is tuned.  The marker stays in the middle; the yellow
//            bar at it shows what the audio filter lets through: both sides of the marker
//            in AM, SAM, CW and FM, the right of it in USB and DIG, the left of it in LSB.
//   When the view changes the waterfall starts again from empty, so the lines on screen
//   always belong to the ruler above them.  When the receiver is retuned in the local view,
//   the lines already on screen are drawn further left or right, so that a station stays
//   under its own frequency.
//
// vid_pData is packed the way Digilent's rgb2dvi expects it: {R, B, G}.

module hdmi_wf #(
    parameter integer LOG2_LINES = 8      // lines of history: 8 = 256 lines, 7 = 128, 6 = 64
)(
    // ---- spectrum side: the 62.5 MHz clock that runs the FFT ----
    input  wire        aclk,
    input  wire [77:0] wf_i,              // spec_avg wf_o: {local, last, valid, bin[10:0], power[63:0]}
    input  wire [31:0] tune_i,            // DDS tuning word: marker, frequency readout, local ruler
    input  wire [15:0] disp_i,            // [15:8] brightness (signed), [7:6] contrast; 0 = automatic
    input  wire [7:0]  stat_i,            // rx_audio stat_o: {clip, lock or squelch open, 0, width[1:0], mode[2:0]}
    // ---- video side: the 74.21875 MHz pixel clock ----
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

    localparam integer LW    = LOG2_LINES;        // bits in a line number
    localparam integer AW    = LOG2_LINES + 10;   // bits in a pixel address: {line, column}

    // One byte per pixel. Written on aclk, read on pix_clk.
    (* ram_style = "block" *) reg [7:0] ram [0:(1<<AW)-1];
    // One tag per stored line: where the local view was centred when the line was taken.
    (* ram_style = "block" *) reg [18:0] tagram [0:(1<<LW)-1];

    // =====================================================================================
    //  aclk side: spectrum line -> 1024 colour numbers -> RAM
    // =====================================================================================

    // ---- stage 0: take the bins in; keep or skip whole lines --------------------------
    // The monitor shows 60 pictures a second, so storing more than one line per picture
    // would be wasted. A line is kept only if 16.74 ms (1,046,528 clocks) have passed since
    // the last kept line began. Averaging 256 FFT frames or more never loses a line.
    localparam [20:0] GAP = 21'd1046528;

    //
    // When the view changes (wide <-> local), a few frames of the old view are still inside
    // the FFT core. So the line that announces the change is not used, lines are ignored for a
    // while after it, and the first line kept after that only sets the noise floor.
    //   into the local view: 66 ms, four of its slow frames
    //   into the wide view:   1 ms, sixteen of its fast frames
    localparam [22:0] HOLD_L = 23'd4128768;       // 2^22 - 2^16 clocks
    localparam [22:0] HOLD_W = 23'd65536;
    //
    // A retune in the local view moves every station sideways. A line holds what arrived
    // since the line before it began, and the FFT core runs up to two of its frames behind,
    // so a line could hold samples from both tunings and would show every station twice.
    // In the local view a line is therefore used only if the line before it began more than
    // 2^22 clocks (four frames, 67 ms) after the tuning word last changed.
    //
    // Every stored line remembers where the local view was centred when it was taken, in
    // pixel columns: (tuning word + half a bin) / 2^13. The video side uses that to draw the
    // line in the right place after a retune.

    wire        in_v = wf_i[75];
    wire        in_l = wf_i[76];
    wire        in_z = wf_i[77];          // the view this line was taken in: 0 wide, 1 local
    reg  [20:0] gap     = GAP;            // clocks since the last kept line began (stops at GAP)
    reg         in_line = 1'b0;           // between the first and last bin of a line
    reg         keep    = 1'b0;           // the current line is being stored
    reg         view    = 1'b0;           // the view on screen
    reg  [22:0] hold    = 23'd0;          // runs down to zero after a change of view
    reg         cal_nx  = 1'b1;           // the next line kept only sets the floor ...
    reg         cal_ln  = 1'b0;           // ... and the current line is that line
    reg  [31:0] tune_q  = 32'd0;          // the tuning word, one clock ago
    reg  [22:0] since   = 23'h7FFFFF;     // clocks since it last changed (stops at the top)
    reg         tq_ok   = 1'b1;           // the line before this one began late enough after that
    reg  [18:0] tag_ln  = 19'd0;          // tag of the line being stored
    wire        tchg    = (tune_i != tune_q);
    wire [31:0] tsum32  = tune_q + 32'd4096;
    wire        gap_ok  = (gap >= GAP);
    wire        first   = in_v && !in_line;
    wire        change  = first && (in_z != view);
    wire        take    = in_line ? keep : (gap_ok && (hold == 23'd0) && !change && (tq_ok || !in_z));

    reg         v0 = 1'b0, l0 = 1'b0, c0 = 1'b0;
    reg  [10:0] a0 = 11'd0;
    reg  [63:0] d0 = 64'd0;

    always @(posedge aclk) begin
        v0 <= in_v && take;
        l0 <= in_l;
        c0 <= in_line ? cal_ln : cal_nx;
        a0 <= wf_i[74:64];
        d0 <= wf_i[63:0];
        if (in_v) begin
            if (!in_line) begin
                keep <= take;
                if (take) begin cal_ln <= cal_nx; cal_nx <= 1'b0; end
            end
            in_line <= !in_l;
        end
        if (first && take) gap <= 21'd1;
        else if (!gap_ok)  gap <= gap + 21'd1;
        if (change) begin
            view   <= in_z;
            hold   <= in_z ? HOLD_L : HOLD_W;
            cal_nx <= 1'b1;
        end else if (hold != 23'd0) begin
            hold <= hold - 23'd1;
        end
        tune_q <= tune_i;
        if (tchg)                      since <= 23'd0;
        else if (since != 23'h7FFFFF)  since <= since + 23'd1;
        if (tchg)                      tq_ok <= 1'b0;
        else if (first)                tq_ok <= since[22];
        if (first && take)             tag_ln <= tsum32[31:13];
    end

    // ---- stages 1 and 2: level = 16 x log2(power), so one step is 0.19 dB ---------------
    // Stage 1 finds the highest byte that is not zero; stage 2 finds the top '1' inside
    // it. The position of that '1' is the whole part of the logarithm and the four bits
    // below it are the fraction.
    reg         v1 = 1'b0, l1 = 1'b0, c1 = 1'b0;
    reg  [10:0] a1 = 11'd0;
    reg  [2:0]  by1 = 3'd0;
    reg  [11:0] w1 = 12'd0;               // that byte, and the four bits below it

    always @(posedge aclk) begin
        v1 <= v0;  l1 <= l0;  c1 <= c0;  a1 <= a0;
        if      (|d0[63:56]) begin by1 <= 3'd7; w1 <= d0[63:52]; end
        else if (|d0[55:48]) begin by1 <= 3'd6; w1 <= d0[55:44]; end
        else if (|d0[47:40]) begin by1 <= 3'd5; w1 <= d0[47:36]; end
        else if (|d0[39:32]) begin by1 <= 3'd4; w1 <= d0[39:28]; end
        else if (|d0[31:24]) begin by1 <= 3'd3; w1 <= d0[31:20]; end
        else if (|d0[23:16]) begin by1 <= 3'd2; w1 <= d0[23:12]; end
        else if (|d0[15:8])  begin by1 <= 3'd1; w1 <= d0[15:4];  end
        else                 begin by1 <= 3'd0; w1 <= {d0[7:0], 4'd0}; end
    end

    reg         v2 = 1'b0, l2 = 1'b0, c2 = 1'b0;
    reg  [10:0] a2 = 11'd0;
    reg  [9:0]  lv2 = 10'd0;

    always @(posedge aclk) begin
        v2 <= v1;  l2 <= l1;  c2 <= c1;  a2 <= a1;
        if      (w1[11]) lv2 <= {by1, 3'd7, w1[10:7]};
        else if (w1[10]) lv2 <= {by1, 3'd6, w1[9:6]};
        else if (w1[9])  lv2 <= {by1, 3'd5, w1[8:5]};
        else if (w1[8])  lv2 <= {by1, 3'd4, w1[7:4]};
        else if (w1[7])  lv2 <= {by1, 3'd3, w1[6:3]};
        else if (w1[6])  lv2 <= {by1, 3'd2, w1[5:2]};
        else if (w1[5])  lv2 <= {by1, 3'd1, w1[4:1]};
        else if (w1[4])  lv2 <= {by1, 3'd0, w1[3:0]};
        else             lv2 <= 10'd0;    // the power was zero
    end

    // ---- stage 3: two bins make one pixel; the stronger of the two is kept --------------
    reg  [9:0]  ev    = 10'd0;            // level of the even bin, waiting for its odd partner
    reg         ev_ok = 1'b0;
    reg         pv = 1'b0, pl = 1'b0;     // a finished pixel: valid, last of its line
    reg         pc = 1'b0;                //                   its line only sets the floor
    reg  [9:0]  px = 10'd0, plv = 10'd0;  //                   column, level

    always @(posedge aclk) begin
        pv  <= v2 && a2[0];
        pl  <= l2;
        pc  <= c2;
        px  <= a2[10:1];
        plv <= (ev_ok && (ev > lv2)) ? ev : lv2;
        if (v2) begin
            if (!a2[0]) begin ev <= lv2; ev_ok <= 1'b1; end
            else        ev_ok <= 1'b0;
        end
    end

    // ---- stages 4 to 6: find the noise floor --------------------------------------------
    // Each line is cut into sixteen 64-pixel pieces and the piece with the lowest average
    // is taken as the noise floor (kept as a sum of 64 levels, i.e. with 6 fraction bits).
    // The floor then follows that value slowly, or jumps to it if it is 6 dB or more away.
    reg  [15:0] seg = 16'd0;              // running sum of the current piece
    wire [15:0] seg_n = seg + {6'd0, plv};
    reg         bv = 1'b0, bl = 1'b0, bc = 1'b0, bend = 1'b0;
    reg  [15:0] bsum = 16'd0;
    reg  [9:0]  bx = 10'd0, blv = 10'd0;

    always @(posedge aclk) begin
        bv <= pv;  bl <= pl;  bc <= pc;  bx <= px;  blv <= plv;
        bend <= (px[5:0] == 6'd63);
        bsum <= seg_n;
        if (pv) seg <= ((px[5:0] == 6'd63) || pl) ? 16'd0 : seg_n;
    end

    reg  [15:0] lmin = 16'hFFFF;          // lowest piece so far in this line (FFFF = none yet)
    wire [15:0] lmin_n = (bend && (bsum < lmin)) ? bsum : lmin;
    reg         cv = 1'b0, cl = 1'b0, cc = 1'b0, cup = 1'b0;
    reg  [15:0] cmin = 16'd0;
    reg  [9:0]  cx = 10'd0, clv = 10'd0;

    always @(posedge aclk) begin
        cv <= bv;  cl <= bl;  cc <= bc;  cx <= bx;  clv <= blv;
        cup  <= bv && bl && (lmin_n != 16'hFFFF);
        cmin <= lmin_n;
        if (bv) lmin <= bl ? 16'hFFFF : lmin_n;
    end

    reg  [15:0] flr = 16'd0;              // the noise floor
    reg  [9:0]  dsp = 10'd0;              // disp_i[15:6], taken over between lines only
    wire signed [16:0] fdiff = $signed({1'b0, cmin}) - $signed({1'b0, flr});
    wire               ffar  = (fdiff >= 17'sd2048) || (fdiff <= -17'sd2048);
    wire        [15:0] fnext = flr + {{2{fdiff[16]}}, fdiff[16:3]};      // flr + fdiff / 8

    // ---- stages 6 to 8: level above the floor -> colour number 0..255 -------------------
    reg         dv = 1'b0, dl = 1'b0, den = 1'b0;
    reg  [9:0]  dx = 10'd0;
    reg  signed [12:0] dd = 13'sd0;       // level above the floor (can be negative)
    reg  [1:0]  dcon = 2'd0;
    reg  [7:0]  dbri = 8'd0;

    always @(posedge aclk) begin
        dv <= cv;  dl <= cl;  dx <= cx;
        den  <= !cc;                      // a line that only sets the floor is not stored
        dd   <= $signed({3'b000, clv}) - $signed({3'b000, flr[15:6]});
        dcon <= dsp[1:0];
        dbri <= dsp[9:2];
        if (cup) begin
            flr <= (cc || ffar) ? cmin : fnext;
            dsp <= disp_i[15:6];
        end
    end

    // contrast: 0 = 48 dB across the colour scale, 1 = 64 dB, 2 = 96 dB, 3 = 24 dB
    wire signed [12:0] dd_q = {{2{dd[12]}}, dd[12:2]};        // dd / 4
    wire signed [12:0] dd_h = {dd[12], dd[12:1]};             // dd / 2
    wire signed [12:0] dd_2 = {dd[11:0], 1'b0};               // dd x 2
    wire signed [12:0] dsc  = (dcon == 2'd0) ? dd :
                              (dcon == 2'd1) ? dd - dd_q :
                              (dcon == 2'd2) ? dd_h : dd_2;
    wire signed [12:0] dbr  = {{5{dbri[7]}}, dbri};

    reg         gv = 1'b0, gl = 1'b0, gen = 1'b0;
    reg  [9:0]  gx = 10'd0;
    reg  signed [12:0] gs = 13'sd0;

    always @(posedge aclk) begin
        gv <= dv;  gl <= dl;  gx <= dx;  gen <= den;
        gs <= dsc + 13'sd24 + dbr;        // the floor itself lands on colour 24
    end

    wire [7:0] gidx = gs[12] ? 8'd0 : (|gs[11:8]) ? 8'd255 : gs[7:0];

    reg  [LW-1:0] wline = {LW{1'b0}};     // the line being filled
    reg  [LW-1:0] wgray = {LW{1'b0}};     // the same number in Gray code, for the video side
    reg           we = 1'b0;
    reg  [AW-1:0] wa = {AW{1'b0}};
    reg  [7:0]    wd = 8'd0;

    always @(posedge aclk) begin
        we <= gv && gen;
        wa <= {wline, gx};
        wd <= gidx;
        if (gv && gen && gl) wline <= wline + 1'b1;
        wgray <= wline ^ (wline >> 1);
    end

    always @(posedge aclk) begin
        if (we) ram[wa] <= wd;            // the block RAM's write port
    end

    always @(posedge aclk) begin
        if (gv && gen && gl) tagram[wline] <= tag_ln;         // the tag RAM's write port
    end

    // =====================================================================================
    //  pix_clk side: 720p timing and the picture
    // =====================================================================================
    localparam H_ACTIVE = 1280;
    localparam H_FP     = 110;
    localparam H_SYNC   = 40;
    localparam H_BP     = 220;
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

    localparam [10:0] PANE_X   = 11'd128;     // the pane is 1024 pixels wide
    localparam [10:0] PANE_END = 11'd1152;
    localparam [9:0]  TR_Y     = 10'd8;       // spectrum trace, 128 rows
    localparam [9:0]  RL_Y     = 10'd136;     // ruler, 32 rows
    localparam [9:0]  WF_Y     = 10'd168;     // waterfall, 512 rows
    localparam [9:0]  WF_END   = 10'd680;
    localparam [9:0]  KEY_Y    = 10'd692;     // colour key, 12 rows
    localparam [9:0]  KEY_END  = 10'd704;
    localparam integer SH      = 9 - LOG2_LINES;   // each stored line is 2^SH rows tall

    // ---- reset: held until the Clocking Wizard reports lock ---------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] lock_ff = 2'b00;
    always @(posedge pix_clk or negedge locked) begin
        if (!locked) lock_ff <= 2'b00;
        else         lock_ff <= {lock_ff[0], 1'b1};
    end
    wire rst = ~lock_ff[1];

    // ---- pixel counters, exactly as in hdmi_bars ---------------------------------------
    reg [10:0] hcnt = 11'd0;              // 0 .. 1649
    reg [9:0]  vcnt = 10'd0;              // 0 .. 749
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

    wire active = (hcnt < H_ACTIVE) && (vcnt < V_ACTIVE);
    wire hs     = (hcnt >= HS_START) && (hcnt < HS_END);
    wire vs     = ((vcnt == VS_START) && (hcnt >= HS_START)) ||
                  ((vcnt >  VS_START) && (vcnt <  VS_END))   ||
                  ((vcnt == VS_END)   && (hcnt <  HS_START));

    // ---- values that cross over from the aclk side --------------------------------------
    // Two flip-flops each. The line pointer travels in Gray code. The tuning word and the
    // status byte are taken only when two samples in a row agree.
    (* ASYNC_REG = "TRUE" *) reg [LW-1:0] wg_m = {LW{1'b0}}, wg_s = {LW{1'b0}};
    (* ASYNC_REG = "TRUE" *) reg [31:0]   tw_m = 32'd0,      tw_s = 32'd0;
    (* ASYNC_REG = "TRUE" *) reg [7:0]    st_m = 8'h08,      st_s = 8'h08;
    (* ASYNC_REG = "TRUE" *) reg          vw_m = 1'b0,       vw_s = 1'b0;
    reg  [31:0]   tw_p = 32'd0;
    reg  [7:0]    st_p = 8'h08;
    wire [LW-1:0] wbin;                   // Gray code back to a plain number
    genvar gi;
    generate
        for (gi = 0; gi < LW; gi = gi + 1) begin : g2b
            assign wbin[gi] = ^(wg_s >> gi);
        end
    endgenerate

    // ---- once a picture, while nothing is being drawn: the numbers the text and the local
    //      ruler need.  Everything here is worked out one small step a clock. -------------
    //   n_fd  the tuned frequency in Hz as eight decimal digits: W x 62.5 MHz / 2^32, rounded
    //   n_ga, n_gn   where the grid of whole 10 kHz stands at the left edge of the pane.
    //         A pixel column is 2^13 of tuning word wide and 10 kHz is 2^32 / 6250 of it, so
    //         (tuning word of a column's left edge) x 6250 steps over a multiple of 2^32
    //         exactly where a tick belongs.  n_gn is the number of the next tick (tick n
    //         stands on n x 10 kHz); it is negative while the pane starts below 0 Hz.
    //   n_b0  that number as four decimal digits (0 while it is negative)
    localparam [20:0] K_HZ   = 21'd1953125;       // 62.5 MHz / 2^5; the product is shifted by 27
    localparam [12:0] K_GRID = 13'd6250;
    reg  [31:0] w_n   = 32'd0;            // tuning word for the next picture
    reg  [7:0]  st_n  = 8'h08;            // status byte for the next picture
    reg  [6:0]  cstep = 7'd127;           // 127 = idle
    reg  [52:0] cacc  = 53'd0;
    reg  [25:0] cbin  = 26'd0;
    reg  [31:0] n_fd  = 32'd0;
    reg  signed [47:0] gacc = 48'sd0;
    reg  [31:0] n_ga  = 32'd0;
    reg  signed [13:0] n_gn = 14'sd0;
    reg  [12:0] nbin  = 13'd0;
    reg  [15:0] n_b0  = 16'd0;

    function [3:0] add3;                  // one digit of the binary to decimal conversion
        input [3:0] d;
        begin
            add3 = (d >= 4'd5) ? (d + 4'd3) : d;
        end
    endfunction

    wire        vb_start = (vcnt == V_ACTIVE) && (hcnt == 11'd0);
    wire [52:0] w_ext  = {21'd0, w_n};
    wire signed [47:0] ledge = $signed({16'd0, w_n}) - 48'sd4196352;      // left edge of column 0, less half a bin
    wire [52:0] crnd   = cacc + 53'd67108864;                          // + 2^26: round to the nearest Hz
    wire [31:0] fd_a   = {add3(n_fd[31:28]), add3(n_fd[27:24]), add3(n_fd[23:20]), add3(n_fd[19:16]),
                          add3(n_fd[15:12]), add3(n_fd[11:8]),  add3(n_fd[7:4]),   add3(n_fd[3:0])};
    wire [15:0] b0_a   = {add3(n_b0[15:12]), add3(n_b0[11:8]), add3(n_b0[7:4]), add3(n_b0[3:0])};
    wire signed [13:0] gn_x = gacc[45:32];
    wire [6:0]  ci     = cstep - 7'd48;

    always @(posedge pix_clk) begin
        if (vb_start) begin
            if (tw_s == tw_p) w_n  <= tw_s;
            if (st_s == st_p) st_n <= st_s;
            cstep <= 7'd0;
            cacc  <= 53'd0;
            gacc  <= 48'sd0;
        end else if (cstep != 7'd127) begin
            cstep <= cstep + 7'd1;
            if (cstep <= 7'd20) begin                                  // W x 1953125, a bit a clock
                if (K_HZ[cstep[4:0]]) cacc <= cacc + (w_ext << cstep[4:0]);
            end else if (cstep == 7'd21) begin
                cbin <= crnd[52:27];
                n_fd <= 32'd0;
            end else if (cstep <= 7'd47) begin                         // 26 bits into 8 decimal digits
                n_fd <= {fd_a[30:0], cbin[25]};
                cbin <= {cbin[24:0], 1'b0};
            end else if (cstep <= 7'd60) begin                         // x 6250, a bit a clock
                if (K_GRID[ci[3:0]]) gacc <= gacc + (ledge <<< ci[3:0]);
            end else if (cstep == 7'd61) begin
                gacc <= gacc + 48'sd4294967295;
            end else if (cstep == 7'd62) begin
                n_ga <= gacc[31:0];
                n_gn <= gn_x;
                nbin <= gn_x[13] ? 13'd0 : gn_x[12:0];                 // the first tick drawn is never below 0
                n_b0 <= 16'd0;
            end else if (cstep <= 7'd75) begin                         // 13 bits into 4 decimal digits
                n_b0 <= {b0_a[14:0], nbin[12]};
                nbin <= {nbin[11:0], 1'b0};
            end else begin
                cstep <= 7'd127;
            end
        end
    end

    wire [13:0] tsum  = {1'b0, w_n[31:19]} + 14'd1;        // wide view: column = (word + half a bin) / 2^21
    wire [10:0] mk_w  = {(w_n[31:19] != 13'd0) && (tsum[13:12] == 2'b00), tsum[11:2]};
    wire [31:0] an32  = w_n + 32'd4096;                    // centre of the local view: (word + half a bin) / 2^13

    reg [LW-1:0] top   = {LW{1'b0}};      // the line being filled when this picture began
    reg [10:0]   mk    = 11'd0;           // marker for this picture: {show it, column}
    reg          pview = 1'b0;            // view of this picture: 0 wide, 1 local
    reg [LW:0]   cnt   = {(LW+1){1'b0}};  // lines stored since the view last changed (stops at 2^LW)
    reg [31:0]   fw    = 32'd0;           // tuning word of this picture
    reg [7:0]    fst   = 8'h08;           // status byte of this picture
    reg [18:0]   anow  = 19'd0;           // centre of the local view, counted in pixel columns
    reg [31:0]   fd    = 32'd0;           // tuned frequency in Hz, eight decimal digits
    reg [31:0]   ga0   = 32'd0;           // the 10 kHz grid at the left edge of the pane
    reg signed [13:0] gn0 = 14'sd0;
    reg [15:0]   b0    = 16'd0;

    wire [LW-1:0] dlt  = wbin - top;      // lines stored during the picture just finished
    wire [LW+1:0] csum = {1'b0, cnt} + {2'b00, dlt};
    wire          cfull = csum[LW+1] || (csum[LW] && (|csum[LW-1:0]));    // csum > 2^LW

    always @(posedge pix_clk) begin
        wg_m <= wgray;   wg_s <= wg_m;
        tw_m <= tune_i;  tw_s <= tw_m;  tw_p <= tw_s;
        st_m <= stat_i;  st_s <= st_m;  st_p <= st_s;
        vw_m <= view;    vw_s <= vw_m;
        if (h_last && v_last) begin
            top   <= wbin;
            pview <= vw_s;
            if (vw_s != pview) cnt <= {(LW+1){1'b0}};               // new view: start from empty
            else               cnt <= cfull ? {1'b1, {LW{1'b0}}} : csum[LW:0];
            mk    <= vw_s ? 11'h600 : mk_w;                         // local: the centre column, 512
            fw    <= w_n;
            fst   <= st_n;
            anow  <= an32[31:13];
            fd    <= n_fd;
            ga0   <= n_ga;
            gn0   <= n_gn;
            b0    <= n_b0;
        end
    end

    // ---- once a line: which stored line is shown here, and how far has it moved? --------
    // A line remembers where the local view was centred when it was taken (its tag). If the
    // receiver has been retuned since, the line is drawn that many columns to the side.
    wire [10:0]   pxw    = hcnt - PANE_X;
    wire [9:0]    wy     = vcnt - WF_Y;
    wire [LW-1:0] wrow   = wy[8:SH];                          // stored line, counted from the top
    wire [LW-1:0] newest = top - 1'b1;
    wire [LW-1:0] last_row = {{(LW-2){1'b1}}, 2'b01};         // LINES - 3: the two oldest lines are not shown
    wire [LW-1:0] rline  = (vcnt >= WF_Y) ? (newest - wrow) : newest;
    wire [9:0]    trw_   = (RL_Y - 10'd1) - vcnt;             // trace: rows up from its base line
    wire [9:0]    rly    = vcnt - RL_Y;                       // ruler: rows down from its top
    reg  [18:0]   tag_q  = 19'd0;
    wire [18:0]   dsub   = anow - tag_q;                      // the tuning word goes round in 2^19 columns
    reg  signed [18:0] shf = 19'sd0;
    always @(posedge pix_clk) begin
        tag_q <= tagram[rline];                               // the tag RAM's read port
        if (hcnt == 11'd8) shf <= pview ? $signed(dsub) : 19'sd0;
    end
    wire signed [19:0] cs    = $signed({9'd0, pxw}) + $signed({shf[18], shf});   // the column to fetch
    wire               cs_ok = (cs[19:10] == 10'd0);          // it exists

    wire [10:0] mkx   = {1'b0, mk[9:0]} + PANE_X;             // marker column on the screen
    wire [11:0] mdiff = {1'b0, hcnt} - {1'b0, mkx};
    wire [11:0] mabs  = mdiff[11] ? (~mdiff + 12'd1) : mdiff;
    wire signed [11:0] mds = $signed(mdiff);                  // pixels to the right of the marker
    // The audio filter's pass band as it lies on the dial, in pixels of 119.2 Hz counted from
    // the marker: first and last column.  AM, SAM, CW and FM hear both sides of the tuned
    // frequency; USB and DIG hear what lies above it (to the right), LSB what lies below it.
    reg signed [11:0] pb_lo, pb_hi;
    always @* begin
        case ({fst[2:0], fst[4:3]})                           // {mode, width}
            {3'd0, 2'd0}: begin pb_lo = -12'sd13; pb_hi = 12'sd13; end          // AM  3 kHz    -1500 ..  +1500 Hz
            {3'd0, 2'd1}: begin pb_lo = -12'sd25; pb_hi = 12'sd25; end          // AM  6 kHz    -3000 ..  +3000 Hz
            {3'd0, 2'd2}: begin pb_lo = -12'sd38; pb_hi = 12'sd38; end          // AM  9 kHz    -4500 ..  +4500 Hz
            {3'd0, 2'd3}: begin pb_lo = -12'sd50; pb_hi = 12'sd50; end          // AM  12 kHz   -6000 ..  +6000 Hz
            {3'd1, 2'd0}: begin pb_lo = -12'sd13; pb_hi = 12'sd13; end          // SAM 3 kHz    -1500 ..  +1500 Hz
            {3'd1, 2'd1}: begin pb_lo = -12'sd25; pb_hi = 12'sd25; end          // SAM 6 kHz    -3000 ..  +3000 Hz
            {3'd1, 2'd2}: begin pb_lo = -12'sd38; pb_hi = 12'sd38; end          // SAM 9 kHz    -4500 ..  +4500 Hz
            {3'd1, 2'd3}: begin pb_lo = -12'sd50; pb_hi = 12'sd50; end          // SAM 12 kHz   -6000 ..  +6000 Hz
            {3'd2, 2'd0}: begin pb_lo = 12'sd1; pb_hi = 12'sd16; end            // USB 1.8 kHz   +150 ..  +1950 Hz
            {3'd2, 2'd1}: begin pb_lo = 12'sd1; pb_hi = 12'sd21; end            // USB 2.4 kHz   +150 ..  +2550 Hz
            {3'd2, 2'd2}: begin pb_lo = 12'sd1; pb_hi = 12'sd26; end            // USB 3 kHz     +150 ..  +3150 Hz
            {3'd2, 2'd3}: begin pb_lo = 12'sd1; pb_hi = 12'sd35; end            // USB 4 kHz     +150 ..  +4150 Hz
            {3'd3, 2'd0}: begin pb_lo = -12'sd16; pb_hi = -12'sd1; end          // LSB 1.8 kHz  -1950 ..   -150 Hz
            {3'd3, 2'd1}: begin pb_lo = -12'sd21; pb_hi = -12'sd1; end          // LSB 2.4 kHz  -2550 ..   -150 Hz
            {3'd3, 2'd2}: begin pb_lo = -12'sd26; pb_hi = -12'sd1; end          // LSB 3 kHz    -3150 ..   -150 Hz
            {3'd3, 2'd3}: begin pb_lo = -12'sd35; pb_hi = -12'sd1; end          // LSB 4 kHz    -4150 ..   -150 Hz
            {3'd4, 2'd0}: begin pb_lo = -12'sd1; pb_hi = 12'sd1; end            // CW  250 Hz    -125 ..   +125 Hz
            {3'd4, 2'd1}: begin pb_lo = -12'sd2; pb_hi = 12'sd2; end            // CW  500 Hz    -250 ..   +250 Hz
            {3'd4, 2'd2}: begin pb_lo = -12'sd3; pb_hi = 12'sd3; end            // CW  800 Hz    -400 ..   +400 Hz
            {3'd4, 2'd3}: begin pb_lo = -12'sd5; pb_hi = 12'sd5; end            // CW  1.2 kHz   -600 ..   +600 Hz
            {3'd5, 2'd0}: begin pb_lo = -12'sd38; pb_hi = 12'sd38; end          // FM  9 kHz    -4500 ..  +4500 Hz
            {3'd5, 2'd1}: begin pb_lo = -12'sd50; pb_hi = 12'sd50; end          // FM  12 kHz   -6000 ..  +6000 Hz
            {3'd5, 2'd2}: begin pb_lo = -12'sd67; pb_hi = 12'sd67; end          // FM  16 kHz   -8000 ..  +8000 Hz
            {3'd5, 2'd3}: begin pb_lo = -12'sd84; pb_hi = 12'sd84; end          // FM  20 kHz  -10000 .. +10000 Hz
            {3'd6, 2'd0}: begin pb_lo = 12'sd10; pb_hi = 12'sd15; end           // DIG 500 Hz   +1250 ..  +1750 Hz
            {3'd6, 2'd1}: begin pb_lo = 12'sd8; pb_hi = 12'sd18; end            // DIG 1.2 kHz   +900 ..  +2100 Hz
            {3'd6, 2'd2}: begin pb_lo = 12'sd1; pb_hi = 12'sd26; end            // DIG 3 kHz     +150 ..  +3150 Hz
            {3'd6, 2'd3}: begin pb_lo = 12'sd1; pb_hi = 12'sd35; end            // DIG 4 kHz     +150 ..  +4150 Hz
            default: begin pb_lo = 12'sd0; pb_hi = 12'sd0; end
        endcase
    end

    // which line of text, if any, this scan line belongs to, and which of its 14 rows
    reg  [2:0] tsel;
    reg  [9:0] trv;
    always @* begin
        tsel = 3'd0;
        trv  = 10'd0;
        if (vcnt >= 10'd146 && vcnt < 10'd160) begin
            tsel = 3'd1;  trv = vcnt - 10'd146;               // ruler labels
        end else if (vcnt >= 10'd12 && vcnt < 10'd26) begin
            tsel = 3'd2;  trv = vcnt - 10'd12;                // tuned frequency
        end else if (vcnt >= 10'd30 && vcnt < 10'd44) begin
            tsel = 3'd3;  trv = vcnt - 10'd30;                // "kHz"
        end else if (vcnt >= 10'd56 && vcnt < 10'd70) begin
            tsel = 3'd4;  trv = vcnt - 10'd56;                // mode
        end else if (vcnt >= 10'd80 && vcnt < 10'd94) begin
            tsel = 3'd5;  trv = vcnt - 10'd80;                // filter width
        end
    end

    // ---- stage 1: where is this pixel, and which RAM byte does it need? -----------------
    reg [AW-1:0] ra = {AW{1'b0}};
    reg [9:0]    s1_px = 10'd0;
    reg [6:0]    s1_hb = 7'd0;
    reg [4:0]    s1_ry = 5'd0;
    reg [2:0]    s1_md = 3'd7;            // pixels from the marker column, 7 = far away
    reg [2:0]    s1_tsel = 3'd0;
    reg [3:0]    s1_trw = 4'd0;
    reg          s1_pane = 1'b0, s1_tr = 1'b0, s1_rl = 1'b0, s1_wf = 1'b0, s1_key = 1'b0;
    reg          s1_ok = 1'b0, s1_pb = 1'b0;

    always @(posedge pix_clk) begin
        ra      <= {rline, cs[9:0]};
        s1_px   <= pxw[9:0];
        s1_hb   <= trw_[6:0];
        s1_ry   <= rly[4:0];
        s1_md   <= (mabs > 12'd6) ? 3'd7 : mabs[2:0];
        s1_tsel <= tsel;
        s1_trw  <= trv[3:0];
        s1_pane <= (hcnt >= PANE_X) && (hcnt < PANE_END);
        s1_tr   <= (vcnt >= TR_Y) && (vcnt < RL_Y) && (cnt != {(LW+1){1'b0}});
        s1_rl   <= (vcnt >= RL_Y) && (vcnt < WF_Y);
        s1_wf   <= (vcnt >= WF_Y) && (vcnt < WF_END) && (wrow <= last_row) && ({1'b0, wrow} < cnt) && cs_ok;
        s1_key  <= (vcnt >= KEY_Y) && (vcnt < KEY_END);
        s1_ok   <= cs_ok;
        s1_pb   <= pview && (mds >= pb_lo) && (mds <= pb_hi);
    end

    // ---- text: characters of 5 x 7 dots, each dot 2 x 2 pixels, so 10 pixels wide --------
    localparam [4:0] G_M = 5'd10, G_H = 5'd11, G_Z = 5'd12, G_K = 5'd13, G_DOT = 5'd14, G_A = 5'd15,
                     G_S = 5'd16, G_U = 5'd17, G_L = 5'd18, G_B = 5'd19, G_C = 5'd20, G_W = 5'd21,
                     G_F = 5'd22, G_D = 5'd23, G_I = 5'd24, G_G = 5'd25, G_SP = 5'd31;

    // wide ruler: fixed places
    reg       c_hit;
    reg [4:0] c_g;
    always @* begin
        c_hit = 1'b1;
        case (hcnt)
            11'd123:  c_g = 5'd0;         // "0"   under the   0 MHz tick
            11'd287:  c_g = 5'd5;         // "5"               5 MHz
            11'd444:  c_g = 5'd1;         // "10"             10 MHz
            11'd456:  c_g = 5'd0;
            11'd608:  c_g = 5'd1;         // "15"
            11'd620:  c_g = 5'd5;
            11'd772:  c_g = 5'd2;         // "20"
            11'd784:  c_g = 5'd0;
            11'd936:  c_g = 5'd2;         // "25"
            11'd948:  c_g = 5'd5;
            11'd1100: c_g = 5'd3;         // "30"
            11'd1112: c_g = 5'd0;
            11'd1164: c_g = G_M;          // "MHz" to the right of the pane
            11'd1176: c_g = G_H;
            11'd1188: c_g = G_Z;
            default: begin c_hit = 1'b0; c_g = 5'd0; end
        endcase
    end

    // local ruler: "kHz" to the right of the pane
    reg       u_hit;
    reg [4:0] u_g;
    always @* begin
        u_hit = 1'b1;
        case (hcnt)
            11'd1192: u_g = G_K;
            11'd1204: u_g = G_H;
            11'd1216: u_g = G_Z;
            default: begin u_hit = 1'b0; u_g = 5'd0; end
        endcase
    end

    // local ruler: the grid of whole 10 kHz, worked out 32 pixels ahead of the beam so that a
    // label can start before its tick.  ga + 51200000 overflows where a tick belongs.
    reg  [31:0] ga   = 32'd0;
    reg  signed [13:0] gn = 14'sd0;       // number of the next tick
    reg  [15:0] bcd  = 16'd0;             // the same as four decimal digits
    wire        la_run = (hcnt >= 11'd96) && (hcnt < 11'd1120);
    wire [32:0] gsum   = {1'b0, ga} + 33'd51200000;
    wire        t_ok   = !gn[13];                             // ticks below 0 Hz are not drawn
    wire        t_new  = la_run && gsum[32] && t_ok;
    wire        t_maj  = t_new && !gn[0];                     // every second one gets a number
    wire [3:0]  bi0    = (bcd[3:0]   == 4'd9) ? 4'd0 : bcd[3:0] + 4'd1;
    wire        bc1    = (bcd[3:0]   == 4'd9);
    wire [3:0]  bi1    = !bc1 ? bcd[7:4]   : (bcd[7:4]   == 4'd9) ? 4'd0 : bcd[7:4] + 4'd1;
    wire        bc2    = bc1 && (bcd[7:4]  == 4'd9);
    wire [3:0]  bi2    = !bc2 ? bcd[11:8]  : (bcd[11:8]  == 4'd9) ? 4'd0 : bcd[11:8] + 4'd1;
    wire        bc3    = bc2 && (bcd[11:8] == 4'd9);
    wire [3:0]  bi3    = !bc3 ? bcd[15:12] : (bcd[15:12] == 4'd9) ? 4'd0 : bcd[15:12] + 4'd1;
    reg         tk_now = 1'b0, mj_now = 1'b0;
    reg  [31:0] tk_sr = 32'd0, mj_sr = 32'd0;                 // 32 clocks later the beam is there
    // the label under a tick: the tick number and a zero, i.e. kHz, without leading zeros
    reg  [19:0] lab_s   = 20'd0;          // five characters
    reg  [2:0]  lab_i   = 3'd0;           // the one to draw next
    reg  [4:0]  lab_w   = 5'd0;           // clocks until it starts
    reg         lab_on  = 1'b0;
    wire [2:0]  lab_len = (bcd[15:12] != 4'd0) ? 3'd5 : (bcd[11:8] != 4'd0) ? 3'd4 :
                          (bcd[7:4]   != 4'd0) ? 3'd3 : (bcd[3:0]  != 4'd0) ? 3'd2 : 3'd1;
    wire [2:0]  lab_sk  = 3'd5 - lab_len;                     // places not needed, 0..4
    wire [4:0]  lab_w0  = 5'd2 + {lab_sk, 2'b00} + {1'b0, lab_sk, 1'b0};      // 2 + 6 x that
    wire        lab_hit = lab_on && (lab_w == 5'd0);
    wire [3:0]  lab_d   = (lab_i == 3'd0) ? lab_s[19:16] : (lab_i == 3'd1) ? lab_s[15:12] :
                          (lab_i == 3'd2) ? lab_s[11:8]  : (lab_i == 3'd3) ? lab_s[7:4]   : lab_s[3:0];

    always @(posedge pix_clk) begin
        if (hcnt == 11'd95) begin
            ga  <= ga0;
            gn  <= gn0;
            bcd <= b0;
        end else if (la_run) begin
            ga <= gsum[31:0];
            if (gsum[32]) begin
                gn <= gn + 14'sd1;
                if (t_ok) bcd <= {bi3, bi2, bi1, bi0};
            end
        end
        tk_now <= t_new;
        mj_now <= t_maj;
        tk_sr  <= {tk_sr[30:0], tk_now};
        mj_sr  <= {mj_sr[30:0], mj_now};
        // the label is centred under its tick: it starts 6 x length - 1 pixels to the left
        if (hcnt == 11'd95) begin
            lab_on <= 1'b0;
        end else if (t_maj) begin
            lab_s  <= {bcd, 4'd0};
            lab_i  <= lab_sk;
            lab_w  <= lab_w0;
            lab_on <= 1'b1;
        end else if (lab_on) begin
            if (lab_w != 5'd0) begin
                lab_w <= lab_w - 5'd1;
            end else if (lab_i == 3'd4) begin
                lab_on <= 1'b0;
            end else begin
                lab_i <= lab_i + 3'd1;
                lab_w <= 5'd11;
            end
        end
    end
    wire ltick = tk_sr[31];
    wire lmaj  = mj_sr[31];

    // status text in the left margin: nine places 12 pixels apart, the first at x = 8
    reg       sc_hit;
    reg [3:0] sc_i;
    always @* begin
        sc_hit = 1'b1;
        case (hcnt)
            11'd8:   sc_i = 4'd0;
            11'd20:  sc_i = 4'd1;
            11'd32:  sc_i = 4'd2;
            11'd44:  sc_i = 4'd3;
            11'd56:  sc_i = 4'd4;
            11'd68:  sc_i = 4'd5;
            11'd80:  sc_i = 4'd6;
            11'd92:  sc_i = 4'd7;
            11'd104: sc_i = 4'd8;
            default: begin sc_hit = 1'b0; sc_i = 4'd0; end
        endcase
    end
    // the mode as three characters and the filter width as seven, most significant first
    reg [14:0] mtxt;
    always @* begin
        case (fst[2:0])
            3'd0: mtxt = {G_A, G_M, G_SP};      // AM
            3'd1: mtxt = {G_S, G_A, G_M};       // SAM
            3'd2: mtxt = {G_U, G_S, G_B};       // USB
            3'd3: mtxt = {G_L, G_S, G_B};       // LSB
            3'd4: mtxt = {G_C, G_W, G_SP};      // CW
            3'd5: mtxt = {G_F, G_M, G_SP};      // FM
            3'd6: mtxt = {G_D, G_I, G_G};       // DIG
            default: mtxt = {3{G_SP}};
        endcase
    end
    reg [34:0] wtxt;
    always @* begin
        case ({fst[2:0], fst[4:3]})                           // {mode, width}
            {3'd0, 2'd0}: wtxt = {5'd3, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // AM  3 kHz
            {3'd0, 2'd1}: wtxt = {5'd6, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // AM  6 kHz
            {3'd0, 2'd2}: wtxt = {5'd9, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // AM  9 kHz
            {3'd0, 2'd3}: wtxt = {5'd1, 5'd2, G_SP, G_K, G_H, G_Z, G_SP};               // AM  12 kHz
            {3'd1, 2'd0}: wtxt = {5'd3, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // SAM 3 kHz
            {3'd1, 2'd1}: wtxt = {5'd6, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // SAM 6 kHz
            {3'd1, 2'd2}: wtxt = {5'd9, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // SAM 9 kHz
            {3'd1, 2'd3}: wtxt = {5'd1, 5'd2, G_SP, G_K, G_H, G_Z, G_SP};               // SAM 12 kHz
            {3'd2, 2'd0}: wtxt = {5'd1, G_DOT, 5'd8, G_SP, G_K, G_H, G_Z};              // USB 1.8 kHz
            {3'd2, 2'd1}: wtxt = {5'd2, G_DOT, 5'd4, G_SP, G_K, G_H, G_Z};              // USB 2.4 kHz
            {3'd2, 2'd2}: wtxt = {5'd3, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // USB 3 kHz
            {3'd2, 2'd3}: wtxt = {5'd4, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // USB 4 kHz
            {3'd3, 2'd0}: wtxt = {5'd1, G_DOT, 5'd8, G_SP, G_K, G_H, G_Z};              // LSB 1.8 kHz
            {3'd3, 2'd1}: wtxt = {5'd2, G_DOT, 5'd4, G_SP, G_K, G_H, G_Z};              // LSB 2.4 kHz
            {3'd3, 2'd2}: wtxt = {5'd3, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // LSB 3 kHz
            {3'd3, 2'd3}: wtxt = {5'd4, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // LSB 4 kHz
            {3'd4, 2'd0}: wtxt = {5'd2, 5'd5, 5'd0, G_SP, G_H, G_Z, G_SP};              // CW  250 Hz
            {3'd4, 2'd1}: wtxt = {5'd5, 5'd0, 5'd0, G_SP, G_H, G_Z, G_SP};              // CW  500 Hz
            {3'd4, 2'd2}: wtxt = {5'd8, 5'd0, 5'd0, G_SP, G_H, G_Z, G_SP};              // CW  800 Hz
            {3'd4, 2'd3}: wtxt = {5'd1, G_DOT, 5'd2, G_SP, G_K, G_H, G_Z};              // CW  1.2 kHz
            {3'd5, 2'd0}: wtxt = {5'd9, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // FM  9 kHz
            {3'd5, 2'd1}: wtxt = {5'd1, 5'd2, G_SP, G_K, G_H, G_Z, G_SP};               // FM  12 kHz
            {3'd5, 2'd2}: wtxt = {5'd1, 5'd6, G_SP, G_K, G_H, G_Z, G_SP};               // FM  16 kHz
            {3'd5, 2'd3}: wtxt = {5'd2, 5'd0, G_SP, G_K, G_H, G_Z, G_SP};               // FM  20 kHz
            {3'd6, 2'd0}: wtxt = {5'd5, 5'd0, 5'd0, G_SP, G_H, G_Z, G_SP};              // DIG 500 Hz
            {3'd6, 2'd1}: wtxt = {5'd1, G_DOT, 5'd2, G_SP, G_K, G_H, G_Z};              // DIG 1.2 kHz
            {3'd6, 2'd2}: wtxt = {5'd3, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // DIG 3 kHz
            {3'd6, 2'd3}: wtxt = {5'd4, G_SP, G_K, G_H, G_Z, G_SP, G_SP};               // DIG 4 kHz
            default: wtxt = {7{G_SP}};
        endcase
    end

    wire z7 = (fd[31:28] == 4'd0);        // leading zeros of the frequency are left blank
    wire z6 = z7 && (fd[27:24] == 4'd0);
    wire z5 = z6 && (fd[23:20] == 4'd0);
    wire z4 = z5 && (fd[19:16] == 4'd0);
    reg [4:0] sg;
    always @* begin
        sg = G_SP;
        case (tsel)
            3'd2: case (sc_i)                                 // 12345.678
                4'd0: sg = z7 ? G_SP : {1'b0, fd[31:28]};
                4'd1: sg = z6 ? G_SP : {1'b0, fd[27:24]};
                4'd2: sg = z5 ? G_SP : {1'b0, fd[23:20]};
                4'd3: sg = z4 ? G_SP : {1'b0, fd[19:16]};
                4'd4: sg = {1'b0, fd[15:12]};
                4'd5: sg = G_DOT;
                4'd6: sg = {1'b0, fd[11:8]};
                4'd7: sg = {1'b0, fd[7:4]};
                default: sg = {1'b0, fd[3:0]};
            endcase
            3'd3: case (sc_i)
                4'd6: sg = G_K;
                4'd7: sg = G_H;
                4'd8: sg = G_Z;
                default: sg = G_SP;
            endcase
            3'd4: case (sc_i)                                 // mode
                4'd0: sg = mtxt[14:10];
                4'd1: sg = mtxt[9:5];
                4'd2: sg = mtxt[4:0];
                default: sg = G_SP;
            endcase
            3'd5: case (sc_i)                                 // filter width
                4'd0: sg = wtxt[34:30];
                4'd1: sg = wtxt[29:25];
                4'd2: sg = wtxt[24:20];
                4'd3: sg = wtxt[19:15];
                4'd4: sg = wtxt[14:10];
                4'd5: sg = wtxt[9:5];
                4'd6: sg = wtxt[4:0];
                default: sg = G_SP;
            endcase
            default: sg = G_SP;
        endcase
    end

    wire       hit   = (tsel == 3'd1) ? (pview ? (lab_hit || u_hit) : c_hit) :
                       (tsel != 3'd0) ? sc_hit : 1'b0;
    wire [4:0] hit_g = (tsel == 3'd1) ? (pview ? (lab_hit ? {1'b0, lab_d} : u_g) : c_g) : sg;
    // SAM: the mode is green when the carrier loop has locked, amber while it has not.
    // FM: green while the squelch is open (a signal is there), amber while it is shut.
    wire [1:0] hit_c = ((tsel == 3'd4) && ((fst[2:0] == 3'd1) || (fst[2:0] == 3'd5))) ? (fst[6] ? 2'd1 : 2'd2) : 2'd0;

    reg       tx_on  = 1'b0;              // inside a character
    reg [4:0] tx_g   = 5'd0;              // which one
    reg [3:0] tx_c   = 4'd9;              // pixel column inside it, 0..9
    reg [1:0] tx_col = 2'd0;              // 0 white, 1 green, 2 amber
    always @(posedge pix_clk) begin
        if (hit) begin
            tx_on <= 1'b1;  tx_g <= hit_g;  tx_c <= 4'd0;  tx_col <= hit_c;
        end else if (tx_c == 4'd9) begin
            tx_on <= 1'b0;
        end else begin
            tx_c <= tx_c + 4'd1;
        end
    end

    // one row of a character; the leftmost dot is bit 4
    function [4:0] font;
        input [4:0] g;
        input [2:0] r;
        begin
            case ({g, r})
                {5'd0, 3'd0}: font = 5'b01110;  {5'd0, 3'd1}: font = 5'b10001;      // 0
                {5'd0, 3'd2}: font = 5'b10011;  {5'd0, 3'd3}: font = 5'b10101;
                {5'd0, 3'd4}: font = 5'b11001;  {5'd0, 3'd5}: font = 5'b10001;
                {5'd0, 3'd6}: font = 5'b01110;
                {5'd1, 3'd0}: font = 5'b00100;  {5'd1, 3'd1}: font = 5'b01100;      // 1
                {5'd1, 3'd2}: font = 5'b00100;  {5'd1, 3'd3}: font = 5'b00100;
                {5'd1, 3'd4}: font = 5'b00100;  {5'd1, 3'd5}: font = 5'b00100;
                {5'd1, 3'd6}: font = 5'b01110;
                {5'd2, 3'd0}: font = 5'b01110;  {5'd2, 3'd1}: font = 5'b10001;      // 2
                {5'd2, 3'd2}: font = 5'b00001;  {5'd2, 3'd3}: font = 5'b00010;
                {5'd2, 3'd4}: font = 5'b00100;  {5'd2, 3'd5}: font = 5'b01000;
                {5'd2, 3'd6}: font = 5'b11111;
                {5'd3, 3'd0}: font = 5'b01110;  {5'd3, 3'd1}: font = 5'b10001;      // 3
                {5'd3, 3'd2}: font = 5'b00001;  {5'd3, 3'd3}: font = 5'b00110;
                {5'd3, 3'd4}: font = 5'b00001;  {5'd3, 3'd5}: font = 5'b10001;
                {5'd3, 3'd6}: font = 5'b01110;
                {5'd4, 3'd0}: font = 5'b00010;  {5'd4, 3'd1}: font = 5'b00110;      // 4
                {5'd4, 3'd2}: font = 5'b01010;  {5'd4, 3'd3}: font = 5'b10010;
                {5'd4, 3'd4}: font = 5'b11111;  {5'd4, 3'd5}: font = 5'b00010;
                {5'd4, 3'd6}: font = 5'b00010;
                {5'd5, 3'd0}: font = 5'b11111;  {5'd5, 3'd1}: font = 5'b10000;      // 5
                {5'd5, 3'd2}: font = 5'b11110;  {5'd5, 3'd3}: font = 5'b00001;
                {5'd5, 3'd4}: font = 5'b00001;  {5'd5, 3'd5}: font = 5'b10001;
                {5'd5, 3'd6}: font = 5'b01110;
                {5'd6, 3'd0}: font = 5'b00110;  {5'd6, 3'd1}: font = 5'b01000;      // 6
                {5'd6, 3'd2}: font = 5'b10000;  {5'd6, 3'd3}: font = 5'b11110;
                {5'd6, 3'd4}: font = 5'b10001;  {5'd6, 3'd5}: font = 5'b10001;
                {5'd6, 3'd6}: font = 5'b01110;
                {5'd7, 3'd0}: font = 5'b11111;  {5'd7, 3'd1}: font = 5'b00001;      // 7
                {5'd7, 3'd2}: font = 5'b00010;  {5'd7, 3'd3}: font = 5'b00100;
                {5'd7, 3'd4}: font = 5'b01000;  {5'd7, 3'd5}: font = 5'b01000;
                {5'd7, 3'd6}: font = 5'b01000;
                {5'd8, 3'd0}: font = 5'b01110;  {5'd8, 3'd1}: font = 5'b10001;      // 8
                {5'd8, 3'd2}: font = 5'b10001;  {5'd8, 3'd3}: font = 5'b01110;
                {5'd8, 3'd4}: font = 5'b10001;  {5'd8, 3'd5}: font = 5'b10001;
                {5'd8, 3'd6}: font = 5'b01110;
                {5'd9, 3'd0}: font = 5'b01110;  {5'd9, 3'd1}: font = 5'b10001;      // 9
                {5'd9, 3'd2}: font = 5'b10001;  {5'd9, 3'd3}: font = 5'b01111;
                {5'd9, 3'd4}: font = 5'b00001;  {5'd9, 3'd5}: font = 5'b00010;
                {5'd9, 3'd6}: font = 5'b01100;
                {5'd10, 3'd0}: font = 5'b10001;  {5'd10, 3'd1}: font = 5'b11011;    // M
                {5'd10, 3'd2}: font = 5'b10101;  {5'd10, 3'd3}: font = 5'b10101;
                {5'd10, 3'd4}: font = 5'b10001;  {5'd10, 3'd5}: font = 5'b10001;
                {5'd10, 3'd6}: font = 5'b10001;
                {5'd11, 3'd0}: font = 5'b10001;  {5'd11, 3'd1}: font = 5'b10001;    // H
                {5'd11, 3'd2}: font = 5'b10001;  {5'd11, 3'd3}: font = 5'b11111;
                {5'd11, 3'd4}: font = 5'b10001;  {5'd11, 3'd5}: font = 5'b10001;
                {5'd11, 3'd6}: font = 5'b10001;
                {5'd12, 3'd2}: font = 5'b11111;  {5'd12, 3'd3}: font = 5'b00010;    // z
                {5'd12, 3'd4}: font = 5'b00100;  {5'd12, 3'd5}: font = 5'b01000;
                {5'd12, 3'd6}: font = 5'b11111;
                {5'd13, 3'd0}: font = 5'b10000;  {5'd13, 3'd1}: font = 5'b10000;    // k
                {5'd13, 3'd2}: font = 5'b10010;  {5'd13, 3'd3}: font = 5'b10100;
                {5'd13, 3'd4}: font = 5'b11000;  {5'd13, 3'd5}: font = 5'b10100;
                {5'd13, 3'd6}: font = 5'b10010;
                {5'd14, 3'd5}: font = 5'b01100;  {5'd14, 3'd6}: font = 5'b01100;    // .
                {5'd15, 3'd0}: font = 5'b01110;  {5'd15, 3'd1}: font = 5'b10001;    // A
                {5'd15, 3'd2}: font = 5'b10001;  {5'd15, 3'd3}: font = 5'b11111;
                {5'd15, 3'd4}: font = 5'b10001;  {5'd15, 3'd5}: font = 5'b10001;
                {5'd15, 3'd6}: font = 5'b10001;
                {5'd16, 3'd0}: font = 5'b01111;  {5'd16, 3'd1}: font = 5'b10000;    // S
                {5'd16, 3'd2}: font = 5'b10000;  {5'd16, 3'd3}: font = 5'b01110;
                {5'd16, 3'd4}: font = 5'b00001;  {5'd16, 3'd5}: font = 5'b00001;
                {5'd16, 3'd6}: font = 5'b11110;
                {5'd17, 3'd0}: font = 5'b10001;  {5'd17, 3'd1}: font = 5'b10001;    // U
                {5'd17, 3'd2}: font = 5'b10001;  {5'd17, 3'd3}: font = 5'b10001;
                {5'd17, 3'd4}: font = 5'b10001;  {5'd17, 3'd5}: font = 5'b10001;
                {5'd17, 3'd6}: font = 5'b01110;
                {5'd18, 3'd0}: font = 5'b10000;  {5'd18, 3'd1}: font = 5'b10000;    // L
                {5'd18, 3'd2}: font = 5'b10000;  {5'd18, 3'd3}: font = 5'b10000;
                {5'd18, 3'd4}: font = 5'b10000;  {5'd18, 3'd5}: font = 5'b10000;
                {5'd18, 3'd6}: font = 5'b11111;
                {5'd19, 3'd0}: font = 5'b11110;  {5'd19, 3'd1}: font = 5'b10001;    // B
                {5'd19, 3'd2}: font = 5'b10001;  {5'd19, 3'd3}: font = 5'b11110;
                {5'd19, 3'd4}: font = 5'b10001;  {5'd19, 3'd5}: font = 5'b10001;
                {5'd19, 3'd6}: font = 5'b11110;
                {5'd20, 3'd0}: font = 5'b01110;  {5'd20, 3'd1}: font = 5'b10001;    // C
                {5'd20, 3'd2}: font = 5'b10000;  {5'd20, 3'd3}: font = 5'b10000;
                {5'd20, 3'd4}: font = 5'b10000;  {5'd20, 3'd5}: font = 5'b10001;
                {5'd20, 3'd6}: font = 5'b01110;
                {5'd21, 3'd0}: font = 5'b10001;  {5'd21, 3'd1}: font = 5'b10001;    // W
                {5'd21, 3'd2}: font = 5'b10001;  {5'd21, 3'd3}: font = 5'b10101;
                {5'd21, 3'd4}: font = 5'b10101;  {5'd21, 3'd5}: font = 5'b10101;
                {5'd21, 3'd6}: font = 5'b01010;
                {5'd22, 3'd0}: font = 5'b11111;  {5'd22, 3'd1}: font = 5'b10000;    // F
                {5'd22, 3'd2}: font = 5'b10000;  {5'd22, 3'd3}: font = 5'b11110;
                {5'd22, 3'd4}: font = 5'b10000;  {5'd22, 3'd5}: font = 5'b10000;
                {5'd22, 3'd6}: font = 5'b10000;
                {5'd23, 3'd0}: font = 5'b11110;  {5'd23, 3'd1}: font = 5'b10001;    // D
                {5'd23, 3'd2}: font = 5'b10001;  {5'd23, 3'd3}: font = 5'b10001;
                {5'd23, 3'd4}: font = 5'b10001;  {5'd23, 3'd5}: font = 5'b10001;
                {5'd23, 3'd6}: font = 5'b11110;
                {5'd24, 3'd0}: font = 5'b01110;  {5'd24, 3'd1}: font = 5'b00100;    // I
                {5'd24, 3'd2}: font = 5'b00100;  {5'd24, 3'd3}: font = 5'b00100;
                {5'd24, 3'd4}: font = 5'b00100;  {5'd24, 3'd5}: font = 5'b00100;
                {5'd24, 3'd6}: font = 5'b01110;
                {5'd25, 3'd0}: font = 5'b01110;  {5'd25, 3'd1}: font = 5'b10001;    // G
                {5'd25, 3'd2}: font = 5'b10000;  {5'd25, 3'd3}: font = 5'b10111;
                {5'd25, 3'd4}: font = 5'b10001;  {5'd25, 3'd5}: font = 5'b10001;
                {5'd25, 3'd6}: font = 5'b01111;
                default:      font = 5'b00000;
            endcase
        end
    endfunction

    // ---- stage 2: RAM read; ruler ticks; one dot of text --------------------------------
    // Wide view: a pixel column covers 30.5 kHz. (500 x column + 375) / 16384 is its frequency
    // in MHz, so a tick belongs wherever the whole-MHz part of that number steps up.
    wire [18:0] tk    = {s1_px, 9'd0} - {6'd0, s1_px, 3'd0} - {7'd0, s1_px, 2'd0} + 19'd375;   // 500 x column + 375
    wire        tick  = (tk[13:0] < 14'd500);
    wire [4:0]  mhz   = tk[18:14];
    wire        five  = (mhz == 5'd0)  || (mhz == 5'd5)  || (mhz == 5'd10) || (mhz == 5'd15) ||
                        (mhz == 5'd20) || (mhz == 5'd25) || (mhz == 5'd30);
    wire [4:0]  fline = font(tx_g, s1_trw[3:1]);
    wire        fdot  = fline[3'd4 - tx_c[3:1]];

    reg [7:0] q1 = 8'd0, q2 = 8'd0, q3 = 8'd0;
    reg [9:0] s2_px = 10'd0;
    reg [6:0] s2_hb = 7'd0;
    reg [4:0] s2_ry = 5'd0;
    reg [2:0] s2_md = 3'd7;
    reg [1:0] s2_tcol = 2'd0;
    reg       s2_pane = 1'b0, s2_tr = 1'b0, s2_rl = 1'b0, s2_wf = 1'b0, s2_key = 1'b0;
    reg       s2_tick = 1'b0, s2_five = 1'b0, s2_txt = 1'b0, s2_ok = 1'b0, s2_pb = 1'b0;

    always @(posedge pix_clk) begin
        q1 <= ram[ra];                    // the block RAM's read port
    end

    always @(posedge pix_clk) begin
        s2_px   <= s1_px;    s2_hb <= s1_hb;  s2_ry <= s1_ry;  s2_md  <= s1_md;
        s2_pane <= s1_pane;  s2_tr <= s1_tr;  s2_rl <= s1_rl;  s2_wf  <= s1_wf;
        s2_key  <= s1_key;   s2_ok <= s1_ok;  s2_pb <= s1_pb;
        s2_tick <= pview ? ltick : tick;
        s2_five <= pview ? lmaj  : (tick && five);
        s2_txt  <= tx_on && (s1_tsel != 3'd0) && fdot;
        s2_tcol <= tx_col;
    end

    // ---- stage 3: second RAM register; what is drawn over the picture here ---------------
    //   0 nothing, 1 grey, 2 white, 3 red, 4 yellow, 5 green, 6 amber
    wire r_white = s2_pane && s2_five && (s2_ry >= 5'd1) && (s2_ry <= 5'd8);
    wire r_grey  = s2_pane && ((s2_ry == 5'd0) || (s2_tick && (s2_ry >= 5'd1) && (s2_ry <= 5'd4)));
    wire r_red   = mk[10] && (s2_ry >= 5'd25) && ({2'b00, s2_md} <= (5'd31 - s2_ry));
    wire r_yel   = s2_pb && (s2_ry >= 5'd26) && (s2_ry <= 5'd30);     // the filter's pass band

    reg [2:0] s3_ov = 3'd0;
    reg       s3_mkl = 1'b0;              // marker line in the trace area
    reg       s3_pbt = 1'b0;              // pass band behind the trace
    reg [7:0] s3_kx = 8'd0;               // colour key: column / 4
    reg [6:0] s3_hb = 7'd0;
    reg       s3_pane = 1'b0, s3_tr = 1'b0, s3_wf = 1'b0, s3_key = 1'b0, s3_ok = 1'b0;

    always @(posedge pix_clk) begin
        q2      <= q1;
        s3_ov   <= s2_txt ? ((s2_tcol == 2'd1) ? 3'd5 : (s2_tcol == 2'd2) ? 3'd6 : 3'd2) :
                   !s2_rl ? 3'd0 : r_white ? 3'd2 : r_red ? 3'd3 : r_yel ? 3'd4 : r_grey ? 3'd1 : 3'd0;
        s3_mkl  <= s2_tr && mk[10] && (s2_md == 3'd0);
        s3_pbt  <= s2_tr && s2_pb;
        s3_kx   <= s2_px[9:2];  s3_hb <= s2_hb;
        s3_pane <= s2_pane;  s3_tr <= s2_tr;  s3_wf <= s2_wf;  s3_key <= s2_key;  s3_ok <= s2_ok;
    end

    // ---- stage 4: third RAM register ---------------------------------------------------
    reg [2:0] s4_ov = 3'd0;
    reg       s4_mkl = 1'b0, s4_pbt = 1'b0;
    reg [7:0] s4_kx = 8'd0;
    reg [6:0] s4_hb = 7'd0;
    reg       s4_pane = 1'b0, s4_tr = 1'b0, s4_wf = 1'b0, s4_key = 1'b0, s4_ok = 1'b0;

    always @(posedge pix_clk) begin
        q3      <= q2;
        s4_ov   <= s3_ov;    s4_mkl <= s3_mkl;  s4_pbt <= s3_pbt;
        s4_kx   <= s3_kx;    s4_hb  <= s3_hb;
        s4_pane <= s3_pane;  s4_tr  <= s3_tr;  s4_wf <= s3_wf;  s4_key <= s3_key;  s4_ok <= s3_ok;
    end

    // ---- stage 5: choose the colour number ----------------------------------------------
    // waterfall: the stored byte. trace: a bar q3/2 rows tall, coloured by height, with a
    // light cap. colour key: 0 at the left edge of the pane to 255 at the right.
    wire [6:0] bar = s4_ok ? q3[7:1] : 7'd0;                  // nothing where the line has moved away
    reg [7:0] s5_idx = 8'd0;
    reg       s5_pal = 1'b0, s5_cap = 1'b0, s5_mkl = 1'b0, s5_pbt = 1'b0;
    reg [2:0] s5_ov  = 3'd0;

    always @(posedge pix_clk) begin
        s5_idx <= s4_wf ? q3 : s4_tr ? {s4_hb, 1'b1} : s4_kx;
        s5_pal <= s4_pane && (s4_wf || s4_key || (s4_tr && (bar > s4_hb)));
        s5_cap <= s4_pane && s4_tr && ({1'b0, bar} == {1'b0, s4_hb} + 8'd1);
        s5_mkl <= s4_mkl;
        s5_pbt <= s4_pbt;
        s5_ov  <= s4_ov;
    end

    // colour scale: black - blue - cyan - yellow - red
    wire [7:0] ramp = {s5_idx[5:0], s5_idx[5:4]};
    reg  [7:0] pr, pg, pb;
    always @* begin
        case (s5_idx[7:6])
            2'd0:    begin pr = 8'h00; pg = 8'h00; pb = ramp;  end
            2'd1:    begin pr = 8'h00; pg = ramp;  pb = 8'hFF; end
            2'd2:    begin pr = ramp;  pg = 8'hFF; pb = ~ramp; end
            default: begin pr = 8'hFF; pg = ~ramp; pb = 8'h00; end
        endcase
    end

    // ---- sync and data-enable ride along, five stages behind the counters ---------------
    reg [4:0] act_d = 5'd0, hs_d = 5'd0, vs_d = 5'd0;
    always @(posedge pix_clk) begin
        if (rst) begin
            act_d <= 5'd0;  hs_d <= 5'd0;  vs_d <= 5'd0;
        end else begin
            act_d <= {act_d[3:0], active};
            hs_d  <= {hs_d[3:0],  hs};
            vs_d  <= {vs_d[3:0],  vs};
        end
    end

    // ---- stage 6: output register, 24'h{RR, BB, GG} -------------------------------------
    always @(posedge pix_clk) begin
        if (rst) begin
            vid_pData  <= 24'd0;
            vid_pHSync <= 1'b0;
            vid_pVSync <= 1'b0;
            vid_pVDE   <= 1'b0;
        end else begin
            if (!act_d[4])            vid_pData <= 24'h000000;
            else if (s5_ov == 3'd2)   vid_pData <= 24'hFFFFFF;       // text, labelled ticks
            else if (s5_ov == 3'd3)   vid_pData <= 24'hFF3030;       // marker
            else if (s5_ov == 3'd4)   vid_pData <= 24'hFF00D0;       // pass band: yellow
            else if (s5_ov == 3'd5)   vid_pData <= 24'h4040FF;       // green
            else if (s5_ov == 3'd6)   vid_pData <= 24'hFF00C0;       // amber
            else if (s5_ov == 3'd1)   vid_pData <= 24'hA0A0A0;       // ruler line, small ticks
            else if (s5_cap)          vid_pData <= 24'hE0E0E0;       // top of a trace bar
            else if (s5_pal)          vid_pData <= {pr, pb, pg};
            else if (s5_mkl)          vid_pData <= 24'hB00000;       // marker line behind the trace
            else if (s5_pbt)          vid_pData <= 24'h280020;       // pass band behind the trace
            else                      vid_pData <= 24'h000000;
            vid_pHSync <= hs_d[4];
            vid_pVSync <= vs_d[4];
            vid_pVDE   <= act_d[4];
        end
    end

    // ---- bring-up LEDs, as in hdmi_bars ----------------------------------------------------
    reg [4:0] fcnt = 5'd0;
    always @(posedge pix_clk) begin
        if (rst) begin
            fcnt      <= 5'd0;
            led_frame <= 1'b0;
        end else if (h_last && v_last) begin
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