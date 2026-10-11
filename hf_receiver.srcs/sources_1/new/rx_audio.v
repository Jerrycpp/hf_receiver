`timescale 1ns / 1ps
// rx_audio.v - everything between the two audio CICs and the I2S transmitter.
//
//   One sample arrives every 1280 clocks (48.83 kS/s) and is worked on one step at a time:
//
//     noise blanker    bridges clicks while they are still short           (ctrl_i[10])
//     sideband turn    USB, LSB, DIG: the samples are turned at the centre frequency of the
//                      wanted sideband, which moves that sideband onto 0 Hz
//     channel filter   511 taps, thirteen widths from 250 Hz to 20 kHz (BTN2 wider, BTN3 narrower)
//     detector         AM:   the length of the I/Q arrow (envelope)
//                      SAM:  a loop locks an oscillator to the carrier and the signal is turned
//                            by its angle; the in-phase part is the audio
//                      USB, LSB, DIG: the filtered sideband is turned back up to its audio
//                            frequencies; the real part is the audio
//                      CW:   the same, turned to 700 Hz: a signal on the tuned frequency
//                            gives a 700 Hz tone
//                      FM:   the step of the angle from one sample to the next is the
//                            frequency; it is de-emphasised, and a squelch mutes the noise
//                            when there is no signal                        (BTN0 / BTN1)
//     hum filter       two high-pass sections at 61 Hz; they also take the carrier away
//     AGC              AM, SAM: the audio is divided by the carrier level.  Sideband modes:
//                      by the peak level.  FM: none.  Fast when the level rises, slow when
//                      it falls.
//     audio filters    four second-order sections.  AM, SAM at 12 kHz: a notch at 5 kHz.
//                      FM: a sixth-order high-pass at 300 Hz and a low-pass at 3.1 kHz.
//     noise reduction  turns down the frequency cells that hold only noise  (SW0 or ctrl_i[11]);
//                      not used in CW and DIG
//     volume           ctrl_i[3:0], 3 dB a step, 7 is the normal setting
//     limiter          rounds peaks off above half of full scale instead of clipping them
//
//   modes    0 AM   1 SAM   2 USB   3 LSB   4 CW   5 FM   6 DIG.   Each mode keeps its own width.
//   widths   AM, SAM: 3, 6, 9, 12 kHz      USB, LSB: 1.8, 2.4, 3, 4 kHz      CW: 250, 500, 800, 1200 Hz
//            FM: 9, 12, 16, 20 kHz         DIG: 0.5 and 1.2 kHz around 1500 Hz, 3, 4 kHz
//
//   ctrl_i   [3:0] volume   [4] test tone   [5] squelch off   [7:6] AGC threshold (0 = normal)
//            [8] notch off  [9] AGC fast    [10] blanker on   [11] noise reduction on
//            [13:12] its depth: 12, 18, 24, 6 dB   [15:14] blanker threshold: 4, 3, 6, 10 times
//   stat_o   {clip, lock, 1'b0, width[1:0], mode[2:0]}   for hdmi_wf.  "lock" is the carrier
//            loop in AM and SAM, and "squelch open" in FM.
//
//   A retune by 1.9 kHz or more, another mode or another width mutes the sound for 21 ms
//   (47 ms with the noise reduction on) and starts the AGC afresh.
//   Every output sample leaves 1002 clocks after its input sample arrived.
//
//   A matching bit-exact model exists for every block.

module rx_audio #(
    parameter integer CIC_BITS = 28,      // bits in an audio CIC's output word
    parameter integer N_MODES  = 7,       // modes the buttons step through, 2..7
    parameter integer DEB_BITS = 20       // a button must hold still for 2^DEB_BITS clocks (17 ms)
)(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [31:0] s_axis_sin_tdata,  // audio CIC on the sine product
    input  wire        s_axis_sin_tvalid,
    input  wire [31:0] s_axis_cos_tdata,  // audio CIC on the cosine product
    input  wire        s_axis_cos_tvalid,
    input  wire [31:0] tune_i,            // DDS tuning word; a change restarts the carrier loop
    input  wire [15:0] ctrl_i,
    input  wire [3:0]  btn_i,             // BTN0 next mode, BTN1 previous, BTN2 wider, BTN3 narrower
    input  wire        sw_i,              // SW0: noise reduction on
    output reg  [23:0] m_axis_tdata = 24'd0,   // to i2s_tx
    output reg         m_axis_tvalid = 1'b0,
    output wire [7:0]  stat_o,
    output wire        led_lock_o,
    output wire        led_clip_o,
    output wire        led_nb_o           // a click was bridged
);

    // =====================================================================================
    //  buttons: two flip-flops, then the level has to hold still before it counts
    // =====================================================================================
    reg  [2:0]  mode = 3'd0;              // 0 AM, 1 SAM, 2 USB, 3 LSB, 4 CW, 5 FM, 6 DIG
    // Every mode remembers its own width, two bits each.  They start at
    // AM 6 kHz, SAM 6 kHz, USB 2.4 kHz, LSB 2.4 kHz, CW 500 Hz, FM 16 kHz, DIG 3 kHz.
    reg  [15:0] bwv  = 16'b01_10_10_01_01_01_01_01;
    wire [1:0]  bw   = bwv[{mode, 1'b0} +: 2];
    wire [3:0] press;
    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : key
            (* ASYNC_REG = "TRUE" *) reg [1:0] ff = 2'b00;
            reg                ok  = 1'b0;
            reg [DEB_BITS-1:0] cnt = {DEB_BITS{1'b0}};
            reg                hit = 1'b0;
            always @(posedge aclk) begin
                ff  <= {ff[0], btn_i[gi]};
                hit <= 1'b0;
                if (ff[1] == ok) begin
                    cnt <= {DEB_BITS{1'b0}};
                end else begin
                    cnt <= cnt + 1'b1;
                    if (&cnt) begin
                        ok  <= ff[1];
                        hit <= ff[1];             // a press counts, a release does not
                    end
                end
            end
            assign press[gi] = hit;
        end
    endgenerate

    wire [2:0] mode_top = N_MODES - 1;
    always @(posedge aclk) begin
        if (press[0] && !press[1])      mode <= (mode == mode_top) ? 3'd0 : mode + 3'd1;
        else if (press[1] && !press[0]) mode <= (mode == 3'd0) ? mode_top : mode - 3'd1;
        if (press[2] && !press[3])      bwv[{mode, 1'b0} +: 2] <= bw + 2'd1;
        else if (press[3] && !press[2]) bwv[{mode, 1'b0} +: 2] <= bw - 2'd1;
    end

    // =====================================================================================
    //  channel filter: 511 taps, one multiply per clock for each of I and Q
    // =====================================================================================
    localparam integer T = CIC_BITS - 1;
    wire               tick = s_axis_sin_tvalid;
    wire signed [24:0] in_i = s_axis_cos_tdata[T -: 25];
    wire signed [24:0] in_q = s_axis_sin_tdata[T -: 25];

    // ---- noise blanker: the filter takes its samples from here, 40 samples late ---------------
    wire signed [24:0] nb_i, nb_q;
    wire               nb_v;
    rxa_nb u_nb (.clk(aclk), .rst(!aresetn), .tick(tick), .i_i(in_i), .q_i(in_q),
                 .on(ctrl_i[10]), .tsel(ctrl_i[15:14]),
                 .i_o(nb_i), .q_o(nb_q), .valid(nb_v), .led(led_nb_o));

    (* ASYNC_REG = "TRUE" *) reg [1:0] sw_ff = 2'b00;             // SW0, brought onto this clock
    always @(posedge aclk) sw_ff <= {sw_ff[0], sw_i};
    wire nr_sw = sw_ff[1];

    (* ram_style = "distributed" *) reg [49:0] mem [0:511];     // the last 512 samples, {Q, I}
    (* rom_style = "block" *) reg signed [17:0] rom [0:4095];     // 16 sets of 256 coefficients, 13 in use
    integer ii;
    initial begin
        for (ii = 0; ii < 512; ii = ii + 1) mem[ii] = 50'd0;
        for (ii = 0; ii < 4096; ii = ii + 1) rom[ii] = 18'sd0;
        // set 0, 0.25 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^22
        rom[   0] = -18'sd13;  rom[   1] = -18'sd15;  rom[   2] = -18'sd16;  rom[   3] = -18'sd18;
        rom[   4] = -18'sd20;  rom[   5] = -18'sd21;  rom[   6] = -18'sd23;  rom[   7] = -18'sd25;
        rom[   8] = -18'sd27;  rom[   9] = -18'sd29;  rom[  10] = -18'sd31;  rom[  11] = -18'sd33;
        rom[  12] = -18'sd35;  rom[  13] = -18'sd37;  rom[  14] = -18'sd40;  rom[  15] = -18'sd42;
        rom[  16] = -18'sd44;  rom[  17] = -18'sd46;  rom[  18] = -18'sd48;  rom[  19] = -18'sd51;
        rom[  20] = -18'sd53;  rom[  21] = -18'sd55;  rom[  22] = -18'sd57;  rom[  23] = -18'sd59;
        rom[  24] = -18'sd61;  rom[  25] = -18'sd64;  rom[  26] = -18'sd66;  rom[  27] = -18'sd67;
        rom[  28] = -18'sd69;  rom[  29] = -18'sd71;  rom[  30] = -18'sd73;  rom[  31] = -18'sd74;
        rom[  32] = -18'sd76;  rom[  33] = -18'sd77;  rom[  34] = -18'sd78;  rom[  35] = -18'sd79;
        rom[  36] = -18'sd80;  rom[  37] = -18'sd80;  rom[  38] = -18'sd81;  rom[  39] = -18'sd81;
        rom[  40] = -18'sd81;  rom[  41] = -18'sd80;  rom[  42] = -18'sd80;  rom[  43] = -18'sd79;
        rom[  44] = -18'sd77;  rom[  45] = -18'sd76;  rom[  46] = -18'sd74;  rom[  47] = -18'sd71;
        rom[  48] = -18'sd68;  rom[  49] = -18'sd65;  rom[  50] = -18'sd62;  rom[  51] = -18'sd57;
        rom[  52] = -18'sd53;  rom[  53] = -18'sd47;  rom[  54] = -18'sd42;  rom[  55] = -18'sd35;
        rom[  56] = -18'sd28;  rom[  57] = -18'sd21;  rom[  58] = -18'sd12;  rom[  59] = -18'sd4;
        rom[  60] =  18'sd6;  rom[  61] =  18'sd17;  rom[  62] =  18'sd28;  rom[  63] =  18'sd40;
        rom[  64] =  18'sd53;  rom[  65] =  18'sd66;  rom[  66] =  18'sd81;  rom[  67] =  18'sd96;
        rom[  68] =  18'sd113;  rom[  69] =  18'sd130;  rom[  70] =  18'sd149;  rom[  71] =  18'sd169;
        rom[  72] =  18'sd189;  rom[  73] =  18'sd211;  rom[  74] =  18'sd234;  rom[  75] =  18'sd258;
        rom[  76] =  18'sd284;  rom[  77] =  18'sd310;  rom[  78] =  18'sd338;  rom[  79] =  18'sd367;
        rom[  80] =  18'sd398;  rom[  81] =  18'sd430;  rom[  82] =  18'sd464;  rom[  83] =  18'sd499;
        rom[  84] =  18'sd535;  rom[  85] =  18'sd573;  rom[  86] =  18'sd613;  rom[  87] =  18'sd654;
        rom[  88] =  18'sd697;  rom[  89] =  18'sd742;  rom[  90] =  18'sd788;  rom[  91] =  18'sd836;
        rom[  92] =  18'sd886;  rom[  93] =  18'sd938;  rom[  94] =  18'sd991;  rom[  95] =  18'sd1047;
        rom[  96] =  18'sd1104;  rom[  97] =  18'sd1163;  rom[  98] =  18'sd1225;  rom[  99] =  18'sd1288;
        rom[ 100] =  18'sd1353;  rom[ 101] =  18'sd1421;  rom[ 102] =  18'sd1490;  rom[ 103] =  18'sd1562;
        rom[ 104] =  18'sd1636;  rom[ 105] =  18'sd1712;  rom[ 106] =  18'sd1790;  rom[ 107] =  18'sd1871;
        rom[ 108] =  18'sd1954;  rom[ 109] =  18'sd2039;  rom[ 110] =  18'sd2126;  rom[ 111] =  18'sd2216;
        rom[ 112] =  18'sd2308;  rom[ 113] =  18'sd2403;  rom[ 114] =  18'sd2500;  rom[ 115] =  18'sd2599;
        rom[ 116] =  18'sd2701;  rom[ 117] =  18'sd2805;  rom[ 118] =  18'sd2912;  rom[ 119] =  18'sd3021;
        rom[ 120] =  18'sd3132;  rom[ 121] =  18'sd3247;  rom[ 122] =  18'sd3363;  rom[ 123] =  18'sd3483;
        rom[ 124] =  18'sd3604;  rom[ 125] =  18'sd3729;  rom[ 126] =  18'sd3855;  rom[ 127] =  18'sd3985;
        rom[ 128] =  18'sd4117;  rom[ 129] =  18'sd4251;  rom[ 130] =  18'sd4388;  rom[ 131] =  18'sd4527;
        rom[ 132] =  18'sd4669;  rom[ 133] =  18'sd4814;  rom[ 134] =  18'sd4961;  rom[ 135] =  18'sd5111;
        rom[ 136] =  18'sd5263;  rom[ 137] =  18'sd5417;  rom[ 138] =  18'sd5574;  rom[ 139] =  18'sd5734;
        rom[ 140] =  18'sd5895;  rom[ 141] =  18'sd6060;  rom[ 142] =  18'sd6226;  rom[ 143] =  18'sd6395;
        rom[ 144] =  18'sd6567;  rom[ 145] =  18'sd6740;  rom[ 146] =  18'sd6916;  rom[ 147] =  18'sd7094;
        rom[ 148] =  18'sd7274;  rom[ 149] =  18'sd7457;  rom[ 150] =  18'sd7641;  rom[ 151] =  18'sd7828;
        rom[ 152] =  18'sd8017;  rom[ 153] =  18'sd8208;  rom[ 154] =  18'sd8400;  rom[ 155] =  18'sd8595;
        rom[ 156] =  18'sd8791;  rom[ 157] =  18'sd8990;  rom[ 158] =  18'sd9190;  rom[ 159] =  18'sd9392;
        rom[ 160] =  18'sd9595;  rom[ 161] =  18'sd9800;  rom[ 162] =  18'sd10007;  rom[ 163] =  18'sd10215;
        rom[ 164] =  18'sd10424;  rom[ 165] =  18'sd10635;  rom[ 166] =  18'sd10847;  rom[ 167] =  18'sd11061;
        rom[ 168] =  18'sd11275;  rom[ 169] =  18'sd11491;  rom[ 170] =  18'sd11707;  rom[ 171] =  18'sd11925;
        rom[ 172] =  18'sd12143;  rom[ 173] =  18'sd12362;  rom[ 174] =  18'sd12582;  rom[ 175] =  18'sd12803;
        rom[ 176] =  18'sd13024;  rom[ 177] =  18'sd13245;  rom[ 178] =  18'sd13467;  rom[ 179] =  18'sd13690;
        rom[ 180] =  18'sd13912;  rom[ 181] =  18'sd14135;  rom[ 182] =  18'sd14357;  rom[ 183] =  18'sd14580;
        rom[ 184] =  18'sd14802;  rom[ 185] =  18'sd15025;  rom[ 186] =  18'sd15247;  rom[ 187] =  18'sd15468;
        rom[ 188] =  18'sd15689;  rom[ 189] =  18'sd15910;  rom[ 190] =  18'sd16130;  rom[ 191] =  18'sd16349;
        rom[ 192] =  18'sd16567;  rom[ 193] =  18'sd16784;  rom[ 194] =  18'sd17001;  rom[ 195] =  18'sd17216;
        rom[ 196] =  18'sd17430;  rom[ 197] =  18'sd17642;  rom[ 198] =  18'sd17853;  rom[ 199] =  18'sd18063;
        rom[ 200] =  18'sd18271;  rom[ 201] =  18'sd18477;  rom[ 202] =  18'sd18682;  rom[ 203] =  18'sd18885;
        rom[ 204] =  18'sd19085;  rom[ 205] =  18'sd19284;  rom[ 206] =  18'sd19480;  rom[ 207] =  18'sd19674;
        rom[ 208] =  18'sd19866;  rom[ 209] =  18'sd20055;  rom[ 210] =  18'sd20242;  rom[ 211] =  18'sd20426;
        rom[ 212] =  18'sd20608;  rom[ 213] =  18'sd20786;  rom[ 214] =  18'sd20962;  rom[ 215] =  18'sd21135;
        rom[ 216] =  18'sd21305;  rom[ 217] =  18'sd21471;  rom[ 218] =  18'sd21635;  rom[ 219] =  18'sd21795;
        rom[ 220] =  18'sd21951;  rom[ 221] =  18'sd22105;  rom[ 222] =  18'sd22254;  rom[ 223] =  18'sd22400;
        rom[ 224] =  18'sd22543;  rom[ 225] =  18'sd22681;  rom[ 226] =  18'sd22816;  rom[ 227] =  18'sd22947;
        rom[ 228] =  18'sd23074;  rom[ 229] =  18'sd23197;  rom[ 230] =  18'sd23316;  rom[ 231] =  18'sd23430;
        rom[ 232] =  18'sd23541;  rom[ 233] =  18'sd23647;  rom[ 234] =  18'sd23749;  rom[ 235] =  18'sd23846;
        rom[ 236] =  18'sd23939;  rom[ 237] =  18'sd24028;  rom[ 238] =  18'sd24112;  rom[ 239] =  18'sd24191;
        rom[ 240] =  18'sd24266;  rom[ 241] =  18'sd24336;  rom[ 242] =  18'sd24402;  rom[ 243] =  18'sd24463;
        rom[ 244] =  18'sd24519;  rom[ 245] =  18'sd24570;  rom[ 246] =  18'sd24617;  rom[ 247] =  18'sd24658;
        rom[ 248] =  18'sd24695;  rom[ 249] =  18'sd24727;  rom[ 250] =  18'sd24754;  rom[ 251] =  18'sd24776;
        rom[ 252] =  18'sd24794;  rom[ 253] =  18'sd24806;  rom[ 254] =  18'sd24813;  rom[ 255] =  18'sd24810;
        // set 1, 0.5 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^22
        rom[ 256] =  18'sd13;  rom[ 257] =  18'sd15;  rom[ 258] =  18'sd17;  rom[ 259] =  18'sd19;
        rom[ 260] =  18'sd21;  rom[ 261] =  18'sd24;  rom[ 262] =  18'sd26;  rom[ 263] =  18'sd29;
        rom[ 264] =  18'sd32;  rom[ 265] =  18'sd35;  rom[ 266] =  18'sd38;  rom[ 267] =  18'sd41;
        rom[ 268] =  18'sd44;  rom[ 269] =  18'sd48;  rom[ 270] =  18'sd51;  rom[ 271] =  18'sd55;
        rom[ 272] =  18'sd58;  rom[ 273] =  18'sd62;  rom[ 274] =  18'sd66;  rom[ 275] =  18'sd70;
        rom[ 276] =  18'sd74;  rom[ 277] =  18'sd78;  rom[ 278] =  18'sd82;  rom[ 279] =  18'sd86;
        rom[ 280] =  18'sd90;  rom[ 281] =  18'sd94;  rom[ 282] =  18'sd98;  rom[ 283] =  18'sd102;
        rom[ 284] =  18'sd106;  rom[ 285] =  18'sd109;  rom[ 286] =  18'sd113;  rom[ 287] =  18'sd116;
        rom[ 288] =  18'sd119;  rom[ 289] =  18'sd122;  rom[ 290] =  18'sd125;  rom[ 291] =  18'sd128;
        rom[ 292] =  18'sd130;  rom[ 293] =  18'sd132;  rom[ 294] =  18'sd133;  rom[ 295] =  18'sd134;
        rom[ 296] =  18'sd135;  rom[ 297] =  18'sd135;  rom[ 298] =  18'sd135;  rom[ 299] =  18'sd134;
        rom[ 300] =  18'sd132;  rom[ 301] =  18'sd130;  rom[ 302] =  18'sd127;  rom[ 303] =  18'sd124;
        rom[ 304] =  18'sd119;  rom[ 305] =  18'sd114;  rom[ 306] =  18'sd108;  rom[ 307] =  18'sd102;
        rom[ 308] =  18'sd94;  rom[ 309] =  18'sd85;  rom[ 310] =  18'sd76;  rom[ 311] =  18'sd65;
        rom[ 312] =  18'sd53;  rom[ 313] =  18'sd40;  rom[ 314] =  18'sd26;  rom[ 315] =  18'sd11;
        rom[ 316] = -18'sd6;  rom[ 317] = -18'sd24;  rom[ 318] = -18'sd43;  rom[ 319] = -18'sd63;
        rom[ 320] = -18'sd85;  rom[ 321] = -18'sd108;  rom[ 322] = -18'sd133;  rom[ 323] = -18'sd159;
        rom[ 324] = -18'sd187;  rom[ 325] = -18'sd216;  rom[ 326] = -18'sd247;  rom[ 327] = -18'sd280;
        rom[ 328] = -18'sd314;  rom[ 329] = -18'sd349;  rom[ 330] = -18'sd386;  rom[ 331] = -18'sd425;
        rom[ 332] = -18'sd466;  rom[ 333] = -18'sd508;  rom[ 334] = -18'sd551;  rom[ 335] = -18'sd597;
        rom[ 336] = -18'sd644;  rom[ 337] = -18'sd692;  rom[ 338] = -18'sd742;  rom[ 339] = -18'sd794;
        rom[ 340] = -18'sd847;  rom[ 341] = -18'sd901;  rom[ 342] = -18'sd957;  rom[ 343] = -18'sd1014;
        rom[ 344] = -18'sd1073;  rom[ 345] = -18'sd1133;  rom[ 346] = -18'sd1194;  rom[ 347] = -18'sd1256;
        rom[ 348] = -18'sd1320;  rom[ 349] = -18'sd1384;  rom[ 350] = -18'sd1449;  rom[ 351] = -18'sd1515;
        rom[ 352] = -18'sd1582;  rom[ 353] = -18'sd1650;  rom[ 354] = -18'sd1717;  rom[ 355] = -18'sd1786;
        rom[ 356] = -18'sd1854;  rom[ 357] = -18'sd1923;  rom[ 358] = -18'sd1992;  rom[ 359] = -18'sd2061;
        rom[ 360] = -18'sd2129;  rom[ 361] = -18'sd2197;  rom[ 362] = -18'sd2265;  rom[ 363] = -18'sd2332;
        rom[ 364] = -18'sd2398;  rom[ 365] = -18'sd2463;  rom[ 366] = -18'sd2526;  rom[ 367] = -18'sd2588;
        rom[ 368] = -18'sd2649;  rom[ 369] = -18'sd2708;  rom[ 370] = -18'sd2765;  rom[ 371] = -18'sd2819;
        rom[ 372] = -18'sd2871;  rom[ 373] = -18'sd2921;  rom[ 374] = -18'sd2967;  rom[ 375] = -18'sd3011;
        rom[ 376] = -18'sd3051;  rom[ 377] = -18'sd3088;  rom[ 378] = -18'sd3121;  rom[ 379] = -18'sd3150;
        rom[ 380] = -18'sd3175;  rom[ 381] = -18'sd3196;  rom[ 382] = -18'sd3211;  rom[ 383] = -18'sd3222;
        rom[ 384] = -18'sd3228;  rom[ 385] = -18'sd3228;  rom[ 386] = -18'sd3222;  rom[ 387] = -18'sd3211;
        rom[ 388] = -18'sd3193;  rom[ 389] = -18'sd3169;  rom[ 390] = -18'sd3139;  rom[ 391] = -18'sd3101;
        rom[ 392] = -18'sd3056;  rom[ 393] = -18'sd3004;  rom[ 394] = -18'sd2944;  rom[ 395] = -18'sd2877;
        rom[ 396] = -18'sd2801;  rom[ 397] = -18'sd2717;  rom[ 398] = -18'sd2625;  rom[ 399] = -18'sd2524;
        rom[ 400] = -18'sd2413;  rom[ 401] = -18'sd2294;  rom[ 402] = -18'sd2165;  rom[ 403] = -18'sd2027;
        rom[ 404] = -18'sd1879;  rom[ 405] = -18'sd1721;  rom[ 406] = -18'sd1554;  rom[ 407] = -18'sd1375;
        rom[ 408] = -18'sd1187;  rom[ 409] = -18'sd988;  rom[ 410] = -18'sd778;  rom[ 411] = -18'sd558;
        rom[ 412] = -18'sd326;  rom[ 413] = -18'sd84;  rom[ 414] =  18'sd170;  rom[ 415] =  18'sd434;
        rom[ 416] =  18'sd710;  rom[ 417] =  18'sd998;  rom[ 418] =  18'sd1296;  rom[ 419] =  18'sd1606;
        rom[ 420] =  18'sd1927;  rom[ 421] =  18'sd2260;  rom[ 422] =  18'sd2604;  rom[ 423] =  18'sd2960;
        rom[ 424] =  18'sd3326;  rom[ 425] =  18'sd3704;  rom[ 426] =  18'sd4094;  rom[ 427] =  18'sd4494;
        rom[ 428] =  18'sd4905;  rom[ 429] =  18'sd5327;  rom[ 430] =  18'sd5760;  rom[ 431] =  18'sd6204;
        rom[ 432] =  18'sd6658;  rom[ 433] =  18'sd7122;  rom[ 434] =  18'sd7597;  rom[ 435] =  18'sd8081;
        rom[ 436] =  18'sd8575;  rom[ 437] =  18'sd9079;  rom[ 438] =  18'sd9592;  rom[ 439] =  18'sd10113;
        rom[ 440] =  18'sd10644;  rom[ 441] =  18'sd11182;  rom[ 442] =  18'sd11729;  rom[ 443] =  18'sd12283;
        rom[ 444] =  18'sd12845;  rom[ 445] =  18'sd13414;  rom[ 446] =  18'sd13990;  rom[ 447] =  18'sd14572;
        rom[ 448] =  18'sd15159;  rom[ 449] =  18'sd15753;  rom[ 450] =  18'sd16351;  rom[ 451] =  18'sd16954;
        rom[ 452] =  18'sd17561;  rom[ 453] =  18'sd18172;  rom[ 454] =  18'sd18787;  rom[ 455] =  18'sd19404;
        rom[ 456] =  18'sd20023;  rom[ 457] =  18'sd20645;  rom[ 458] =  18'sd21268;  rom[ 459] =  18'sd21891;
        rom[ 460] =  18'sd22516;  rom[ 461] =  18'sd23140;  rom[ 462] =  18'sd23763;  rom[ 463] =  18'sd24385;
        rom[ 464] =  18'sd25005;  rom[ 465] =  18'sd25623;  rom[ 466] =  18'sd26239;  rom[ 467] =  18'sd26851;
        rom[ 468] =  18'sd27459;  rom[ 469] =  18'sd28062;  rom[ 470] =  18'sd28661;  rom[ 471] =  18'sd29254;
        rom[ 472] =  18'sd29841;  rom[ 473] =  18'sd30421;  rom[ 474] =  18'sd30995;  rom[ 475] =  18'sd31560;
        rom[ 476] =  18'sd32117;  rom[ 477] =  18'sd32666;  rom[ 478] =  18'sd33205;  rom[ 479] =  18'sd33735;
        rom[ 480] =  18'sd34254;  rom[ 481] =  18'sd34762;  rom[ 482] =  18'sd35259;  rom[ 483] =  18'sd35744;
        rom[ 484] =  18'sd36217;  rom[ 485] =  18'sd36677;  rom[ 486] =  18'sd37123;  rom[ 487] =  18'sd37557;
        rom[ 488] =  18'sd37976;  rom[ 489] =  18'sd38380;  rom[ 490] =  18'sd38770;  rom[ 491] =  18'sd39144;
        rom[ 492] =  18'sd39502;  rom[ 493] =  18'sd39845;  rom[ 494] =  18'sd40170;  rom[ 495] =  18'sd40480;
        rom[ 496] =  18'sd40772;  rom[ 497] =  18'sd41046;  rom[ 498] =  18'sd41303;  rom[ 499] =  18'sd41542;
        rom[ 500] =  18'sd41763;  rom[ 501] =  18'sd41966;  rom[ 502] =  18'sd42149;  rom[ 503] =  18'sd42314;
        rom[ 504] =  18'sd42460;  rom[ 505] =  18'sd42587;  rom[ 506] =  18'sd42695;  rom[ 507] =  18'sd42783;
        rom[ 508] =  18'sd42851;  rom[ 509] =  18'sd42901;  rom[ 510] =  18'sd42930;  rom[ 511] =  18'sd42934;
        // set 2, 0.8 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^22
        rom[ 512] =  18'sd7;  rom[ 513] =  18'sd8;  rom[ 514] =  18'sd8;  rom[ 515] =  18'sd8;
        rom[ 516] =  18'sd7;  rom[ 517] =  18'sd7;  rom[ 518] =  18'sd6;  rom[ 519] =  18'sd6;
        rom[ 520] =  18'sd4;  rom[ 521] =  18'sd3;  rom[ 522] =  18'sd1;  rom[ 523] = -18'sd1;
        rom[ 524] = -18'sd3;  rom[ 525] = -18'sd6;  rom[ 526] = -18'sd9;  rom[ 527] = -18'sd12;
        rom[ 528] = -18'sd16;  rom[ 529] = -18'sd20;  rom[ 530] = -18'sd25;  rom[ 531] = -18'sd30;
        rom[ 532] = -18'sd36;  rom[ 533] = -18'sd42;  rom[ 534] = -18'sd48;  rom[ 535] = -18'sd55;
        rom[ 536] = -18'sd62;  rom[ 537] = -18'sd70;  rom[ 538] = -18'sd79;  rom[ 539] = -18'sd87;
        rom[ 540] = -18'sd96;  rom[ 541] = -18'sd106;  rom[ 542] = -18'sd116;  rom[ 543] = -18'sd126;
        rom[ 544] = -18'sd137;  rom[ 545] = -18'sd148;  rom[ 546] = -18'sd159;  rom[ 547] = -18'sd170;
        rom[ 548] = -18'sd181;  rom[ 549] = -18'sd193;  rom[ 550] = -18'sd205;  rom[ 551] = -18'sd216;
        rom[ 552] = -18'sd228;  rom[ 553] = -18'sd239;  rom[ 554] = -18'sd250;  rom[ 555] = -18'sd261;
        rom[ 556] = -18'sd271;  rom[ 557] = -18'sd281;  rom[ 558] = -18'sd290;  rom[ 559] = -18'sd299;
        rom[ 560] = -18'sd306;  rom[ 561] = -18'sd313;  rom[ 562] = -18'sd319;  rom[ 563] = -18'sd324;
        rom[ 564] = -18'sd328;  rom[ 565] = -18'sd330;  rom[ 566] = -18'sd331;  rom[ 567] = -18'sd330;
        rom[ 568] = -18'sd328;  rom[ 569] = -18'sd324;  rom[ 570] = -18'sd318;  rom[ 571] = -18'sd310;
        rom[ 572] = -18'sd300;  rom[ 573] = -18'sd288;  rom[ 574] = -18'sd274;  rom[ 575] = -18'sd257;
        rom[ 576] = -18'sd238;  rom[ 577] = -18'sd217;  rom[ 578] = -18'sd192;  rom[ 579] = -18'sd166;
        rom[ 580] = -18'sd136;  rom[ 581] = -18'sd104;  rom[ 582] = -18'sd69;  rom[ 583] = -18'sd31;
        rom[ 584] =  18'sd10;  rom[ 585] =  18'sd53;  rom[ 586] =  18'sd99;  rom[ 587] =  18'sd149;
        rom[ 588] =  18'sd200;  rom[ 589] =  18'sd255;  rom[ 590] =  18'sd312;  rom[ 591] =  18'sd372;
        rom[ 592] =  18'sd435;  rom[ 593] =  18'sd499;  rom[ 594] =  18'sd566;  rom[ 595] =  18'sd635;
        rom[ 596] =  18'sd707;  rom[ 597] =  18'sd779;  rom[ 598] =  18'sd854;  rom[ 599] =  18'sd930;
        rom[ 600] =  18'sd1006;  rom[ 601] =  18'sd1084;  rom[ 602] =  18'sd1163;  rom[ 603] =  18'sd1242;
        rom[ 604] =  18'sd1320;  rom[ 605] =  18'sd1399;  rom[ 606] =  18'sd1477;  rom[ 607] =  18'sd1554;
        rom[ 608] =  18'sd1630;  rom[ 609] =  18'sd1704;  rom[ 610] =  18'sd1776;  rom[ 611] =  18'sd1846;
        rom[ 612] =  18'sd1913;  rom[ 613] =  18'sd1977;  rom[ 614] =  18'sd2037;  rom[ 615] =  18'sd2093;
        rom[ 616] =  18'sd2145;  rom[ 617] =  18'sd2191;  rom[ 618] =  18'sd2232;  rom[ 619] =  18'sd2268;
        rom[ 620] =  18'sd2297;  rom[ 621] =  18'sd2319;  rom[ 622] =  18'sd2335;  rom[ 623] =  18'sd2343;
        rom[ 624] =  18'sd2342;  rom[ 625] =  18'sd2334;  rom[ 626] =  18'sd2317;  rom[ 627] =  18'sd2290;
        rom[ 628] =  18'sd2255;  rom[ 629] =  18'sd2209;  rom[ 630] =  18'sd2153;  rom[ 631] =  18'sd2087;
        rom[ 632] =  18'sd2010;  rom[ 633] =  18'sd1922;  rom[ 634] =  18'sd1823;  rom[ 635] =  18'sd1712;
        rom[ 636] =  18'sd1590;  rom[ 637] =  18'sd1456;  rom[ 638] =  18'sd1311;  rom[ 639] =  18'sd1153;
        rom[ 640] =  18'sd984;  rom[ 641] =  18'sd804;  rom[ 642] =  18'sd611;  rom[ 643] =  18'sd407;
        rom[ 644] =  18'sd192;  rom[ 645] = -18'sd35;  rom[ 646] = -18'sd272;  rom[ 647] = -18'sd520;
        rom[ 648] = -18'sd779;  rom[ 649] = -18'sd1047;  rom[ 650] = -18'sd1324;  rom[ 651] = -18'sd1610;
        rom[ 652] = -18'sd1904;  rom[ 653] = -18'sd2206;  rom[ 654] = -18'sd2515;  rom[ 655] = -18'sd2830;
        rom[ 656] = -18'sd3150;  rom[ 657] = -18'sd3475;  rom[ 658] = -18'sd3803;  rom[ 659] = -18'sd4134;
        rom[ 660] = -18'sd4467;  rom[ 661] = -18'sd4801;  rom[ 662] = -18'sd5134;  rom[ 663] = -18'sd5465;
        rom[ 664] = -18'sd5794;  rom[ 665] = -18'sd6119;  rom[ 666] = -18'sd6439;  rom[ 667] = -18'sd6752;
        rom[ 668] = -18'sd7058;  rom[ 669] = -18'sd7354;  rom[ 670] = -18'sd7641;  rom[ 671] = -18'sd7915;
        rom[ 672] = -18'sd8176;  rom[ 673] = -18'sd8422;  rom[ 674] = -18'sd8653;  rom[ 675] = -18'sd8866;
        rom[ 676] = -18'sd9060;  rom[ 677] = -18'sd9233;  rom[ 678] = -18'sd9385;  rom[ 679] = -18'sd9513;
        rom[ 680] = -18'sd9617;  rom[ 681] = -18'sd9695;  rom[ 682] = -18'sd9745;  rom[ 683] = -18'sd9766;
        rom[ 684] = -18'sd9758;  rom[ 685] = -18'sd9717;  rom[ 686] = -18'sd9645;  rom[ 687] = -18'sd9538;
        rom[ 688] = -18'sd9396;  rom[ 689] = -18'sd9218;  rom[ 690] = -18'sd9003;  rom[ 691] = -18'sd8750;
        rom[ 692] = -18'sd8457;  rom[ 693] = -18'sd8125;  rom[ 694] = -18'sd7752;  rom[ 695] = -18'sd7338;
        rom[ 696] = -18'sd6881;  rom[ 697] = -18'sd6383;  rom[ 698] = -18'sd5841;  rom[ 699] = -18'sd5255;
        rom[ 700] = -18'sd4626;  rom[ 701] = -18'sd3954;  rom[ 702] = -18'sd3237;  rom[ 703] = -18'sd2477;
        rom[ 704] = -18'sd1673;  rom[ 705] = -18'sd826;  rom[ 706] =  18'sd64;  rom[ 707] =  18'sd997;
        rom[ 708] =  18'sd1972;  rom[ 709] =  18'sd2988;  rom[ 710] =  18'sd4045;  rom[ 711] =  18'sd5141;
        rom[ 712] =  18'sd6276;  rom[ 713] =  18'sd7449;  rom[ 714] =  18'sd8658;  rom[ 715] =  18'sd9901;
        rom[ 716] =  18'sd11179;  rom[ 717] =  18'sd12488;  rom[ 718] =  18'sd13828;  rom[ 719] =  18'sd15196;
        rom[ 720] =  18'sd16592;  rom[ 721] =  18'sd18012;  rom[ 722] =  18'sd19455;  rom[ 723] =  18'sd20919;
        rom[ 724] =  18'sd22402;  rom[ 725] =  18'sd23901;  rom[ 726] =  18'sd25414;  rom[ 727] =  18'sd26939;
        rom[ 728] =  18'sd28473;  rom[ 729] =  18'sd30014;  rom[ 730] =  18'sd31560;  rom[ 731] =  18'sd33107;
        rom[ 732] =  18'sd34653;  rom[ 733] =  18'sd36196;  rom[ 734] =  18'sd37732;  rom[ 735] =  18'sd39259;
        rom[ 736] =  18'sd40775;  rom[ 737] =  18'sd42277;  rom[ 738] =  18'sd43761;  rom[ 739] =  18'sd45225;
        rom[ 740] =  18'sd46667;  rom[ 741] =  18'sd48084;  rom[ 742] =  18'sd49473;  rom[ 743] =  18'sd50832;
        rom[ 744] =  18'sd52157;  rom[ 745] =  18'sd53447;  rom[ 746] =  18'sd54699;  rom[ 747] =  18'sd55910;
        rom[ 748] =  18'sd57078;  rom[ 749] =  18'sd58202;  rom[ 750] =  18'sd59278;  rom[ 751] =  18'sd60305;
        rom[ 752] =  18'sd61280;  rom[ 753] =  18'sd62202;  rom[ 754] =  18'sd63068;  rom[ 755] =  18'sd63878;
        rom[ 756] =  18'sd64629;  rom[ 757] =  18'sd65319;  rom[ 758] =  18'sd65949;  rom[ 759] =  18'sd66515;
        rom[ 760] =  18'sd67018;  rom[ 761] =  18'sd67456;  rom[ 762] =  18'sd67828;  rom[ 763] =  18'sd68133;
        rom[ 764] =  18'sd68371;  rom[ 765] =  18'sd68542;  rom[ 766] =  18'sd68644;  rom[ 767] =  18'sd68674;
        // set 3, 1.2 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^22
        rom[ 768] =  18'sd10;  rom[ 769] =  18'sd11;  rom[ 770] =  18'sd11;  rom[ 771] =  18'sd11;
        rom[ 772] =  18'sd11;  rom[ 773] =  18'sd11;  rom[ 774] =  18'sd10;  rom[ 775] =  18'sd9;
        rom[ 776] =  18'sd7;  rom[ 777] =  18'sd5;  rom[ 778] =  18'sd3;  rom[ 779] =  18'sd0;
        rom[ 780] = -18'sd4;  rom[ 781] = -18'sd8;  rom[ 782] = -18'sd12;  rom[ 783] = -18'sd17;
        rom[ 784] = -18'sd23;  rom[ 785] = -18'sd29;  rom[ 786] = -18'sd35;  rom[ 787] = -18'sd42;
        rom[ 788] = -18'sd50;  rom[ 789] = -18'sd58;  rom[ 790] = -18'sd66;  rom[ 791] = -18'sd75;
        rom[ 792] = -18'sd84;  rom[ 793] = -18'sd93;  rom[ 794] = -18'sd102;  rom[ 795] = -18'sd111;
        rom[ 796] = -18'sd120;  rom[ 797] = -18'sd129;  rom[ 798] = -18'sd138;  rom[ 799] = -18'sd146;
        rom[ 800] = -18'sd154;  rom[ 801] = -18'sd161;  rom[ 802] = -18'sd167;  rom[ 803] = -18'sd172;
        rom[ 804] = -18'sd176;  rom[ 805] = -18'sd179;  rom[ 806] = -18'sd180;  rom[ 807] = -18'sd180;
        rom[ 808] = -18'sd178;  rom[ 809] = -18'sd175;  rom[ 810] = -18'sd169;  rom[ 811] = -18'sd161;
        rom[ 812] = -18'sd152;  rom[ 813] = -18'sd140;  rom[ 814] = -18'sd125;  rom[ 815] = -18'sd109;
        rom[ 816] = -18'sd90;  rom[ 817] = -18'sd68;  rom[ 818] = -18'sd44;  rom[ 819] = -18'sd17;
        rom[ 820] =  18'sd12;  rom[ 821] =  18'sd43;  rom[ 822] =  18'sd77;  rom[ 823] =  18'sd113;
        rom[ 824] =  18'sd151;  rom[ 825] =  18'sd191;  rom[ 826] =  18'sd232;  rom[ 827] =  18'sd275;
        rom[ 828] =  18'sd319;  rom[ 829] =  18'sd364;  rom[ 830] =  18'sd410;  rom[ 831] =  18'sd455;
        rom[ 832] =  18'sd501;  rom[ 833] =  18'sd546;  rom[ 834] =  18'sd591;  rom[ 835] =  18'sd634;
        rom[ 836] =  18'sd675;  rom[ 837] =  18'sd714;  rom[ 838] =  18'sd750;  rom[ 839] =  18'sd783;
        rom[ 840] =  18'sd813;  rom[ 841] =  18'sd838;  rom[ 842] =  18'sd859;  rom[ 843] =  18'sd874;
        rom[ 844] =  18'sd884;  rom[ 845] =  18'sd888;  rom[ 846] =  18'sd886;  rom[ 847] =  18'sd877;
        rom[ 848] =  18'sd860;  rom[ 849] =  18'sd837;  rom[ 850] =  18'sd805;  rom[ 851] =  18'sd765;
        rom[ 852] =  18'sd717;  rom[ 853] =  18'sd661;  rom[ 854] =  18'sd596;  rom[ 855] =  18'sd523;
        rom[ 856] =  18'sd441;  rom[ 857] =  18'sd351;  rom[ 858] =  18'sd253;  rom[ 859] =  18'sd147;
        rom[ 860] =  18'sd33;  rom[ 861] = -18'sd88;  rom[ 862] = -18'sd215;  rom[ 863] = -18'sd349;
        rom[ 864] = -18'sd488;  rom[ 865] = -18'sd631;  rom[ 866] = -18'sd779;  rom[ 867] = -18'sd930;
        rom[ 868] = -18'sd1083;  rom[ 869] = -18'sd1237;  rom[ 870] = -18'sd1391;  rom[ 871] = -18'sd1544;
        rom[ 872] = -18'sd1694;  rom[ 873] = -18'sd1842;  rom[ 874] = -18'sd1984;  rom[ 875] = -18'sd2120;
        rom[ 876] = -18'sd2250;  rom[ 877] = -18'sd2370;  rom[ 878] = -18'sd2480;  rom[ 879] = -18'sd2580;
        rom[ 880] = -18'sd2666;  rom[ 881] = -18'sd2739;  rom[ 882] = -18'sd2796;  rom[ 883] = -18'sd2837;
        rom[ 884] = -18'sd2860;  rom[ 885] = -18'sd2865;  rom[ 886] = -18'sd2851;  rom[ 887] = -18'sd2816;
        rom[ 888] = -18'sd2759;  rom[ 889] = -18'sd2681;  rom[ 890] = -18'sd2580;  rom[ 891] = -18'sd2457;
        rom[ 892] = -18'sd2310;  rom[ 893] = -18'sd2140;  rom[ 894] = -18'sd1947;  rom[ 895] = -18'sd1731;
        rom[ 896] = -18'sd1492;  rom[ 897] = -18'sd1232;  rom[ 898] = -18'sd950;  rom[ 899] = -18'sd648;
        rom[ 900] = -18'sd326;  rom[ 901] =  18'sd13;  rom[ 902] =  18'sd369;  rom[ 903] =  18'sd739;
        rom[ 904] =  18'sd1122;  rom[ 905] =  18'sd1517;  rom[ 906] =  18'sd1919;  rom[ 907] =  18'sd2328;
        rom[ 908] =  18'sd2741;  rom[ 909] =  18'sd3155;  rom[ 910] =  18'sd3567;  rom[ 911] =  18'sd3974;
        rom[ 912] =  18'sd4374;  rom[ 913] =  18'sd4763;  rom[ 914] =  18'sd5139;  rom[ 915] =  18'sd5497;
        rom[ 916] =  18'sd5836;  rom[ 917] =  18'sd6151;  rom[ 918] =  18'sd6439;  rom[ 919] =  18'sd6698;
        rom[ 920] =  18'sd6924;  rom[ 921] =  18'sd7114;  rom[ 922] =  18'sd7266;  rom[ 923] =  18'sd7376;
        rom[ 924] =  18'sd7442;  rom[ 925] =  18'sd7462;  rom[ 926] =  18'sd7433;  rom[ 927] =  18'sd7354;
        rom[ 928] =  18'sd7222;  rom[ 929] =  18'sd7037;  rom[ 930] =  18'sd6797;  rom[ 931] =  18'sd6501;
        rom[ 932] =  18'sd6149;  rom[ 933] =  18'sd5740;  rom[ 934] =  18'sd5276;  rom[ 935] =  18'sd4756;
        rom[ 936] =  18'sd4181;  rom[ 937] =  18'sd3553;  rom[ 938] =  18'sd2873;  rom[ 939] =  18'sd2144;
        rom[ 940] =  18'sd1369;  rom[ 941] =  18'sd550;  rom[ 942] = -18'sd310;  rom[ 943] = -18'sd1206;
        rom[ 944] = -18'sd2135;  rom[ 945] = -18'sd3091;  rom[ 946] = -18'sd4070;  rom[ 947] = -18'sd5067;
        rom[ 948] = -18'sd6075;  rom[ 949] = -18'sd7089;  rom[ 950] = -18'sd8104;  rom[ 951] = -18'sd9111;
        rom[ 952] = -18'sd10105;  rom[ 953] = -18'sd11078;  rom[ 954] = -18'sd12023;  rom[ 955] = -18'sd12934;
        rom[ 956] = -18'sd13803;  rom[ 957] = -18'sd14622;  rom[ 958] = -18'sd15383;  rom[ 959] = -18'sd16080;
        rom[ 960] = -18'sd16706;  rom[ 961] = -18'sd17252;  rom[ 962] = -18'sd17711;  rom[ 963] = -18'sd18078;
        rom[ 964] = -18'sd18344;  rom[ 965] = -18'sd18504;  rom[ 966] = -18'sd18551;  rom[ 967] = -18'sd18480;
        rom[ 968] = -18'sd18284;  rom[ 969] = -18'sd17960;  rom[ 970] = -18'sd17503;  rom[ 971] = -18'sd16908;
        rom[ 972] = -18'sd16173;  rom[ 973] = -18'sd15294;  rom[ 974] = -18'sd14269;  rom[ 975] = -18'sd13096;
        rom[ 976] = -18'sd11775;  rom[ 977] = -18'sd10305;  rom[ 978] = -18'sd8686;  rom[ 979] = -18'sd6919;
        rom[ 980] = -18'sd5007;  rom[ 981] = -18'sd2951;  rom[ 982] = -18'sd755;  rom[ 983] =  18'sd1578;
        rom[ 984] =  18'sd4042;  rom[ 985] =  18'sd6632;  rom[ 986] =  18'sd9343;  rom[ 987] =  18'sd12168;
        rom[ 988] =  18'sd15099;  rom[ 989] =  18'sd18129;  rom[ 990] =  18'sd21249;  rom[ 991] =  18'sd24450;
        rom[ 992] =  18'sd27721;  rom[ 993] =  18'sd31054;  rom[ 994] =  18'sd34438;  rom[ 995] =  18'sd37861;
        rom[ 996] =  18'sd41312;  rom[ 997] =  18'sd44780;  rom[ 998] =  18'sd48253;  rom[ 999] =  18'sd51718;
        rom[1000] =  18'sd55163;  rom[1001] =  18'sd58577;  rom[1002] =  18'sd61946;  rom[1003] =  18'sd65258;
        rom[1004] =  18'sd68501;  rom[1005] =  18'sd71663;  rom[1006] =  18'sd74732;  rom[1007] =  18'sd77695;
        rom[1008] =  18'sd80543;  rom[1009] =  18'sd83264;  rom[1010] =  18'sd85847;  rom[1011] =  18'sd88282;
        rom[1012] =  18'sd90559;  rom[1013] =  18'sd92671;  rom[1014] =  18'sd94607;  rom[1015] =  18'sd96361;
        rom[1016] =  18'sd97926;  rom[1017] =  18'sd99295;  rom[1018] =  18'sd100463;  rom[1019] =  18'sd101425;
        rom[1020] =  18'sd102177;  rom[1021] =  18'sd102717;  rom[1022] =  18'sd103042;  rom[1023] =  18'sd103164;
        // set 4, 1.8 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^21
        rom[1024] = -18'sd7;  rom[1025] = -18'sd7;  rom[1026] = -18'sd8;  rom[1027] = -18'sd8;
        rom[1028] = -18'sd8;  rom[1029] = -18'sd8;  rom[1030] = -18'sd7;  rom[1031] = -18'sd6;
        rom[1032] = -18'sd5;  rom[1033] = -18'sd4;  rom[1034] = -18'sd2;  rom[1035] =  18'sd1;
        rom[1036] =  18'sd3;  rom[1037] =  18'sd6;  rom[1038] =  18'sd9;  rom[1039] =  18'sd13;
        rom[1040] =  18'sd17;  rom[1041] =  18'sd21;  rom[1042] =  18'sd25;  rom[1043] =  18'sd30;
        rom[1044] =  18'sd34;  rom[1045] =  18'sd38;  rom[1046] =  18'sd42;  rom[1047] =  18'sd46;
        rom[1048] =  18'sd49;  rom[1049] =  18'sd52;  rom[1050] =  18'sd55;  rom[1051] =  18'sd56;
        rom[1052] =  18'sd57;  rom[1053] =  18'sd56;  rom[1054] =  18'sd55;  rom[1055] =  18'sd52;
        rom[1056] =  18'sd49;  rom[1057] =  18'sd44;  rom[1058] =  18'sd37;  rom[1059] =  18'sd30;
        rom[1060] =  18'sd21;  rom[1061] =  18'sd10;  rom[1062] = -18'sd1;  rom[1063] = -18'sd14;
        rom[1064] = -18'sd27;  rom[1065] = -18'sd42;  rom[1066] = -18'sd57;  rom[1067] = -18'sd73;
        rom[1068] = -18'sd89;  rom[1069] = -18'sd105;  rom[1070] = -18'sd121;  rom[1071] = -18'sd136;
        rom[1072] = -18'sd150;  rom[1073] = -18'sd163;  rom[1074] = -18'sd175;  rom[1075] = -18'sd185;
        rom[1076] = -18'sd193;  rom[1077] = -18'sd198;  rom[1078] = -18'sd200;  rom[1079] = -18'sd200;
        rom[1080] = -18'sd196;  rom[1081] = -18'sd189;  rom[1082] = -18'sd178;  rom[1083] = -18'sd164;
        rom[1084] = -18'sd146;  rom[1085] = -18'sd124;  rom[1086] = -18'sd98;  rom[1087] = -18'sd69;
        rom[1088] = -18'sd37;  rom[1089] = -18'sd2;  rom[1090] =  18'sd36;  rom[1091] =  18'sd77;
        rom[1092] =  18'sd119;  rom[1093] =  18'sd162;  rom[1094] =  18'sd206;  rom[1095] =  18'sd250;
        rom[1096] =  18'sd293;  rom[1097] =  18'sd335;  rom[1098] =  18'sd375;  rom[1099] =  18'sd411;
        rom[1100] =  18'sd444;  rom[1101] =  18'sd473;  rom[1102] =  18'sd496;  rom[1103] =  18'sd513;
        rom[1104] =  18'sd523;  rom[1105] =  18'sd527;  rom[1106] =  18'sd522;  rom[1107] =  18'sd510;
        rom[1108] =  18'sd488;  rom[1109] =  18'sd459;  rom[1110] =  18'sd420;  rom[1111] =  18'sd373;
        rom[1112] =  18'sd317;  rom[1113] =  18'sd252;  rom[1114] =  18'sd180;  rom[1115] =  18'sd101;
        rom[1116] =  18'sd16;  rom[1117] = -18'sd75;  rom[1118] = -18'sd170;  rom[1119] = -18'sd269;
        rom[1120] = -18'sd369;  rom[1121] = -18'sd470;  rom[1122] = -18'sd569;  rom[1123] = -18'sd667;
        rom[1124] = -18'sd760;  rom[1125] = -18'sd847;  rom[1126] = -18'sd926;  rom[1127] = -18'sd997;
        rom[1128] = -18'sd1057;  rom[1129] = -18'sd1104;  rom[1130] = -18'sd1139;  rom[1131] = -18'sd1158;
        rom[1132] = -18'sd1162;  rom[1133] = -18'sd1148;  rom[1134] = -18'sd1117;  rom[1135] = -18'sd1069;
        rom[1136] = -18'sd1002;  rom[1137] = -18'sd916;  rom[1138] = -18'sd813;  rom[1139] = -18'sd692;
        rom[1140] = -18'sd555;  rom[1141] = -18'sd403;  rom[1142] = -18'sd237;  rom[1143] = -18'sd59;
        rom[1144] =  18'sd129;  rom[1145] =  18'sd326;  rom[1146] =  18'sd527;  rom[1147] =  18'sd731;
        rom[1148] =  18'sd935;  rom[1149] =  18'sd1136;  rom[1150] =  18'sd1330;  rom[1151] =  18'sd1515;
        rom[1152] =  18'sd1687;  rom[1153] =  18'sd1844;  rom[1154] =  18'sd1981;  rom[1155] =  18'sd2097;
        rom[1156] =  18'sd2189;  rom[1157] =  18'sd2253;  rom[1158] =  18'sd2288;  rom[1159] =  18'sd2292;
        rom[1160] =  18'sd2263;  rom[1161] =  18'sd2200;  rom[1162] =  18'sd2103;  rom[1163] =  18'sd1971;
        rom[1164] =  18'sd1804;  rom[1165] =  18'sd1604;  rom[1166] =  18'sd1371;  rom[1167] =  18'sd1107;
        rom[1168] =  18'sd816;  rom[1169] =  18'sd499;  rom[1170] =  18'sd161;  rom[1171] = -18'sd196;
        rom[1172] = -18'sd566;  rom[1173] = -18'sd946;  rom[1174] = -18'sd1329;  rom[1175] = -18'sd1711;
        rom[1176] = -18'sd2086;  rom[1177] = -18'sd2449;  rom[1178] = -18'sd2793;  rom[1179] = -18'sd3113;
        rom[1180] = -18'sd3403;  rom[1181] = -18'sd3658;  rom[1182] = -18'sd3872;  rom[1183] = -18'sd4041;
        rom[1184] = -18'sd4160;  rom[1185] = -18'sd4226;  rom[1186] = -18'sd4234;  rom[1187] = -18'sd4183;
        rom[1188] = -18'sd4070;  rom[1189] = -18'sd3894;  rom[1190] = -18'sd3655;  rom[1191] = -18'sd3354;
        rom[1192] = -18'sd2991;  rom[1193] = -18'sd2570;  rom[1194] = -18'sd2093;  rom[1195] = -18'sd1566;
        rom[1196] = -18'sd993;  rom[1197] = -18'sd380;  rom[1198] =  18'sd266;  rom[1199] =  18'sd937;
        rom[1200] =  18'sd1625;  rom[1201] =  18'sd2320;  rom[1202] =  18'sd3015;  rom[1203] =  18'sd3698;
        rom[1204] =  18'sd4359;  rom[1205] =  18'sd4989;  rom[1206] =  18'sd5577;  rom[1207] =  18'sd6113;
        rom[1208] =  18'sd6588;  rom[1209] =  18'sd6990;  rom[1210] =  18'sd7313;  rom[1211] =  18'sd7547;
        rom[1212] =  18'sd7686;  rom[1213] =  18'sd7722;  rom[1214] =  18'sd7651;  rom[1215] =  18'sd7468;
        rom[1216] =  18'sd7172;  rom[1217] =  18'sd6760;  rom[1218] =  18'sd6233;  rom[1219] =  18'sd5593;
        rom[1220] =  18'sd4843;  rom[1221] =  18'sd3988;  rom[1222] =  18'sd3035;  rom[1223] =  18'sd1992;
        rom[1224] =  18'sd868;  rom[1225] = -18'sd324;  rom[1226] = -18'sd1572;  rom[1227] = -18'sd2861;
        rom[1228] = -18'sd4177;  rom[1229] = -18'sd5503;  rom[1230] = -18'sd6821;  rom[1231] = -18'sd8113;
        rom[1232] = -18'sd9360;  rom[1233] = -18'sd10543;  rom[1234] = -18'sd11643;  rom[1235] = -18'sd12640;
        rom[1236] = -18'sd13516;  rom[1237] = -18'sd14251;  rom[1238] = -18'sd14827;  rom[1239] = -18'sd15228;
        rom[1240] = -18'sd15438;  rom[1241] = -18'sd15442;  rom[1242] = -18'sd15227;  rom[1243] = -18'sd14783;
        rom[1244] = -18'sd14099;  rom[1245] = -18'sd13170;  rom[1246] = -18'sd11989;  rom[1247] = -18'sd10555;
        rom[1248] = -18'sd8868;  rom[1249] = -18'sd6929;  rom[1250] = -18'sd4743;  rom[1251] = -18'sd2319;
        rom[1252] =  18'sd335;  rom[1253] =  18'sd3205;  rom[1254] =  18'sd6276;  rom[1255] =  18'sd9533;
        rom[1256] =  18'sd12954;  rom[1257] =  18'sd16518;  rom[1258] =  18'sd20203;  rom[1259] =  18'sd23983;
        rom[1260] =  18'sd27833;  rom[1261] =  18'sd31723;  rom[1262] =  18'sd35626;  rom[1263] =  18'sd39513;
        rom[1264] =  18'sd43353;  rom[1265] =  18'sd47118;  rom[1266] =  18'sd50777;  rom[1267] =  18'sd54301;
        rom[1268] =  18'sd57663;  rom[1269] =  18'sd60835;  rom[1270] =  18'sd63790;  rom[1271] =  18'sd66505;
        rom[1272] =  18'sd68957;  rom[1273] =  18'sd71126;  rom[1274] =  18'sd72992;  rom[1275] =  18'sd74542;
        rom[1276] =  18'sd75761;  rom[1277] =  18'sd76639;  rom[1278] =  18'sd77169;  rom[1279] =  18'sd77356;
        // set 5, 2.4 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^21
        rom[1280] =  18'sd7;  rom[1281] =  18'sd8;  rom[1282] =  18'sd9;  rom[1283] =  18'sd9;
        rom[1284] =  18'sd10;  rom[1285] =  18'sd10;  rom[1286] =  18'sd9;  rom[1287] =  18'sd8;
        rom[1288] =  18'sd7;  rom[1289] =  18'sd5;  rom[1290] =  18'sd2;  rom[1291] =  18'sd0;
        rom[1292] = -18'sd4;  rom[1293] = -18'sd8;  rom[1294] = -18'sd12;  rom[1295] = -18'sd17;
        rom[1296] = -18'sd21;  rom[1297] = -18'sd26;  rom[1298] = -18'sd30;  rom[1299] = -18'sd35;
        rom[1300] = -18'sd38;  rom[1301] = -18'sd41;  rom[1302] = -18'sd43;  rom[1303] = -18'sd45;
        rom[1304] = -18'sd44;  rom[1305] = -18'sd43;  rom[1306] = -18'sd40;  rom[1307] = -18'sd36;
        rom[1308] = -18'sd29;  rom[1309] = -18'sd22;  rom[1310] = -18'sd13;  rom[1311] = -18'sd2;
        rom[1312] =  18'sd10;  rom[1313] =  18'sd22;  rom[1314] =  18'sd36;  rom[1315] =  18'sd50;
        rom[1316] =  18'sd64;  rom[1317] =  18'sd78;  rom[1318] =  18'sd90;  rom[1319] =  18'sd102;
        rom[1320] =  18'sd112;  rom[1321] =  18'sd120;  rom[1322] =  18'sd125;  rom[1323] =  18'sd127;
        rom[1324] =  18'sd126;  rom[1325] =  18'sd122;  rom[1326] =  18'sd113;  rom[1327] =  18'sd101;
        rom[1328] =  18'sd85;  rom[1329] =  18'sd65;  rom[1330] =  18'sd42;  rom[1331] =  18'sd15;
        rom[1332] = -18'sd14;  rom[1333] = -18'sd45;  rom[1334] = -18'sd78;  rom[1335] = -18'sd111;
        rom[1336] = -18'sd144;  rom[1337] = -18'sd176;  rom[1338] = -18'sd206;  rom[1339] = -18'sd233;
        rom[1340] = -18'sd256;  rom[1341] = -18'sd274;  rom[1342] = -18'sd285;  rom[1343] = -18'sd291;
        rom[1344] = -18'sd288;  rom[1345] = -18'sd279;  rom[1346] = -18'sd261;  rom[1347] = -18'sd235;
        rom[1348] = -18'sd200;  rom[1349] = -18'sd158;  rom[1350] = -18'sd109;  rom[1351] = -18'sd54;
        rom[1352] =  18'sd7;  rom[1353] =  18'sd72;  rom[1354] =  18'sd140;  rom[1355] =  18'sd209;
        rom[1356] =  18'sd277;  rom[1357] =  18'sd342;  rom[1358] =  18'sd404;  rom[1359] =  18'sd458;
        rom[1360] =  18'sd505;  rom[1361] =  18'sd541;  rom[1362] =  18'sd566;  rom[1363] =  18'sd578;
        rom[1364] =  18'sd576;  rom[1365] =  18'sd559;  rom[1366] =  18'sd526;  rom[1367] =  18'sd478;
        rom[1368] =  18'sd415;  rom[1369] =  18'sd336;  rom[1370] =  18'sd245;  rom[1371] =  18'sd141;
        rom[1372] =  18'sd27;  rom[1373] = -18'sd94;  rom[1374] = -18'sd220;  rom[1375] = -18'sd348;
        rom[1376] = -18'sd475;  rom[1377] = -18'sd597;  rom[1378] = -18'sd711;  rom[1379] = -18'sd814;
        rom[1380] = -18'sd902;  rom[1381] = -18'sd972;  rom[1382] = -18'sd1021;  rom[1383] = -18'sd1047;
        rom[1384] = -18'sd1049;  rom[1385] = -18'sd1023;  rom[1386] = -18'sd970;  rom[1387] = -18'sd890;
        rom[1388] = -18'sd782;  rom[1389] = -18'sd649;  rom[1390] = -18'sd493;  rom[1391] = -18'sd315;
        rom[1392] = -18'sd119;  rom[1393] =  18'sd90;  rom[1394] =  18'sd307;  rom[1395] =  18'sd529;
        rom[1396] =  18'sd749;  rom[1397] =  18'sd962;  rom[1398] =  18'sd1161;  rom[1399] =  18'sd1342;
        rom[1400] =  18'sd1498;  rom[1401] =  18'sd1625;  rom[1402] =  18'sd1717;  rom[1403] =  18'sd1771;
        rom[1404] =  18'sd1783;  rom[1405] =  18'sd1751;  rom[1406] =  18'sd1673;  rom[1407] =  18'sd1549;
        rom[1408] =  18'sd1381;  rom[1409] =  18'sd1169;  rom[1410] =  18'sd917;  rom[1411] =  18'sd630;
        rom[1412] =  18'sd314;  rom[1413] = -18'sd26;  rom[1414] = -18'sd382;  rom[1415] = -18'sd746;
        rom[1416] = -18'sd1108;  rom[1417] = -18'sd1460;  rom[1418] = -18'sd1793;  rom[1419] = -18'sd2096;
        rom[1420] = -18'sd2361;  rom[1421] = -18'sd2580;  rom[1422] = -18'sd2745;  rom[1423] = -18'sd2850;
        rom[1424] = -18'sd2888;  rom[1425] = -18'sd2855;  rom[1426] = -18'sd2750;  rom[1427] = -18'sd2572;
        rom[1428] = -18'sd2321;  rom[1429] = -18'sd2001;  rom[1430] = -18'sd1616;  rom[1431] = -18'sd1173;
        rom[1432] = -18'sd681;  rom[1433] = -18'sd150;  rom[1434] =  18'sd410;  rom[1435] =  18'sd984;
        rom[1436] =  18'sd1561;  rom[1437] =  18'sd2124;  rom[1438] =  18'sd2661;  rom[1439] =  18'sd3155;
        rom[1440] =  18'sd3593;  rom[1441] =  18'sd3962;  rom[1442] =  18'sd4249;  rom[1443] =  18'sd4443;
        rom[1444] =  18'sd4536;  rom[1445] =  18'sd4520;  rom[1446] =  18'sd4391;  rom[1447] =  18'sd4148;
        rom[1448] =  18'sd3791;  rom[1449] =  18'sd3323;  rom[1450] =  18'sd2752;  rom[1451] =  18'sd2088;
        rom[1452] =  18'sd1342;  rom[1453] =  18'sd529;  rom[1454] = -18'sd333;  rom[1455] = -18'sd1226;
        rom[1456] = -18'sd2128;  rom[1457] = -18'sd3018;  rom[1458] = -18'sd3874;  rom[1459] = -18'sd4672;
        rom[1460] = -18'sd5391;  rom[1461] = -18'sd6008;  rom[1462] = -18'sd6505;  rom[1463] = -18'sd6863;
        rom[1464] = -18'sd7068;  rom[1465] = -18'sd7107;  rom[1466] = -18'sd6971;  rom[1467] = -18'sd6657;
        rom[1468] = -18'sd6163;  rom[1469] = -18'sd5494;  rom[1470] = -18'sd4657;  rom[1471] = -18'sd3666;
        rom[1472] = -18'sd2537;  rom[1473] = -18'sd1292;  rom[1474] =  18'sd45;  rom[1475] =  18'sd1445;
        rom[1476] =  18'sd2876;  rom[1477] =  18'sd4307;  rom[1478] =  18'sd5700;  rom[1479] =  18'sd7022;
        rom[1480] =  18'sd8235;  rom[1481] =  18'sd9305;  rom[1482] =  18'sd10199;  rom[1483] =  18'sd10886;
        rom[1484] =  18'sd11338;  rom[1485] =  18'sd11533;  rom[1486] =  18'sd11452;  rom[1487] =  18'sd11083;
        rom[1488] =  18'sd10419;  rom[1489] =  18'sd9460;  rom[1490] =  18'sd8213;  rom[1491] =  18'sd6691;
        rom[1492] =  18'sd4917;  rom[1493] =  18'sd2918;  rom[1494] =  18'sd728;  rom[1495] = -18'sd1611;
        rom[1496] = -18'sd4052;  rom[1497] = -18'sd6543;  rom[1498] = -18'sd9029;  rom[1499] = -18'sd11450;
        rom[1500] = -18'sd13746;  rom[1501] = -18'sd15852;  rom[1502] = -18'sd17708;  rom[1503] = -18'sd19252;
        rom[1504] = -18'sd20426;  rom[1505] = -18'sd21177;  rom[1506] = -18'sd21454;  rom[1507] = -18'sd21216;
        rom[1508] = -18'sd20425;  rom[1509] = -18'sd19056;  rom[1510] = -18'sd17090;  rom[1511] = -18'sd14517;
        rom[1512] = -18'sd11340;  rom[1513] = -18'sd7571;  rom[1514] = -18'sd3231;  rom[1515] =  18'sd1647;
        rom[1516] =  18'sd7018;  rom[1517] =  18'sd12832;  rom[1518] =  18'sd19025;  rom[1519] =  18'sd25528;
        rom[1520] =  18'sd32265;  rom[1521] =  18'sd39152;  rom[1522] =  18'sd46103;  rom[1523] =  18'sd53025;
        rom[1524] =  18'sd59829;  rom[1525] =  18'sd66420;  rom[1526] =  18'sd72708;  rom[1527] =  18'sd78604;
        rom[1528] =  18'sd84026;  rom[1529] =  18'sd88896;  rom[1530] =  18'sd93144;  rom[1531] =  18'sd96708;
        rom[1532] =  18'sd99535;  rom[1533] =  18'sd101585;  rom[1534] =  18'sd102827;  rom[1535] =  18'sd103246;
        // set 6, 3 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^21
        rom[1536] = -18'sd6;  rom[1537] = -18'sd8;  rom[1538] = -18'sd9;  rom[1539] = -18'sd10;
        rom[1540] = -18'sd11;  rom[1541] = -18'sd11;  rom[1542] = -18'sd11;  rom[1543] = -18'sd10;
        rom[1544] = -18'sd8;  rom[1545] = -18'sd6;  rom[1546] = -18'sd3;  rom[1547] =  18'sd0;
        rom[1548] =  18'sd5;  rom[1549] =  18'sd10;  rom[1550] =  18'sd15;  rom[1551] =  18'sd20;
        rom[1552] =  18'sd25;  rom[1553] =  18'sd30;  rom[1554] =  18'sd33;  rom[1555] =  18'sd36;
        rom[1556] =  18'sd38;  rom[1557] =  18'sd38;  rom[1558] =  18'sd37;  rom[1559] =  18'sd34;
        rom[1560] =  18'sd29;  rom[1561] =  18'sd21;  rom[1562] =  18'sd12;  rom[1563] =  18'sd2;
        rom[1564] = -18'sd10;  rom[1565] = -18'sd23;  rom[1566] = -18'sd36;  rom[1567] = -18'sd50;
        rom[1568] = -18'sd62;  rom[1569] = -18'sd74;  rom[1570] = -18'sd83;  rom[1571] = -18'sd90;
        rom[1572] = -18'sd94;  rom[1573] = -18'sd94;  rom[1574] = -18'sd91;  rom[1575] = -18'sd83;
        rom[1576] = -18'sd71;  rom[1577] = -18'sd55;  rom[1578] = -18'sd35;  rom[1579] = -18'sd11;
        rom[1580] =  18'sd15;  rom[1581] =  18'sd43;  rom[1582] =  18'sd71;  rom[1583] =  18'sd100;
        rom[1584] =  18'sd127;  rom[1585] =  18'sd151;  rom[1586] =  18'sd170;  rom[1587] =  18'sd185;
        rom[1588] =  18'sd193;  rom[1589] =  18'sd194;  rom[1590] =  18'sd188;  rom[1591] =  18'sd173;
        rom[1592] =  18'sd150;  rom[1593] =  18'sd118;  rom[1594] =  18'sd80;  rom[1595] =  18'sd35;
        rom[1596] = -18'sd15;  rom[1597] = -18'sd68;  rom[1598] = -18'sd122;  rom[1599] = -18'sd176;
        rom[1600] = -18'sd227;  rom[1601] = -18'sd273;  rom[1602] = -18'sd311;  rom[1603] = -18'sd339;
        rom[1604] = -18'sd356;  rom[1605] = -18'sd360;  rom[1606] = -18'sd349;  rom[1607] = -18'sd324;
        rom[1608] = -18'sd285;  rom[1609] = -18'sd231;  rom[1610] = -18'sd164;  rom[1611] = -18'sd86;
        rom[1612] =  18'sd1;  rom[1613] =  18'sd94;  rom[1614] =  18'sd189;  rom[1615] =  18'sd284;
        rom[1616] =  18'sd373;  rom[1617] =  18'sd453;  rom[1618] =  18'sd521;  rom[1619] =  18'sd572;
        rom[1620] =  18'sd605;  rom[1621] =  18'sd615;  rom[1622] =  18'sd602;  rom[1623] =  18'sd564;
        rom[1624] =  18'sd501;  rom[1625] =  18'sd415;  rom[1626] =  18'sd307;  rom[1627] =  18'sd180;
        rom[1628] =  18'sd39;  rom[1629] = -18'sd113;  rom[1630] = -18'sd269;  rom[1631] = -18'sd425;
        rom[1632] = -18'sd572;  rom[1633] = -18'sd706;  rom[1634] = -18'sd821;  rom[1635] = -18'sd909;
        rom[1636] = -18'sd968;  rom[1637] = -18'sd991;  rom[1638] = -18'sd977;  rom[1639] = -18'sd925;
        rom[1640] = -18'sd833;  rom[1641] = -18'sd703;  rom[1642] = -18'sd538;  rom[1643] = -18'sd344;
        rom[1644] = -18'sd125;  rom[1645] =  18'sd111;  rom[1646] =  18'sd355;  rom[1647] =  18'sd599;
        rom[1648] =  18'sd833;  rom[1649] =  18'sd1047;  rom[1650] =  18'sd1231;  rom[1651] =  18'sd1377;
        rom[1652] =  18'sd1477;  rom[1653] =  18'sd1525;  rom[1654] =  18'sd1517;  rom[1655] =  18'sd1448;
        rom[1656] =  18'sd1320;  rom[1657] =  18'sd1135;  rom[1658] =  18'sd895;  rom[1659] =  18'sd608;
        rom[1660] =  18'sd284;  rom[1661] = -18'sd68;  rom[1662] = -18'sd436;  rom[1663] = -18'sd804;
        rom[1664] = -18'sd1160;  rom[1665] = -18'sd1489;  rom[1666] = -18'sd1776;  rom[1667] = -18'sd2008;
        rom[1668] = -18'sd2174;  rom[1669] = -18'sd2264;  rom[1670] = -18'sd2271;  rom[1671] = -18'sd2190;
        rom[1672] = -18'sd2020;  rom[1673] = -18'sd1765;  rom[1674] = -18'sd1428;  rom[1675] = -18'sd1021;
        rom[1676] = -18'sd554;  rom[1677] = -18'sd45;  rom[1678] =  18'sd491;  rom[1679] =  18'sd1033;
        rom[1680] =  18'sd1560;  rom[1681] =  18'sd2052;  rom[1682] =  18'sd2487;  rom[1683] =  18'sd2846;
        rom[1684] =  18'sd3113;  rom[1685] =  18'sd3271;  rom[1686] =  18'sd3311;  rom[1687] =  18'sd3225;
        rom[1688] =  18'sd3010;  rom[1689] =  18'sd2670;  rom[1690] =  18'sd2212;  rom[1691] =  18'sd1647;
        rom[1692] =  18'sd993;  rom[1693] =  18'sd271;  rom[1694] = -18'sd494;  rom[1695] = -18'sd1274;
        rom[1696] = -18'sd2040;  rom[1697] = -18'sd2763;  rom[1698] = -18'sd3411;  rom[1699] = -18'sd3957;
        rom[1700] = -18'sd4376;  rom[1701] = -18'sd4645;  rom[1702] = -18'sd4747;  rom[1703] = -18'sd4671;
        rom[1704] = -18'sd4413;  rom[1705] = -18'sd3973;  rom[1706] = -18'sd3360;  rom[1707] = -18'sd2591;
        rom[1708] = -18'sd1688;  rom[1709] = -18'sd679;  rom[1710] =  18'sd401;  rom[1711] =  18'sd1514;
        rom[1712] =  18'sd2618;  rom[1713] =  18'sd3672;  rom[1714] =  18'sd4632;  rom[1715] =  18'sd5458;
        rom[1716] =  18'sd6111;  rom[1717] =  18'sd6559;  rom[1718] =  18'sd6775;  rom[1719] =  18'sd6740;
        rom[1720] =  18'sd6445;  rom[1721] =  18'sd5889;  rom[1722] =  18'sd5081;  rom[1723] =  18'sd4041;
        rom[1724] =  18'sd2796;  rom[1725] =  18'sd1387;  rom[1726] = -18'sd141;  rom[1727] = -18'sd1736;
        rom[1728] = -18'sd3340;  rom[1729] = -18'sd4891;  rom[1730] = -18'sd6328;  rom[1731] = -18'sd7591;
        rom[1732] = -18'sd8624;  rom[1733] = -18'sd9375;  rom[1734] = -18'sd9802;  rom[1735] = -18'sd9874;
        rom[1736] = -18'sd9569;  rom[1737] = -18'sd8881;  rom[1738] = -18'sd7817;  rom[1739] = -18'sd6397;
        rom[1740] = -18'sd4657;  rom[1741] = -18'sd2648;  rom[1742] = -18'sd430;  rom[1743] =  18'sd1923;
        rom[1744] =  18'sd4329;  rom[1745] =  18'sd6700;  rom[1746] =  18'sd8944;  rom[1747] =  18'sd10970;
        rom[1748] =  18'sd12688;  rom[1749] =  18'sd14016;  rom[1750] =  18'sd14882;  rom[1751] =  18'sd15224;
        rom[1752] =  18'sd14999;  rom[1753] =  18'sd14181;  rom[1754] =  18'sd12763;  rom[1755] =  18'sd10759;
        rom[1756] =  18'sd8207;  rom[1757] =  18'sd5166;  rom[1758] =  18'sd1715;  rom[1759] = -18'sd2045;
        rom[1760] = -18'sd5998;  rom[1761] = -18'sd10010;  rom[1762] = -18'sd13939;  rom[1763] = -18'sd17633;
        rom[1764] = -18'sd20939;  rom[1765] = -18'sd23703;  rom[1766] = -18'sd25780;  rom[1767] = -18'sd27034;
        rom[1768] = -18'sd27345;  rom[1769] = -18'sd26614;  rom[1770] = -18'sd24762;  rom[1771] = -18'sd21741;
        rom[1772] = -18'sd17530;  rom[1773] = -18'sd12140;  rom[1774] = -18'sd5611;  rom[1775] =  18'sd1982;
        rom[1776] =  18'sd10533;  rom[1777] =  18'sd19910;  rom[1778] =  18'sd29953;  rom[1779] =  18'sd40480;
        rom[1780] =  18'sd51289;  rom[1781] =  18'sd62166;  rom[1782] =  18'sd72891;  rom[1783] =  18'sd83237;
        rom[1784] =  18'sd92986;  rom[1785] =  18'sd101924;  rom[1786] =  18'sd109855;  rom[1787] =  18'sd116603;
        rom[1788] =  18'sd122017;  rom[1789] =  18'sd125974;  rom[1790] =  18'sd128385;  rom[1791] =  18'sd129198;
        // set 7, 4 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^20
        rom[1792] =  18'sd1;  rom[1793] =  18'sd2;  rom[1794] =  18'sd3;  rom[1795] =  18'sd4;
        rom[1796] =  18'sd5;  rom[1797] =  18'sd6;  rom[1798] =  18'sd6;  rom[1799] =  18'sd6;
        rom[1800] =  18'sd5;  rom[1801] =  18'sd4;  rom[1802] =  18'sd2;  rom[1803] =  18'sd0;
        rom[1804] = -18'sd3;  rom[1805] = -18'sd6;  rom[1806] = -18'sd9;  rom[1807] = -18'sd12;
        rom[1808] = -18'sd15;  rom[1809] = -18'sd16;  rom[1810] = -18'sd17;  rom[1811] = -18'sd16;
        rom[1812] = -18'sd14;  rom[1813] = -18'sd11;  rom[1814] = -18'sd6;  rom[1815] =  18'sd0;
        rom[1816] =  18'sd6;  rom[1817] =  18'sd13;  rom[1818] =  18'sd19;  rom[1819] =  18'sd25;
        rom[1820] =  18'sd30;  rom[1821] =  18'sd33;  rom[1822] =  18'sd34;  rom[1823] =  18'sd33;
        rom[1824] =  18'sd29;  rom[1825] =  18'sd23;  rom[1826] =  18'sd14;  rom[1827] =  18'sd3;
        rom[1828] = -18'sd9;  rom[1829] = -18'sd21;  rom[1830] = -18'sd34;  rom[1831] = -18'sd45;
        rom[1832] = -18'sd54;  rom[1833] = -18'sd60;  rom[1834] = -18'sd63;  rom[1835] = -18'sd61;
        rom[1836] = -18'sd54;  rom[1837] = -18'sd43;  rom[1838] = -18'sd28;  rom[1839] = -18'sd10;
        rom[1840] =  18'sd11;  rom[1841] =  18'sd33;  rom[1842] =  18'sd54;  rom[1843] =  18'sd74;
        rom[1844] =  18'sd89;  rom[1845] =  18'sd100;  rom[1846] =  18'sd105;  rom[1847] =  18'sd103;
        rom[1848] =  18'sd93;  rom[1849] =  18'sd76;  rom[1850] =  18'sd52;  rom[1851] =  18'sd23;
        rom[1852] = -18'sd10;  rom[1853] = -18'sd46;  rom[1854] = -18'sd80;  rom[1855] = -18'sd112;
        rom[1856] = -18'sd139;  rom[1857] = -18'sd157;  rom[1858] = -18'sd167;  rom[1859] = -18'sd165;
        rom[1860] = -18'sd151;  rom[1861] = -18'sd126;  rom[1862] = -18'sd90;  rom[1863] = -18'sd46;
        rom[1864] =  18'sd5;  rom[1865] =  18'sd59;  rom[1866] =  18'sd113;  rom[1867] =  18'sd163;
        rom[1868] =  18'sd205;  rom[1869] =  18'sd235;  rom[1870] =  18'sd251;  rom[1871] =  18'sd251;
        rom[1872] =  18'sd234;  rom[1873] =  18'sd199;  rom[1874] =  18'sd148;  rom[1875] =  18'sd83;
        rom[1876] =  18'sd9;  rom[1877] = -18'sd71;  rom[1878] = -18'sd151;  rom[1879] = -18'sd226;
        rom[1880] = -18'sd290;  rom[1881] = -18'sd337;  rom[1882] = -18'sd365;  rom[1883] = -18'sd369;
        rom[1884] = -18'sd348;  rom[1885] = -18'sd301;  rom[1886] = -18'sd231;  rom[1887] = -18'sd141;
        rom[1888] = -18'sd36;  rom[1889] =  18'sd78;  rom[1890] =  18'sd193;  rom[1891] =  18'sd302;
        rom[1892] =  18'sd396;  rom[1893] =  18'sd469;  rom[1894] =  18'sd513;  rom[1895] =  18'sd525;
        rom[1896] =  18'sd501;  rom[1897] =  18'sd442;  rom[1898] =  18'sd349;  rom[1899] =  18'sd227;
        rom[1900] =  18'sd82;  rom[1901] = -18'sd76;  rom[1902] = -18'sd237;  rom[1903] = -18'sd391;
        rom[1904] = -18'sd527;  rom[1905] = -18'sd633;  rom[1906] = -18'sd702;  rom[1907] = -18'sd727;
        rom[1908] = -18'sd703;  rom[1909] = -18'sd630;  rom[1910] = -18'sd510;  rom[1911] = -18'sd349;
        rom[1912] = -18'sd156;  rom[1913] =  18'sd58;  rom[1914] =  18'sd279;  rom[1915] =  18'sd492;
        rom[1916] =  18'sd683;  rom[1917] =  18'sd836;  rom[1918] =  18'sd940;  rom[1919] =  18'sd985;
        rom[1920] =  18'sd965;  rom[1921] =  18'sd879;  rom[1922] =  18'sd728;  rom[1923] =  18'sd520;
        rom[1924] =  18'sd266;  rom[1925] = -18'sd18;  rom[1926] = -18'sd314;  rom[1927] = -18'sd604;
        rom[1928] = -18'sd866;  rom[1929] = -18'sd1082;  rom[1930] = -18'sd1235;  rom[1931] = -18'sd1311;
        rom[1932] = -18'sd1301;  rom[1933] = -18'sd1203;  rom[1934] = -18'sd1017;  rom[1935] = -18'sd754;
        rom[1936] = -18'sd428;  rom[1937] = -18'sd57;  rom[1938] =  18'sd335;  rom[1939] =  18'sd723;
        rom[1940] =  18'sd1079;  rom[1941] =  18'sd1379;  rom[1942] =  18'sd1598;  rom[1943] =  18'sd1719;
        rom[1944] =  18'sd1729;  rom[1945] =  18'sd1622;  rom[1946] =  18'sd1399;  rom[1947] =  18'sd1072;
        rom[1948] =  18'sd657;  rom[1949] =  18'sd179;  rom[1950] = -18'sd333;  rom[1951] = -18'sd846;
        rom[1952] = -18'sd1324;  rom[1953] = -18'sd1734;  rom[1954] = -18'sd2045;  rom[1955] = -18'sd2231;
        rom[1956] = -18'sd2274;  rom[1957] = -18'sd2165;  rom[1958] = -18'sd1903;  rom[1959] = -18'sd1501;
        rom[1960] = -18'sd980;  rom[1961] = -18'sd368;  rom[1962] =  18'sd296;  rom[1963] =  18'sd969;
        rom[1964] =  18'sd1607;  rom[1965] =  18'sd2165;  rom[1966] =  18'sd2601;  rom[1967] =  18'sd2880;
        rom[1968] =  18'sd2976;  rom[1969] =  18'sd2875;  rom[1970] =  18'sd2574;  rom[1971] =  18'sd2086;
        rom[1972] =  18'sd1434;  rom[1973] =  18'sd655;  rom[1974] = -18'sd204;  rom[1975] = -18'sd1088;
        rom[1976] = -18'sd1938;  rom[1977] = -18'sd2697;  rom[1978] = -18'sd3307;  rom[1979] = -18'sd3721;
        rom[1980] = -18'sd3902;  rom[1981] = -18'sd3826;  rom[1982] = -18'sd3488;  rom[1983] = -18'sd2898;
        rom[1984] = -18'sd2082;  rom[1985] = -18'sd1087;  rom[1986] =  18'sd29;  rom[1987] =  18'sd1197;
        rom[1988] =  18'sd2340;  rom[1989] =  18'sd3380;  rom[1990] =  18'sd4242;  rom[1991] =  18'sd4858;
        rom[1992] =  18'sd5176;  rom[1993] =  18'sd5157;  rom[1994] =  18'sd4788;  rom[1995] =  18'sd4075;
        rom[1996] =  18'sd3047;  rom[1997] =  18'sd1759;  rom[1998] =  18'sd284;  rom[1999] = -18'sd1290;
        rom[2000] = -18'sd2861;  rom[2001] = -18'sd4324;  rom[2002] = -18'sd5573;  rom[2003] = -18'sd6514;
        rom[2004] = -18'sd7064;  rom[2005] = -18'sd7165;  rom[2006] = -18'sd6784;  rom[2007] = -18'sd5917;
        rom[2008] = -18'sd4594;  rom[2009] = -18'sd2876;  rom[2010] = -18'sd852;  rom[2011] =  18'sd1360;
        rom[2012] =  18'sd3624;  rom[2013] =  18'sd5793;  rom[2014] =  18'sd7715;  rom[2015] =  18'sd9244;
        rom[2016] =  18'sd10249;  rom[2017] =  18'sd10625;  rom[2018] =  18'sd10301;  rom[2019] =  18'sd9243;
        rom[2020] =  18'sd7463;  rom[2021] =  18'sd5021;  rom[2022] =  18'sd2021;  rom[2023] = -18'sd1385;
        rom[2024] = -18'sd5012;  rom[2025] = -18'sd8641;  rom[2026] = -18'sd12035;  rom[2027] = -18'sd14947;
        rom[2028] = -18'sd17138;  rom[2029] = -18'sd18385;  rom[2030] = -18'sd18495;  rom[2031] = -18'sd17320;
        rom[2032] = -18'sd14763;  rom[2033] = -18'sd10787;  rom[2034] = -18'sd5424;  rom[2035] =  18'sd1233;
        rom[2036] =  18'sd9021;  rom[2037] =  18'sd17720;  rom[2038] =  18'sd27058;  rom[2039] =  18'sd36719;
        rom[2040] =  18'sd46366;  rom[2041] =  18'sd55644;  rom[2042] =  18'sd64207;  rom[2043] =  18'sd71724;
        rom[2044] =  18'sd77905;  rom[2045] =  18'sd82504;  rom[2046] =  18'sd85339;  rom[2047] =  18'sd86294;
        // set 8, 6 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^20
        rom[2048] = -18'sd3;  rom[2049] = -18'sd3;  rom[2050] = -18'sd1;  rom[2051] =  18'sd1;
        rom[2052] =  18'sd3;  rom[2053] =  18'sd5;  rom[2054] =  18'sd7;  rom[2055] =  18'sd7;
        rom[2056] =  18'sd7;  rom[2057] =  18'sd6;  rom[2058] =  18'sd3;  rom[2059] = -18'sd1;
        rom[2060] = -18'sd5;  rom[2061] = -18'sd9;  rom[2062] = -18'sd12;  rom[2063] = -18'sd14;
        rom[2064] = -18'sd14;  rom[2065] = -18'sd11;  rom[2066] = -18'sd7;  rom[2067] =  18'sd0;
        rom[2068] =  18'sd8;  rom[2069] =  18'sd15;  rom[2070] =  18'sd21;  rom[2071] =  18'sd24;
        rom[2072] =  18'sd24;  rom[2073] =  18'sd20;  rom[2074] =  18'sd12;  rom[2075] =  18'sd2;
        rom[2076] = -18'sd10;  rom[2077] = -18'sd22;  rom[2078] = -18'sd32;  rom[2079] = -18'sd37;
        rom[2080] = -18'sd38;  rom[2081] = -18'sd32;  rom[2082] = -18'sd21;  rom[2083] = -18'sd5;
        rom[2084] =  18'sd13;  rom[2085] =  18'sd32;  rom[2086] =  18'sd46;  rom[2087] =  18'sd56;
        rom[2088] =  18'sd57;  rom[2089] =  18'sd49;  rom[2090] =  18'sd33;  rom[2091] =  18'sd11;
        rom[2092] = -18'sd16;  rom[2093] = -18'sd42;  rom[2094] = -18'sd65;  rom[2095] = -18'sd79;
        rom[2096] = -18'sd82;  rom[2097] = -18'sd73;  rom[2098] = -18'sd51;  rom[2099] = -18'sd20;
        rom[2100] =  18'sd17;  rom[2101] =  18'sd55;  rom[2102] =  18'sd87;  rom[2103] =  18'sd109;
        rom[2104] =  18'sd115;  rom[2105] =  18'sd104;  rom[2106] =  18'sd76;  rom[2107] =  18'sd34;
        rom[2108] = -18'sd16;  rom[2109] = -18'sd69;  rom[2110] = -18'sd114;  rom[2111] = -18'sd146;
        rom[2112] = -18'sd157;  rom[2113] = -18'sd145;  rom[2114] = -18'sd110;  rom[2115] = -18'sd55;
        rom[2116] =  18'sd12;  rom[2117] =  18'sd83;  rom[2118] =  18'sd145;  rom[2119] =  18'sd190;
        rom[2120] =  18'sd209;  rom[2121] =  18'sd197;  rom[2122] =  18'sd154;  rom[2123] =  18'sd84;
        rom[2124] = -18'sd3;  rom[2125] = -18'sd96;  rom[2126] = -18'sd181;  rom[2127] = -18'sd243;
        rom[2128] = -18'sd273;  rom[2129] = -18'sd262;  rom[2130] = -18'sd211;  rom[2131] = -18'sd124;
        rom[2132] = -18'sd12;  rom[2133] =  18'sd108;  rom[2134] =  18'sd220;  rom[2135] =  18'sd305;
        rom[2136] =  18'sd349;  rom[2137] =  18'sd343;  rom[2138] =  18'sd283;  rom[2139] =  18'sd177;
        rom[2140] =  18'sd37;  rom[2141] = -18'sd117;  rom[2142] = -18'sd263;  rom[2143] = -18'sd377;
        rom[2144] = -18'sd440;  rom[2145] = -18'sd440;  rom[2146] = -18'sd373;  rom[2147] = -18'sd245;
        rom[2148] = -18'sd72;  rom[2149] =  18'sd121;  rom[2150] =  18'sd308;  rom[2151] =  18'sd458;
        rom[2152] =  18'sd547;  rom[2153] =  18'sd557;  rom[2154] =  18'sd484;  rom[2155] =  18'sd333;
        rom[2156] =  18'sd123;  rom[2157] = -18'sd118;  rom[2158] = -18'sd353;  rom[2159] = -18'sd548;
        rom[2160] = -18'sd670;  rom[2161] = -18'sd697;  rom[2162] = -18'sd619;  rom[2163] = -18'sd443;
        rom[2164] = -18'sd191;  rom[2165] =  18'sd104;  rom[2166] =  18'sd398;  rom[2167] =  18'sd647;
        rom[2168] =  18'sd811;  rom[2169] =  18'sd861;  rom[2170] =  18'sd782;  rom[2171] =  18'sd581;
        rom[2172] =  18'sd281;  rom[2173] = -18'sd76;  rom[2174] = -18'sd440;  rom[2175] = -18'sd755;
        rom[2176] = -18'sd972;  rom[2177] = -18'sd1052;  rom[2178] = -18'sd977;  rom[2179] = -18'sd750;
        rom[2180] = -18'sd398;  rom[2181] =  18'sd30;  rom[2182] =  18'sd475;  rom[2183] =  18'sd870;
        rom[2184] =  18'sd1152;  rom[2185] =  18'sd1273;  rom[2186] =  18'sd1208;  rom[2187] =  18'sd956;
        rom[2188] =  18'sd548;  rom[2189] =  18'sd38;  rom[2190] = -18'sd502;  rom[2191] = -18'sd991;
        rom[2192] = -18'sd1354;  rom[2193] = -18'sd1529;  rom[2194] = -18'sd1480;  rom[2195] = -18'sd1206;
        rom[2196] = -18'sd737;  rom[2197] = -18'sd135;  rom[2198] =  18'sd515;  rom[2199] =  18'sd1118;
        rom[2200] =  18'sd1579;  rom[2201] =  18'sd1822;  rom[2202] =  18'sd1802;  rom[2203] =  18'sd1508;
        rom[2204] =  18'sd974;  rom[2205] =  18'sd268;  rom[2206] = -18'sd511;  rom[2207] = -18'sd1247;
        rom[2208] = -18'sd1828;  rom[2209] = -18'sd2159;  rom[2210] = -18'sd2180;  rom[2211] = -18'sd1872;
        rom[2212] = -18'sd1268;  rom[2213] = -18'sd445;  rom[2214] =  18'sd481;  rom[2215] =  18'sd1377;
        rom[2216] =  18'sd2105;  rom[2217] =  18'sd2547;  rom[2218] =  18'sd2625;  rom[2219] =  18'sd2311;
        rom[2220] =  18'sd1633;  rom[2221] =  18'sd679;  rom[2222] = -18'sd420;  rom[2223] = -18'sd1506;
        rom[2224] = -18'sd2413;  rom[2225] = -18'sd2996;  rom[2226] = -18'sd3154;  rom[2227] = -18'sd2843;
        rom[2228] = -18'sd2089;  rom[2229] = -18'sd985;  rom[2230] =  18'sd317;  rom[2231] =  18'sd1631;
        rom[2232] =  18'sd2759;  rom[2233] =  18'sd3522;  rom[2234] =  18'sd3788;  rom[2235] =  18'sd3495;
        rom[2236] =  18'sd2661;  rom[2237] =  18'sd1386;  rom[2238] = -18'sd158;  rom[2239] = -18'sd1750;
        rom[2240] = -18'sd3153;  rom[2241] = -18'sd4147;  rom[2242] = -18'sd4561;  rom[2243] = -18'sd4307;
        rom[2244] = -18'sd3389;  rom[2245] = -18'sd1915;  rom[2246] = -18'sd79;  rom[2247] =  18'sd1860;
        rom[2248] =  18'sd3614;  rom[2249] =  18'sd4910;  rom[2250] =  18'sd5529;  rom[2251] =  18'sd5343;
        rom[2252] =  18'sd4340;  rom[2253] =  18'sd2627;  rom[2254] =  18'sd425;  rom[2255] = -18'sd1958;
        rom[2256] = -18'sd4173;  rom[2257] = -18'sd5878;  rom[2258] = -18'sd6786;  rom[2259] = -18'sd6717;
        rom[2260] = -18'sd5624;  rom[2261] = -18'sd3616;  rom[2262] = -18'sd940;  rom[2263] =  18'sd2039;
        rom[2264] =  18'sd4889;  rom[2265] =  18'sd7173;  rom[2266] =  18'sd8510;  rom[2267] =  18'sd8638;
        rom[2268] =  18'sd7460;  rom[2269] =  18'sd5065;  rom[2270] =  18'sd1733;  rom[2271] = -18'sd2098;
        rom[2272] = -18'sd5885;  rom[2273] = -18'sd9053;  rom[2274] = -18'sd11078;  rom[2275] = -18'sd11565;
        rom[2276] = -18'sd10318;  rom[2277] = -18'sd7384;  rom[2278] = -18'sd3062;  rom[2279] =  18'sd2118;
        rom[2280] =  18'sd7450;  rom[2281] =  18'sd12147;  rom[2282] =  18'sd15437;  rom[2283] =  18'sd16680;
        rom[2284] =  18'sd15470;  rom[2285] =  18'sd11712;  rom[2286] =  18'sd5668;  rom[2287] = -18'sd2042;
        rom[2288] = -18'sd10484;  rom[2289] = -18'sd18506;  rom[2290] = -18'sd24851;  rom[2291] = -18'sd28312;
        rom[2292] = -18'sd27874;  rom[2293] = -18'sd22854;  rom[2294] = -18'sd13005;  rom[2295] =  18'sd1424;
        rom[2296] =  18'sd19684;  rom[2297] =  18'sd40572;  rom[2298] =  18'sd62542;  rom[2299] =  18'sd83845;
        rom[2300] =  18'sd102707;  rom[2301] =  18'sd117507;  rom[2302] =  18'sd126947;  rom[2303] =  18'sd130198;
        // set 9, 9 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^19
        rom[2304] =  18'sd0;  rom[2305] =  18'sd1;  rom[2306] =  18'sd2;  rom[2307] =  18'sd3;
        rom[2308] =  18'sd2;  rom[2309] =  18'sd1;  rom[2310] = -18'sd1;  rom[2311] = -18'sd3;
        rom[2312] = -18'sd4;  rom[2313] = -18'sd4;  rom[2314] = -18'sd2;  rom[2315] =  18'sd0;
        rom[2316] =  18'sd4;  rom[2317] =  18'sd6;  rom[2318] =  18'sd7;  rom[2319] =  18'sd5;
        rom[2320] =  18'sd1;  rom[2321] = -18'sd3;  rom[2322] = -18'sd8;  rom[2323] = -18'sd10;
        rom[2324] = -18'sd9;  rom[2325] = -18'sd4;  rom[2326] =  18'sd2;  rom[2327] =  18'sd9;
        rom[2328] =  18'sd13;  rom[2329] =  18'sd13;  rom[2330] =  18'sd9;  rom[2331] =  18'sd1;
        rom[2332] = -18'sd8;  rom[2333] = -18'sd16;  rom[2334] = -18'sd18;  rom[2335] = -18'sd15;
        rom[2336] = -18'sd6;  rom[2337] =  18'sd6;  rom[2338] =  18'sd17;  rom[2339] =  18'sd24;
        rom[2340] =  18'sd23;  rom[2341] =  18'sd14;  rom[2342] =  18'sd0;  rom[2343] = -18'sd16;
        rom[2344] = -18'sd28;  rom[2345] = -18'sd32;  rom[2346] = -18'sd24;  rom[2347] = -18'sd8;
        rom[2348] =  18'sd12;  rom[2349] =  18'sd31;  rom[2350] =  18'sd40;  rom[2351] =  18'sd36;
        rom[2352] =  18'sd20;  rom[2353] = -18'sd5;  rom[2354] = -18'sd30;  rom[2355] = -18'sd47;
        rom[2356] = -18'sd50;  rom[2357] = -18'sd36;  rom[2358] = -18'sd8;  rom[2359] =  18'sd24;
        rom[2360] =  18'sd51;  rom[2361] =  18'sd63;  rom[2362] =  18'sd54;  rom[2363] =  18'sd26;
        rom[2364] = -18'sd13;  rom[2365] = -18'sd51;  rom[2366] = -18'sd74;  rom[2367] = -18'sd74;
        rom[2368] = -18'sd49;  rom[2369] = -18'sd5;  rom[2370] =  18'sd44;  rom[2371] =  18'sd81;
        rom[2372] =  18'sd94;  rom[2373] =  18'sd76;  rom[2374] =  18'sd31;  rom[2375] = -18'sd28;
        rom[2376] = -18'sd82;  rom[2377] = -18'sd111;  rom[2378] = -18'sd105;  rom[2379] = -18'sd63;
        rom[2380] =  18'sd3;  rom[2381] =  18'sd73;  rom[2382] =  18'sd123;  rom[2383] =  18'sd135;
        rom[2384] =  18'sd102;  rom[2385] =  18'sd32;  rom[2386] = -18'sd53;  rom[2387] = -18'sd126;
        rom[2388] = -18'sd161;  rom[2389] = -18'sd144;  rom[2390] = -18'sd77;  rom[2391] =  18'sd20;
        rom[2392] =  18'sd116;  rom[2393] =  18'sd179;  rom[2394] =  18'sd186;  rom[2395] =  18'sd130;
        rom[2396] =  18'sd28;  rom[2397] = -18'sd91;  rom[2398] = -18'sd186;  rom[2399] = -18'sd224;
        rom[2400] = -18'sd189;  rom[2401] = -18'sd88;  rom[2402] =  18'sd48;  rom[2403] =  18'sd176;
        rom[2404] =  18'sd252;  rom[2405] =  18'sd248;  rom[2406] =  18'sd161;  rom[2407] =  18'sd14;
        rom[2408] = -18'sd146;  rom[2409] = -18'sd265;  rom[2410] = -18'sd303;  rom[2411] = -18'sd240;
        rom[2412] = -18'sd94;  rom[2413] =  18'sd92;  rom[2414] =  18'sd257;  rom[2415] =  18'sd345;
        rom[2416] =  18'sd322;  rom[2417] =  18'sd189;  rom[2418] = -18'sd14;  rom[2419] = -18'sd223;
        rom[2420] = -18'sd369;  rom[2421] = -18'sd398;  rom[2422] = -18'sd296;  rom[2423] = -18'sd89;
        rom[2424] =  18'sd158;  rom[2425] =  18'sd365;  rom[2426] =  18'sd461;  rom[2427] =  18'sd407;
        rom[2428] =  18'sd214;  rom[2429] = -18'sd61;  rom[2430] = -18'sd329;  rom[2431] = -18'sd500;
        rom[2432] = -18'sd512;  rom[2433] = -18'sd354;  rom[2434] = -18'sd69;  rom[2435] =  18'sd252;
        rom[2436] =  18'sd506;  rom[2437] =  18'sd602;  rom[2438] =  18'sd502;  rom[2439] =  18'sd229;
        rom[2440] = -18'sd134;  rom[2441] = -18'sd469;  rom[2442] = -18'sd663;  rom[2443] = -18'sd645;
        rom[2444] = -18'sd411;  rom[2445] = -18'sd28;  rom[2446] =  18'sd383;  rom[2447] =  18'sd684;
        rom[2448] =  18'sd772;  rom[2449] =  18'sd605;  rom[2450] =  18'sd229;  rom[2451] = -18'sd242;
        rom[2452] = -18'sd653;  rom[2453] = -18'sd865;  rom[2454] = -18'sd798;  rom[2455] = -18'sd462;
        rom[2456] =  18'sd45;  rom[2457] =  18'sd559;  rom[2458] =  18'sd910;  rom[2459] =  18'sd973;
        rom[2460] =  18'sd715;  rom[2461] =  18'sd206;  rom[2462] = -18'sd395;  rom[2463] = -18'sd891;
        rom[2464] = -18'sd1112;  rom[2465] = -18'sd972;  rom[2466] = -18'sd501;  rom[2467] =  18'sd158;
        rom[2468] =  18'sd794;  rom[2469] =  18'sd1193;  rom[2470] =  18'sd1212;  rom[2471] =  18'sd828;
        rom[2472] =  18'sd149;  rom[2473] = -18'sd609;  rom[2474] = -18'sd1198;  rom[2475] = -18'sd1414;
        rom[2476] = -18'sd1167;  rom[2477] = -18'sd520;  rom[2478] =  18'sd329;  rom[2479] =  18'sd1107;
        rom[2480] =  18'sd1551;  rom[2481] =  18'sd1497;  rom[2482] =  18'sd941;  rom[2483] =  18'sd46;
        rom[2484] = -18'sd905;  rom[2485] = -18'sd1596;  rom[2486] = -18'sd1787;  rom[2487] = -18'sd1389;
        rom[2488] = -18'sd510;  rom[2489] =  18'sd580;  rom[2490] =  18'sd1526;  rom[2491] =  18'sd2007;
        rom[2492] =  18'sd1840;  rom[2493] =  18'sd1051;  rom[2494] = -18'sd126;  rom[2495] = -18'sd1316;
        rom[2496] = -18'sd2123;  rom[2497] = -18'sd2258;  rom[2498] = -18'sd1646;  rom[2499] = -18'sd455;
        rom[2500] =  18'sd947;  rom[2501] =  18'sd2100;  rom[2502] =  18'sd2607;  rom[2503] =  18'sd2269;
        rom[2504] =  18'sd1154;  rom[2505] = -18'sd402;  rom[2506] = -18'sd1903;  rom[2507] = -18'sd2844;
        rom[2508] = -18'sd2882;  rom[2509] = -18'sd1957;  rom[2510] = -18'sd328;  rom[2511] =  18'sd1498;
        rom[2512] =  18'sd2921;  rom[2513] =  18'sd3442;  rom[2514] =  18'sd2839;  rom[2515] =  18'sd1250;
        rom[2516] = -18'sd850;  rom[2517] = -18'sd2786;  rom[2518] = -18'sd3900;  rom[2519] = -18'sd3771;
        rom[2520] = -18'sd2367;  rom[2521] = -18'sd80;  rom[2522] =  18'sd2379;  rom[2523] =  18'sd4195;
        rom[2524] =  18'sd4717;  rom[2525] =  18'sd3685;  rom[2526] =  18'sd1339;  rom[2527] = -18'sd1625;
        rom[2528] = -18'sd4253;  rom[2529] = -18'sd5639;  rom[2530] = -18'sd5221;  rom[2531] = -18'sd3002;
        rom[2532] =  18'sd414;  rom[2533] =  18'sd3975;  rom[2534] =  18'sd6491;  rom[2535] =  18'sd7023;
        rom[2536] =  18'sd5209;  rom[2537] =  18'sd1442;  rom[2538] = -18'sd3197;  rom[2539] = -18'sd7227;
        rom[2540] = -18'sd9217;  rom[2541] = -18'sd8263;  rom[2542] = -18'sd4339;  rom[2543] =  18'sd1593;
        rom[2544] =  18'sd7782;  rom[2545] =  18'sd12159;  rom[2546] =  18'sd12961;  rom[2547] =  18'sd9334;
        rom[2548] =  18'sd1736;  rom[2549] = -18'sd8006;  rom[2550] = -18'sd17031;  rom[2551] = -18'sd22111;
        rom[2552] = -18'sd20491;  rom[2553] = -18'sd10690;  rom[2554] =  18'sd6967;  rom[2555] =  18'sd30250;
        rom[2556] =  18'sd55389;  rom[2557] =  18'sd77859;  rom[2558] =  18'sd93387;  rom[2559] =  18'sd98922;
        // set 10, 12 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^18
        rom[2560] =  18'sd1;  rom[2561] =  18'sd1;  rom[2562] =  18'sd1;  rom[2563] =  18'sd0;
        rom[2564] = -18'sd1;  rom[2565] = -18'sd2;  rom[2566] = -18'sd1;  rom[2567] =  18'sd0;
        rom[2568] =  18'sd2;  rom[2569] =  18'sd2;  rom[2570] =  18'sd2;  rom[2571] =  18'sd0;
        rom[2572] = -18'sd2;  rom[2573] = -18'sd3;  rom[2574] = -18'sd2;  rom[2575] =  18'sd0;
        rom[2576] =  18'sd3;  rom[2577] =  18'sd4;  rom[2578] =  18'sd3;  rom[2579] =  18'sd0;
        rom[2580] = -18'sd4;  rom[2581] = -18'sd6;  rom[2582] = -18'sd5;  rom[2583] =  18'sd0;
        rom[2584] =  18'sd5;  rom[2585] =  18'sd7;  rom[2586] =  18'sd6;  rom[2587] =  18'sd1;
        rom[2588] = -18'sd5;  rom[2589] = -18'sd9;  rom[2590] = -18'sd8;  rom[2591] = -18'sd2;
        rom[2592] =  18'sd6;  rom[2593] =  18'sd11;  rom[2594] =  18'sd10;  rom[2595] =  18'sd3;
        rom[2596] = -18'sd7;  rom[2597] = -18'sd14;  rom[2598] = -18'sd13;  rom[2599] = -18'sd4;
        rom[2600] =  18'sd8;  rom[2601] =  18'sd16;  rom[2602] =  18'sd16;  rom[2603] =  18'sd6;
        rom[2604] = -18'sd8;  rom[2605] = -18'sd19;  rom[2606] = -18'sd19;  rom[2607] = -18'sd8;
        rom[2608] =  18'sd9;  rom[2609] =  18'sd22;  rom[2610] =  18'sd23;  rom[2611] =  18'sd11;
        rom[2612] = -18'sd9;  rom[2613] = -18'sd25;  rom[2614] = -18'sd28;  rom[2615] = -18'sd14;
        rom[2616] =  18'sd9;  rom[2617] =  18'sd29;  rom[2618] =  18'sd33;  rom[2619] =  18'sd19;
        rom[2620] = -18'sd9;  rom[2621] = -18'sd33;  rom[2622] = -18'sd39;  rom[2623] = -18'sd23;
        rom[2624] =  18'sd8;  rom[2625] =  18'sd37;  rom[2626] =  18'sd46;  rom[2627] =  18'sd29;
        rom[2628] = -18'sd6;  rom[2629] = -18'sd40;  rom[2630] = -18'sd53;  rom[2631] = -18'sd36;
        rom[2632] =  18'sd4;  rom[2633] =  18'sd44;  rom[2634] =  18'sd61;  rom[2635] =  18'sd43;
        rom[2636] = -18'sd1;  rom[2637] = -18'sd48;  rom[2638] = -18'sd70;  rom[2639] = -18'sd52;
        rom[2640] = -18'sd3;  rom[2641] =  18'sd52;  rom[2642] =  18'sd80;  rom[2643] =  18'sd62;
        rom[2644] =  18'sd7;  rom[2645] = -18'sd55;  rom[2646] = -18'sd90;  rom[2647] = -18'sd74;
        rom[2648] = -18'sd13;  rom[2649] =  18'sd58;  rom[2650] =  18'sd101;  rom[2651] =  18'sd87;
        rom[2652] =  18'sd21;  rom[2653] = -18'sd61;  rom[2654] = -18'sd112;  rom[2655] = -18'sd101;
        rom[2656] = -18'sd30;  rom[2657] =  18'sd63;  rom[2658] =  18'sd124;  rom[2659] =  18'sd116;
        rom[2660] =  18'sd40;  rom[2661] = -18'sd64;  rom[2662] = -18'sd137;  rom[2663] = -18'sd134;
        rom[2664] = -18'sd53;  rom[2665] =  18'sd64;  rom[2666] =  18'sd149;  rom[2667] =  18'sd153;
        rom[2668] =  18'sd67;  rom[2669] = -18'sd62;  rom[2670] = -18'sd163;  rom[2671] = -18'sd173;
        rom[2672] = -18'sd84;  rom[2673] =  18'sd60;  rom[2674] =  18'sd176;  rom[2675] =  18'sd196;
        rom[2676] =  18'sd103;  rom[2677] = -18'sd55;  rom[2678] = -18'sd189;  rom[2679] = -18'sd220;
        rom[2680] = -18'sd124;  rom[2681] =  18'sd49;  rom[2682] =  18'sd202;  rom[2683] =  18'sd246;
        rom[2684] =  18'sd149;  rom[2685] = -18'sd40;  rom[2686] = -18'sd215;  rom[2687] = -18'sd273;
        rom[2688] = -18'sd176;  rom[2689] =  18'sd29;  rom[2690] =  18'sd227;  rom[2691] =  18'sd303;
        rom[2692] =  18'sd206;  rom[2693] = -18'sd15;  rom[2694] = -18'sd238;  rom[2695] = -18'sd334;
        rom[2696] = -18'sd240;  rom[2697] = -18'sd2;  rom[2698] =  18'sd248;  rom[2699] =  18'sd366;
        rom[2700] =  18'sd277;  rom[2701] =  18'sd23;  rom[2702] = -18'sd256;  rom[2703] = -18'sd400;
        rom[2704] = -18'sd318;  rom[2705] = -18'sd47;  rom[2706] =  18'sd263;  rom[2707] =  18'sd436;
        rom[2708] =  18'sd363;  rom[2709] =  18'sd76;  rom[2710] = -18'sd268;  rom[2711] = -18'sd472;
        rom[2712] = -18'sd412;  rom[2713] = -18'sd110;  rom[2714] =  18'sd270;  rom[2715] =  18'sd510;
        rom[2716] =  18'sd465;  rom[2717] =  18'sd148;  rom[2718] = -18'sd269;  rom[2719] = -18'sd549;
        rom[2720] = -18'sd523;  rom[2721] = -18'sd192;  rom[2722] =  18'sd265;  rom[2723] =  18'sd589;
        rom[2724] =  18'sd586;  rom[2725] =  18'sd243;  rom[2726] = -18'sd257;  rom[2727] = -18'sd629;
        rom[2728] = -18'sd654;  rom[2729] = -18'sd300;  rom[2730] =  18'sd244;  rom[2731] =  18'sd670;
        rom[2732] =  18'sd727;  rom[2733] =  18'sd365;  rom[2734] = -18'sd225;  rom[2735] = -18'sd711;
        rom[2736] = -18'sd807;  rom[2737] = -18'sd438;  rom[2738] =  18'sd201;  rom[2739] =  18'sd753;
        rom[2740] =  18'sd892;  rom[2741] =  18'sd520;  rom[2742] = -18'sd170;  rom[2743] = -18'sd793;
        rom[2744] = -18'sd985;  rom[2745] = -18'sd613;  rom[2746] =  18'sd131;  rom[2747] =  18'sd834;
        rom[2748] =  18'sd1085;  rom[2749] =  18'sd718;  rom[2750] = -18'sd83;  rom[2751] = -18'sd873;
        rom[2752] = -18'sd1194;  rom[2753] = -18'sd836;  rom[2754] =  18'sd24;  rom[2755] =  18'sd912;
        rom[2756] =  18'sd1313;  rom[2757] =  18'sd970;  rom[2758] =  18'sd47;  rom[2759] = -18'sd949;
        rom[2760] = -18'sd1443;  rom[2761] = -18'sd1123;  rom[2762] = -18'sd133;  rom[2763] =  18'sd985;
        rom[2764] =  18'sd1588;  rom[2765] =  18'sd1298;  rom[2766] =  18'sd237;  rom[2767] = -18'sd1018;
        rom[2768] = -18'sd1748;  rom[2769] = -18'sd1500;  rom[2770] = -18'sd363;  rom[2771] =  18'sd1049;
        rom[2772] =  18'sd1930;  rom[2773] =  18'sd1736;  rom[2774] =  18'sd516;  rom[2775] = -18'sd1078;
        rom[2776] = -18'sd2139;  rom[2777] = -18'sd2015;  rom[2778] = -18'sd705;  rom[2779] =  18'sd1102;
        rom[2780] =  18'sd2383;  rom[2781] =  18'sd2353;  rom[2782] =  18'sd941;  rom[2783] = -18'sd1123;
        rom[2784] = -18'sd2676;  rom[2785] = -18'sd2771;  rom[2786] = -18'sd1243;  rom[2787] =  18'sd1138;
        rom[2788] =  18'sd3039;  rom[2789] =  18'sd3305;  rom[2790] =  18'sd1640;  rom[2791] = -18'sd1145;
        rom[2792] = -18'sd3509;  rom[2793] = -18'sd4017;  rom[2794] = -18'sd2187;  rom[2795] =  18'sd1139;
        rom[2796] =  18'sd4150;  rom[2797] =  18'sd5024;  rom[2798] =  18'sd2987;  rom[2799] = -18'sd1110;
        rom[2800] = -18'sd5097;  rom[2801] = -18'sd6574;  rom[2802] = -18'sd4276;  rom[2803] =  18'sd1029;
        rom[2804] =  18'sd6671;  rom[2805] =  18'sd9308;  rom[2806] =  18'sd6709;  rom[2807] = -18'sd773;
        rom[2808] = -18'sd9869;  rom[2809] = -18'sd15516;  rom[2810] = -18'sd13059;  rom[2811] = -18'sd545;
        rom[2812] =  18'sd19955;  rom[2813] =  18'sd42740;  rom[2814] =  18'sd60505;  rom[2815] =  18'sd67192;
        // set 11, 16 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^18
        rom[2816] = -18'sd1;  rom[2817] = -18'sd1;  rom[2818] =  18'sd0;  rom[2819] =  18'sd1;
        rom[2820] =  18'sd1;  rom[2821] =  18'sd0;  rom[2822] = -18'sd2;  rom[2823] = -18'sd2;
        rom[2824] =  18'sd0;  rom[2825] =  18'sd3;  rom[2826] =  18'sd2;  rom[2827] =  18'sd0;
        rom[2828] = -18'sd3;  rom[2829] = -18'sd3;  rom[2830] =  18'sd0;  rom[2831] =  18'sd4;
        rom[2832] =  18'sd4;  rom[2833] =  18'sd0;  rom[2834] = -18'sd5;  rom[2835] = -18'sd5;
        rom[2836] =  18'sd0;  rom[2837] =  18'sd5;  rom[2838] =  18'sd6;  rom[2839] =  18'sd1;
        rom[2840] = -18'sd6;  rom[2841] = -18'sd7;  rom[2842] = -18'sd1;  rom[2843] =  18'sd7;
        rom[2844] =  18'sd9;  rom[2845] =  18'sd2;  rom[2846] = -18'sd8;  rom[2847] = -18'sd11;
        rom[2848] = -18'sd3;  rom[2849] =  18'sd9;  rom[2850] =  18'sd13;  rom[2851] =  18'sd4;
        rom[2852] = -18'sd10;  rom[2853] = -18'sd15;  rom[2854] = -18'sd5;  rom[2855] =  18'sd11;
        rom[2856] =  18'sd18;  rom[2857] =  18'sd7;  rom[2858] = -18'sd12;  rom[2859] = -18'sd20;
        rom[2860] = -18'sd9;  rom[2861] =  18'sd12;  rom[2862] =  18'sd23;  rom[2863] =  18'sd12;
        rom[2864] = -18'sd13;  rom[2865] = -18'sd27;  rom[2866] = -18'sd15;  rom[2867] =  18'sd13;
        rom[2868] =  18'sd30;  rom[2869] =  18'sd18;  rom[2870] = -18'sd13;  rom[2871] = -18'sd34;
        rom[2872] = -18'sd22;  rom[2873] =  18'sd13;  rom[2874] =  18'sd38;  rom[2875] =  18'sd26;
        rom[2876] = -18'sd13;  rom[2877] = -18'sd42;  rom[2878] = -18'sd31;  rom[2879] =  18'sd12;
        rom[2880] =  18'sd46;  rom[2881] =  18'sd36;  rom[2882] = -18'sd11;  rom[2883] = -18'sd50;
        rom[2884] = -18'sd42;  rom[2885] =  18'sd9;  rom[2886] =  18'sd55;  rom[2887] =  18'sd49;
        rom[2888] = -18'sd7;  rom[2889] = -18'sd59;  rom[2890] = -18'sd56;  rom[2891] =  18'sd4;
        rom[2892] =  18'sd64;  rom[2893] =  18'sd64;  rom[2894] =  18'sd0;  rom[2895] = -18'sd68;
        rom[2896] = -18'sd73;  rom[2897] = -18'sd5;  rom[2898] =  18'sd73;  rom[2899] =  18'sd82;
        rom[2900] =  18'sd10;  rom[2901] = -18'sd77;  rom[2902] = -18'sd93;  rom[2903] = -18'sd17;
        rom[2904] =  18'sd81;  rom[2905] =  18'sd103;  rom[2906] =  18'sd24;  rom[2907] = -18'sd84;
        rom[2908] = -18'sd115;  rom[2909] = -18'sd33;  rom[2910] =  18'sd87;  rom[2911] =  18'sd127;
        rom[2912] =  18'sd42;  rom[2913] = -18'sd90;  rom[2914] = -18'sd140;  rom[2915] = -18'sd54;
        rom[2916] =  18'sd92;  rom[2917] =  18'sd154;  rom[2918] =  18'sd66;  rom[2919] = -18'sd92;
        rom[2920] = -18'sd168;  rom[2921] = -18'sd80;  rom[2922] =  18'sd92;  rom[2923] =  18'sd182;
        rom[2924] =  18'sd95;  rom[2925] = -18'sd91;  rom[2926] = -18'sd197;  rom[2927] = -18'sd113;
        rom[2928] =  18'sd89;  rom[2929] =  18'sd213;  rom[2930] =  18'sd131;  rom[2931] = -18'sd85;
        rom[2932] = -18'sd228;  rom[2933] = -18'sd152;  rom[2934] =  18'sd80;  rom[2935] =  18'sd244;
        rom[2936] =  18'sd174;  rom[2937] = -18'sd73;  rom[2938] = -18'sd260;  rom[2939] = -18'sd198;
        rom[2940] =  18'sd64;  rom[2941] =  18'sd275;  rom[2942] =  18'sd224;  rom[2943] = -18'sd53;
        rom[2944] = -18'sd291;  rom[2945] = -18'sd251;  rom[2946] =  18'sd39;  rom[2947] =  18'sd306;
        rom[2948] =  18'sd281;  rom[2949] = -18'sd24;  rom[2950] = -18'sd320;  rom[2951] = -18'sd313;
        rom[2952] =  18'sd5;  rom[2953] =  18'sd333;  rom[2954] =  18'sd346;  rom[2955] =  18'sd16;
        rom[2956] = -18'sd346;  rom[2957] = -18'sd381;  rom[2958] = -18'sd40;  rom[2959] =  18'sd357;
        rom[2960] =  18'sd418;  rom[2961] =  18'sd67;  rom[2962] = -18'sd367;  rom[2963] = -18'sd457;
        rom[2964] = -18'sd98;  rom[2965] =  18'sd375;  rom[2966] =  18'sd497;  rom[2967] =  18'sd133;
        rom[2968] = -18'sd381;  rom[2969] = -18'sd540;  rom[2970] = -18'sd171;  rom[2971] =  18'sd384;
        rom[2972] =  18'sd583;  rom[2973] =  18'sd213;  rom[2974] = -18'sd385;  rom[2975] = -18'sd629;
        rom[2976] = -18'sd260;  rom[2977] =  18'sd383;  rom[2978] =  18'sd675;  rom[2979] =  18'sd311;
        rom[2980] = -18'sd378;  rom[2981] = -18'sd723;  rom[2982] = -18'sd367;  rom[2983] =  18'sd369;
        rom[2984] =  18'sd772;  rom[2985] =  18'sd428;  rom[2986] = -18'sd356;  rom[2987] = -18'sd822;
        rom[2988] = -18'sd494;  rom[2989] =  18'sd338;  rom[2990] =  18'sd872;  rom[2991] =  18'sd566;
        rom[2992] = -18'sd315;  rom[2993] = -18'sd924;  rom[2994] = -18'sd644;  rom[2995] =  18'sd287;
        rom[2996] =  18'sd975;  rom[2997] =  18'sd728;  rom[2998] = -18'sd253;  rom[2999] = -18'sd1027;
        rom[3000] = -18'sd818;  rom[3001] =  18'sd211;  rom[3002] =  18'sd1079;  rom[3003] =  18'sd916;
        rom[3004] = -18'sd163;  rom[3005] = -18'sd1131;  rom[3006] = -18'sd1022;  rom[3007] =  18'sd106;
        rom[3008] =  18'sd1182;  rom[3009] =  18'sd1136;  rom[3010] = -18'sd39;  rom[3011] = -18'sd1232;
        rom[3012] = -18'sd1259;  rom[3013] = -18'sd37;  rom[3014] =  18'sd1282;  rom[3015] =  18'sd1393;
        rom[3016] =  18'sd125;  rom[3017] = -18'sd1330;  rom[3018] = -18'sd1538;  rom[3019] = -18'sd227;
        rom[3020] =  18'sd1377;  rom[3021] =  18'sd1695;  rom[3022] =  18'sd344;  rom[3023] = -18'sd1422;
        rom[3024] = -18'sd1868;  rom[3025] = -18'sd478;  rom[3026] =  18'sd1465;  rom[3027] =  18'sd2057;
        rom[3028] =  18'sd632;  rom[3029] = -18'sd1506;  rom[3030] = -18'sd2268;  rom[3031] = -18'sd811;
        rom[3032] =  18'sd1544;  rom[3033] =  18'sd2502;  rom[3034] =  18'sd1020;  rom[3035] = -18'sd1578;
        rom[3036] = -18'sd2768;  rom[3037] = -18'sd1265;  rom[3038] =  18'sd1609;  rom[3039] =  18'sd3072;
        rom[3040] =  18'sd1557;  rom[3041] = -18'sd1634;  rom[3042] = -18'sd3425;  rom[3043] = -18'sd1908;
        rom[3044] =  18'sd1654;  rom[3045] =  18'sd3845;  rom[3046] =  18'sd2340;  rom[3047] = -18'sd1667;
        rom[3048] = -18'sd4356;  rom[3049] = -18'sd2884;  rom[3050] =  18'sd1668;  rom[3051] =  18'sd4998;
        rom[3052] =  18'sd3592;  rom[3053] = -18'sd1653;  rom[3054] = -18'sd5839;  rom[3055] = -18'sd4555;
        rom[3056] =  18'sd1610;  rom[3057] =  18'sd7004;  rom[3058] =  18'sd5948;  rom[3059] = -18'sd1511;
        rom[3060] = -18'sd8746;  rom[3061] = -18'sd8161;  rom[3062] =  18'sd1279;  rom[3063] =  18'sd11679;
        rom[3064] =  18'sd12239;  rom[3065] = -18'sd610;  rom[3066] = -18'sd17721;  rom[3067] = -18'sd22266;
        rom[3068] = -18'sd2607;  rom[3069] =  18'sd36870;  rom[3070] =  18'sd76252;  rom[3071] =  18'sd92722;
        // set 12, 20 kHz wide: 511 taps, the first 256 (the rest mirror them); scale 2^18
        rom[3072] =  18'sd1;  rom[3073] =  18'sd0;  rom[3074] = -18'sd1;  rom[3075] = -18'sd1;
        rom[3076] =  18'sd1;  rom[3077] =  18'sd2;  rom[3078] =  18'sd0;  rom[3079] = -18'sd3;
        rom[3080] = -18'sd1;  rom[3081] =  18'sd2;  rom[3082] =  18'sd3;  rom[3083] = -18'sd1;
        rom[3084] = -18'sd4;  rom[3085] = -18'sd2;  rom[3086] =  18'sd4;  rom[3087] =  18'sd4;
        rom[3088] = -18'sd2;  rom[3089] = -18'sd6;  rom[3090] = -18'sd1;  rom[3091] =  18'sd6;
        rom[3092] =  18'sd5;  rom[3093] = -18'sd3;  rom[3094] = -18'sd8;  rom[3095] = -18'sd1;
        rom[3096] =  18'sd8;  rom[3097] =  18'sd6;  rom[3098] = -18'sd6;  rom[3099] = -18'sd10;
        rom[3100] =  18'sd1;  rom[3101] =  18'sd11;  rom[3102] =  18'sd6;  rom[3103] = -18'sd9;
        rom[3104] = -18'sd12;  rom[3105] =  18'sd3;  rom[3106] =  18'sd15;  rom[3107] =  18'sd5;
        rom[3108] = -18'sd13;  rom[3109] = -18'sd14;  rom[3110] =  18'sd7;  rom[3111] =  18'sd19;
        rom[3112] =  18'sd4;  rom[3113] = -18'sd19;  rom[3114] = -18'sd15;  rom[3115] =  18'sd12;
        rom[3116] =  18'sd24;  rom[3117] =  18'sd1;  rom[3118] = -18'sd25;  rom[3119] = -18'sd16;
        rom[3120] =  18'sd18;  rom[3121] =  18'sd28;  rom[3122] = -18'sd3;  rom[3123] = -18'sd32;
        rom[3124] = -18'sd15;  rom[3125] =  18'sd26;  rom[3126] =  18'sd32;  rom[3127] = -18'sd10;
        rom[3128] = -18'sd40;  rom[3129] = -18'sd13;  rom[3130] =  18'sd36;  rom[3131] =  18'sd35;
        rom[3132] = -18'sd18;  rom[3133] = -18'sd48;  rom[3134] = -18'sd8;  rom[3135] =  18'sd47;
        rom[3136] =  18'sd36;  rom[3137] = -18'sd30;  rom[3138] = -18'sd56;  rom[3139] = -18'sd1;
        rom[3140] =  18'sd60;  rom[3141] =  18'sd36;  rom[3142] = -18'sd44;  rom[3143] = -18'sd63;
        rom[3144] =  18'sd10;  rom[3145] =  18'sd74;  rom[3146] =  18'sd32;  rom[3147] = -18'sd60;
        rom[3148] = -18'sd69;  rom[3149] =  18'sd24;  rom[3150] =  18'sd88;  rom[3151] =  18'sd25;
        rom[3152] = -18'sd79;  rom[3153] = -18'sd73;  rom[3154] =  18'sd42;  rom[3155] =  18'sd102;
        rom[3156] =  18'sd14;  rom[3157] = -18'sd101;  rom[3158] = -18'sd73;  rom[3159] =  18'sd65;
        rom[3160] =  18'sd115;  rom[3161] = -18'sd3;  rom[3162] = -18'sd123;  rom[3163] = -18'sd69;
        rom[3164] =  18'sd91;  rom[3165] =  18'sd125;  rom[3166] = -18'sd25;  rom[3167] = -18'sd147;
        rom[3168] = -18'sd59;  rom[3169] =  18'sd122;  rom[3170] =  18'sd133;  rom[3171] = -18'sd53;
        rom[3172] = -18'sd171;  rom[3173] = -18'sd43;  rom[3174] =  18'sd156;  rom[3175] =  18'sd135;
        rom[3176] = -18'sd87;  rom[3177] = -18'sd193;  rom[3178] = -18'sd19;  rom[3179] =  18'sd192;
        rom[3180] =  18'sd131;  rom[3181] = -18'sd127;  rom[3182] = -18'sd211;  rom[3183] =  18'sd13;
        rom[3184] =  18'sd230;  rom[3185] =  18'sd119;  rom[3186] = -18'sd174;  rom[3187] = -18'sd225;
        rom[3188] =  18'sd54;  rom[3189] =  18'sd268;  rom[3190] =  18'sd98;  rom[3191] = -18'sd226;
        rom[3192] = -18'sd233;  rom[3193] =  18'sd104;  rom[3194] =  18'sd304;  rom[3195] =  18'sd66;
        rom[3196] = -18'sd282;  rom[3197] = -18'sd231;  rom[3198] =  18'sd163;  rom[3199] =  18'sd336;
        rom[3200] =  18'sd22;  rom[3201] = -18'sd341;  rom[3202] = -18'sd218;  rom[3203] =  18'sd232;
        rom[3204] =  18'sd362;  rom[3205] = -18'sd35;  rom[3206] = -18'sd400;  rom[3207] = -18'sd193;
        rom[3208] =  18'sd309;  rom[3209] =  18'sd379;  rom[3210] = -18'sd106;  rom[3211] = -18'sd458;
        rom[3212] = -18'sd151;  rom[3213] =  18'sd393;  rom[3214] =  18'sd384;  rom[3215] = -18'sd191;
        rom[3216] = -18'sd512;  rom[3217] = -18'sd93;  rom[3218] =  18'sd482;  rom[3219] =  18'sd374;
        rom[3220] = -18'sd290;  rom[3221] = -18'sd557;  rom[3222] = -18'sd16;  rom[3223] =  18'sd574;
        rom[3224] =  18'sd346;  rom[3225] = -18'sd402;  rom[3226] = -18'sd592;  rom[3227] =  18'sd81;
        rom[3228] =  18'sd665;  rom[3229] =  18'sd297;  rom[3230] = -18'sd525;  rom[3231] = -18'sd611;
        rom[3232] =  18'sd198;  rom[3233] =  18'sd752;  rom[3234] =  18'sd224;  rom[3235] = -18'sd658;
        rom[3236] = -18'sd611;  rom[3237] =  18'sd337;  rom[3238] =  18'sd831;  rom[3239] =  18'sd124;
        rom[3240] = -18'sd798;  rom[3241] = -18'sd586;  rom[3242] =  18'sd497;  rom[3243] =  18'sd897;
        rom[3244] = -18'sd5;  rom[3245] = -18'sd941;  rom[3246] = -18'sd533;  rom[3247] =  18'sd677;
        rom[3248] =  18'sd946;  rom[3249] = -18'sd166;  rom[3250] = -18'sd1083;  rom[3251] = -18'sd448;
        rom[3252] =  18'sd876;  rom[3253] =  18'sd970;  rom[3254] = -18'sd359;  rom[3255] = -18'sd1220;
        rom[3256] = -18'sd324;  rom[3257] =  18'sd1090;  rom[3258] =  18'sd964;  rom[3259] = -18'sd588;
        rom[3260] = -18'sd1346;  rom[3261] = -18'sd156;  rom[3262] =  18'sd1319;  rom[3263] =  18'sd920;
        rom[3264] = -18'sd853;  rom[3265] = -18'sd1453;  rom[3266] =  18'sd59;  rom[3267] =  18'sd1557;
        rom[3268] =  18'sd832;  rom[3269] = -18'sd1155;  rom[3270] = -18'sd1537;  rom[3271] =  18'sd330;
        rom[3272] =  18'sd1803;  rom[3273] =  18'sd690;  rom[3274] = -18'sd1497;  rom[3275] = -18'sd1586;
        rom[3276] =  18'sd662;  rom[3277] =  18'sd2050;  rom[3278] =  18'sd483;  rom[3279] = -18'sd1882;
        rom[3280] = -18'sd1593;  rom[3281] =  18'sd1067;  rom[3282] =  18'sd2296;  rom[3283] =  18'sd197;
        rom[3284] = -18'sd2314;  rom[3285] = -18'sd1544;  rom[3286] =  18'sd1559;  rom[3287] =  18'sd2536;
        rom[3288] = -18'sd187;  rom[3289] = -18'sd2802;  rom[3290] = -18'sd1423;  rom[3291] =  18'sd2158;
        rom[3292] =  18'sd2767;  rom[3293] = -18'sd698;  rom[3294] = -18'sd3363;  rom[3295] = -18'sd1207;
        rom[3296] =  18'sd2904;  rom[3297] =  18'sd2987;  rom[3298] = -18'sd1384;  rom[3299] = -18'sd4029;
        rom[3300] = -18'sd859;  rom[3301] =  18'sd3862;  rom[3302] =  18'sd3197;  rom[3303] = -18'sd2331;
        rom[3304] = -18'sd4864;  rom[3305] = -18'sd312;  rom[3306] =  18'sd5168;  rom[3307] =  18'sd3405;
        rom[3308] = -18'sd3714;  rom[3309] = -18'sd6011;  rom[3310] =  18'sd572;  rom[3311] =  18'sd7128;
        rom[3312] =  18'sd3648;  rom[3313] = -18'sd5956;  rom[3314] = -18'sd7851;  rom[3315] =  18'sd2137;
        rom[3316] =  18'sd10613;  rom[3317] =  18'sd4077;  rom[3318] = -18'sd10381;  rom[3319] = -18'sd11774;
        rom[3320] =  18'sd5562;  rom[3321] =  18'sd19359;  rom[3322] =  18'sd5866;  rom[3323] = -18'sd24142;
        rom[3324] = -18'sd28826;  rom[3325] =  18'sd17617;  rom[3326] =  18'sd87594;  rom[3327] =  18'sd121248;
    end

    // which filter set a mode and width use, and by how much its output is shifted
    function [3:0] fset_of;
        input [2:0] m;
        input [1:0] w;
        begin
            case ({m, w})
                {3'd0, 2'd0}: fset_of = 4'd6 ;     // AM   3000 Hz
                {3'd0, 2'd1}: fset_of = 4'd8 ;     // AM   6000 Hz
                {3'd0, 2'd2}: fset_of = 4'd9 ;     // AM   9000 Hz
                {3'd0, 2'd3}: fset_of = 4'd10;     // AM  12000 Hz
                {3'd1, 2'd0}: fset_of = 4'd6 ;     // SAM  3000 Hz
                {3'd1, 2'd1}: fset_of = 4'd8 ;     // SAM  6000 Hz
                {3'd1, 2'd2}: fset_of = 4'd9 ;     // SAM  9000 Hz
                {3'd1, 2'd3}: fset_of = 4'd10;     // SAM 12000 Hz
                {3'd2, 2'd0}: fset_of = 4'd4 ;     // USB  1800 Hz
                {3'd2, 2'd1}: fset_of = 4'd5 ;     // USB  2400 Hz
                {3'd2, 2'd2}: fset_of = 4'd6 ;     // USB  3000 Hz
                {3'd2, 2'd3}: fset_of = 4'd7 ;     // USB  4000 Hz
                {3'd3, 2'd0}: fset_of = 4'd4 ;     // LSB  1800 Hz
                {3'd3, 2'd1}: fset_of = 4'd5 ;     // LSB  2400 Hz
                {3'd3, 2'd2}: fset_of = 4'd6 ;     // LSB  3000 Hz
                {3'd3, 2'd3}: fset_of = 4'd7 ;     // LSB  4000 Hz
                {3'd4, 2'd0}: fset_of = 4'd0 ;     // CW    250 Hz
                {3'd4, 2'd1}: fset_of = 4'd1 ;     // CW    500 Hz
                {3'd4, 2'd2}: fset_of = 4'd2 ;     // CW    800 Hz
                {3'd4, 2'd3}: fset_of = 4'd3 ;     // CW   1200 Hz
                {3'd5, 2'd0}: fset_of = 4'd9 ;     // FM   9000 Hz
                {3'd5, 2'd1}: fset_of = 4'd10;     // FM  12000 Hz
                {3'd5, 2'd2}: fset_of = 4'd11;     // FM  16000 Hz
                {3'd5, 2'd3}: fset_of = 4'd12;     // FM  20000 Hz
                {3'd6, 2'd0}: fset_of = 4'd1 ;     // DIG   500 Hz
                {3'd6, 2'd1}: fset_of = 4'd3 ;     // DIG  1200 Hz
                {3'd6, 2'd2}: fset_of = 4'd6 ;     // DIG  3000 Hz
                {3'd6, 2'd3}: fset_of = 4'd7 ;     // DIG  4000 Hz
                default: fset_of = 4'd8;
            endcase
        end
    endfunction
    function [4:0] sh_of;
        input [3:0] k;
        begin
            case (k)
                4'd0 : sh_of = 5'd19;
                4'd1 : sh_of = 5'd19;
                4'd2 : sh_of = 5'd19;
                4'd3 : sh_of = 5'd19;
                4'd4 : sh_of = 5'd18;
                4'd5 : sh_of = 5'd18;
                4'd6 : sh_of = 5'd18;
                4'd7 : sh_of = 5'd17;
                4'd8 : sh_of = 5'd17;
                4'd9 : sh_of = 5'd16;
                4'd10: sh_of = 5'd15;
                4'd11: sh_of = 5'd15;
                4'd12: sh_of = 5'd15;
                default: sh_of = 5'd17;
            endcase
        end
    endfunction
    // the step of the turning angle for a sample: the centre of the sideband (CW: the pitch)
    function [31:0] pinc_of;
        input [2:0] m;
        input [1:0] w;
        begin
            case ({m, w})
                {3'd2, 2'd0}: pinc_of = 32'd92358977  ;     // USB  1800 Hz: 1050.0 Hz
                {3'd2, 2'd1}: pinc_of = 32'd118747256 ;     // USB  2400 Hz: 1350.0 Hz
                {3'd2, 2'd2}: pinc_of = 32'd145135535 ;     // USB  3000 Hz: 1650.0 Hz
                {3'd2, 2'd3}: pinc_of = 32'd189116000 ;     // USB  4000 Hz: 2150.0 Hz
                {3'd3, 2'd0}: pinc_of = 32'd92358977  ;     // LSB  1800 Hz: 1050.0 Hz
                {3'd3, 2'd1}: pinc_of = 32'd118747256 ;     // LSB  2400 Hz: 1350.0 Hz
                {3'd3, 2'd2}: pinc_of = 32'd145135535 ;     // LSB  3000 Hz: 1650.0 Hz
                {3'd3, 2'd3}: pinc_of = 32'd189116000 ;     // LSB  4000 Hz: 2150.0 Hz
                {3'd4, 2'd0}: pinc_of = 32'd61572651  ;     // CW    250 Hz: 700.0 Hz
                {3'd4, 2'd1}: pinc_of = 32'd61572651  ;     // CW    500 Hz: 700.0 Hz
                {3'd4, 2'd2}: pinc_of = 32'd61572651  ;     // CW    800 Hz: 700.0 Hz
                {3'd4, 2'd3}: pinc_of = 32'd61572651  ;     // CW   1200 Hz: 700.0 Hz
                {3'd6, 2'd0}: pinc_of = 32'd131941395 ;     // DIG   500 Hz: 1500.0 Hz
                {3'd6, 2'd1}: pinc_of = 32'd131941395 ;     // DIG  1200 Hz: 1500.0 Hz
                {3'd6, 2'd2}: pinc_of = 32'd145135535 ;     // DIG  3000 Hz: 1650.0 Hz
                {3'd6, 2'd3}: pinc_of = 32'd189116000 ;     // DIG  4000 Hz: 2150.0 Hz
                default: pinc_of = 32'd0;
            endcase
        end
    endfunction
    // the audio band for the noise reduction: {highest cell kept, highest cell measured}
    function [15:0] nrb_of;
        input [2:0] m;
        input [1:0] w;
        begin
            case ({m, w})
                {3'd0, 2'd0}: nrb_of = {8'd37, 8'd25};
                {3'd0, 2'd1}: nrb_of = {8'd68, 8'd56};
                {3'd0, 2'd2}: nrb_of = {8'd100, 8'd88};
                {3'd0, 2'd3}: nrb_of = {8'd131, 8'd119};
                {3'd1, 2'd0}: nrb_of = {8'd37, 8'd25};
                {3'd1, 2'd1}: nrb_of = {8'd68, 8'd56};
                {3'd1, 2'd2}: nrb_of = {8'd100, 8'd88};
                {3'd1, 2'd3}: nrb_of = {8'd131, 8'd119};
                {3'd2, 2'd0}: nrb_of = {8'd46, 8'd35};
                {3'd2, 2'd1}: nrb_of = {8'd59, 8'd47};
                {3'd2, 2'd2}: nrb_of = {8'd71, 8'd60};
                {3'd2, 2'd3}: nrb_of = {8'd92, 8'd81};
                {3'd3, 2'd0}: nrb_of = {8'd46, 8'd35};
                {3'd3, 2'd1}: nrb_of = {8'd59, 8'd47};
                {3'd3, 2'd2}: nrb_of = {8'd71, 8'd60};
                {3'd3, 2'd3}: nrb_of = {8'd92, 8'd81};
                {3'd4, 2'd0}: nrb_of = {8'd37, 8'd25};
                {3'd4, 2'd1}: nrb_of = {8'd37, 8'd25};
                {3'd4, 2'd2}: nrb_of = {8'd37, 8'd25};
                {3'd4, 2'd3}: nrb_of = {8'd37, 8'd25};
                {3'd5, 2'd0}: nrb_of = {8'd71, 8'd60};
                {3'd5, 2'd1}: nrb_of = {8'd71, 8'd60};
                {3'd5, 2'd2}: nrb_of = {8'd71, 8'd60};
                {3'd5, 2'd3}: nrb_of = {8'd71, 8'd60};
                {3'd6, 2'd0}: nrb_of = {8'd42, 8'd30};
                {3'd6, 2'd1}: nrb_of = {8'd49, 8'd38};
                {3'd6, 2'd2}: nrb_of = {8'd71, 8'd60};
                {3'd6, 2'd3}: nrb_of = {8'd92, 8'd81};
                default: nrb_of = {8'd68, 8'd56};
            endcase
        end
    endfunction
    // squelch: the detector's reading on noise alone depends on the width; open below, close above
    function [35:0] sqo_of;
        input [1:0] w;
        begin
            case (w)
                2'd0: sqo_of = 36'd58720256;
                2'd1: sqo_of = 36'd95420416;
                2'd2: sqo_of = 36'd157286400;
                2'd3: sqo_of = 36'd225443840;
            endcase
        end
    endfunction
    function [35:0] sqc_of;
        input [1:0] w;
        begin
            case (w)
                2'd0: sqc_of = 36'd116391936;
                2'd1: sqc_of = 36'd190840832;
                2'd2: sqc_of = 36'd314572800;
                2'd3: sqc_of = 36'd450887680;
            endcase
        end
    endfunction
    // the fast reading shuts the squelch above this: a station has just gone
    function [35:0] sqf_of;
        input [1:0] w;
        begin
            case (w)
                2'd0: sqf_of = 36'd189792256;
                2'd1: sqf_of = 36'd310378496;
                2'd2: sqf_of = 36'd513802240;
                2'd3: sqf_of = 36'd735051776;
            endcase
        end
    endfunction
    function signed [24:0] sat25;
        input signed [31:0] v;
        begin
            sat25 = (v > 32'sd16777215) ? 25'sd16777215 : (v < -32'sd16777216) ? -25'sd16777216 : v[24:0];
        end
    endfunction

    reg  [8:0]  wp = 9'd0;                // where the newest sample is
    reg  [8:0]  fk = 9'd0;                // tap number, 0..510
    reg  [1:0]  bw_s = 2'd1;              // settings taken over at the tick
    reg  [2:0]  mode_s = 3'd0;
    reg  [15:0] ctrl_s = 16'd0;
    reg  [3:0]  fset_s = 4'd8;            // the filter set of this sample
    reg  [4:0]  sh_s   = 5'd17;           // and its output shift
    reg  [49:0] wdat   = 50'd0;           // the sample on its way into the filter's memory
    wire        ssb_s  = (mode_s == 3'd2) || (mode_s == 3'd3) || (mode_s == 3'd4) || (mode_s == 3'd6);
    wire        fm_s   = (mode_s == 3'd5);
    wire        prerot = (mode_s == 3'd2) || (mode_s == 3'd3) || (mode_s == 3'd6);
    wire        chg    = (mode != mode_s) || (bw != bw_s);   // another mode or width than the last sample had
    wire [7:0]  cidx = fk[8] ? (8'd254 - fk[7:0]) : fk[7:0];     // taps 256..510 mirror 254..0
    wire [8:0]  ra   = wp - fk;

    reg  [49:0]        mq = 50'd0;
    reg  signed [17:0] cq = 18'sd0;
    reg  signed [42:0] pi = 43'sd0, pq = 43'sd0;
    reg  signed [47:0] ai = 48'sd0, aq = 48'sd0;
    reg  [1:0]         fv = 2'b00;        // [0] the RAM word is valid, [1] the product is valid
    reg                frd = 1'b0;        // a read is under way

    always @(posedge aclk) begin
        mq <= mem[ra];
        cq <= rom[{fset_s, cidx}];
        pi <= $signed(mq[24:0])  * cq;
        pq <= $signed(mq[49:25]) * cq;
    end

    reg signed [29:0] yi = 30'sd0, yq = 30'sd0;                  // filter output
    wire signed [47:0] rnd = 48'sd1 <<< (sh_s - 5'd1);
    wire signed [47:0] ri = ai + rnd;
    wire signed [47:0] rq = aq + rnd;
    wire signed [47:0] si = ri >>> sh_s;
    wire signed [47:0] sq = rq >>> sh_s;

    // =====================================================================================
    //  the two CORDICs and the multiplier, each a small sequential unit
    // =====================================================================================
    reg                cstart = 1'b0;
    wire               cdone;
    wire signed [31:0] v_mag, v_unused, r_i, r_q;
    wire [31:0]        v_phi, r_unused;
    reg  [31:0]        theta = 32'd0;     // the loop's oscillator angle, 2^32 = one turn

    rxa_cordic u_vec (.clk(aclk), .start(cstart), .rot(1'b0), .xi(yi), .yi(yq), .ai(32'd0),
                      .xo(v_mag), .yo(v_unused), .ao(v_phi), .done(cdone));
    // The rotating CORDIC has two jobs in a sample.  In front of the filter (USB, LSB, DIG) it
    // turns the sample at the sideband's centre frequency.  Behind the filter it turns the
    // result by the carrier loop's angle (AM, SAM, FM) or back up to audio (USB, LSB, CW, DIG).
    reg                rstart = 1'b0;     // start it alone, for the turn in front of the filter
    reg                pre    = 1'b0;     // it is doing that turn
    reg  [31:0]        phc    = 32'd0;    // angle of the sideband's centre frequency (CW: of the pitch)
    wire               lsb_s  = (mode_s == 3'd3);
    wire signed [29:0] rxi  = pre ? {{5{nb_i[24]}}, nb_i} : yi;
    wire signed [29:0] ryi  = pre ? {{5{nb_q[24]}}, nb_q} : yq;
    wire [31:0]        rang = pre   ? (lsb_s ? (32'd0 - phc) : phc) :
                              ssb_s ? (lsb_s ? phc : (32'd0 - phc)) : (32'd0 - theta);
    wire               rdone;
    rxa_cordic u_rot (.clk(aclk), .start(cstart || rstart), .rot(1'b1), .xi(rxi), .yi(ryi), .ai(rang),
                      .xo(r_i), .yo(r_q), .ao(r_unused), .done(rdone));

    reg                mstart = 1'b0;
    reg  signed [31:0] ma = 32'sd0;
    reg  signed [23:0] mb = 24'sd0;
    wire signed [55:0] mp;
    wire               mdone;
    rxa_smul u_mul (.clk(aclk), .start(mstart), .a(ma), .b(mb), .p(mp), .done(mdone));

    // =====================================================================================
    //  one sample, step by step
    // =====================================================================================
    localparam [4:0] S_IDLE = 5'd0,  S_FIR  = 5'd1,  S_FLUSH = 5'd2, S_ROUND = 5'd3, S_CORD = 5'd4,
                     S_PLL  = 5'd5,  S_PLL2 = 5'd6,  S_DC1  = 5'd7,  S_DC2   = 5'd8, S_AGC  = 5'd9,
                     S_AGC2 = 5'd10, S_DIV  = 5'd11, S_DIV2 = 5'd12, S_B0    = 5'd13, S_B1  = 5'd14,
                     S_B2   = 5'd15, S_B3   = 5'd16, S_VOL  = 5'd17, S_LIM   = 5'd18, S_LIM2 = 5'd19,
                     S_OUT  = 5'd20, S_NB   = 5'd21, S_NR   = 5'd22, S_NR2   = 5'd23, S_NR3 = 5'd24,
                     S_PRE  = 5'd25, S_STORE = 5'd26, S_B4  = 5'd27;
    reg [4:0] st = S_IDLE;

    // ---- noise reduction: it always runs; its output is used when it is switched on -----------
    reg                nr_tick = 1'b0;
    reg                big_r   = 1'b0;    // this sample came right after a big retune
    reg                nr_on   = 1'b0;
    wire signed [21:0] nr_y;

    // carrier loop
    reg  signed [47:0] omega  = 48'sd0;   // its frequency, 16 more bits below theta's
    reg  [31:0]        mlock  = 32'h40000000;      // mean size of the phase error, starts at 90 degrees
    reg                locked = 1'b0;
    reg                pol    = 1'b0;     // 1 = the in-phase part is upside down
    reg  signed [42:0] cr     = 43'sd0;   // slow mean of the in-phase part
    reg  [31:0]        tune_r = 32'd0;
    reg  [11:0]        mute   = 12'd0;
    reg  [9:0]         settle = 10'd0;    // samples left of the settling time after a big retune
    reg  [10:0]        pc     = 11'd0;    // clocks since the sample arrived
    localparam [10:0]  OUT_AT = 11'd1000;  // the count at which a sample leaves: two clocks later it is out
    // detector, hum filter, AGC
    reg  signed [31:0] mag = 32'sd0, rI = 32'sd0, rQ = 32'sd0;
    reg  [31:0]        phi = 32'd0;
    reg  signed [32:0] d = 33'sd0, a1 = 33'sd0, a2 = 33'sd0;
    reg  signed [40:0] s1f = 41'sd0, s2f = 41'sd0;
    reg  signed [42:0] cf = 43'sd0;
    reg  [41:0]        lf = 42'd0;        // the level the audio is divided by, 10 bits of fraction
    reg                sam = 1'b0;
    // divider
    reg  [47:0]        dnum = 48'd0, dquo = 48'd0;
    reg  [32:0]        drem = 33'd0;
    reg  [31:0]        dden = 32'd1;
    reg  [5:0]         dcnt = 6'd0;
    reg                dneg = 1'b0;
    reg  signed [20:0] q = 21'sd0;
    // FM: frequency, de-emphasis, squelch
    reg  [31:0]        phi1 = 32'd0;      // the angle one sample ago
    reg  signed [31:0] f1 = 32'sd0, f2 = 32'sd0;                  // the frequency one and two samples ago
    reg  signed [36:0] de_f = 37'sd0;     // de-emphasised frequency, 5 bits of fraction
    reg  [45:0]        sq_avg = 46'd0;    // squelch: mean size of the noise above the voice, x 1024
    reg  [41:0]        sq_fst = 42'd0;    // the same over 128 samples only, x 128: quick to see a station go
    reg                sq_open = 1'b0;
    // four second-order sections, one after the other: the last two inputs and outputs of each
    reg  signed [21:0] bx1 [0:3], bx2 [0:3];                      // inputs, one and two samples ago
    reg  signed [31:0] by1 [0:3], by2 [0:3];                      // outputs, 8 of the 32 bits are fraction
    integer            bi;
    initial for (bi = 0; bi < 4; bi = bi + 1) begin
        bx1[bi] = 22'sd0;  bx2[bi] = 22'sd0;  by1[bi] = 32'sd0;  by2[bi] = 32'sd0;
    end
    reg  [1:0]         slot = 2'd0;
    reg  signed [21:0] v = 22'sd0;        // the sample on its way through them
    reg  signed [55:0] nacc = 56'sd0;
    reg  signed [21:0] qn = 22'sd0;
    // FM: the audio is held back by 384 samples (7.9 ms).  When a station stops, the detector
    // turns to loud noise at once; the squelch needs a few milliseconds to notice, and by then
    // it has shut before that noise gets here.
    (* ram_style = "block" *) reg [21:0] dly [0:511];
    reg  [8:0]         dp    = 9'd0;      // where the next sample is written
    reg  [8:0]         dhold = 9'd0;      // samples for which the held-back audio is still from before a change
    reg  signed [21:0] qdr   = 22'sd0;
    integer di;
    initial for (di = 0; di < 512; di = di + 1) dly[di] = 22'd0;
    wire               dl_re = (st == S_PLL2);
    wire               dl_we = (st == S_NR);
    always @(posedge aclk) begin
        if (dl_re) qdr <= dly[dp + 9'd128];                       // the one written 384 samples ago
        if (dl_we) dly[dp] <= qn;
    end
    wire signed [21:0] qx = !fm_s ? qn : (dhold != 9'd0) ? 22'sd0 : qdr;
    wire               nr_use = nr_on && !((mode_s == 3'd4) || (mode_s == 3'd6));
    wire signed [21:0] qv = nr_use ? nr_y : qx;                   // what goes on to the volume control
    wire [15:0]        nrb = nrb_of(mode_s, bw_s);

    rxa_nr u_nr (.clk(aclk), .rst(!aresetn), .tick(nr_tick), .x_i(qx), .rq_i(rQ), .lden_i(dden),
                 .locked_i(locked && (mode_s <= 3'd1)), .khi_i(nrb[15:8]), .ohi_i(nrb[7:0]),
                 .fsel_i(ctrl_s[13:12]), .big_i(big_r), .y_o(nr_y));
    // volume, limiter, output
    reg  signed [33:0] u = 34'sd0;
    reg  [5:0]         tc = 6'd0;
    wire [5:0]         tc_n = tc + 6'd1;
    reg                clip = 1'b0;
    reg  [11:0]        clip_hold = 12'd0;

    // ---- carrier loop arithmetic -----------------------------------------------------------
    wire signed [31:0] e    = phi - theta;                        // phase error, a full turn wide
    wire signed [31:0] ec   = {e[30], e[30:0]};                   // the same, modulo half a turn
    wire               hold = ($unsigned(mag) < lf[41:13]);       // hardly any signal: coast
    wire signed [31:0] ee   = hold ? 32'sd0 : ec;
    wire signed [47:0] ee48 = {{16{ee[31]}}, ee};
    wire signed [47:0] om_n = omega + (locked ? (ee48 >>> 6) : (ee48 <<< 2));     // gains 2^-22, 2^-14
    localparam signed [47:0] OM_MAX = 48'sd35184372088832;        // 2^45: an eighth of a turn per sample
    wire signed [47:0] om_c = (om_n > OM_MAX) ? OM_MAX : (om_n < -OM_MAX) ? -OM_MAX : om_n;
    wire signed [31:0] eep  = locked ? (ee >>> 10) : (ee >>> 6);  // gains 2^-10, 2^-6
    wire [31:0]        ae   = ec[31] ? (32'd0 - ec) : ec;
    wire signed [32:0] mdif = $signed({1'b0, ae}) - $signed({1'b0, mlock});
    wire signed [32:0] mstp = (locked && (ae > mlock)) ? (mdif >>> 14) : (mdif >>> 11);
    wire [31:0]        ml_n = mlock + mstp[31:0];
    wire signed [42:0] cr_n = cr + (($signed({rI, 10'd0}) - cr) >>> 10);

    // ---- hum filter and AGC arithmetic -----------------------------------------------------
    // For 768 samples after a big retune the sound is muted anyway. The filter still holds the
    // old station for the first 576 of them. In that time the slow averages are made fast, so
    // that they sit on the new station when the mute ends:
    //   hum filter 0.3 ms instead of 2.6 ms; carrier level: no memory at first, then 1.3 ms
    //   instead of 21 ms; and the AGC level follows downwards at once, too.
    wire               fast  = (settle != 10'd0);
    wire               fresh = (settle > 10'd192);
    wire signed [40:0] s1_d = $signed({d, 8'd0}) - s1f;
    wire signed [40:0] s1_n = s1f + (fast ? (s1_d >>> 4) : (s1_d >>> 7));
    wire signed [40:0] s1_s = s1_n >>> 8;
    wire signed [32:0] a1_n = d - s1_s[32:0];
    wire signed [40:0] s2_d = $signed({a1, 8'd0}) - s2f;
    wire signed [40:0] s2_n = s2f + (fast ? (s2_d >>> 4) : (s2_d >>> 7));
    wire signed [40:0] s2_s = s2_n >>> 8;
    wire signed [32:0] a2_n = a1 - s2_s[32:0];
    wire signed [42:0] cf_d = $signed({d, 10'd0}) - cf;
    wire signed [42:0] cf_n = fresh ? $signed({d, 10'd0}) : (cf + (fast ? (cf_d >>> 6) : (cf_d >>> 10)));
    wire [41:0]        cabs = cf[42] ? (42'd0 - cf[41:0]) : cf[41:0];
    wire [41:0]        mpk  = {1'b0, mag[31:0], 9'd0};            // half the instantaneous size
    // AM, SAM (and FM, where it is not used): the carrier level, or half a sudden peak.
    // The sideband modes have no carrier: there the peak itself is the level.
    wire [41:0]        c10  = ssb_s ? {mag[31:0], 10'd0} : ((mpk > cabs) ? mpk : cabs);
    // how fast the level falls: 26 dB a second; "fast" 104; DIG 6.5 (fast: 26)
    wire [41:0]        ldif = lf - c10;
    wire [41:0]        lstp = (mode_s == 3'd6) ? (ctrl_s[9] ? (ldif >> 14) : (ldif >> 16))
                                               : (ctrl_s[9] ? (ldif >> 12) : (ldif >> 14));
    wire [41:0]        ldec = lf - lstp;
    wire [41:0]        lf_n = ((c10 > lf) || fast) ? c10 : ldec;
    wire [31:0]        lmin = (ctrl_s[7:6] == 2'd0) ? 32'd1024 : (ctrl_s[7:6] == 2'd1) ? 32'd4096 :
                              (ctrl_s[7:6] == 2'd2) ? 32'd16384 : 32'd256;
    wire [31:0]        lint = lf[41:10];
    wire signed [47:0] num  = {a2[32:0], 15'd0};
    wire [47:0]        nabs = num[47] ? (48'd0 - num) : num;

    // ---- divider step ----------------------------------------------------------------------
    wire [32:0] dsh  = {drem[31:0], dnum[47]};
    wire        dge  = (dsh >= {1'b0, dden});
    wire [32:0] dsub = dsh - {1'b0, dden};

    // ---- FM arithmetic ---------------------------------------------------------------------
    wire signed [31:0] fw   = phi - phi1;                         // the step of the angle = the frequency
    wire signed [34:0] n2w  = $signed({{3{fw[31]}}, fw}) - $signed({{2{f1[31]}}, f1, 1'b0}) + $signed({{3{f2[31]}}, f2});
    wire [34:0]        n2a  = n2w[34] ? (35'd0 - n2w) : n2w;      // size of its second difference: the high notes
    wire signed [38:0] de_d = $signed({{2{fw[31]}}, fw, 5'd0}) - $signed({{2{de_f[36]}}, de_f});
    wire signed [38:0] de_n = $signed({{2{de_f[36]}}, de_f}) + (de_d >>> 5);
    wire [45:0]        sq_n = sq_avg + {11'd0, n2a} - {10'd0, sq_avg[45:10]};
    wire [35:0]        sq_l = sq_avg[45:10];
    wire [41:0]        sqf_n = sq_fst + {7'd0, n2a} - {7'd0, sq_fst[41:7]};
    wire [35:0]        sqf_l = {1'b0, sq_fst[41:7]};
    wire               sq_mute = fm_s && !sq_open && !ctrl_s[5];

    // ---- the four second-order sections ----------------------------------------------------
    //   y = x + b1 x1 + x2 + a1 y1 + a2 y2  (the low-pass takes x, x1, x2 at 1/32)
    //   which one a slot holds: 0 none, 1 the 5 kHz notch, 2..4 the high-pass, 5 the low-pass
    wire [2:0]         bsel = (((mode_s == 3'd0) || (mode_s == 3'd1)) && (bw_s == 2'd3) && !ctrl_s[8] && (slot == 2'd0)) ? 3'd1 :
                              fm_s ? (3'd2 + {1'b0, slot}) : 3'd0;
    reg  signed [23:0] bq_b1, bq_a1, bq_a2;
    always @* begin
        case (bsel)
            3'd1: begin bq_b1 = -24'sd104866;  bq_a1 = 24'sd104046;  bq_a2 = -24'sd64516; end     // the 5 kHz notch
            3'd2: begin bq_b1 = -24'sd131072;  bq_a1 = 24'sd126267;  bq_a2 = -24'sd60825; end     // high-pass, first part
            3'd3: begin bq_b1 = -24'sd131072;  bq_a1 = 24'sd127495;  bq_a2 = -24'sd62054; end     // high-pass, second part
            3'd4: begin bq_b1 = -24'sd131072;  bq_a1 = 24'sd129679;  bq_a2 = -24'sd64240; end     // high-pass, third part
            3'd5: begin bq_b1 = 24'sd4096;  bq_a1 = 24'sd94465;  bq_a2 = -24'sd37121; end     // low-pass 3.1 kHz
            default: begin bq_b1 = 24'sd0;  bq_a1 = 24'sd0;  bq_a2 = 24'sd0; end
        endcase
    end
    wire signed [21:0] x1s = bx1[slot];
    wire signed [21:0] x2s = bx2[slot];
    wire signed [31:0] y1s = by1[slot];
    wire signed [31:0] y2s = by2[slot];
    wire signed [55:0] bxin = (bsel == 3'd5) ? ($signed({v, 19'd0}) + $signed({x2s, 19'd0}))
                                             : ($signed({v, 24'd0}) + $signed({x2s, 24'd0}));
    wire signed [55:0] bsum = nacc + 56'sd32768;
    wire signed [55:0] bshf = bsum >>> 16;                        // the new output, 8 bits of fraction
    wire signed [31:0] y8s  = (bshf > 56'sd2147483647) ? 32'sh7FFFFFFF :
                              (bshf < -56'sd2147483648) ? 32'sh80000000 : bshf[31:0];
    wire signed [32:0] bo   = ($signed({y8s[31], y8s}) + 33'sd128) >>> 8;
    wire signed [21:0] vs   = (bo > 33'sd2097151) ? 22'sd2097151 : (bo < -33'sd2097152) ? -22'sd2097152 : bo[21:0];
    wire signed [20:0] qc   = (dquo > 48'd524287) ? (dneg ? -21'sd524287 : 21'sd524287)
                                                  : (dneg ? -$signed({1'b0, dquo[19:0]}) : $signed({1'b0, dquo[19:0]}));

    // ---- volume and limiter arithmetic -----------------------------------------------------
    wire signed [30:0] tv  = ctrl_s[0] ? $signed({qv, 8'd0})       // x 256
                                       : ($signed({qv, 7'd0}) + $signed({qv, 5'd0}) + $signed({qv, 4'd0}) +
                                          $signed({qv, 2'd0}) + qv);          // x 181
    wire signed [37:0] tsh = $signed({{7{tv[30]}}, tv}) <<< ctrl_s[3:1];
    wire signed [37:0] u_n = tsh >>> 5;
    wire [33:0]        ax  = u[33] ? (34'd0 - u) : u;
    localparam [33:0]  LIM_T = 34'd4194304;                       // half of full scale
    wire               over = (ax > LIM_T);
    wire               flat = (ax >= 34'd12582912);               // three times that: fully limited
    wire [33:0]        dd   = ax - LIM_T;
    wire [33:0]        yl_k = ax - {2'b00, mp[55:24]};            // ax - dd^2 / 2^24
    wire [33:0]        yl   = !over ? ax : flat ? 34'd8388607 : (yl_k > 34'd8388607) ? 34'd8388607 : yl_k;
    wire [23:0]        outv = u[33] ? (24'd0 - yl[23:0]) : yl[23:0];

    wire signed [31:0] dw  = tune_i - tune_r;
    wire               big = (dw >= 32'sd131072) || (dw <= -32'sd131072);

    always @(posedge aclk) begin
        cstart        <= 1'b0;
        rstart        <= 1'b0;
        mstart        <= 1'b0;
        nr_tick       <= 1'b0;
        m_axis_tvalid <= 1'b0;
        pc            <= pc + 11'd1;
        fv            <= {fv[0], frd};
        if (fv[1]) begin
            ai <= ai + pi;
            aq <= aq + pq;
        end

        if (!aresetn) begin
            st <= S_IDLE;   frd <= 1'b0;   fv <= 2'b00;
            theta <= 32'd0; omega <= 48'sd0; mlock <= 32'h40000000; locked <= 1'b0; pol <= 1'b0;
            cr <= 43'sd0;   cf <= 43'sd0;  s1f <= 41'sd0; s2f <= 41'sd0; lf <= 42'd0;
            tc <= 6'd0;     mute <= 12'd0; tune_r <= 32'd0; clip <= 1'b0;  big_r <= 1'b0;  nr_on <= 1'b0;
            settle <= 10'd0;
            bw_s <= 2'd1;   mode_s <= 3'd0; fset_s <= 4'd8; sh_s <= 5'd17; pre <= 1'b0; phc <= 32'd0;
            phi1 <= 32'd0;  f1 <= 32'sd0;  f2 <= 32'sd0;  de_f <= 37'sd0;  sq_avg <= 46'd0;  sq_open <= 1'b0;
            sq_fst <= 42'd0;  dp <= 9'd0;  dhold <= 9'd0;
            for (bi = 0; bi < 4; bi = bi + 1) begin
                bx1[bi] <= 22'sd0;  bx2[bi] <= 22'sd0;  by1[bi] <= 32'sd0;  by2[bi] <= 32'sd0;
            end
        end else begin
            case (st)
                // ---- wait for a sample; take the settings over ------------------------------
                S_IDLE: if (tick) begin
                    pc     <= 11'd0;
                    bw_s   <= bw;
                    mode_s <= mode;
                    ctrl_s <= ctrl_i;
                    fset_s <= fset_of(mode, bw);
                    sh_s   <= sh_of(fset_of(mode, bw));
                    nr_on  <= ctrl_i[11] || nr_sw;
                    big_r  <= 1'b0;
                    // A retune starts the carrier loop again.  A big step, another mode or another
                    // width also starts the AGC and the squelch again, and mutes for a moment.
                    if ((tune_i != tune_r) || chg) begin
                        tune_r <= tune_i;
                        locked <= 1'b0;
                        mlock  <= 32'h40000000;
                        if (big || chg) begin
                            lf      <= 42'd0;
                            mute    <= (ctrl_i[11] || nr_sw) ? 12'd2304 : 12'd1024;   // the noise reduction is 1280 samples late
                            settle  <= 10'd768;
                            big_r   <= 1'b1;
                            sq_open <= 1'b0;
                            sq_avg  <= (mode == 3'd5) ? {sqc_of(bw), 10'd0} : 46'd0;
                            sq_fst  <= 42'd0;
                            dhold   <= 9'd384;
                        end
                    end
                    if (chg) begin
                        for (bi = 0; bi < 4; bi = bi + 1) begin
                            bx1[bi] <= 22'sd0;  bx2[bi] <= 22'sd0;  by1[bi] <= 32'sd0;  by2[bi] <= 32'sd0;
                        end
                        de_f <= 37'sd0;
                    end
                    st <= S_NB;
                end
                // ---- the blanker hands over the sample from 40 samples ago -------------------
                S_NB: if (nb_v) begin
                    if (prerot) begin                             // USB, LSB, DIG: turn it first
                        pre    <= 1'b1;
                        rstart <= 1'b1;
                        st     <= S_PRE;
                    end else begin
                        wdat <= {nb_q, nb_i};
                        st   <= S_STORE;
                    end
                end
                S_PRE: if (rdone) begin
                    wdat <= {sat25(r_q), sat25(r_i)};
                    pre  <= 1'b0;
                    st   <= S_STORE;
                end
                // ---- into the filter's memory ------------------------------------------------
                S_STORE: begin
                    mem[wp + 9'd1] <= wdat;
                    wp  <= wp + 9'd1;
                    fk  <= 9'd0;
                    ai  <= 48'sd0;
                    aq  <= 48'sd0;
                    frd <= 1'b1;
                    st  <= S_FIR;
                end
                // ---- 511 reads, one a clock ----------------------------------------------
                S_FIR: begin
                    if (fk == 9'd510) begin
                        frd <= 1'b0;
                        st  <= S_FLUSH;
                    end else begin
                        fk <= fk + 9'd1;
                    end
                end
                S_FLUSH: if (fv == 2'b00 && !frd) st <= S_ROUND;
                S_ROUND: begin
                    yi     <= si[29:0];
                    yq     <= sq[29:0];
                    cstart <= 1'b1;
                    st     <= S_CORD;
                end
                // ---- size and angle; and the signal turned by the loop's angle ------------
                S_CORD: if (cdone) begin
                    mag <= v_mag;
                    phi <= v_phi;
                    rI  <= r_i;
                    rQ  <= r_q;
                    st  <= S_PLL;
                end
                // ---- carrier loop ---------------------------------------------------------
                S_PLL: begin
                    omega <= om_c;
                    theta <= theta + om_c[47:16] + eep;
                    mlock <= ml_n;
                    cr    <= cr_n;
                    sam   <= (mode_s == 3'd1) && locked;
                    if (!((mode_s == 3'd1) && locked)) pol <= cr_n[42];
                    if (locked) begin
                        if (ml_n > 32'd477218588) locked <= 1'b0;
                    end else begin
                        if (ml_n < 32'd268435456) locked <= 1'b1;
                    end
                    // FM: frequency, de-emphasis, and the noise above the voice for the squelch
                    phi1 <= phi;
                    f2   <= f1;
                    f1   <= fw;
                    if (fm_s) begin
                        de_f   <= de_n[36:0];
                        sq_avg <= sq_n;
                        sq_fst <= sqf_n;
                    end
                    st <= S_PLL2;
                end
                S_PLL2: begin
                    d  <= fm_s  ? $signed({de_f[36], de_f[36:5]}) :
                          ssb_s ? $signed({rI[31], rI}) :
                          sam   ? (pol ? -$signed({rI[31], rI}) : $signed({rI[31], rI})) : $signed({mag[31], mag});
                    if (fm_s) begin
                        if (sq_open) begin
                            if (sq_l > sqc_of(bw_s)) begin
                                sq_open <= 1'b0;
                            end else if (sqf_l > sqf_of(bw_s)) begin          // the station has just gone
                                sq_open <= 1'b0;
                                sq_avg  <= {sqc_of(bw_s), 10'd0};             // the slow reading has to come down again
                            end
                        end else if ((settle == 10'd0) && (sq_l < sqo_of(bw_s))) begin
                            sq_open <= 1'b1;
                        end
                    end
                    st <= S_DC1;
                end
                // ---- hum filter -----------------------------------------------------------
                S_DC1: begin
                    s1f <= s1_n;
                    a1  <= a1_n;
                    cf  <= cf_n;
                    st  <= S_DC2;
                end
                S_DC2: begin
                    s2f <= s2_n;
                    a2  <= a2_n;
                    st  <= S_AGC;
                end
                // ---- AGC level, then audio x 2^15 / level --------------------------------
                S_AGC: begin
                    lf <= lf_n;
                    if (fast) settle <= settle - 10'd1;
                    st <= S_AGC2;
                end
                S_AGC2: begin
                    dden <= fm_s ? 32'd67108864 : ((lint > lmin) ? lint : lmin);
                    dnum <= nabs;
                    dneg <= num[47];
                    drem <= 33'd0;
                    dquo <= 48'd0;
                    dcnt <= 6'd0;
                    st   <= S_DIV;
                end
                S_DIV: begin
                    dnum <= {dnum[46:0], 1'b0};
                    drem <= dge ? dsub : dsh;
                    dquo <= {dquo[46:0], dge};
                    dcnt <= dcnt + 6'd1;
                    if (dcnt == 6'd47) st <= S_DIV2;
                end
                S_DIV2: begin
                    q    <= qc;
                    v    <= {qc[20], qc};
                    slot <= 2'd0;
                    st   <= S_B0;
                end
                // ---- four second-order sections, three products each ------------------------
                S_B0: begin
                    ma     <= {{2{x1s[21]}}, x1s, 8'd0};
                    mb     <= bq_b1;
                    mstart <= 1'b1;
                    st     <= S_B1;
                end
                S_B1: if (mdone) begin
                    nacc   <= mp + bxin;
                    ma     <= y1s;
                    mb     <= bq_a1;
                    mstart <= 1'b1;
                    st     <= S_B2;
                end
                S_B2: if (mdone) begin
                    nacc   <= nacc + mp;
                    ma     <= y2s;
                    mb     <= bq_a2;
                    mstart <= 1'b1;
                    st     <= S_B3;
                end
                S_B3: if (mdone) begin
                    nacc <= nacc + mp;
                    st   <= S_B4;
                end
                S_B4: begin
                    if (bsel != 3'd0) begin
                        bx2[slot] <= x1s;
                        bx1[slot] <= v;
                        by2[slot] <= y1s;
                        by1[slot] <= y8s;
                        v <= vs;
                    end else begin                                // not in circuit: it forgets
                        bx2[slot] <= 22'sd0;
                        bx1[slot] <= 22'sd0;
                        by2[slot] <= 32'sd0;
                        by1[slot] <= 32'sd0;
                    end
                    slot <= slot + 2'd1;
                    if (slot == 2'd3) begin
                        qn <= (bsel != 3'd0) ? vs : v;
                        st <= S_NR;
                    end else begin
                        st <= S_B0;
                    end
                end
                // ---- noise reduction: hand the sample over, take the one from 26 ms ago ------
                S_NR: begin
                    nr_tick <= 1'b1;
                    dp      <= dp + 9'd1;                         // the sample goes into the delay line at dp
                    st      <= S_NR2;
                end
                S_NR2: st <= S_NR3;
                S_NR3: st <= S_VOL;
                // ---- volume ---------------------------------------------------------------
                S_VOL: begin
                    u  <= u_n[33:0];
                    st <= S_LIM;
                end
                // ---- limiter: above half scale, bend the peak over ------------------------
                S_LIM: begin
                    ma     <= {8'd0, dd[23:0]};
                    mb     <= dd[23:0];
                    mstart <= over && !flat;
                    st     <= (over && !flat) ? S_LIM2 : S_OUT;
                end
                S_LIM2: if (mdone) st <= S_OUT;
                // The work above takes longer for some samples than for others (a bridged click,
                // a peak in the limiter). The sample waits here, so that it always leaves the same
                // number of clocks after it arrived.
                S_OUT: if (pc >= OUT_AT) begin
                    tc   <= tc + 6'd1;
                    clip <= over && (mute == 12'd0) && !sq_mute;      // a muted sample cannot clip
                    phc  <= phc + pinc_of(mode_s, bw_s);
                    if (mute != 12'd0) mute <= mute - 12'd1;
                    if (dhold != 9'd0) dhold <= dhold - 9'd1;
                    if (ctrl_s[4])                        m_axis_tdata <= tc_n[5] ? 24'h100000 : 24'hF00000;
                    else if ((mute != 12'd0) || sq_mute)  m_axis_tdata <= 24'd0;
                    else                                  m_axis_tdata <= outv;
                    m_axis_tvalid <= 1'b1;
                    st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

    // ---- status ----------------------------------------------------------------------------
    // the clip light stays on for 4096 samples (84 ms) so that it can be seen
    always @(posedge aclk) begin
        if (!aresetn)                              clip_hold <= 12'd0;
        else if (m_axis_tvalid && clip)            clip_hold <= 12'hFFF;
        else if (m_axis_tvalid && clip_hold != 0)  clip_hold <= clip_hold - 12'd1;
    end
    // "lock": the carrier loop in AM and SAM, "squelch open" in FM, nothing in the other modes
    wire lock_o = (mode_s <= 3'd1) ? locked : (fm_s ? sq_open : 1'b0);
    assign led_lock_o = lock_o;
    assign led_clip_o = (clip_hold != 12'd0);
    assign stat_o     = {led_clip_o, lock_o, 1'b0, bw, mode};

endmodule

// -----------------------------------------------------------------------------------------
// CORDIC, one step a clock, 24 steps.  Its result is 1.6468 times too long; nothing here minds.
//   rot = 0: turns the arrow (xi, yi) onto the x axis: xo = its length, ao = its angle
//   rot = 1: turns the arrow (xi, yi) by the angle ai
// Angles: 2^32 is one full turn.
// -----------------------------------------------------------------------------------------
module rxa_cordic (
    input  wire               clk,
    input  wire               start,
    input  wire               rot,
    input  wire signed [29:0] xi,
    input  wire signed [29:0] yi,
    input  wire [31:0]        ai,
    output wire signed [31:0] xo,
    output wire signed [31:0] yo,
    output wire [31:0]        ao,
    output reg                done = 1'b0
);
    reg signed [35:0] x = 36'sd0, y = 36'sd0;
    reg [31:0]        z = 32'd0;
    reg [4:0]         i = 5'd0;
    reg               busy = 1'b0, r = 1'b0;

    function [31:0] atan;
        input [4:0] k;
        begin
            case (k)
                5'd0 : atan = 32'h20000000;
                5'd1 : atan = 32'h12E4051E;
                5'd2 : atan = 32'h09FB385B;
                5'd3 : atan = 32'h051111D4;
                5'd4 : atan = 32'h028B0D43;
                5'd5 : atan = 32'h0145D7E1;
                5'd6 : atan = 32'h00A2F61E;
                5'd7 : atan = 32'h00517C55;
                5'd8 : atan = 32'h0028BE53;
                5'd9 : atan = 32'h00145F2F;
                5'd10: atan = 32'h000A2F98;
                5'd11: atan = 32'h000517CC;
                5'd12: atan = 32'h00028BE6;
                5'd13: atan = 32'h000145F3;
                5'd14: atan = 32'h0000A2FA;
                5'd15: atan = 32'h0000517D;
                5'd16: atan = 32'h000028BE;
                5'd17: atan = 32'h0000145F;
                5'd18: atan = 32'h00000A30;
                5'd19: atan = 32'h00000518;
                5'd20: atan = 32'h0000028C;
                5'd21: atan = 32'h00000146;
                5'd22: atan = 32'h000000A3;
                5'd23: atan = 32'h00000051;
                default: atan = 32'd0;
            endcase
        end
    endfunction

    wire signed [35:0] x0 = {{3{xi[29]}}, xi, 3'b000};
    wire signed [35:0] y0 = {{3{yi[29]}}, yi, 3'b000};
    wire               flip = rot ? (ai[31] ^ ai[30]) : xi[29];   // start in the right half plane
    wire               dpos = r ? !z[31] : y[35];
    wire signed [35:0] xs = x >>> i;
    wire signed [35:0] ys = y >>> i;

    always @(posedge clk) begin
        done <= 1'b0;
        if (start) begin
            r    <= rot;
            i    <= 5'd0;
            busy <= 1'b1;
            x    <= flip ? -x0 : x0;
            y    <= flip ? -y0 : y0;
            z    <= rot ? (flip ? {~ai[31], ai[30:0]} : ai) : (flip ? 32'h80000000 : 32'd0);
        end else if (busy) begin
            if (dpos) begin
                x <= x - ys;  y <= y + xs;  z <= z - atan(i);
            end else begin
                x <= x + ys;  y <= y - xs;  z <= z + atan(i);
            end
            i <= i + 5'd1;
            if (i == 5'd23) begin
                busy <= 1'b0;
                done <= 1'b1;
            end
        end
    end

    wire signed [35:0] xr = x >>> 3;
    wire signed [35:0] yr = y >>> 3;
    assign xo = xr[31:0];
    assign yo = yr[31:0];
    assign ao = z;
endmodule

// -----------------------------------------------------------------------------------------
// p = a x b, signed, two bits of b a clock: ready 12 clocks after start.
// -----------------------------------------------------------------------------------------
module rxa_smul (
    input  wire               clk,
    input  wire               start,
    input  wire signed [31:0] a,
    input  wire signed [23:0] b,
    output wire signed [55:0] p,
    output reg                done = 1'b0
);
    reg signed [55:0] acc = 56'sd0, as = 56'sd0;
    reg [23:0]        bs = 24'd0;
    reg [3:0]         n = 4'd0;
    reg               busy = 1'b0;
    wire signed [55:0] t0 = bs[0] ? as : 56'sd0;                  // this bit of b
    wire signed [55:0] t1 = bs[1] ? (as <<< 1) : 56'sd0;          // and the next one, worth twice as much

    always @(posedge clk) begin
        done <= 1'b0;
        if (start) begin
            acc  <= 56'sd0;
            as   <= {{24{a[31]}}, a};
            bs   <= b;
            n    <= 4'd0;
            busy <= 1'b1;
        end else if (busy) begin
            acc <= (n == 4'd11) ? (acc + t0 - t1) : (acc + t0 + t1);      // the top bit of b is negative
            as  <= as <<< 2;
            bs  <= {2'b00, bs[23:2]};
            n   <= n + 4'd1;
            if (n == 4'd11) begin
                busy <= 1'b0;
                done <= 1'b1;
            end
        end
    end
    assign p = acc;
endmodule

// -----------------------------------------------------------------------------------------
// Noise blanker.  It works on the samples before the channel filter, where a click is still
// short: the five samples of the CIC's own memory.
//
//   Every sample is delayed by 40 samples (0.8 ms) in a small RAM.  A sample that is larger
//   than T times the running average opens a gap.  The gap starts two samples before the
//   first large sample and ends two samples after the last one.  When it has closed, the
//   samples in it are overwritten in the RAM by a straight line from the last good sample
//   before it to the first good sample after it.  A burst longer than 31 samples is not a
//   click; it is left alone and the average is allowed to catch up with it quickly.
//
//   tsel: T = 4, 3, 6, 10.    led: on for 42 ms after a click was bridged.
// -----------------------------------------------------------------------------------------
module rxa_nb (
    input  wire               clk,
    input  wire               rst,
    input  wire               tick,           // one clock: i_i, q_i hold a new sample
    input  wire signed [24:0] i_i,
    input  wire signed [24:0] q_i,
    input  wire               on,
    input  wire [1:0]         tsel,
    output reg  signed [24:0] i_o = 25'sd0,   // the sample from 40 samples ago
    output reg  signed [24:0] q_o = 25'sd0,
    output reg                valid = 1'b0,   // one clock: i_o, q_o are ready
    output wire               led
);
    (* ram_style = "distributed" *) reg [49:0] ram [0:63];
    integer ri;
    initial for (ri = 0; ri < 64; ri = ri + 1) ram[ri] = 50'd0;

    reg  [5:0]  n       = 6'd0;           // where the newest sample goes
    reg  [35:0] avg_f   = 36'd0;          // average size, 10 bits of fraction
    reg         giveup  = 1'b1;           // learning the level: nothing is blanked
    reg         in_gap  = 1'b0;
    reg  [4:0]  since_s = 5'd0;           // samples since the gap began
    reg  [1:0]  since_l = 2'd3;           // samples since the last large one (stops at 3)
    reg  [1:0]  since_e = 2'd0;           // samples since the last gap ended (stops at 3)
    reg  [11:0] ledc    = 12'd0;

    // ---- how large is this sample, and is that too large? ------------------------------
    wire [24:0] ai  = i_i[24] ? (25'd0 - i_i) : i_i;
    wire [24:0] aq  = q_i[24] ? (25'd0 - q_i) : q_i;
    wire [24:0] mx  = (ai > aq) ? ai : aq;
    wire [24:0] mn  = (ai > aq) ? aq : ai;
    wire [25:0] m   = {1'b0, mx} + {2'b00, mn[24:1]};             // larger + half the smaller
    wire [25:0] avg = avg_f[35:10];
    wire [29:0] a1  = {4'd0, avg};
    wire [29:0] tm  = (tsel == 2'd0) ? {a1[27:0], 2'b00} :                        // x 4
                      (tsel == 2'd1) ? ({a1[28:0], 1'b0} + a1) :                  // x 3
                      (tsel == 2'd2) ? ({a1[27:0], 2'b00} + {a1[28:0], 1'b0}) :   // x 6
                                       ({a1[26:0], 3'b000} + {a1[28:0], 1'b0});   // x 10
    wire [29:0] thr  = (tm < 30'd64) ? 30'd64 : tm;
    wire        big  = ({4'd0, m} > thr);
    wire        flag = on && big && !giveup;

    // ---- the running average: a large sample counts as twice the average at most -------
    wire [26:0] cap  = {avg, 1'b1};                               // 2 x average + 1
    wire [25:0] lim  = ({1'b0, m} < cap) ? m : cap[25:0];
    wire signed [37:0] dfast = $signed({2'b00, m,   10'd0}) - $signed({2'b00, avg_f});
    wire signed [37:0] dslow = $signed({2'b00, lim, 10'd0}) - $signed({2'b00, avg_f});
    wire signed [37:0] astep = giveup ? (dfast >>> 3) : (dslow >>> 10);
    wire signed [37:0] anew  = $signed({2'b00, avg_f}) + astep;

    // ---- where the gap stands at this sample ------------------------------------------
    wire [1:0]  sl_now = (since_l == 2'd3) ? 2'd3 : since_l + 2'd1;
    wire [1:0]  se_now = (since_e == 2'd3) ? 2'd3 : since_e + 2'd1;
    wire [4:0]  ss_now = in_gap ? (since_s + 5'd1) : ((se_now == 2'd3) ? 5'd2 : {3'd0, se_now - 2'd1});
    wire        drop   = flag && (ss_now >= 5'd28);               // too long to be a click
    wire        close  = !flag && in_gap && (sl_now == 2'd3);     // two quiet samples have passed

    // ---- 2^16 / (gap + 1) --------------------------------------------------------------
    function [16:0] recip;
        input [5:0] d;
        begin
            case (d)
                6'd2: recip = 17'd32768;
                6'd3: recip = 17'd21845;
                6'd4: recip = 17'd16384;
                6'd5: recip = 17'd13107;
                6'd6: recip = 17'd10923;
                6'd7: recip = 17'd9362;
                6'd8: recip = 17'd8192;
                6'd9: recip = 17'd7282;
                6'd10: recip = 17'd6554;
                6'd11: recip = 17'd5958;
                6'd12: recip = 17'd5461;
                6'd13: recip = 17'd5041;
                6'd14: recip = 17'd4681;
                6'd15: recip = 17'd4369;
                6'd16: recip = 17'd4096;
                6'd17: recip = 17'd3855;
                6'd18: recip = 17'd3641;
                6'd19: recip = 17'd3449;
                6'd20: recip = 17'd3277;
                6'd21: recip = 17'd3121;
                6'd22: recip = 17'd2979;
                6'd23: recip = 17'd2849;
                6'd24: recip = 17'd2731;
                6'd25: recip = 17'd2621;
                6'd26: recip = 17'd2521;
                6'd27: recip = 17'd2427;
                6'd28: recip = 17'd2341;
                6'd29: recip = 17'd2260;
                6'd30: recip = 17'd2185;
                6'd31: recip = 17'd2114;
                6'd32: recip = 17'd2048;
                default: recip = 17'd0;
            endcase
        end
    endfunction

    localparam [1:0] B_IDLE = 2'd0, B_BASE = 2'd1, B_FILL = 2'd2, B_OUT = 2'd3;
    reg  [1:0]         bs = B_IDLE;
    reg  signed [24:0] ci = 25'sd0, cq = 25'sd0;                  // the sample that just came in
    reg  signed [24:0] bi = 25'sd0, bq = 25'sd0;                  // the good sample before the gap
    reg  [4:0]         g  = 5'd0, k = 5'd0;
    reg  [5:0]         wa = 6'd0;
    reg                fill0 = 1'b0;                              // the two products are being formed
    reg  signed [24:0] di = 25'sd0, dq = 25'sd0;
    reg  [16:0]        rr = 17'd0;
    reg  signed [42:0] si = 43'sd0, sq = 43'sd0;                  // one step of the straight line
    reg  signed [47:0] acc_i = 48'sd0, acc_q = 48'sd0;

    wire [5:0]  ra = (bs == B_BASE) ? (n - {1'b0, g} - 6'd1) : (n - 6'd40);
    wire [49:0] rd = ram[ra];
    wire signed [25:0] ei = {ci[24], ci} - {rd[24], rd[24:0]};
    wire signed [25:0] eq = {cq[24], cq} - {rd[49], rd[49:25]};
    wire signed [47:0] ani = acc_i + si;
    wire signed [47:0] anq = acc_q + sq;
    wire signed [47:0] vi  = (ani + 48'sd16384) >>> 15;
    wire signed [47:0] vq  = (anq + 48'sd16384) >>> 15;
    wire signed [24:0] fi  = bi + vi[24:0];
    wire signed [24:0] fq  = bq + vq[24:0];

    // the RAM has one write port: the new sample at the tick, or one sample of the straight line
    wire        w_new  = (bs == B_IDLE) && tick;
    wire        w_fill = (bs == B_FILL) && !fill0;
    wire [5:0]  w_adr  = w_new ? n : wa;
    wire [49:0] w_dat  = w_new ? {q_i, i_i} : {fq, fi};
    always @(posedge clk) begin
        if (!rst && (w_new || w_fill)) ram[w_adr] <= w_dat;
    end

    always @(posedge clk) begin
        valid <= 1'b0;
        if (rst) begin
            bs <= B_IDLE;  n <= 6'd0;  avg_f <= 36'd0;  giveup <= 1'b1;  in_gap <= 1'b0;
            since_s <= 5'd0;  since_l <= 2'd3;  since_e <= 2'd0;  ledc <= 12'd0;  fill0 <= 1'b0;
        end else begin
            case (bs)
                B_IDLE: if (tick) begin
                    ci     <= i_i;
                    cq     <= q_i;
                    avg_f  <= anew[35:0];
                    if (giveup && !big) giveup <= 1'b0;
                    since_l <= flag ? 2'd0 : sl_now;
                    since_s <= ss_now;
                    since_e <= (drop || close) ? 2'd0 : se_now;
                    if (flag) begin
                        in_gap <= !drop;
                        if (drop) giveup <= 1'b1;
                    end else if (close) begin
                        in_gap <= 1'b0;
                        ledc   <= 12'd2048;
                    end else if (ledc != 12'd0) begin
                        ledc <= ledc - 12'd1;
                    end
                    g  <= ss_now;
                    bs <= close ? B_BASE : B_OUT;
                end
                // the good sample before the gap; half the way from it to the new sample
                B_BASE: begin
                    bi    <= rd[24:0];
                    bq    <= rd[49:25];
                    di    <= ei[25:1];
                    dq    <= eq[25:1];
                    rr    <= recip({1'b0, g} + 6'd1);
                    wa    <= n - {1'b0, g};
                    k     <= 5'd0;
                    acc_i <= 48'sd0;
                    acc_q <= 48'sd0;
                    fill0 <= 1'b1;
                    bs    <= B_FILL;
                end
                // one sample of the straight line a clock
                B_FILL: begin
                    if (fill0) begin
                        si    <= di * $signed({1'b0, rr});
                        sq    <= dq * $signed({1'b0, rr});
                        fill0 <= 1'b0;
                    end else begin
                        acc_i   <= ani;
                        acc_q   <= anq;
                        wa      <= wa + 6'd1;
                        k       <= k + 5'd1;
                        if (k + 5'd1 == g) bs <= B_OUT;
                    end
                end
                B_OUT: begin
                    i_o   <= rd[24:0];
                    q_o   <= rd[49:25];
                    valid <= 1'b1;
                    n     <= n + 6'd1;
                    bs    <= B_IDLE;
                end
            endcase
        end
    end
    assign led = (ledc != 12'd0);
endmodule

// -----------------------------------------------------------------------------------------
// Spectral noise reduction.
//
//   The audio is cut into overlapping pieces of 1024 samples (21 ms), a new one every 256
//   samples.  A piece is weighted with a Hann window and split into frequency cells 47.7 Hz
//   wide by an FFT.  Every cell gets a gain between "floor" and 1, the cells go back
//   through an inverse FFT, the piece is weighted again and added to its neighbours.
//
//   gain         decision-directed Wiener rule: a cell that stands well above the noise
//                keeps its level, a cell at the noise level goes down to the floor, and the
//                estimate leans on the previous piece so that single noise peaks do not
//                open a cell (that would be heard as tinkling).
//   noise level  one number for the whole audio band, because the noise behind an AM
//                detector is flat: the level that one cell in eight stays below.  It is
//                kept as it would be before the AGC, where it does not move with fading.
//                While the carrier loop is locked it is measured in the quadrature
//                channel, which holds the same noise and none of the programme.  Otherwise
//                it is measured in the audio, and then it is allowed to rise only slowly.
//   band         cells above half the filter width + 250 Hz hold no programme: set to zero.
//
//   Levels are 64 x log2(magnitude), 0.094 dB a step, so ratios are subtractions.
//   The output is 1279 samples (26 ms) behind the input.
// -----------------------------------------------------------------------------------------
module rxa_nr (
    input  wire               clk,
    input  wire               rst,
    input  wire               tick,           // one clock: the inputs belong to a new sample
    input  wire signed [21:0] x_i,            // audio after the AGC
    input  wire signed [31:0] rq_i,           // quadrature output of the carrier loop
    input  wire [31:0]        lden_i,         // what the AGC divides by
    input  wire               locked_i,       // carrier loop locked
    input  wire [7:0]         khi_i,          // the highest audio cell that can hold programme
    input  wire [7:0]         ohi_i,          // the highest cell used to measure the noise
    input  wire [1:0]         fsel_i,         // depth: 12, 18, 24, 6 dB
    input  wire               big_i,          // a big retune at this sample: learn the noise level afresh
    output reg  signed [21:0] y_o = 22'sd0    // ready two clocks after tick
);
    // ---- memories --------------------------------------------------------------------------
    (* ram_style = "block" *) reg [43:0] ring [0:1023];           // the last 1024 samples, {quadrature, audio}
    (* ram_style = "block" *) reg [63:0] fr   [0:1023];           // the piece being worked on, {imaginary, real}
    (* ram_style = "block" *) reg [25:0] ola  [0:1023];           // pieces added up
    (* ram_style = "block" *) reg [21:0] obuf [0:511];            // finished samples: one half is played, one filled
    (* ram_style = "distributed" *) reg [19:0] bst [0:255];       // per cell: {previous gamma, previous gain}
    (* rom_style = "block" *) reg [35:0] tw [0:511];              // {-sin, cos} of 2 pi k / 1024, 2^16 = 1
    integer zi;
    initial begin
        for (zi = 0; zi < 1024; zi = zi + 1) begin ring[zi] = 44'd0; fr[zi] = 64'd0; ola[zi] = 26'd0; end
        for (zi = 0; zi < 512; zi = zi + 1) obuf[zi] = 22'd0;
        for (zi = 0; zi < 256; zi = zi + 1) bst[zi] = 20'd0;
        tw[  0] = 36'h000010000;  tw[  1] = 36'hFF9B8FFFF;  tw[  2] = 36'hFF370FFFB;  tw[  3] = 36'hFED28FFF5;
        tw[  4] = 36'hFE6E0FFEC;  tw[  5] = 36'hFE098FFE1;  tw[  6] = 36'hFDA50FFD4;  tw[  7] = 36'hFD408FFC4;
        tw[  8] = 36'hFCDC0FFB1;  tw[  9] = 36'hFC77CFF9C;  tw[ 10] = 36'hFC134FF85;  tw[ 11] = 36'hFBAF0FF6B;
        tw[ 12] = 36'hFB4ACFF4E;  tw[ 13] = 36'hFAE68FF30;  tw[ 14] = 36'hFA824FF0E;  tw[ 15] = 36'hFA1E4FEEB;
        tw[ 16] = 36'hF9BA0FEC4;  tw[ 17] = 36'hF9560FE9C;  tw[ 18] = 36'hF8F20FE71;  tw[ 19] = 36'hF88E4FE43;
        tw[ 20] = 36'hF82A8FE13;  tw[ 21] = 36'hF7C6CFDE1;  tw[ 22] = 36'hF7630FDAC;  tw[ 23] = 36'hF6FF8FD74;
        tw[ 24] = 36'hF69C0FD3B;  tw[ 25] = 36'hF6388FCFE;  tw[ 26] = 36'hF5D54FCC0;  tw[ 27] = 36'hF5720FC7F;
        tw[ 28] = 36'hF50F0FC3B;  tw[ 29] = 36'hF4AC0FBF5;  tw[ 30] = 36'hF4490FBAD;  tw[ 31] = 36'hF3E64FB62;
        tw[ 32] = 36'hF383CFB15;  tw[ 33] = 36'hF3210FAC5;  tw[ 34] = 36'hF2BECFA73;  tw[ 35] = 36'hF25C8FA1F;
        tw[ 36] = 36'hF1FA4F9C8;  tw[ 37] = 36'hF1984F96E;  tw[ 38] = 36'hF1364F913;  tw[ 39] = 36'hF0D48F8B4;
        tw[ 40] = 36'hF0730F854;  tw[ 41] = 36'hF0118F7F1;  tw[ 42] = 36'hEFB04F78C;  tw[ 43] = 36'hEF4F4F724;
        tw[ 44] = 36'hEEEE4F6BA;  tw[ 45] = 36'hEE8D4F64E;  tw[ 46] = 36'hEE2CCF5DF;  tw[ 47] = 36'hEDCC4F56E;
        tw[ 48] = 36'hED6C0F4FA;  tw[ 49] = 36'hED0BCF484;  tw[ 50] = 36'hECAC0F40C;  tw[ 51] = 36'hEC4C4F391;
        tw[ 52] = 36'hEBECCF314;  tw[ 53] = 36'hEB8D4F295;  tw[ 54] = 36'hEB2E0F213;  tw[ 55] = 36'hEACF4F18F;
        tw[ 56] = 36'hEA708F109;  tw[ 57] = 36'hEA11CF080;  tw[ 58] = 36'hE9B38EFF5;  tw[ 59] = 36'hE9558EF68;
        tw[ 60] = 36'hE8F78EED9;  tw[ 61] = 36'hE899CEE47;  tw[ 62] = 36'hE83C4EDB3;  tw[ 63] = 36'hE7DF0ED1C;
        tw[ 64] = 36'hE7820EC83;  tw[ 65] = 36'hE7254EBE8;  tw[ 66] = 36'hE6C8CEB4B;  tw[ 67] = 36'hE66C8EAAB;
        tw[ 68] = 36'hE6108EA0A;  tw[ 69] = 36'hE5B4CE966;  tw[ 70] = 36'hE5594E8BF;  tw[ 71] = 36'hE4FE0E817;
        tw[ 72] = 36'hE4A30E76C;  tw[ 73] = 36'hE4484E6BF;  tw[ 74] = 36'hE3EDCE610;  tw[ 75] = 36'hE3938E55E;
        tw[ 76] = 36'hE3398E4AA;  tw[ 77] = 36'hE2E00E3F4;  tw[ 78] = 36'hE2868E33C;  tw[ 79] = 36'hE22D8E282;
        tw[ 80] = 36'hE1D4CE1C6;  tw[ 81] = 36'hE17C0E107;  tw[ 82] = 36'hE1240E046;  tw[ 83] = 36'hE0CC0DF83;
        tw[ 84] = 36'hE0744DEBE;  tw[ 85] = 36'hE01D0DDF7;  tw[ 86] = 36'hDFC60DD2D;  tw[ 87] = 36'hDF6F4DC62;
        tw[ 88] = 36'hDF190DB94;  tw[ 89] = 36'hDEC2CDAC4;  tw[ 90] = 36'hDE6D0D9F2;  tw[ 91] = 36'hDE17CD91E;
        tw[ 92] = 36'hDDC28D848;  tw[ 93] = 36'hDD6DCD770;  tw[ 94] = 36'hDD198D696;  tw[ 95] = 36'hDCC54D5BA;
        tw[ 96] = 36'hDC718D4DB;  tw[ 97] = 36'hDC1E0D3FB;  tw[ 98] = 36'hDBCB0D318;  tw[ 99] = 36'hDB784D234;
        tw[100] = 36'hDB260D14D;  tw[101] = 36'hDAD40D065;  tw[102] = 36'hDA824CF7A;  tw[103] = 36'hDA310CE8E;
        tw[104] = 36'hD9E00CD9F;  tw[105] = 36'hD98F8CCAE;  tw[106] = 36'hD93F4CBBC;  tw[107] = 36'hD8EF8CAC7;
        tw[108] = 36'hD8A00C9D1;  tw[109] = 36'hD8510C8D9;  tw[110] = 36'hD8024C7DE;  tw[111] = 36'hD7B40C6E2;
        tw[112] = 36'hD7660C5E4;  tw[113] = 36'hD7188C4E4;  tw[114] = 36'hD6CB8C3E2;  tw[115] = 36'hD67ECC2DE;
        tw[116] = 36'hD6328C1D8;  tw[117] = 36'hD5E68C0D1;  tw[118] = 36'hD59B0BFC7;  tw[119] = 36'hD54FCBEBC;
        tw[120] = 36'hD5054BDAF;  tw[121] = 36'hD4BB0BCA0;  tw[122] = 36'hD4710BB8F;  tw[123] = 36'hD4278BA7D;
        tw[124] = 36'hD3DE8B968;  tw[125] = 36'hD3960B852;  tw[126] = 36'hD34DCB73A;  tw[127] = 36'hD3060B620;
        tw[128] = 36'hD2BECB505;  tw[129] = 36'hD2780B3E8;  tw[130] = 36'hD2318B2C9;  tw[131] = 36'hD1EB8B1A8;
        tw[132] = 36'hD1A60B086;  tw[133] = 36'hD160CAF62;  tw[134] = 36'hD11C4AE3C;  tw[135] = 36'hD0D80AD14;
        tw[136] = 36'hD0944ABEB;  tw[137] = 36'hD0510AAC1;  tw[138] = 36'hD00E4A994;  tw[139] = 36'hCFCBCA866;
        tw[140] = 36'hCF8A0A736;  tw[141] = 36'hCF488A605;  tw[142] = 36'hCF078A4D2;  tw[143] = 36'hCEC70A39E;
        tw[144] = 36'hCE870A268;  tw[145] = 36'hCE478A130;  tw[146] = 36'hCE0889FF7;  tw[147] = 36'hCDC9C9EBC;
        tw[148] = 36'hCD8BC9D80;  tw[149] = 36'hCD4E49C42;  tw[150] = 36'hCD1109B03;  tw[151] = 36'hCCD4899C2;
        tw[152] = 36'hCC9849880;  tw[153] = 36'hCC5C8973C;  tw[154] = 36'hCC21895F7;  tw[155] = 36'hCBE6C94B0;
        tw[156] = 36'hCBACC9368;  tw[157] = 36'hCB730921F;  tw[158] = 36'hCB3A090D4;  tw[159] = 36'hCB0148F88;
        tw[160] = 36'hCAC948E3A;  tw[161] = 36'hCA9188CEB;  tw[162] = 36'hCA5A88B9A;  tw[163] = 36'hCA2408A49;
        tw[164] = 36'hC9EE088F6;  tw[165] = 36'hC9B8887A1;  tw[166] = 36'hC9838864C;  tw[167] = 36'hC94F084F5;
        tw[168] = 36'hC91B0839C;  tw[169] = 36'hC8E788243;  tw[170] = 36'hC8B4C80E8;  tw[171] = 36'hC88247F8C;
        tw[172] = 36'hC85087E2F;  tw[173] = 36'hC81F47CD0;  tw[174] = 36'hC7EE87B70;  tw[175] = 36'hC7BE47A10;
        tw[176] = 36'hC78E878AD;  tw[177] = 36'hC75F8774A;  tw[178] = 36'hC731075E6;  tw[179] = 36'hC70307480;
        tw[180] = 36'hC6D58731A;  tw[181] = 36'hC6A8871B2;  tw[182] = 36'hC67C07049;  tw[183] = 36'hC65046EDF;
        tw[184] = 36'hC62506D74;  tw[185] = 36'hC5FA46C08;  tw[186] = 36'hC5D046A9B;  tw[187] = 36'hC5A68692D;
        tw[188] = 36'hC57D867BE;  tw[189] = 36'hC5554664E;  tw[190] = 36'hC52D464DD;  tw[191] = 36'hC5060636B;
        tw[192] = 36'hC4DF461F8;  tw[193] = 36'hC4B906084;  tw[194] = 36'hC49345F0F;  tw[195] = 36'hC46E45D99;
        tw[196] = 36'hC449C5C22;  tw[197] = 36'hC42605AAA;  tw[198] = 36'hC402C5932;  tw[199] = 36'hC3E0057B9;
        tw[200] = 36'hC3BDC563E;  tw[201] = 36'hC39C454C3;  tw[202] = 36'hC37B45348;  tw[203] = 36'hC35AC51CB;
        tw[204] = 36'hC33B0504D;  tw[205] = 36'hC31BC4ECF;  tw[206] = 36'hC2FD04D50;  tw[207] = 36'hC2DF04BD1;
        tw[208] = 36'hC2C184A50;  tw[209] = 36'hC2A4848CF;  tw[210] = 36'hC2884474D;  tw[211] = 36'hC26C845CB;
        tw[212] = 36'hC25184447;  tw[213] = 36'hC237042C3;  tw[214] = 36'hC21D0413F;  tw[215] = 36'hC203C3FBA;
        tw[216] = 36'hC1EB03E34;  tw[217] = 36'hC1D303CAE;  tw[218] = 36'hC1BB43B27;  tw[219] = 36'hC1A48399F;
        tw[220] = 36'hC18E03817;  tw[221] = 36'hC1784368E;  tw[222] = 36'hC16343505;  tw[223] = 36'hC14EC337C;
        tw[224] = 36'hC13AC31F1;  tw[225] = 36'hC12783067;  tw[226] = 36'hC114C2EDC;  tw[227] = 36'hC102C2D50;
        tw[228] = 36'hC0F142BC4;  tw[229] = 36'hC0E042A38;  tw[230] = 36'hC0D0028AB;  tw[231] = 36'hC0C08271E;
        tw[232] = 36'hC0B142590;  tw[233] = 36'hC0A302402;  tw[234] = 36'hC09502274;  tw[235] = 36'hC087C20E5;
        tw[236] = 36'hC07B41F56;  tw[237] = 36'hC06F41DC7;  tw[238] = 36'hC063C1C38;  tw[239] = 36'hC05901AA8;
        tw[240] = 36'hC04F01918;  tw[241] = 36'hC04541787;  tw[242] = 36'hC03C815F7;  tw[243] = 36'hC03401466;
        tw[244] = 36'hC02C812D5;  tw[245] = 36'hC02541144;  tw[246] = 36'hC01EC0FB3;  tw[247] = 36'hC01900E21;
        tw[248] = 36'hC013C0C90;  tw[249] = 36'hC00F00AFE;  tw[250] = 36'hC00B0096C;  tw[251] = 36'hC007C07DA;
        tw[252] = 36'hC00500648;  tw[253] = 36'hC002C04B6;  tw[254] = 36'hC00140324;  tw[255] = 36'hC00040192;
        tw[256] = 36'hC00000000;  tw[257] = 36'hC0007FE6E;  tw[258] = 36'hC0017FCDC;  tw[259] = 36'hC002FFB4A;
        tw[260] = 36'hC0053F9B8;  tw[261] = 36'hC007FF826;  tw[262] = 36'hC00B3F694;  tw[263] = 36'hC00F3F502;
        tw[264] = 36'hC013FF370;  tw[265] = 36'hC0193F1DF;  tw[266] = 36'hC01EFF04D;  tw[267] = 36'hC0257EEBC;
        tw[268] = 36'hC02CBED2B;  tw[269] = 36'hC0343EB9A;  tw[270] = 36'hC03CBEA09;  tw[271] = 36'hC0457E879;
        tw[272] = 36'hC04F3E6E8;  tw[273] = 36'hC0593E558;  tw[274] = 36'hC063FE3C8;  tw[275] = 36'hC06F7E239;
        tw[276] = 36'hC07B7E0AA;  tw[277] = 36'hC087FDF1B;  tw[278] = 36'hC0953DD8C;  tw[279] = 36'hC0A33DBFE;
        tw[280] = 36'hC0B17DA70;  tw[281] = 36'hC0C0BD8E2;  tw[282] = 36'hC0D03D755;  tw[283] = 36'hC0E07D5C8;
        tw[284] = 36'hC0F17D43C;  tw[285] = 36'hC102FD2B0;  tw[286] = 36'hC114FD124;  tw[287] = 36'hC127BCF99;
        tw[288] = 36'hC13AFCE0F;  tw[289] = 36'hC14EFCC84;  tw[290] = 36'hC1637CAFB;  tw[291] = 36'hC1787C972;
        tw[292] = 36'hC18E3C7E9;  tw[293] = 36'hC1A4BC661;  tw[294] = 36'hC1BB7C4D9;  tw[295] = 36'hC1D33C352;
        tw[296] = 36'hC1EB3C1CC;  tw[297] = 36'hC203FC046;  tw[298] = 36'hC21D3BEC1;  tw[299] = 36'hC2373BD3D;
        tw[300] = 36'hC251BBBB9;  tw[301] = 36'hC26CBBA35;  tw[302] = 36'hC2887B8B3;  tw[303] = 36'hC2A4BB731;
        tw[304] = 36'hC2C1BB5B0;  tw[305] = 36'hC2DF3B42F;  tw[306] = 36'hC2FD3B2B0;  tw[307] = 36'hC31BFB131;
        tw[308] = 36'hC33B3AFB3;  tw[309] = 36'hC35AFAE35;  tw[310] = 36'hC37B7ACB8;  tw[311] = 36'hC39C7AB3D;
        tw[312] = 36'hC3BDFA9C2;  tw[313] = 36'hC3E03A847;  tw[314] = 36'hC402FA6CE;  tw[315] = 36'hC4263A556;
        tw[316] = 36'hC449FA3DE;  tw[317] = 36'hC46E7A267;  tw[318] = 36'hC4937A0F1;  tw[319] = 36'hC4B939F7C;
        tw[320] = 36'hC4DF79E08;  tw[321] = 36'hC50639C95;  tw[322] = 36'hC52D79B23;  tw[323] = 36'hC555799B2;
        tw[324] = 36'hC57DB9842;  tw[325] = 36'hC5A6B96D3;  tw[326] = 36'hC5D079565;  tw[327] = 36'hC5FA793F8;
        tw[328] = 36'hC6253928C;  tw[329] = 36'hC65079121;  tw[330] = 36'hC67C38FB7;  tw[331] = 36'hC6A8B8E4E;
        tw[332] = 36'hC6D5B8CE6;  tw[333] = 36'hC70338B80;  tw[334] = 36'hC73138A1A;  tw[335] = 36'hC75FB88B6;
        tw[336] = 36'hC78EB8753;  tw[337] = 36'hC7BE785F0;  tw[338] = 36'hC7EEB8490;  tw[339] = 36'hC81F78330;
        tw[340] = 36'hC850B81D1;  tw[341] = 36'hC88278074;  tw[342] = 36'hC8B4F7F18;  tw[343] = 36'hC8E7B7DBD;
        tw[344] = 36'hC91B37C64;  tw[345] = 36'hC94F37B0B;  tw[346] = 36'hC983B79B4;  tw[347] = 36'hC9B8B785F;
        tw[348] = 36'hC9EE3770A;  tw[349] = 36'hCA24375B7;  tw[350] = 36'hCA5AB7466;  tw[351] = 36'hCA91B7315;
        tw[352] = 36'hCAC9771C6;  tw[353] = 36'hCB0177078;  tw[354] = 36'hCB3A36F2C;  tw[355] = 36'hCB7336DE1;
        tw[356] = 36'hCBACF6C98;  tw[357] = 36'hCBE6F6B50;  tw[358] = 36'hCC21B6A09;  tw[359] = 36'hCC5CB68C4;
        tw[360] = 36'hCC9876780;  tw[361] = 36'hCCD4B663E;  tw[362] = 36'hCD11364FD;  tw[363] = 36'hCD4E763BE;
        tw[364] = 36'hCD8BF6280;  tw[365] = 36'hCDC9F6144;  tw[366] = 36'hCE08B6009;  tw[367] = 36'hCE47B5ED0;
        tw[368] = 36'hCE8735D98;  tw[369] = 36'hCEC735C62;  tw[370] = 36'hCF07B5B2E;  tw[371] = 36'hCF48B59FB;
        tw[372] = 36'hCF8A358CA;  tw[373] = 36'hCFCBF579A;  tw[374] = 36'hD00E7566C;  tw[375] = 36'hD0513553F;
        tw[376] = 36'hD09475415;  tw[377] = 36'hD0D8352EC;  tw[378] = 36'hD11C751C4;  tw[379] = 36'hD160F509E;
        tw[380] = 36'hD1A634F7A;  tw[381] = 36'hD1EBB4E58;  tw[382] = 36'hD231B4D37;  tw[383] = 36'hD27834C18;
        tw[384] = 36'hD2BEF4AFB;  tw[385] = 36'hD306349E0;  tw[386] = 36'hD34DF48C6;  tw[387] = 36'hD396347AE;
        tw[388] = 36'hD3DEB4698;  tw[389] = 36'hD427B4583;  tw[390] = 36'hD47134471;  tw[391] = 36'hD4BB34360;
        tw[392] = 36'hD50574251;  tw[393] = 36'hD54FF4144;  tw[394] = 36'hD59B34039;  tw[395] = 36'hD5E6B3F2F;
        tw[396] = 36'hD632B3E28;  tw[397] = 36'hD67EF3D22;  tw[398] = 36'hD6CBB3C1E;  tw[399] = 36'hD718B3B1C;
        tw[400] = 36'hD76633A1C;  tw[401] = 36'hD7B43391E;  tw[402] = 36'hD80273822;  tw[403] = 36'hD85133727;
        tw[404] = 36'hD8A03362F;  tw[405] = 36'hD8EFB3539;  tw[406] = 36'hD93F73444;  tw[407] = 36'hD98FB3352;
        tw[408] = 36'hD9E033261;  tw[409] = 36'hDA3133172;  tw[410] = 36'hDA8273086;  tw[411] = 36'hDAD432F9B;
        tw[412] = 36'hDB2632EB3;  tw[413] = 36'hDB7872DCC;  tw[414] = 36'hDBCB32CE8;  tw[415] = 36'hDC1E32C05;
        tw[416] = 36'hDC71B2B25;  tw[417] = 36'hDCC572A46;  tw[418] = 36'hDD19B296A;  tw[419] = 36'hDD6DF2890;
        tw[420] = 36'hDDC2B27B8;  tw[421] = 36'hDE17F26E2;  tw[422] = 36'hDE6D3260E;  tw[423] = 36'hDEC2F253C;
        tw[424] = 36'hDF193246C;  tw[425] = 36'hDF6F7239E;  tw[426] = 36'hDFC6322D3;  tw[427] = 36'hE01D32209;
        tw[428] = 36'hE07472142;  tw[429] = 36'hE0CC3207D;  tw[430] = 36'hE12431FBA;  tw[431] = 36'hE17C31EF9;
        tw[432] = 36'hE1D4F1E3A;  tw[433] = 36'hE22DB1D7E;  tw[434] = 36'hE286B1CC4;  tw[435] = 36'hE2E031C0C;
        tw[436] = 36'hE339B1B56;  tw[437] = 36'hE393B1AA2;  tw[438] = 36'hE3EDF19F0;  tw[439] = 36'hE44871941;
        tw[440] = 36'hE4A331894;  tw[441] = 36'hE4FE317E9;  tw[442] = 36'hE55971741;  tw[443] = 36'hE5B4F169A;
        tw[444] = 36'hE610B15F6;  tw[445] = 36'hE66CB1555;  tw[446] = 36'hE6C8F14B5;  tw[447] = 36'hE72571418;
        tw[448] = 36'hE7823137D;  tw[449] = 36'hE7DF312E4;  tw[450] = 36'hE83C7124D;  tw[451] = 36'hE899F11B9;
        tw[452] = 36'hE8F7B1127;  tw[453] = 36'hE955B1098;  tw[454] = 36'hE9B3B100B;  tw[455] = 36'hEA11F0F80;
        tw[456] = 36'hEA70B0EF7;  tw[457] = 36'hEACF70E71;  tw[458] = 36'hEB2E30DED;  tw[459] = 36'hEB8D70D6B;
        tw[460] = 36'hEBECF0CEC;  tw[461] = 36'hEC4C70C6F;  tw[462] = 36'hECAC30BF4;  tw[463] = 36'hED0BF0B7C;
        tw[464] = 36'hED6C30B06;  tw[465] = 36'hEDCC70A92;  tw[466] = 36'hEE2CF0A21;  tw[467] = 36'hEE8D709B2;
        tw[468] = 36'hEEEE70946;  tw[469] = 36'hEF4F708DC;  tw[470] = 36'hEFB070874;  tw[471] = 36'hF011B080F;
        tw[472] = 36'hF073307AC;  tw[473] = 36'hF0D4B074C;  tw[474] = 36'hF136706ED;  tw[475] = 36'hF19870692;
        tw[476] = 36'hF1FA70638;  tw[477] = 36'hF25CB05E1;  tw[478] = 36'hF2BEF058D;  tw[479] = 36'hF3213053B;
        tw[480] = 36'hF383F04EB;  tw[481] = 36'hF3E67049E;  tw[482] = 36'hF44930453;  tw[483] = 36'hF4AC3040B;
        tw[484] = 36'hF50F303C5;  tw[485] = 36'hF57230381;  tw[486] = 36'hF5D570340;  tw[487] = 36'hF638B0302;
        tw[488] = 36'hF69C302C5;  tw[489] = 36'hF6FFB028C;  tw[490] = 36'hF76330254;  tw[491] = 36'hF7C6F021F;
        tw[492] = 36'hF82AB01ED;  tw[493] = 36'hF88E701BD;  tw[494] = 36'hF8F23018F;  tw[495] = 36'hF95630164;
        tw[496] = 36'hF9BA3013C;  tw[497] = 36'hFA1E70115;  tw[498] = 36'hFA82700F2;  tw[499] = 36'hFAE6B00D0;
        tw[500] = 36'hFB4AF00B2;  tw[501] = 36'hFBAF30095;  tw[502] = 36'hFC137007B;  tw[503] = 36'hFC77F0064;
        tw[504] = 36'hFCDC3004F;  tw[505] = 36'hFD40B003C;  tw[506] = 36'hFDA53002C;  tw[507] = 36'hFE09B001F;
        tw[508] = 36'hFE6E30014;  tw[509] = 36'hFED2B000B;  tw[510] = 36'hFF3730005;  tw[511] = 36'hFF9BB0001;
    end

    // ---- small tables ----------------------------------------------------------------------
    function [5:0] mlut;                  // 64 x log2(1 + m / 64)
        input [5:0] a;
        begin
            case (a)
                6'd0: mlut = 6'd0;  6'd1: mlut = 6'd1;  6'd2: mlut = 6'd3;  6'd3: mlut = 6'd4;  6'd4: mlut = 6'd6;  6'd5: mlut = 6'd7;  6'd6: mlut = 6'd8;  6'd7: mlut = 6'd10;
                6'd8: mlut = 6'd11;  6'd9: mlut = 6'd12;  6'd10: mlut = 6'd13;  6'd11: mlut = 6'd15;  6'd12: mlut = 6'd16;  6'd13: mlut = 6'd17;  6'd14: mlut = 6'd18;  6'd15: mlut = 6'd19;
                6'd16: mlut = 6'd21;  6'd17: mlut = 6'd22;  6'd18: mlut = 6'd23;  6'd19: mlut = 6'd24;  6'd20: mlut = 6'd25;  6'd21: mlut = 6'd26;  6'd22: mlut = 6'd27;  6'd23: mlut = 6'd28;
                6'd24: mlut = 6'd29;  6'd25: mlut = 6'd30;  6'd26: mlut = 6'd31;  6'd27: mlut = 6'd32;  6'd28: mlut = 6'd34;  6'd29: mlut = 6'd35;  6'd30: mlut = 6'd35;  6'd31: mlut = 6'd36;
                6'd32: mlut = 6'd37;  6'd33: mlut = 6'd38;  6'd34: mlut = 6'd39;  6'd35: mlut = 6'd40;  6'd36: mlut = 6'd41;  6'd37: mlut = 6'd42;  6'd38: mlut = 6'd43;  6'd39: mlut = 6'd44;
                6'd40: mlut = 6'd45;  6'd41: mlut = 6'd46;  6'd42: mlut = 6'd47;  6'd43: mlut = 6'd47;  6'd44: mlut = 6'd48;  6'd45: mlut = 6'd49;  6'd46: mlut = 6'd50;  6'd47: mlut = 6'd51;
                6'd48: mlut = 6'd52;  6'd49: mlut = 6'd52;  6'd50: mlut = 6'd53;  6'd51: mlut = 6'd54;  6'd52: mlut = 6'd55;  6'd53: mlut = 6'd56;  6'd54: mlut = 6'd56;  6'd55: mlut = 6'd57;
                6'd56: mlut = 6'd58;  6'd57: mlut = 6'd59;  6'd58: mlut = 6'd60;  6'd59: mlut = 6'd60;  6'd60: mlut = 6'd61;  6'd61: mlut = 6'd62;  6'd62: mlut = 6'd63;  6'd63: mlut = 6'd63;
                default: mlut = 6'd0;
            endcase
        end
    endfunction
    function [8:0] c1m;                   // -64 x log2(1 - 2^(-x/64))
        input [8:0] a;
        begin
            case (a)
                9'd0: c1m = 9'd0;  9'd1: c1m = 9'd418;  9'd2: c1m = 9'd355;  9'd3: c1m = 9'd318;  9'd4: c1m = 9'd292;  9'd5: c1m = 9'd272;
                9'd6: c1m = 9'd255;  9'd7: c1m = 9'd242;  9'd8: c1m = 9'd230;  9'd9: c1m = 9'd219;  9'd10: c1m = 9'd210;  9'd11: c1m = 9'd202;
                9'd12: c1m = 9'd194;  9'd13: c1m = 9'd187;  9'd14: c1m = 9'd181;  9'd15: c1m = 9'd175;  9'd16: c1m = 9'd170;  9'd17: c1m = 9'd165;
                9'd18: c1m = 9'd160;  9'd19: c1m = 9'd155;  9'd20: c1m = 9'd151;  9'd21: c1m = 9'd147;  9'd22: c1m = 9'd143;  9'd23: c1m = 9'd140;
                9'd24: c1m = 9'd136;  9'd25: c1m = 9'd133;  9'd26: c1m = 9'd130;  9'd27: c1m = 9'd127;  9'd28: c1m = 9'd124;  9'd29: c1m = 9'd121;
                9'd30: c1m = 9'd118;  9'd31: c1m = 9'd116;  9'd32: c1m = 9'd113;  9'd33: c1m = 9'd111;  9'd34: c1m = 9'd109;  9'd35: c1m = 9'd107;
                9'd36: c1m = 9'd104;  9'd37: c1m = 9'd102;  9'd38: c1m = 9'd100;  9'd39: c1m = 9'd98;  9'd40: c1m = 9'd97;  9'd41: c1m = 9'd95;
                9'd42: c1m = 9'd93;  9'd43: c1m = 9'd91;  9'd44: c1m = 9'd90;  9'd45: c1m = 9'd88;  9'd46: c1m = 9'd86;  9'd47: c1m = 9'd85;
                9'd48: c1m = 9'd83;  9'd49: c1m = 9'd82;  9'd50: c1m = 9'd81;  9'd51: c1m = 9'd79;  9'd52: c1m = 9'd78;  9'd53: c1m = 9'd76;
                9'd54: c1m = 9'd75;  9'd55: c1m = 9'd74;  9'd56: c1m = 9'd73;  9'd57: c1m = 9'd72;  9'd58: c1m = 9'd70;  9'd59: c1m = 9'd69;
                9'd60: c1m = 9'd68;  9'd61: c1m = 9'd67;  9'd62: c1m = 9'd66;  9'd63: c1m = 9'd65;  9'd64: c1m = 9'd64;  9'd65: c1m = 9'd63;
                9'd66: c1m = 9'd62;  9'd67: c1m = 9'd61;  9'd68: c1m = 9'd60;  9'd69: c1m = 9'd59;  9'd70: c1m = 9'd58;  9'd71: c1m = 9'd57;
                9'd72: c1m = 9'd57;  9'd73: c1m = 9'd56;  9'd74: c1m = 9'd55;  9'd75: c1m = 9'd54;  9'd76: c1m = 9'd53;  9'd77: c1m = 9'd53;
                9'd78: c1m = 9'd52;  9'd79: c1m = 9'd51;  9'd80: c1m = 9'd50;  9'd81: c1m = 9'd50;  9'd82: c1m = 9'd49;  9'd83: c1m = 9'd48;
                9'd84: c1m = 9'd48;  9'd85: c1m = 9'd47;  9'd86: c1m = 9'd46;  9'd87: c1m = 9'd46;  9'd88: c1m = 9'd45;  9'd89: c1m = 9'd44;
                9'd90: c1m = 9'd44;  9'd91: c1m = 9'd43;  9'd92: c1m = 9'd43;  9'd93: c1m = 9'd42;  9'd94: c1m = 9'd41;  9'd95: c1m = 9'd41;
                9'd96: c1m = 9'd40;  9'd97: c1m = 9'd40;  9'd98: c1m = 9'd39;  9'd99: c1m = 9'd39;  9'd100: c1m = 9'd38;  9'd101: c1m = 9'd38;
                9'd102: c1m = 9'd37;  9'd103: c1m = 9'd37;  9'd104: c1m = 9'd36;  9'd105: c1m = 9'd36;  9'd106: c1m = 9'd35;  9'd107: c1m = 9'd35;
                9'd108: c1m = 9'd34;  9'd109: c1m = 9'd34;  9'd110: c1m = 9'd33;  9'd111: c1m = 9'd33;  9'd112: c1m = 9'd33;  9'd113: c1m = 9'd32;
                9'd114: c1m = 9'd32;  9'd115: c1m = 9'd31;  9'd116: c1m = 9'd31;  9'd117: c1m = 9'd31;  9'd118: c1m = 9'd30;  9'd119: c1m = 9'd30;
                9'd120: c1m = 9'd29;  9'd121: c1m = 9'd29;  9'd122: c1m = 9'd29;  9'd123: c1m = 9'd28;  9'd124: c1m = 9'd28;  9'd125: c1m = 9'd28;
                9'd126: c1m = 9'd27;  9'd127: c1m = 9'd27;  9'd128: c1m = 9'd27;  9'd129: c1m = 9'd26;  9'd130: c1m = 9'd26;  9'd131: c1m = 9'd26;
                9'd132: c1m = 9'd25;  9'd133: c1m = 9'd25;  9'd134: c1m = 9'd25;  9'd135: c1m = 9'd24;  9'd136: c1m = 9'd24;  9'd137: c1m = 9'd24;
                9'd138: c1m = 9'd23;  9'd139: c1m = 9'd23;  9'd140: c1m = 9'd23;  9'd141: c1m = 9'd23;  9'd142: c1m = 9'd22;  9'd143: c1m = 9'd22;
                9'd144: c1m = 9'd22;  9'd145: c1m = 9'd22;  9'd146: c1m = 9'd21;  9'd147: c1m = 9'd21;  9'd148: c1m = 9'd21;  9'd149: c1m = 9'd21;
                9'd150: c1m = 9'd20;  9'd151: c1m = 9'd20;  9'd152: c1m = 9'd20;  9'd153: c1m = 9'd20;  9'd154: c1m = 9'd19;  9'd155: c1m = 9'd19;
                9'd156: c1m = 9'd19;  9'd157: c1m = 9'd19;  9'd158: c1m = 9'd18;  9'd159: c1m = 9'd18;  9'd160: c1m = 9'd18;  9'd161: c1m = 9'd18;
                9'd162: c1m = 9'd18;  9'd163: c1m = 9'd17;  9'd164: c1m = 9'd17;  9'd165: c1m = 9'd17;  9'd166: c1m = 9'd17;  9'd167: c1m = 9'd17;
                9'd168: c1m = 9'd16;  9'd169: c1m = 9'd16;  9'd170: c1m = 9'd16;  9'd171: c1m = 9'd16;  9'd172: c1m = 9'd16;  9'd173: c1m = 9'd15;
                9'd174: c1m = 9'd15;  9'd175: c1m = 9'd15;  9'd176: c1m = 9'd15;  9'd177: c1m = 9'd15;  9'd178: c1m = 9'd15;  9'd179: c1m = 9'd14;
                9'd180: c1m = 9'd14;  9'd181: c1m = 9'd14;  9'd182: c1m = 9'd14;  9'd183: c1m = 9'd14;  9'd184: c1m = 9'd14;  9'd185: c1m = 9'd13;
                9'd186: c1m = 9'd13;  9'd187: c1m = 9'd13;  9'd188: c1m = 9'd13;  9'd189: c1m = 9'd13;  9'd190: c1m = 9'd13;  9'd191: c1m = 9'd12;
                9'd192: c1m = 9'd12;  9'd193: c1m = 9'd12;  9'd194: c1m = 9'd12;  9'd195: c1m = 9'd12;  9'd196: c1m = 9'd12;  9'd197: c1m = 9'd12;
                9'd198: c1m = 9'd12;  9'd199: c1m = 9'd11;  9'd200: c1m = 9'd11;  9'd201: c1m = 9'd11;  9'd202: c1m = 9'd11;  9'd203: c1m = 9'd11;
                9'd204: c1m = 9'd11;  9'd205: c1m = 9'd11;  9'd206: c1m = 9'd10;  9'd207: c1m = 9'd10;  9'd208: c1m = 9'd10;  9'd209: c1m = 9'd10;
                9'd210: c1m = 9'd10;  9'd211: c1m = 9'd10;  9'd212: c1m = 9'd10;  9'd213: c1m = 9'd10;  9'd214: c1m = 9'd10;  9'd215: c1m = 9'd9;
                9'd216: c1m = 9'd9;  9'd217: c1m = 9'd9;  9'd218: c1m = 9'd9;  9'd219: c1m = 9'd9;  9'd220: c1m = 9'd9;  9'd221: c1m = 9'd9;
                9'd222: c1m = 9'd9;  9'd223: c1m = 9'd9;  9'd224: c1m = 9'd9;  9'd225: c1m = 9'd8;  9'd226: c1m = 9'd8;  9'd227: c1m = 9'd8;
                9'd228: c1m = 9'd8;  9'd229: c1m = 9'd8;  9'd230: c1m = 9'd8;  9'd231: c1m = 9'd8;  9'd232: c1m = 9'd8;  9'd233: c1m = 9'd8;
                9'd234: c1m = 9'd8;  9'd235: c1m = 9'd8;  9'd236: c1m = 9'd7;  9'd237: c1m = 9'd7;  9'd238: c1m = 9'd7;  9'd239: c1m = 9'd7;
                9'd240: c1m = 9'd7;  9'd241: c1m = 9'd7;  9'd242: c1m = 9'd7;  9'd243: c1m = 9'd7;  9'd244: c1m = 9'd7;  9'd245: c1m = 9'd7;
                9'd246: c1m = 9'd7;  9'd247: c1m = 9'd7;  9'd248: c1m = 9'd7;  9'd249: c1m = 9'd6;  9'd250: c1m = 9'd6;  9'd251: c1m = 9'd6;
                9'd252: c1m = 9'd6;  9'd253: c1m = 9'd6;  9'd254: c1m = 9'd6;  9'd255: c1m = 9'd6;  9'd256: c1m = 9'd6;  9'd257: c1m = 9'd6;
                9'd258: c1m = 9'd6;  9'd259: c1m = 9'd6;  9'd260: c1m = 9'd6;  9'd261: c1m = 9'd6;  9'd262: c1m = 9'd6;  9'd263: c1m = 9'd6;
                9'd264: c1m = 9'd5;  9'd265: c1m = 9'd5;  9'd266: c1m = 9'd5;  9'd267: c1m = 9'd5;  9'd268: c1m = 9'd5;  9'd269: c1m = 9'd5;
                9'd270: c1m = 9'd5;  9'd271: c1m = 9'd5;  9'd272: c1m = 9'd5;  9'd273: c1m = 9'd5;  9'd274: c1m = 9'd5;  9'd275: c1m = 9'd5;
                9'd276: c1m = 9'd5;  9'd277: c1m = 9'd5;  9'd278: c1m = 9'd5;  9'd279: c1m = 9'd5;  9'd280: c1m = 9'd5;  9'd281: c1m = 9'd5;
                9'd282: c1m = 9'd4;  9'd283: c1m = 9'd4;  9'd284: c1m = 9'd4;  9'd285: c1m = 9'd4;  9'd286: c1m = 9'd4;  9'd287: c1m = 9'd4;
                9'd288: c1m = 9'd4;  9'd289: c1m = 9'd4;  9'd290: c1m = 9'd4;  9'd291: c1m = 9'd4;  9'd292: c1m = 9'd4;  9'd293: c1m = 9'd4;
                9'd294: c1m = 9'd4;  9'd295: c1m = 9'd4;  9'd296: c1m = 9'd4;  9'd297: c1m = 9'd4;  9'd298: c1m = 9'd4;  9'd299: c1m = 9'd4;
                9'd300: c1m = 9'd4;  9'd301: c1m = 9'd4;  9'd302: c1m = 9'd4;  9'd303: c1m = 9'd4;  9'd304: c1m = 9'd3;  9'd305: c1m = 9'd3;
                9'd306: c1m = 9'd3;  9'd307: c1m = 9'd3;  9'd308: c1m = 9'd3;  9'd309: c1m = 9'd3;  9'd310: c1m = 9'd3;  9'd311: c1m = 9'd3;
                9'd312: c1m = 9'd3;  9'd313: c1m = 9'd3;  9'd314: c1m = 9'd3;  9'd315: c1m = 9'd3;  9'd316: c1m = 9'd3;  9'd317: c1m = 9'd3;
                9'd318: c1m = 9'd3;  9'd319: c1m = 9'd3;  9'd320: c1m = 9'd3;  9'd321: c1m = 9'd3;  9'd322: c1m = 9'd3;  9'd323: c1m = 9'd3;
                9'd324: c1m = 9'd3;  9'd325: c1m = 9'd3;  9'd326: c1m = 9'd3;  9'd327: c1m = 9'd3;  9'd328: c1m = 9'd3;  9'd329: c1m = 9'd3;
                9'd330: c1m = 9'd3;  9'd331: c1m = 9'd3;  9'd332: c1m = 9'd3;  9'd333: c1m = 9'd3;  9'd334: c1m = 9'd3;  9'd335: c1m = 9'd2;
                9'd336: c1m = 9'd2;  9'd337: c1m = 9'd2;  9'd338: c1m = 9'd2;  9'd339: c1m = 9'd2;  9'd340: c1m = 9'd2;  9'd341: c1m = 9'd2;
                9'd342: c1m = 9'd2;  9'd343: c1m = 9'd2;  9'd344: c1m = 9'd2;  9'd345: c1m = 9'd2;  9'd346: c1m = 9'd2;  9'd347: c1m = 9'd2;
                9'd348: c1m = 9'd2;  9'd349: c1m = 9'd2;  9'd350: c1m = 9'd2;  9'd351: c1m = 9'd2;  9'd352: c1m = 9'd2;  9'd353: c1m = 9'd2;
                9'd354: c1m = 9'd2;  9'd355: c1m = 9'd2;  9'd356: c1m = 9'd2;  9'd357: c1m = 9'd2;  9'd358: c1m = 9'd2;  9'd359: c1m = 9'd2;
                9'd360: c1m = 9'd2;  9'd361: c1m = 9'd2;  9'd362: c1m = 9'd2;  9'd363: c1m = 9'd2;  9'd364: c1m = 9'd2;  9'd365: c1m = 9'd2;
                9'd366: c1m = 9'd2;  9'd367: c1m = 9'd2;  9'd368: c1m = 9'd2;  9'd369: c1m = 9'd2;  9'd370: c1m = 9'd2;  9'd371: c1m = 9'd2;
                9'd372: c1m = 9'd2;  9'd373: c1m = 9'd2;  9'd374: c1m = 9'd2;  9'd375: c1m = 9'd2;  9'd376: c1m = 9'd2;  9'd377: c1m = 9'd2;
                9'd378: c1m = 9'd2;  9'd379: c1m = 9'd2;  9'd380: c1m = 9'd2;  9'd381: c1m = 9'd2;  9'd382: c1m = 9'd1;  9'd383: c1m = 9'd1;
                9'd384: c1m = 9'd1;  9'd385: c1m = 9'd1;  9'd386: c1m = 9'd1;  9'd387: c1m = 9'd1;  9'd388: c1m = 9'd1;  9'd389: c1m = 9'd1;
                9'd390: c1m = 9'd1;  9'd391: c1m = 9'd1;  9'd392: c1m = 9'd1;  9'd393: c1m = 9'd1;  9'd394: c1m = 9'd1;  9'd395: c1m = 9'd1;
                9'd396: c1m = 9'd1;  9'd397: c1m = 9'd1;  9'd398: c1m = 9'd1;  9'd399: c1m = 9'd1;  9'd400: c1m = 9'd1;  9'd401: c1m = 9'd1;
                9'd402: c1m = 9'd1;  9'd403: c1m = 9'd1;  9'd404: c1m = 9'd1;  9'd405: c1m = 9'd1;  9'd406: c1m = 9'd1;  9'd407: c1m = 9'd1;
                9'd408: c1m = 9'd1;  9'd409: c1m = 9'd1;  9'd410: c1m = 9'd1;  9'd411: c1m = 9'd1;  9'd412: c1m = 9'd1;  9'd413: c1m = 9'd1;
                9'd414: c1m = 9'd1;  9'd415: c1m = 9'd1;  9'd416: c1m = 9'd1;  9'd417: c1m = 9'd1;  9'd418: c1m = 9'd1;  9'd419: c1m = 9'd1;
                9'd420: c1m = 9'd1;  9'd421: c1m = 9'd1;  9'd422: c1m = 9'd1;  9'd423: c1m = 9'd1;  9'd424: c1m = 9'd1;  9'd425: c1m = 9'd1;
                9'd426: c1m = 9'd1;  9'd427: c1m = 9'd1;  9'd428: c1m = 9'd1;  9'd429: c1m = 9'd1;  9'd430: c1m = 9'd1;  9'd431: c1m = 9'd1;
                9'd432: c1m = 9'd1;  9'd433: c1m = 9'd1;  9'd434: c1m = 9'd1;  9'd435: c1m = 9'd1;  9'd436: c1m = 9'd1;  9'd437: c1m = 9'd1;
                9'd438: c1m = 9'd1;  9'd439: c1m = 9'd1;  9'd440: c1m = 9'd1;  9'd441: c1m = 9'd1;  9'd442: c1m = 9'd1;  9'd443: c1m = 9'd1;
                9'd444: c1m = 9'd1;  9'd445: c1m = 9'd1;  9'd446: c1m = 9'd1;  9'd447: c1m = 9'd1;  9'd448: c1m = 9'd1;  9'd449: c1m = 9'd1;
                9'd450: c1m = 9'd1;  9'd451: c1m = 9'd1;  9'd452: c1m = 9'd1;  9'd453: c1m = 9'd1;  9'd454: c1m = 9'd1;  9'd455: c1m = 9'd1;
                9'd456: c1m = 9'd1;  9'd457: c1m = 9'd1;  9'd458: c1m = 9'd1;  9'd459: c1m = 9'd1;  9'd460: c1m = 9'd1;  9'd461: c1m = 9'd1;
                9'd462: c1m = 9'd1;  9'd463: c1m = 9'd1;  9'd464: c1m = 9'd1;  9'd465: c1m = 9'd1;  9'd466: c1m = 9'd1;  9'd467: c1m = 9'd1;
                9'd468: c1m = 9'd1;  9'd469: c1m = 9'd1;  9'd470: c1m = 9'd1;  9'd471: c1m = 9'd1;  9'd472: c1m = 9'd1;  9'd473: c1m = 9'd1;
                9'd474: c1m = 9'd1;  9'd475: c1m = 9'd1;  9'd476: c1m = 9'd1;  9'd477: c1m = 9'd1;  9'd478: c1m = 9'd1;  9'd479: c1m = 9'd1;
                9'd480: c1m = 9'd1;  9'd481: c1m = 9'd1;  9'd482: c1m = 9'd1;  9'd483: c1m = 9'd0;  9'd484: c1m = 9'd0;  9'd485: c1m = 9'd0;
                9'd486: c1m = 9'd0;  9'd487: c1m = 9'd0;  9'd488: c1m = 9'd0;  9'd489: c1m = 9'd0;  9'd490: c1m = 9'd0;  9'd491: c1m = 9'd0;
                9'd492: c1m = 9'd0;  9'd493: c1m = 9'd0;  9'd494: c1m = 9'd0;  9'd495: c1m = 9'd0;  9'd496: c1m = 9'd0;  9'd497: c1m = 9'd0;
                9'd498: c1m = 9'd0;  9'd499: c1m = 9'd0;  9'd500: c1m = 9'd0;  9'd501: c1m = 9'd0;  9'd502: c1m = 9'd0;  9'd503: c1m = 9'd0;
                9'd504: c1m = 9'd0;  9'd505: c1m = 9'd0;  9'd506: c1m = 9'd0;  9'd507: c1m = 9'd0;  9'd508: c1m = 9'd0;  9'd509: c1m = 9'd0;
                9'd510: c1m = 9'd0;  9'd511: c1m = 9'd0;
                default: c1m = 9'd0;
            endcase
        end
    endfunction
    function [6:0] lam;                   // 64 x log2(1 + 2^(-d/64))
        input [8:0] a;
        begin
            case (a)
                9'd0: lam = 7'd64;  9'd1: lam = 7'd64;  9'd2: lam = 7'd63;  9'd3: lam = 7'd63;  9'd4: lam = 7'd62;  9'd5: lam = 7'd62;
                9'd6: lam = 7'd61;  9'd7: lam = 7'd61;  9'd8: lam = 7'd60;  9'd9: lam = 7'd60;  9'd10: lam = 7'd59;  9'd11: lam = 7'd59;
                9'd12: lam = 7'd58;  9'd13: lam = 7'd58;  9'd14: lam = 7'd57;  9'd15: lam = 7'd57;  9'd16: lam = 7'd56;  9'd17: lam = 7'd56;
                9'd18: lam = 7'd55;  9'd19: lam = 7'd55;  9'd20: lam = 7'd55;  9'd21: lam = 7'd54;  9'd22: lam = 7'd54;  9'd23: lam = 7'd53;
                9'd24: lam = 7'd53;  9'd25: lam = 7'd52;  9'd26: lam = 7'd52;  9'd27: lam = 7'd51;  9'd28: lam = 7'd51;  9'd29: lam = 7'd51;
                9'd30: lam = 7'd50;  9'd31: lam = 7'd50;  9'd32: lam = 7'd49;  9'd33: lam = 7'd49;  9'd34: lam = 7'd49;  9'd35: lam = 7'd48;
                9'd36: lam = 7'd48;  9'd37: lam = 7'd47;  9'd38: lam = 7'd47;  9'd39: lam = 7'd47;  9'd40: lam = 7'd46;  9'd41: lam = 7'd46;
                9'd42: lam = 7'd45;  9'd43: lam = 7'd45;  9'd44: lam = 7'd45;  9'd45: lam = 7'd44;  9'd46: lam = 7'd44;  9'd47: lam = 7'd43;
                9'd48: lam = 7'd43;  9'd49: lam = 7'd43;  9'd50: lam = 7'd42;  9'd51: lam = 7'd42;  9'd52: lam = 7'd42;  9'd53: lam = 7'd41;
                9'd54: lam = 7'd41;  9'd55: lam = 7'd41;  9'd56: lam = 7'd40;  9'd57: lam = 7'd40;  9'd58: lam = 7'd39;  9'd59: lam = 7'd39;
                9'd60: lam = 7'd39;  9'd61: lam = 7'd38;  9'd62: lam = 7'd38;  9'd63: lam = 7'd38;  9'd64: lam = 7'd37;  9'd65: lam = 7'd37;
                9'd66: lam = 7'd37;  9'd67: lam = 7'd36;  9'd68: lam = 7'd36;  9'd69: lam = 7'd36;  9'd70: lam = 7'd35;  9'd71: lam = 7'd35;
                9'd72: lam = 7'd35;  9'd73: lam = 7'd35;  9'd74: lam = 7'd34;  9'd75: lam = 7'd34;  9'd76: lam = 7'd34;  9'd77: lam = 7'd33;
                9'd78: lam = 7'd33;  9'd79: lam = 7'd33;  9'd80: lam = 7'd32;  9'd81: lam = 7'd32;  9'd82: lam = 7'd32;  9'd83: lam = 7'd32;
                9'd84: lam = 7'd31;  9'd85: lam = 7'd31;  9'd86: lam = 7'd31;  9'd87: lam = 7'd30;  9'd88: lam = 7'd30;  9'd89: lam = 7'd30;
                9'd90: lam = 7'd30;  9'd91: lam = 7'd29;  9'd92: lam = 7'd29;  9'd93: lam = 7'd29;  9'd94: lam = 7'd28;  9'd95: lam = 7'd28;
                9'd96: lam = 7'd28;  9'd97: lam = 7'd28;  9'd98: lam = 7'd27;  9'd99: lam = 7'd27;  9'd100: lam = 7'd27;  9'd101: lam = 7'd27;
                9'd102: lam = 7'd26;  9'd103: lam = 7'd26;  9'd104: lam = 7'd26;  9'd105: lam = 7'd26;  9'd106: lam = 7'd25;  9'd107: lam = 7'd25;
                9'd108: lam = 7'd25;  9'd109: lam = 7'd25;  9'd110: lam = 7'd24;  9'd111: lam = 7'd24;  9'd112: lam = 7'd24;  9'd113: lam = 7'd24;
                9'd114: lam = 7'd24;  9'd115: lam = 7'd23;  9'd116: lam = 7'd23;  9'd117: lam = 7'd23;  9'd118: lam = 7'd23;  9'd119: lam = 7'd22;
                9'd120: lam = 7'd22;  9'd121: lam = 7'd22;  9'd122: lam = 7'd22;  9'd123: lam = 7'd22;  9'd124: lam = 7'd21;  9'd125: lam = 7'd21;
                9'd126: lam = 7'd21;  9'd127: lam = 7'd21;  9'd128: lam = 7'd21;  9'd129: lam = 7'd20;  9'd130: lam = 7'd20;  9'd131: lam = 7'd20;
                9'd132: lam = 7'd20;  9'd133: lam = 7'd20;  9'd134: lam = 7'd19;  9'd135: lam = 7'd19;  9'd136: lam = 7'd19;  9'd137: lam = 7'd19;
                9'd138: lam = 7'd19;  9'd139: lam = 7'd19;  9'd140: lam = 7'd18;  9'd141: lam = 7'd18;  9'd142: lam = 7'd18;  9'd143: lam = 7'd18;
                9'd144: lam = 7'd18;  9'd145: lam = 7'd17;  9'd146: lam = 7'd17;  9'd147: lam = 7'd17;  9'd148: lam = 7'd17;  9'd149: lam = 7'd17;
                9'd150: lam = 7'd17;  9'd151: lam = 7'd16;  9'd152: lam = 7'd16;  9'd153: lam = 7'd16;  9'd154: lam = 7'd16;  9'd155: lam = 7'd16;
                9'd156: lam = 7'd16;  9'd157: lam = 7'd15;  9'd158: lam = 7'd15;  9'd159: lam = 7'd15;  9'd160: lam = 7'd15;  9'd161: lam = 7'd15;
                9'd162: lam = 7'd15;  9'd163: lam = 7'd15;  9'd164: lam = 7'd14;  9'd165: lam = 7'd14;  9'd166: lam = 7'd14;  9'd167: lam = 7'd14;
                9'd168: lam = 7'd14;  9'd169: lam = 7'd14;  9'd170: lam = 7'd14;  9'd171: lam = 7'd13;  9'd172: lam = 7'd13;  9'd173: lam = 7'd13;
                9'd174: lam = 7'd13;  9'd175: lam = 7'd13;  9'd176: lam = 7'd13;  9'd177: lam = 7'd13;  9'd178: lam = 7'd13;  9'd179: lam = 7'd12;
                9'd180: lam = 7'd12;  9'd181: lam = 7'd12;  9'd182: lam = 7'd12;  9'd183: lam = 7'd12;  9'd184: lam = 7'd12;  9'd185: lam = 7'd12;
                9'd186: lam = 7'd12;  9'd187: lam = 7'd11;  9'd188: lam = 7'd11;  9'd189: lam = 7'd11;  9'd190: lam = 7'd11;  9'd191: lam = 7'd11;
                9'd192: lam = 7'd11;  9'd193: lam = 7'd11;  9'd194: lam = 7'd11;  9'd195: lam = 7'd11;  9'd196: lam = 7'd10;  9'd197: lam = 7'd10;
                9'd198: lam = 7'd10;  9'd199: lam = 7'd10;  9'd200: lam = 7'd10;  9'd201: lam = 7'd10;  9'd202: lam = 7'd10;  9'd203: lam = 7'd10;
                9'd204: lam = 7'd10;  9'd205: lam = 7'd10;  9'd206: lam = 7'd9;  9'd207: lam = 7'd9;  9'd208: lam = 7'd9;  9'd209: lam = 7'd9;
                9'd210: lam = 7'd9;  9'd211: lam = 7'd9;  9'd212: lam = 7'd9;  9'd213: lam = 7'd9;  9'd214: lam = 7'd9;  9'd215: lam = 7'd9;
                9'd216: lam = 7'd8;  9'd217: lam = 7'd8;  9'd218: lam = 7'd8;  9'd219: lam = 7'd8;  9'd220: lam = 7'd8;  9'd221: lam = 7'd8;
                9'd222: lam = 7'd8;  9'd223: lam = 7'd8;  9'd224: lam = 7'd8;  9'd225: lam = 7'd8;  9'd226: lam = 7'd8;  9'd227: lam = 7'd8;
                9'd228: lam = 7'd8;  9'd229: lam = 7'd7;  9'd230: lam = 7'd7;  9'd231: lam = 7'd7;  9'd232: lam = 7'd7;  9'd233: lam = 7'd7;
                9'd234: lam = 7'd7;  9'd235: lam = 7'd7;  9'd236: lam = 7'd7;  9'd237: lam = 7'd7;  9'd238: lam = 7'd7;  9'd239: lam = 7'd7;
                9'd240: lam = 7'd7;  9'd241: lam = 7'd7;  9'd242: lam = 7'd6;  9'd243: lam = 7'd6;  9'd244: lam = 7'd6;  9'd245: lam = 7'd6;
                9'd246: lam = 7'd6;  9'd247: lam = 7'd6;  9'd248: lam = 7'd6;  9'd249: lam = 7'd6;  9'd250: lam = 7'd6;  9'd251: lam = 7'd6;
                9'd252: lam = 7'd6;  9'd253: lam = 7'd6;  9'd254: lam = 7'd6;  9'd255: lam = 7'd6;  9'd256: lam = 7'd6;  9'd257: lam = 7'd6;
                9'd258: lam = 7'd5;  9'd259: lam = 7'd5;  9'd260: lam = 7'd5;  9'd261: lam = 7'd5;  9'd262: lam = 7'd5;  9'd263: lam = 7'd5;
                9'd264: lam = 7'd5;  9'd265: lam = 7'd5;  9'd266: lam = 7'd5;  9'd267: lam = 7'd5;  9'd268: lam = 7'd5;  9'd269: lam = 7'd5;
                9'd270: lam = 7'd5;  9'd271: lam = 7'd5;  9'd272: lam = 7'd5;  9'd273: lam = 7'd5;  9'd274: lam = 7'd5;  9'd275: lam = 7'd5;
                9'd276: lam = 7'd5;  9'd277: lam = 7'd4;  9'd278: lam = 7'd4;  9'd279: lam = 7'd4;  9'd280: lam = 7'd4;  9'd281: lam = 7'd4;
                9'd282: lam = 7'd4;  9'd283: lam = 7'd4;  9'd284: lam = 7'd4;  9'd285: lam = 7'd4;  9'd286: lam = 7'd4;  9'd287: lam = 7'd4;
                9'd288: lam = 7'd4;  9'd289: lam = 7'd4;  9'd290: lam = 7'd4;  9'd291: lam = 7'd4;  9'd292: lam = 7'd4;  9'd293: lam = 7'd4;
                9'd294: lam = 7'd4;  9'd295: lam = 7'd4;  9'd296: lam = 7'd4;  9'd297: lam = 7'd4;  9'd298: lam = 7'd4;  9'd299: lam = 7'd4;
                9'd300: lam = 7'd4;  9'd301: lam = 7'd3;  9'd302: lam = 7'd3;  9'd303: lam = 7'd3;  9'd304: lam = 7'd3;  9'd305: lam = 7'd3;
                9'd306: lam = 7'd3;  9'd307: lam = 7'd3;  9'd308: lam = 7'd3;  9'd309: lam = 7'd3;  9'd310: lam = 7'd3;  9'd311: lam = 7'd3;
                9'd312: lam = 7'd3;  9'd313: lam = 7'd3;  9'd314: lam = 7'd3;  9'd315: lam = 7'd3;  9'd316: lam = 7'd3;  9'd317: lam = 7'd3;
                9'd318: lam = 7'd3;  9'd319: lam = 7'd3;  9'd320: lam = 7'd3;  9'd321: lam = 7'd3;  9'd322: lam = 7'd3;  9'd323: lam = 7'd3;
                9'd324: lam = 7'd3;  9'd325: lam = 7'd3;  9'd326: lam = 7'd3;  9'd327: lam = 7'd3;  9'd328: lam = 7'd3;  9'd329: lam = 7'd3;
                9'd330: lam = 7'd3;  9'd331: lam = 7'd3;  9'd332: lam = 7'd2;  9'd333: lam = 7'd2;  9'd334: lam = 7'd2;  9'd335: lam = 7'd2;
                9'd336: lam = 7'd2;  9'd337: lam = 7'd2;  9'd338: lam = 7'd2;  9'd339: lam = 7'd2;  9'd340: lam = 7'd2;  9'd341: lam = 7'd2;
                9'd342: lam = 7'd2;  9'd343: lam = 7'd2;  9'd344: lam = 7'd2;  9'd345: lam = 7'd2;  9'd346: lam = 7'd2;  9'd347: lam = 7'd2;
                9'd348: lam = 7'd2;  9'd349: lam = 7'd2;  9'd350: lam = 7'd2;  9'd351: lam = 7'd2;  9'd352: lam = 7'd2;  9'd353: lam = 7'd2;
                9'd354: lam = 7'd2;  9'd355: lam = 7'd2;  9'd356: lam = 7'd2;  9'd357: lam = 7'd2;  9'd358: lam = 7'd2;  9'd359: lam = 7'd2;
                9'd360: lam = 7'd2;  9'd361: lam = 7'd2;  9'd362: lam = 7'd2;  9'd363: lam = 7'd2;  9'd364: lam = 7'd2;  9'd365: lam = 7'd2;
                9'd366: lam = 7'd2;  9'd367: lam = 7'd2;  9'd368: lam = 7'd2;  9'd369: lam = 7'd2;  9'd370: lam = 7'd2;  9'd371: lam = 7'd2;
                9'd372: lam = 7'd2;  9'd373: lam = 7'd2;  9'd374: lam = 7'd2;  9'd375: lam = 7'd2;  9'd376: lam = 7'd2;  9'd377: lam = 7'd2;
                9'd378: lam = 7'd2;  9'd379: lam = 7'd2;  9'd380: lam = 7'd1;  9'd381: lam = 7'd1;  9'd382: lam = 7'd1;  9'd383: lam = 7'd1;
                9'd384: lam = 7'd1;  9'd385: lam = 7'd1;  9'd386: lam = 7'd1;  9'd387: lam = 7'd1;  9'd388: lam = 7'd1;  9'd389: lam = 7'd1;
                9'd390: lam = 7'd1;  9'd391: lam = 7'd1;  9'd392: lam = 7'd1;  9'd393: lam = 7'd1;  9'd394: lam = 7'd1;  9'd395: lam = 7'd1;
                9'd396: lam = 7'd1;  9'd397: lam = 7'd1;  9'd398: lam = 7'd1;  9'd399: lam = 7'd1;  9'd400: lam = 7'd1;  9'd401: lam = 7'd1;
                9'd402: lam = 7'd1;  9'd403: lam = 7'd1;  9'd404: lam = 7'd1;  9'd405: lam = 7'd1;  9'd406: lam = 7'd1;  9'd407: lam = 7'd1;
                9'd408: lam = 7'd1;  9'd409: lam = 7'd1;  9'd410: lam = 7'd1;  9'd411: lam = 7'd1;  9'd412: lam = 7'd1;  9'd413: lam = 7'd1;
                9'd414: lam = 7'd1;  9'd415: lam = 7'd1;  9'd416: lam = 7'd1;  9'd417: lam = 7'd1;  9'd418: lam = 7'd1;  9'd419: lam = 7'd1;
                9'd420: lam = 7'd1;  9'd421: lam = 7'd1;  9'd422: lam = 7'd1;  9'd423: lam = 7'd1;  9'd424: lam = 7'd1;  9'd425: lam = 7'd1;
                9'd426: lam = 7'd1;  9'd427: lam = 7'd1;  9'd428: lam = 7'd1;  9'd429: lam = 7'd1;  9'd430: lam = 7'd1;  9'd431: lam = 7'd1;
                9'd432: lam = 7'd1;  9'd433: lam = 7'd1;  9'd434: lam = 7'd1;  9'd435: lam = 7'd1;  9'd436: lam = 7'd1;  9'd437: lam = 7'd1;
                9'd438: lam = 7'd1;  9'd439: lam = 7'd1;  9'd440: lam = 7'd1;  9'd441: lam = 7'd1;  9'd442: lam = 7'd1;  9'd443: lam = 7'd1;
                9'd444: lam = 7'd1;  9'd445: lam = 7'd1;  9'd446: lam = 7'd1;  9'd447: lam = 7'd1;  9'd448: lam = 7'd1;  9'd449: lam = 7'd1;
                9'd450: lam = 7'd1;  9'd451: lam = 7'd1;  9'd452: lam = 7'd1;  9'd453: lam = 7'd1;  9'd454: lam = 7'd1;  9'd455: lam = 7'd1;
                9'd456: lam = 7'd1;  9'd457: lam = 7'd1;  9'd458: lam = 7'd1;  9'd459: lam = 7'd1;  9'd460: lam = 7'd1;  9'd461: lam = 7'd1;
                9'd462: lam = 7'd1;  9'd463: lam = 7'd1;  9'd464: lam = 7'd1;  9'd465: lam = 7'd1;  9'd466: lam = 7'd1;  9'd467: lam = 7'd1;
                9'd468: lam = 7'd1;  9'd469: lam = 7'd1;  9'd470: lam = 7'd1;  9'd471: lam = 7'd1;  9'd472: lam = 7'd1;  9'd473: lam = 7'd1;
                9'd474: lam = 7'd1;  9'd475: lam = 7'd1;  9'd476: lam = 7'd1;  9'd477: lam = 7'd1;  9'd478: lam = 7'd1;  9'd479: lam = 7'd1;
                9'd480: lam = 7'd1;  9'd481: lam = 7'd1;  9'd482: lam = 7'd0;  9'd483: lam = 7'd0;  9'd484: lam = 7'd0;  9'd485: lam = 7'd0;
                9'd486: lam = 7'd0;  9'd487: lam = 7'd0;  9'd488: lam = 7'd0;  9'd489: lam = 7'd0;  9'd490: lam = 7'd0;  9'd491: lam = 7'd0;
                9'd492: lam = 7'd0;  9'd493: lam = 7'd0;  9'd494: lam = 7'd0;  9'd495: lam = 7'd0;  9'd496: lam = 7'd0;  9'd497: lam = 7'd0;
                9'd498: lam = 7'd0;  9'd499: lam = 7'd0;  9'd500: lam = 7'd0;  9'd501: lam = 7'd0;  9'd502: lam = 7'd0;  9'd503: lam = 7'd0;
                9'd504: lam = 7'd0;  9'd505: lam = 7'd0;  9'd506: lam = 7'd0;  9'd507: lam = 7'd0;  9'd508: lam = 7'd0;  9'd509: lam = 7'd0;
                9'd510: lam = 7'd0;  9'd511: lam = 7'd0;
                default: lam = 7'd0;
            endcase
        end
    endfunction
    function [9:0] gtm;                   // 128 x log2(1 + 2^(-(j - 256)/64)): how far the gain^2 is below 1
        input [9:0] a;
        begin
            case (a)
                10'd0: gtm = 10'd523;  10'd1: gtm = 10'd521;  10'd2: gtm = 10'd519;  10'd3: gtm = 10'd518;  10'd4: gtm = 10'd516;  10'd5: gtm = 10'd514;
                10'd6: gtm = 10'd512;  10'd7: gtm = 10'd510;  10'd8: gtm = 10'd508;  10'd9: gtm = 10'd506;  10'd10: gtm = 10'd504;  10'd11: gtm = 10'd503;
                10'd12: gtm = 10'd501;  10'd13: gtm = 10'd499;  10'd14: gtm = 10'd497;  10'd15: gtm = 10'd495;  10'd16: gtm = 10'd493;  10'd17: gtm = 10'd491;
                10'd18: gtm = 10'd490;  10'd19: gtm = 10'd488;  10'd20: gtm = 10'd486;  10'd21: gtm = 10'd484;  10'd22: gtm = 10'd482;  10'd23: gtm = 10'd480;
                10'd24: gtm = 10'd478;  10'd25: gtm = 10'd477;  10'd26: gtm = 10'd475;  10'd27: gtm = 10'd473;  10'd28: gtm = 10'd471;  10'd29: gtm = 10'd469;
                10'd30: gtm = 10'd467;  10'd31: gtm = 10'd465;  10'd32: gtm = 10'd464;  10'd33: gtm = 10'd462;  10'd34: gtm = 10'd460;  10'd35: gtm = 10'd458;
                10'd36: gtm = 10'd456;  10'd37: gtm = 10'd454;  10'd38: gtm = 10'd453;  10'd39: gtm = 10'd451;  10'd40: gtm = 10'd449;  10'd41: gtm = 10'd447;
                10'd42: gtm = 10'd445;  10'd43: gtm = 10'd444;  10'd44: gtm = 10'd442;  10'd45: gtm = 10'd440;  10'd46: gtm = 10'd438;  10'd47: gtm = 10'd436;
                10'd48: gtm = 10'd434;  10'd49: gtm = 10'd433;  10'd50: gtm = 10'd431;  10'd51: gtm = 10'd429;  10'd52: gtm = 10'd427;  10'd53: gtm = 10'd425;
                10'd54: gtm = 10'd424;  10'd55: gtm = 10'd422;  10'd56: gtm = 10'd420;  10'd57: gtm = 10'd418;  10'd58: gtm = 10'd416;  10'd59: gtm = 10'd415;
                10'd60: gtm = 10'd413;  10'd61: gtm = 10'd411;  10'd62: gtm = 10'd409;  10'd63: gtm = 10'd408;  10'd64: gtm = 10'd406;  10'd65: gtm = 10'd404;
                10'd66: gtm = 10'd402;  10'd67: gtm = 10'd400;  10'd68: gtm = 10'd399;  10'd69: gtm = 10'd397;  10'd70: gtm = 10'd395;  10'd71: gtm = 10'd393;
                10'd72: gtm = 10'd392;  10'd73: gtm = 10'd390;  10'd74: gtm = 10'd388;  10'd75: gtm = 10'd386;  10'd76: gtm = 10'd385;  10'd77: gtm = 10'd383;
                10'd78: gtm = 10'd381;  10'd79: gtm = 10'd379;  10'd80: gtm = 10'd378;  10'd81: gtm = 10'd376;  10'd82: gtm = 10'd374;  10'd83: gtm = 10'd372;
                10'd84: gtm = 10'd371;  10'd85: gtm = 10'd369;  10'd86: gtm = 10'd367;  10'd87: gtm = 10'd365;  10'd88: gtm = 10'd364;  10'd89: gtm = 10'd362;
                10'd90: gtm = 10'd360;  10'd91: gtm = 10'd359;  10'd92: gtm = 10'd357;  10'd93: gtm = 10'd355;  10'd94: gtm = 10'd353;  10'd95: gtm = 10'd352;
                10'd96: gtm = 10'd350;  10'd97: gtm = 10'd348;  10'd98: gtm = 10'd347;  10'd99: gtm = 10'd345;  10'd100: gtm = 10'd343;  10'd101: gtm = 10'd342;
                10'd102: gtm = 10'd340;  10'd103: gtm = 10'd338;  10'd104: gtm = 10'd337;  10'd105: gtm = 10'd335;  10'd106: gtm = 10'd333;  10'd107: gtm = 10'd332;
                10'd108: gtm = 10'd330;  10'd109: gtm = 10'd328;  10'd110: gtm = 10'd327;  10'd111: gtm = 10'd325;  10'd112: gtm = 10'd323;  10'd113: gtm = 10'd322;
                10'd114: gtm = 10'd320;  10'd115: gtm = 10'd318;  10'd116: gtm = 10'd317;  10'd117: gtm = 10'd315;  10'd118: gtm = 10'd313;  10'd119: gtm = 10'd312;
                10'd120: gtm = 10'd310;  10'd121: gtm = 10'd308;  10'd122: gtm = 10'd307;  10'd123: gtm = 10'd305;  10'd124: gtm = 10'd304;  10'd125: gtm = 10'd302;
                10'd126: gtm = 10'd300;  10'd127: gtm = 10'd299;  10'd128: gtm = 10'd297;  10'd129: gtm = 10'd296;  10'd130: gtm = 10'd294;  10'd131: gtm = 10'd292;
                10'd132: gtm = 10'd291;  10'd133: gtm = 10'd289;  10'd134: gtm = 10'd288;  10'd135: gtm = 10'd286;  10'd136: gtm = 10'd285;  10'd137: gtm = 10'd283;
                10'd138: gtm = 10'd281;  10'd139: gtm = 10'd280;  10'd140: gtm = 10'd278;  10'd141: gtm = 10'd277;  10'd142: gtm = 10'd275;  10'd143: gtm = 10'd274;
                10'd144: gtm = 10'd272;  10'd145: gtm = 10'd271;  10'd146: gtm = 10'd269;  10'd147: gtm = 10'd267;  10'd148: gtm = 10'd266;  10'd149: gtm = 10'd264;
                10'd150: gtm = 10'd263;  10'd151: gtm = 10'd261;  10'd152: gtm = 10'd260;  10'd153: gtm = 10'd258;  10'd154: gtm = 10'd257;  10'd155: gtm = 10'd255;
                10'd156: gtm = 10'd254;  10'd157: gtm = 10'd252;  10'd158: gtm = 10'd251;  10'd159: gtm = 10'd249;  10'd160: gtm = 10'd248;  10'd161: gtm = 10'd246;
                10'd162: gtm = 10'd245;  10'd163: gtm = 10'd243;  10'd164: gtm = 10'd242;  10'd165: gtm = 10'd241;  10'd166: gtm = 10'd239;  10'd167: gtm = 10'd238;
                10'd168: gtm = 10'd236;  10'd169: gtm = 10'd235;  10'd170: gtm = 10'd233;  10'd171: gtm = 10'd232;  10'd172: gtm = 10'd230;  10'd173: gtm = 10'd229;
                10'd174: gtm = 10'd228;  10'd175: gtm = 10'd226;  10'd176: gtm = 10'd225;  10'd177: gtm = 10'd223;  10'd178: gtm = 10'd222;  10'd179: gtm = 10'd221;
                10'd180: gtm = 10'd219;  10'd181: gtm = 10'd218;  10'd182: gtm = 10'd216;  10'd183: gtm = 10'd215;  10'd184: gtm = 10'd214;  10'd185: gtm = 10'd212;
                10'd186: gtm = 10'd211;  10'd187: gtm = 10'd210;  10'd188: gtm = 10'd208;  10'd189: gtm = 10'd207;  10'd190: gtm = 10'd206;  10'd191: gtm = 10'd204;
                10'd192: gtm = 10'd203;  10'd193: gtm = 10'd202;  10'd194: gtm = 10'd200;  10'd195: gtm = 10'd199;  10'd196: gtm = 10'd198;  10'd197: gtm = 10'd196;
                10'd198: gtm = 10'd195;  10'd199: gtm = 10'd194;  10'd200: gtm = 10'd192;  10'd201: gtm = 10'd191;  10'd202: gtm = 10'd190;  10'd203: gtm = 10'd189;
                10'd204: gtm = 10'd187;  10'd205: gtm = 10'd186;  10'd206: gtm = 10'd185;  10'd207: gtm = 10'd183;  10'd208: gtm = 10'd182;  10'd209: gtm = 10'd181;
                10'd210: gtm = 10'd180;  10'd211: gtm = 10'd178;  10'd212: gtm = 10'd177;  10'd213: gtm = 10'd176;  10'd214: gtm = 10'd175;  10'd215: gtm = 10'd174;
                10'd216: gtm = 10'd172;  10'd217: gtm = 10'd171;  10'd218: gtm = 10'd170;  10'd219: gtm = 10'd169;  10'd220: gtm = 10'd167;  10'd221: gtm = 10'd166;
                10'd222: gtm = 10'd165;  10'd223: gtm = 10'd164;  10'd224: gtm = 10'd163;  10'd225: gtm = 10'd162;  10'd226: gtm = 10'd160;  10'd227: gtm = 10'd159;
                10'd228: gtm = 10'd158;  10'd229: gtm = 10'd157;  10'd230: gtm = 10'd156;  10'd231: gtm = 10'd155;  10'd232: gtm = 10'd154;  10'd233: gtm = 10'd152;
                10'd234: gtm = 10'd151;  10'd235: gtm = 10'd150;  10'd236: gtm = 10'd149;  10'd237: gtm = 10'd148;  10'd238: gtm = 10'd147;  10'd239: gtm = 10'd146;
                10'd240: gtm = 10'd145;  10'd241: gtm = 10'd144;  10'd242: gtm = 10'd143;  10'd243: gtm = 10'd141;  10'd244: gtm = 10'd140;  10'd245: gtm = 10'd139;
                10'd246: gtm = 10'd138;  10'd247: gtm = 10'd137;  10'd248: gtm = 10'd136;  10'd249: gtm = 10'd135;  10'd250: gtm = 10'd134;  10'd251: gtm = 10'd133;
                10'd252: gtm = 10'd132;  10'd253: gtm = 10'd131;  10'd254: gtm = 10'd130;  10'd255: gtm = 10'd129;  10'd256: gtm = 10'd128;  10'd257: gtm = 10'd127;
                10'd258: gtm = 10'd126;  10'd259: gtm = 10'd125;  10'd260: gtm = 10'd124;  10'd261: gtm = 10'd123;  10'd262: gtm = 10'd122;  10'd263: gtm = 10'd121;
                10'd264: gtm = 10'd120;  10'd265: gtm = 10'd119;  10'd266: gtm = 10'd118;  10'd267: gtm = 10'd117;  10'd268: gtm = 10'd116;  10'd269: gtm = 10'd115;
                10'd270: gtm = 10'd115;  10'd271: gtm = 10'd114;  10'd272: gtm = 10'd113;  10'd273: gtm = 10'd112;  10'd274: gtm = 10'd111;  10'd275: gtm = 10'd110;
                10'd276: gtm = 10'd109;  10'd277: gtm = 10'd108;  10'd278: gtm = 10'd107;  10'd279: gtm = 10'd106;  10'd280: gtm = 10'd106;  10'd281: gtm = 10'd105;
                10'd282: gtm = 10'd104;  10'd283: gtm = 10'd103;  10'd284: gtm = 10'd102;  10'd285: gtm = 10'd101;  10'd286: gtm = 10'd100;  10'd287: gtm = 10'd100;
                10'd288: gtm = 10'd99;  10'd289: gtm = 10'd98;  10'd290: gtm = 10'd97;  10'd291: gtm = 10'd96;  10'd292: gtm = 10'd95;  10'd293: gtm = 10'd95;
                10'd294: gtm = 10'd94;  10'd295: gtm = 10'd93;  10'd296: gtm = 10'd92;  10'd297: gtm = 10'd92;  10'd298: gtm = 10'd91;  10'd299: gtm = 10'd90;
                10'd300: gtm = 10'd89;  10'd301: gtm = 10'd88;  10'd302: gtm = 10'd88;  10'd303: gtm = 10'd87;  10'd304: gtm = 10'd86;  10'd305: gtm = 10'd85;
                10'd306: gtm = 10'd85;  10'd307: gtm = 10'd84;  10'd308: gtm = 10'd83;  10'd309: gtm = 10'd83;  10'd310: gtm = 10'd82;  10'd311: gtm = 10'd81;
                10'd312: gtm = 10'd80;  10'd313: gtm = 10'd80;  10'd314: gtm = 10'd79;  10'd315: gtm = 10'd78;  10'd316: gtm = 10'd78;  10'd317: gtm = 10'd77;
                10'd318: gtm = 10'd76;  10'd319: gtm = 10'd76;  10'd320: gtm = 10'd75;  10'd321: gtm = 10'd74;  10'd322: gtm = 10'd74;  10'd323: gtm = 10'd73;
                10'd324: gtm = 10'd72;  10'd325: gtm = 10'd72;  10'd326: gtm = 10'd71;  10'd327: gtm = 10'd70;  10'd328: gtm = 10'd70;  10'd329: gtm = 10'd69;
                10'd330: gtm = 10'd68;  10'd331: gtm = 10'd68;  10'd332: gtm = 10'd67;  10'd333: gtm = 10'd67;  10'd334: gtm = 10'd66;  10'd335: gtm = 10'd65;
                10'd336: gtm = 10'd65;  10'd337: gtm = 10'd64;  10'd338: gtm = 10'd64;  10'd339: gtm = 10'd63;  10'd340: gtm = 10'd62;  10'd341: gtm = 10'd62;
                10'd342: gtm = 10'd61;  10'd343: gtm = 10'd61;  10'd344: gtm = 10'd60;  10'd345: gtm = 10'd60;  10'd346: gtm = 10'd59;  10'd347: gtm = 10'd59;
                10'd348: gtm = 10'd58;  10'd349: gtm = 10'd57;  10'd350: gtm = 10'd57;  10'd351: gtm = 10'd56;  10'd352: gtm = 10'd56;  10'd353: gtm = 10'd55;
                10'd354: gtm = 10'd55;  10'd355: gtm = 10'd54;  10'd356: gtm = 10'd54;  10'd357: gtm = 10'd53;  10'd358: gtm = 10'd53;  10'd359: gtm = 10'd52;
                10'd360: gtm = 10'd52;  10'd361: gtm = 10'd51;  10'd362: gtm = 10'd51;  10'd363: gtm = 10'd50;  10'd364: gtm = 10'd50;  10'd365: gtm = 10'd49;
                10'd366: gtm = 10'd49;  10'd367: gtm = 10'd49;  10'd368: gtm = 10'd48;  10'd369: gtm = 10'd48;  10'd370: gtm = 10'd47;  10'd371: gtm = 10'd47;
                10'd372: gtm = 10'd46;  10'd373: gtm = 10'd46;  10'd374: gtm = 10'd45;  10'd375: gtm = 10'd45;  10'd376: gtm = 10'd45;  10'd377: gtm = 10'd44;
                10'd378: gtm = 10'd44;  10'd379: gtm = 10'd43;  10'd380: gtm = 10'd43;  10'd381: gtm = 10'd42;  10'd382: gtm = 10'd42;  10'd383: gtm = 10'd42;
                10'd384: gtm = 10'd41;  10'd385: gtm = 10'd41;  10'd386: gtm = 10'd40;  10'd387: gtm = 10'd40;  10'd388: gtm = 10'd40;  10'd389: gtm = 10'd39;
                10'd390: gtm = 10'd39;  10'd391: gtm = 10'd38;  10'd392: gtm = 10'd38;  10'd393: gtm = 10'd38;  10'd394: gtm = 10'd37;  10'd395: gtm = 10'd37;
                10'd396: gtm = 10'd37;  10'd397: gtm = 10'd36;  10'd398: gtm = 10'd36;  10'd399: gtm = 10'd36;  10'd400: gtm = 10'd35;  10'd401: gtm = 10'd35;
                10'd402: gtm = 10'd35;  10'd403: gtm = 10'd34;  10'd404: gtm = 10'd34;  10'd405: gtm = 10'd34;  10'd406: gtm = 10'd33;  10'd407: gtm = 10'd33;
                10'd408: gtm = 10'd33;  10'd409: gtm = 10'd32;  10'd410: gtm = 10'd32;  10'd411: gtm = 10'd32;  10'd412: gtm = 10'd31;  10'd413: gtm = 10'd31;
                10'd414: gtm = 10'd31;  10'd415: gtm = 10'd30;  10'd416: gtm = 10'd30;  10'd417: gtm = 10'd30;  10'd418: gtm = 10'd29;  10'd419: gtm = 10'd29;
                10'd420: gtm = 10'd29;  10'd421: gtm = 10'd29;  10'd422: gtm = 10'd28;  10'd423: gtm = 10'd28;  10'd424: gtm = 10'd28;  10'd425: gtm = 10'd27;
                10'd426: gtm = 10'd27;  10'd427: gtm = 10'd27;  10'd428: gtm = 10'd27;  10'd429: gtm = 10'd26;  10'd430: gtm = 10'd26;  10'd431: gtm = 10'd26;
                10'd432: gtm = 10'd26;  10'd433: gtm = 10'd25;  10'd434: gtm = 10'd25;  10'd435: gtm = 10'd25;  10'd436: gtm = 10'd25;  10'd437: gtm = 10'd24;
                10'd438: gtm = 10'd24;  10'd439: gtm = 10'd24;  10'd440: gtm = 10'd24;  10'd441: gtm = 10'd23;  10'd442: gtm = 10'd23;  10'd443: gtm = 10'd23;
                10'd444: gtm = 10'd23;  10'd445: gtm = 10'd22;  10'd446: gtm = 10'd22;  10'd447: gtm = 10'd22;  10'd448: gtm = 10'd22;  10'd449: gtm = 10'd22;
                10'd450: gtm = 10'd21;  10'd451: gtm = 10'd21;  10'd452: gtm = 10'd21;  10'd453: gtm = 10'd21;  10'd454: gtm = 10'd20;  10'd455: gtm = 10'd20;
                10'd456: gtm = 10'd20;  10'd457: gtm = 10'd20;  10'd458: gtm = 10'd20;  10'd459: gtm = 10'd19;  10'd460: gtm = 10'd19;  10'd461: gtm = 10'd19;
                10'd462: gtm = 10'd19;  10'd463: gtm = 10'd19;  10'd464: gtm = 10'd18;  10'd465: gtm = 10'd18;  10'd466: gtm = 10'd18;  10'd467: gtm = 10'd18;
                10'd468: gtm = 10'd18;  10'd469: gtm = 10'd18;  10'd470: gtm = 10'd17;  10'd471: gtm = 10'd17;  10'd472: gtm = 10'd17;  10'd473: gtm = 10'd17;
                10'd474: gtm = 10'd17;  10'd475: gtm = 10'd16;  10'd476: gtm = 10'd16;  10'd477: gtm = 10'd16;  10'd478: gtm = 10'd16;  10'd479: gtm = 10'd16;
                10'd480: gtm = 10'd16;  10'd481: gtm = 10'd15;  10'd482: gtm = 10'd15;  10'd483: gtm = 10'd15;  10'd484: gtm = 10'd15;  10'd485: gtm = 10'd15;
                10'd486: gtm = 10'd15;  10'd487: gtm = 10'd15;  10'd488: gtm = 10'd14;  10'd489: gtm = 10'd14;  10'd490: gtm = 10'd14;  10'd491: gtm = 10'd14;
                10'd492: gtm = 10'd14;  10'd493: gtm = 10'd14;  10'd494: gtm = 10'd14;  10'd495: gtm = 10'd13;  10'd496: gtm = 10'd13;  10'd497: gtm = 10'd13;
                10'd498: gtm = 10'd13;  10'd499: gtm = 10'd13;  10'd500: gtm = 10'd13;  10'd501: gtm = 10'd13;  10'd502: gtm = 10'd12;  10'd503: gtm = 10'd12;
                10'd504: gtm = 10'd12;  10'd505: gtm = 10'd12;  10'd506: gtm = 10'd12;  10'd507: gtm = 10'd12;  10'd508: gtm = 10'd12;  10'd509: gtm = 10'd12;
                10'd510: gtm = 10'd11;  10'd511: gtm = 10'd11;  10'd512: gtm = 10'd11;  10'd513: gtm = 10'd11;  10'd514: gtm = 10'd11;  10'd515: gtm = 10'd11;
                10'd516: gtm = 10'd11;  10'd517: gtm = 10'd11;  10'd518: gtm = 10'd11;  10'd519: gtm = 10'd10;  10'd520: gtm = 10'd10;  10'd521: gtm = 10'd10;
                10'd522: gtm = 10'd10;  10'd523: gtm = 10'd10;  10'd524: gtm = 10'd10;  10'd525: gtm = 10'd10;  10'd526: gtm = 10'd10;  10'd527: gtm = 10'd10;
                10'd528: gtm = 10'd9;  10'd529: gtm = 10'd9;  10'd530: gtm = 10'd9;  10'd531: gtm = 10'd9;  10'd532: gtm = 10'd9;  10'd533: gtm = 10'd9;
                10'd534: gtm = 10'd9;  10'd535: gtm = 10'd9;  10'd536: gtm = 10'd9;  10'd537: gtm = 10'd9;  10'd538: gtm = 10'd9;  10'd539: gtm = 10'd8;
                10'd540: gtm = 10'd8;  10'd541: gtm = 10'd8;  10'd542: gtm = 10'd8;  10'd543: gtm = 10'd8;  10'd544: gtm = 10'd8;  10'd545: gtm = 10'd8;
                10'd546: gtm = 10'd8;  10'd547: gtm = 10'd8;  10'd548: gtm = 10'd8;  10'd549: gtm = 10'd8;  10'd550: gtm = 10'd7;  10'd551: gtm = 10'd7;
                10'd552: gtm = 10'd7;  10'd553: gtm = 10'd7;  10'd554: gtm = 10'd7;  10'd555: gtm = 10'd7;  10'd556: gtm = 10'd7;  10'd557: gtm = 10'd7;
                10'd558: gtm = 10'd7;  10'd559: gtm = 10'd7;  10'd560: gtm = 10'd7;  10'd561: gtm = 10'd7;  10'd562: gtm = 10'd7;  10'd563: gtm = 10'd7;
                10'd564: gtm = 10'd6;  10'd565: gtm = 10'd6;  10'd566: gtm = 10'd6;  10'd567: gtm = 10'd6;  10'd568: gtm = 10'd6;  10'd569: gtm = 10'd6;
                10'd570: gtm = 10'd6;  10'd571: gtm = 10'd6;  10'd572: gtm = 10'd6;  10'd573: gtm = 10'd6;  10'd574: gtm = 10'd6;  10'd575: gtm = 10'd6;
                10'd576: gtm = 10'd6;  10'd577: gtm = 10'd6;  10'd578: gtm = 10'd6;  10'd579: gtm = 10'd6;  10'd580: gtm = 10'd5;  10'd581: gtm = 10'd5;
                10'd582: gtm = 10'd5;  10'd583: gtm = 10'd5;  10'd584: gtm = 10'd5;  10'd585: gtm = 10'd5;  10'd586: gtm = 10'd5;  10'd587: gtm = 10'd5;
                10'd588: gtm = 10'd5;  10'd589: gtm = 10'd5;  10'd590: gtm = 10'd5;  10'd591: gtm = 10'd5;  10'd592: gtm = 10'd5;  10'd593: gtm = 10'd5;
                10'd594: gtm = 10'd5;  10'd595: gtm = 10'd5;  10'd596: gtm = 10'd5;  10'd597: gtm = 10'd5;  10'd598: gtm = 10'd4;  10'd599: gtm = 10'd4;
                10'd600: gtm = 10'd4;  10'd601: gtm = 10'd4;  10'd602: gtm = 10'd4;  10'd603: gtm = 10'd4;  10'd604: gtm = 10'd4;  10'd605: gtm = 10'd4;
                10'd606: gtm = 10'd4;  10'd607: gtm = 10'd4;  10'd608: gtm = 10'd4;  10'd609: gtm = 10'd4;  10'd610: gtm = 10'd4;  10'd611: gtm = 10'd4;
                10'd612: gtm = 10'd4;  10'd613: gtm = 10'd4;  10'd614: gtm = 10'd4;  10'd615: gtm = 10'd4;  10'd616: gtm = 10'd4;  10'd617: gtm = 10'd4;
                10'd618: gtm = 10'd4;  10'd619: gtm = 10'd4;  10'd620: gtm = 10'd4;  10'd621: gtm = 10'd4;  10'd622: gtm = 10'd3;  10'd623: gtm = 10'd3;
                10'd624: gtm = 10'd3;  10'd625: gtm = 10'd3;  10'd626: gtm = 10'd3;  10'd627: gtm = 10'd3;  10'd628: gtm = 10'd3;  10'd629: gtm = 10'd3;
                10'd630: gtm = 10'd3;  10'd631: gtm = 10'd3;  10'd632: gtm = 10'd3;  10'd633: gtm = 10'd3;  10'd634: gtm = 10'd3;  10'd635: gtm = 10'd3;
                10'd636: gtm = 10'd3;  10'd637: gtm = 10'd3;  10'd638: gtm = 10'd3;  10'd639: gtm = 10'd3;  10'd640: gtm = 10'd3;  10'd641: gtm = 10'd3;
                10'd642: gtm = 10'd3;  10'd643: gtm = 10'd3;  10'd644: gtm = 10'd3;  10'd645: gtm = 10'd3;  10'd646: gtm = 10'd3;  10'd647: gtm = 10'd3;
                10'd648: gtm = 10'd3;  10'd649: gtm = 10'd3;  10'd650: gtm = 10'd3;  10'd651: gtm = 10'd3;  10'd652: gtm = 10'd3;  10'd653: gtm = 10'd2;
                10'd654: gtm = 10'd2;  10'd655: gtm = 10'd2;  10'd656: gtm = 10'd2;  10'd657: gtm = 10'd2;  10'd658: gtm = 10'd2;  10'd659: gtm = 10'd2;
                10'd660: gtm = 10'd2;  10'd661: gtm = 10'd2;  10'd662: gtm = 10'd2;  10'd663: gtm = 10'd2;  10'd664: gtm = 10'd2;  10'd665: gtm = 10'd2;
                10'd666: gtm = 10'd2;  10'd667: gtm = 10'd2;  10'd668: gtm = 10'd2;  10'd669: gtm = 10'd2;  10'd670: gtm = 10'd2;  10'd671: gtm = 10'd2;
                10'd672: gtm = 10'd2;  10'd673: gtm = 10'd2;  10'd674: gtm = 10'd2;  10'd675: gtm = 10'd2;  10'd676: gtm = 10'd2;  10'd677: gtm = 10'd2;
                10'd678: gtm = 10'd2;  10'd679: gtm = 10'd2;  10'd680: gtm = 10'd2;  10'd681: gtm = 10'd2;  10'd682: gtm = 10'd2;  10'd683: gtm = 10'd2;
                10'd684: gtm = 10'd2;  10'd685: gtm = 10'd2;  10'd686: gtm = 10'd2;  10'd687: gtm = 10'd2;  10'd688: gtm = 10'd2;  10'd689: gtm = 10'd2;
                10'd690: gtm = 10'd2;  10'd691: gtm = 10'd2;  10'd692: gtm = 10'd2;  10'd693: gtm = 10'd2;  10'd694: gtm = 10'd2;  10'd695: gtm = 10'd2;
                10'd696: gtm = 10'd2;  10'd697: gtm = 10'd2;  10'd698: gtm = 10'd2;  10'd699: gtm = 10'd2;  10'd700: gtm = 10'd2;  10'd701: gtm = 10'd1;
                10'd702: gtm = 10'd1;  10'd703: gtm = 10'd1;  10'd704: gtm = 10'd1;  10'd705: gtm = 10'd1;  10'd706: gtm = 10'd1;  10'd707: gtm = 10'd1;
                10'd708: gtm = 10'd1;  10'd709: gtm = 10'd1;  10'd710: gtm = 10'd1;  10'd711: gtm = 10'd1;  10'd712: gtm = 10'd1;  10'd713: gtm = 10'd1;
                10'd714: gtm = 10'd1;  10'd715: gtm = 10'd1;  10'd716: gtm = 10'd1;  10'd717: gtm = 10'd1;  10'd718: gtm = 10'd1;  10'd719: gtm = 10'd1;
                10'd720: gtm = 10'd1;  10'd721: gtm = 10'd1;  10'd722: gtm = 10'd1;  10'd723: gtm = 10'd1;  10'd724: gtm = 10'd1;  10'd725: gtm = 10'd1;
                10'd726: gtm = 10'd1;  10'd727: gtm = 10'd1;  10'd728: gtm = 10'd1;  10'd729: gtm = 10'd1;  10'd730: gtm = 10'd1;  10'd731: gtm = 10'd1;
                10'd732: gtm = 10'd1;  10'd733: gtm = 10'd1;  10'd734: gtm = 10'd1;  10'd735: gtm = 10'd1;  10'd736: gtm = 10'd1;  10'd737: gtm = 10'd1;
                10'd738: gtm = 10'd1;  10'd739: gtm = 10'd1;  10'd740: gtm = 10'd1;  10'd741: gtm = 10'd1;  10'd742: gtm = 10'd1;  10'd743: gtm = 10'd1;
                10'd744: gtm = 10'd1;  10'd745: gtm = 10'd1;  10'd746: gtm = 10'd1;  10'd747: gtm = 10'd1;  10'd748: gtm = 10'd1;  10'd749: gtm = 10'd1;
                10'd750: gtm = 10'd1;  10'd751: gtm = 10'd1;  10'd752: gtm = 10'd1;  10'd753: gtm = 10'd1;  10'd754: gtm = 10'd1;  10'd755: gtm = 10'd1;
                10'd756: gtm = 10'd1;  10'd757: gtm = 10'd1;  10'd758: gtm = 10'd1;  10'd759: gtm = 10'd1;  10'd760: gtm = 10'd1;  10'd761: gtm = 10'd1;
                10'd762: gtm = 10'd1;  10'd763: gtm = 10'd1;  10'd764: gtm = 10'd1;  10'd765: gtm = 10'd1;  10'd766: gtm = 10'd1;  10'd767: gtm = 10'd1;
                10'd768: gtm = 10'd1;  10'd769: gtm = 10'd1;  10'd770: gtm = 10'd1;  10'd771: gtm = 10'd1;  10'd772: gtm = 10'd1;  10'd773: gtm = 10'd1;
                10'd774: gtm = 10'd1;  10'd775: gtm = 10'd1;  10'd776: gtm = 10'd1;  10'd777: gtm = 10'd1;  10'd778: gtm = 10'd1;  10'd779: gtm = 10'd1;
                10'd780: gtm = 10'd1;  10'd781: gtm = 10'd1;  10'd782: gtm = 10'd1;  10'd783: gtm = 10'd1;  10'd784: gtm = 10'd1;  10'd785: gtm = 10'd1;
                10'd786: gtm = 10'd1;  10'd787: gtm = 10'd1;  10'd788: gtm = 10'd1;  10'd789: gtm = 10'd1;  10'd790: gtm = 10'd1;  10'd791: gtm = 10'd1;
                10'd792: gtm = 10'd1;  10'd793: gtm = 10'd1;  10'd794: gtm = 10'd1;  10'd795: gtm = 10'd1;  10'd796: gtm = 10'd1;  10'd797: gtm = 10'd1;
                10'd798: gtm = 10'd1;  10'd799: gtm = 10'd1;  10'd800: gtm = 10'd1;  10'd801: gtm = 10'd1;  10'd802: gtm = 10'd0;  10'd803: gtm = 10'd0;
                10'd804: gtm = 10'd0;  10'd805: gtm = 10'd0;  10'd806: gtm = 10'd0;  10'd807: gtm = 10'd0;  10'd808: gtm = 10'd0;  10'd809: gtm = 10'd0;
                10'd810: gtm = 10'd0;  10'd811: gtm = 10'd0;  10'd812: gtm = 10'd0;  10'd813: gtm = 10'd0;  10'd814: gtm = 10'd0;  10'd815: gtm = 10'd0;
                10'd816: gtm = 10'd0;  10'd817: gtm = 10'd0;  10'd818: gtm = 10'd0;  10'd819: gtm = 10'd0;  10'd820: gtm = 10'd0;  10'd821: gtm = 10'd0;
                10'd822: gtm = 10'd0;  10'd823: gtm = 10'd0;  10'd824: gtm = 10'd0;  10'd825: gtm = 10'd0;  10'd826: gtm = 10'd0;  10'd827: gtm = 10'd0;
                10'd828: gtm = 10'd0;  10'd829: gtm = 10'd0;  10'd830: gtm = 10'd0;  10'd831: gtm = 10'd0;  10'd832: gtm = 10'd0;  10'd833: gtm = 10'd0;
                10'd834: gtm = 10'd0;  10'd835: gtm = 10'd0;  10'd836: gtm = 10'd0;  10'd837: gtm = 10'd0;  10'd838: gtm = 10'd0;  10'd839: gtm = 10'd0;
                10'd840: gtm = 10'd0;  10'd841: gtm = 10'd0;  10'd842: gtm = 10'd0;  10'd843: gtm = 10'd0;  10'd844: gtm = 10'd0;  10'd845: gtm = 10'd0;
                10'd846: gtm = 10'd0;  10'd847: gtm = 10'd0;  10'd848: gtm = 10'd0;  10'd849: gtm = 10'd0;  10'd850: gtm = 10'd0;  10'd851: gtm = 10'd0;
                10'd852: gtm = 10'd0;  10'd853: gtm = 10'd0;  10'd854: gtm = 10'd0;  10'd855: gtm = 10'd0;  10'd856: gtm = 10'd0;  10'd857: gtm = 10'd0;
                10'd858: gtm = 10'd0;  10'd859: gtm = 10'd0;  10'd860: gtm = 10'd0;  10'd861: gtm = 10'd0;  10'd862: gtm = 10'd0;  10'd863: gtm = 10'd0;
                10'd864: gtm = 10'd0;  10'd865: gtm = 10'd0;  10'd866: gtm = 10'd0;  10'd867: gtm = 10'd0;  10'd868: gtm = 10'd0;  10'd869: gtm = 10'd0;
                10'd870: gtm = 10'd0;  10'd871: gtm = 10'd0;  10'd872: gtm = 10'd0;  10'd873: gtm = 10'd0;  10'd874: gtm = 10'd0;  10'd875: gtm = 10'd0;
                10'd876: gtm = 10'd0;  10'd877: gtm = 10'd0;  10'd878: gtm = 10'd0;  10'd879: gtm = 10'd0;  10'd880: gtm = 10'd0;  10'd881: gtm = 10'd0;
                10'd882: gtm = 10'd0;  10'd883: gtm = 10'd0;  10'd884: gtm = 10'd0;  10'd885: gtm = 10'd0;  10'd886: gtm = 10'd0;  10'd887: gtm = 10'd0;
                10'd888: gtm = 10'd0;  10'd889: gtm = 10'd0;  10'd890: gtm = 10'd0;  10'd891: gtm = 10'd0;  10'd892: gtm = 10'd0;  10'd893: gtm = 10'd0;
                10'd894: gtm = 10'd0;  10'd895: gtm = 10'd0;  10'd896: gtm = 10'd0;  10'd897: gtm = 10'd0;  10'd898: gtm = 10'd0;  10'd899: gtm = 10'd0;
                10'd900: gtm = 10'd0;  10'd901: gtm = 10'd0;  10'd902: gtm = 10'd0;  10'd903: gtm = 10'd0;  10'd904: gtm = 10'd0;  10'd905: gtm = 10'd0;
                10'd906: gtm = 10'd0;  10'd907: gtm = 10'd0;  10'd908: gtm = 10'd0;  10'd909: gtm = 10'd0;  10'd910: gtm = 10'd0;  10'd911: gtm = 10'd0;
                10'd912: gtm = 10'd0;  10'd913: gtm = 10'd0;  10'd914: gtm = 10'd0;  10'd915: gtm = 10'd0;  10'd916: gtm = 10'd0;  10'd917: gtm = 10'd0;
                10'd918: gtm = 10'd0;  10'd919: gtm = 10'd0;  10'd920: gtm = 10'd0;  10'd921: gtm = 10'd0;  10'd922: gtm = 10'd0;  10'd923: gtm = 10'd0;
                10'd924: gtm = 10'd0;  10'd925: gtm = 10'd0;  10'd926: gtm = 10'd0;  10'd927: gtm = 10'd0;  10'd928: gtm = 10'd0;  10'd929: gtm = 10'd0;
                10'd930: gtm = 10'd0;  10'd931: gtm = 10'd0;  10'd932: gtm = 10'd0;  10'd933: gtm = 10'd0;  10'd934: gtm = 10'd0;  10'd935: gtm = 10'd0;
                10'd936: gtm = 10'd0;  10'd937: gtm = 10'd0;  10'd938: gtm = 10'd0;  10'd939: gtm = 10'd0;  10'd940: gtm = 10'd0;  10'd941: gtm = 10'd0;
                10'd942: gtm = 10'd0;  10'd943: gtm = 10'd0;  10'd944: gtm = 10'd0;  10'd945: gtm = 10'd0;  10'd946: gtm = 10'd0;  10'd947: gtm = 10'd0;
                10'd948: gtm = 10'd0;  10'd949: gtm = 10'd0;  10'd950: gtm = 10'd0;  10'd951: gtm = 10'd0;  10'd952: gtm = 10'd0;  10'd953: gtm = 10'd0;
                10'd954: gtm = 10'd0;  10'd955: gtm = 10'd0;  10'd956: gtm = 10'd0;  10'd957: gtm = 10'd0;  10'd958: gtm = 10'd0;  10'd959: gtm = 10'd0;
                10'd960: gtm = 10'd0;  10'd961: gtm = 10'd0;  10'd962: gtm = 10'd0;  10'd963: gtm = 10'd0;  10'd964: gtm = 10'd0;  10'd965: gtm = 10'd0;
                10'd966: gtm = 10'd0;  10'd967: gtm = 10'd0;  10'd968: gtm = 10'd0;  10'd969: gtm = 10'd0;  10'd970: gtm = 10'd0;  10'd971: gtm = 10'd0;
                10'd972: gtm = 10'd0;  10'd973: gtm = 10'd0;  10'd974: gtm = 10'd0;  10'd975: gtm = 10'd0;  10'd976: gtm = 10'd0;  10'd977: gtm = 10'd0;
                10'd978: gtm = 10'd0;  10'd979: gtm = 10'd0;  10'd980: gtm = 10'd0;  10'd981: gtm = 10'd0;  10'd982: gtm = 10'd0;  10'd983: gtm = 10'd0;
                10'd984: gtm = 10'd0;  10'd985: gtm = 10'd0;  10'd986: gtm = 10'd0;  10'd987: gtm = 10'd0;  10'd988: gtm = 10'd0;  10'd989: gtm = 10'd0;
                10'd990: gtm = 10'd0;  10'd991: gtm = 10'd0;  10'd992: gtm = 10'd0;  10'd993: gtm = 10'd0;  10'd994: gtm = 10'd0;  10'd995: gtm = 10'd0;
                10'd996: gtm = 10'd0;  10'd997: gtm = 10'd0;  10'd998: gtm = 10'd0;  10'd999: gtm = 10'd0;  10'd1000: gtm = 10'd0;  10'd1001: gtm = 10'd0;
                10'd1002: gtm = 10'd0;  10'd1003: gtm = 10'd0;  10'd1004: gtm = 10'd0;  10'd1005: gtm = 10'd0;  10'd1006: gtm = 10'd0;  10'd1007: gtm = 10'd0;
                10'd1008: gtm = 10'd0;  10'd1009: gtm = 10'd0;  10'd1010: gtm = 10'd0;  10'd1011: gtm = 10'd0;  10'd1012: gtm = 10'd0;  10'd1013: gtm = 10'd0;
                10'd1014: gtm = 10'd0;  10'd1015: gtm = 10'd0;  10'd1016: gtm = 10'd0;  10'd1017: gtm = 10'd0;  10'd1018: gtm = 10'd0;  10'd1019: gtm = 10'd0;
                10'd1020: gtm = 10'd0;  10'd1021: gtm = 10'd0;  10'd1022: gtm = 10'd0;  10'd1023: gtm = 10'd0;
                default: gtm = 10'd0;
            endcase
        end
    endfunction
    function [16:0] exl;                  // 65536 x 2^(-i/128): the gain as a number
        input [8:0] a;
        begin
            case (a)
                9'd0: exl = 17'd65536;  9'd1: exl = 17'd65182;  9'd2: exl = 17'd64830;  9'd3: exl = 17'd64480;  9'd4: exl = 17'd64132;  9'd5: exl = 17'd63785;
                9'd6: exl = 17'd63441;  9'd7: exl = 17'd63098;  9'd8: exl = 17'd62757;  9'd9: exl = 17'd62419;  9'd10: exl = 17'd62081;  9'd11: exl = 17'd61746;
                9'd12: exl = 17'd61413;  9'd13: exl = 17'd61081;  9'd14: exl = 17'd60751;  9'd15: exl = 17'd60423;  9'd16: exl = 17'd60097;  9'd17: exl = 17'd59772;
                9'd18: exl = 17'd59449;  9'd19: exl = 17'd59128;  9'd20: exl = 17'd58809;  9'd21: exl = 17'd58491;  9'd22: exl = 17'd58176;  9'd23: exl = 17'd57861;
                9'd24: exl = 17'd57549;  9'd25: exl = 17'd57238;  9'd26: exl = 17'd56929;  9'd27: exl = 17'd56622;  9'd28: exl = 17'd56316;  9'd29: exl = 17'd56012;
                9'd30: exl = 17'd55709;  9'd31: exl = 17'd55408;  9'd32: exl = 17'd55109;  9'd33: exl = 17'd54811;  9'd34: exl = 17'd54515;  9'd35: exl = 17'd54221;
                9'd36: exl = 17'd53928;  9'd37: exl = 17'd53637;  9'd38: exl = 17'd53347;  9'd39: exl = 17'd53059;  9'd40: exl = 17'd52773;  9'd41: exl = 17'd52488;
                9'd42: exl = 17'd52204;  9'd43: exl = 17'd51922;  9'd44: exl = 17'd51642;  9'd45: exl = 17'd51363;  9'd46: exl = 17'd51085;  9'd47: exl = 17'd50810;
                9'd48: exl = 17'd50535;  9'd49: exl = 17'd50262;  9'd50: exl = 17'd49991;  9'd51: exl = 17'd49721;  9'd52: exl = 17'd49452;  9'd53: exl = 17'd49185;
                9'd54: exl = 17'd48920;  9'd55: exl = 17'd48655;  9'd56: exl = 17'd48393;  9'd57: exl = 17'd48131;  9'd58: exl = 17'd47871;  9'd59: exl = 17'd47613;
                9'd60: exl = 17'd47356;  9'd61: exl = 17'd47100;  9'd62: exl = 17'd46846;  9'd63: exl = 17'd46593;  9'd64: exl = 17'd46341;  9'd65: exl = 17'd46091;
                9'd66: exl = 17'd45842;  9'd67: exl = 17'd45594;  9'd68: exl = 17'd45348;  9'd69: exl = 17'd45103;  9'd70: exl = 17'd44859;  9'd71: exl = 17'd44617;
                9'd72: exl = 17'd44376;  9'd73: exl = 17'd44137;  9'd74: exl = 17'd43898;  9'd75: exl = 17'd43661;  9'd76: exl = 17'd43425;  9'd77: exl = 17'd43191;
                9'd78: exl = 17'd42958;  9'd79: exl = 17'd42726;  9'd80: exl = 17'd42495;  9'd81: exl = 17'd42265;  9'd82: exl = 17'd42037;  9'd83: exl = 17'd41810;
                9'd84: exl = 17'd41584;  9'd85: exl = 17'd41360;  9'd86: exl = 17'd41136;  9'd87: exl = 17'd40914;  9'd88: exl = 17'd40693;  9'd89: exl = 17'd40473;
                9'd90: exl = 17'd40255;  9'd91: exl = 17'd40037;  9'd92: exl = 17'd39821;  9'd93: exl = 17'd39606;  9'd94: exl = 17'd39392;  9'd95: exl = 17'd39180;
                9'd96: exl = 17'd38968;  9'd97: exl = 17'd38757;  9'd98: exl = 17'd38548;  9'd99: exl = 17'd38340;  9'd100: exl = 17'd38133;  9'd101: exl = 17'd37927;
                9'd102: exl = 17'd37722;  9'd103: exl = 17'd37518;  9'd104: exl = 17'd37316;  9'd105: exl = 17'd37114;  9'd106: exl = 17'd36914;  9'd107: exl = 17'd36715;
                9'd108: exl = 17'd36516;  9'd109: exl = 17'd36319;  9'd110: exl = 17'd36123;  9'd111: exl = 17'd35928;  9'd112: exl = 17'd35734;  9'd113: exl = 17'd35541;
                9'd114: exl = 17'd35349;  9'd115: exl = 17'd35158;  9'd116: exl = 17'd34968;  9'd117: exl = 17'd34779;  9'd118: exl = 17'd34591;  9'd119: exl = 17'd34405;
                9'd120: exl = 17'd34219;  9'd121: exl = 17'd34034;  9'd122: exl = 17'd33850;  9'd123: exl = 17'd33667;  9'd124: exl = 17'd33486;  9'd125: exl = 17'd33305;
                9'd126: exl = 17'd33125;  9'd127: exl = 17'd32946;  9'd128: exl = 17'd32768;  9'd129: exl = 17'd32591;  9'd130: exl = 17'd32415;  9'd131: exl = 17'd32240;
                9'd132: exl = 17'd32066;  9'd133: exl = 17'd31893;  9'd134: exl = 17'd31720;  9'd135: exl = 17'd31549;  9'd136: exl = 17'd31379;  9'd137: exl = 17'd31209;
                9'd138: exl = 17'd31041;  9'd139: exl = 17'd30873;  9'd140: exl = 17'd30706;  9'd141: exl = 17'd30541;  9'd142: exl = 17'd30376;  9'd143: exl = 17'd30212;
                9'd144: exl = 17'd30048;  9'd145: exl = 17'd29886;  9'd146: exl = 17'd29725;  9'd147: exl = 17'd29564;  9'd148: exl = 17'd29405;  9'd149: exl = 17'd29246;
                9'd150: exl = 17'd29088;  9'd151: exl = 17'd28931;  9'd152: exl = 17'd28774;  9'd153: exl = 17'd28619;  9'd154: exl = 17'd28464;  9'd155: exl = 17'd28311;
                9'd156: exl = 17'd28158;  9'd157: exl = 17'd28006;  9'd158: exl = 17'd27855;  9'd159: exl = 17'd27704;  9'd160: exl = 17'd27554;  9'd161: exl = 17'd27406;
                9'd162: exl = 17'd27258;  9'd163: exl = 17'd27110;  9'd164: exl = 17'd26964;  9'd165: exl = 17'd26818;  9'd166: exl = 17'd26674;  9'd167: exl = 17'd26530;
                9'd168: exl = 17'd26386;  9'd169: exl = 17'd26244;  9'd170: exl = 17'd26102;  9'd171: exl = 17'd25961;  9'd172: exl = 17'd25821;  9'd173: exl = 17'd25681;
                9'd174: exl = 17'd25543;  9'd175: exl = 17'd25405;  9'd176: exl = 17'd25268;  9'd177: exl = 17'd25131;  9'd178: exl = 17'd24995;  9'd179: exl = 17'd24860;
                9'd180: exl = 17'd24726;  9'd181: exl = 17'd24593;  9'd182: exl = 17'd24460;  9'd183: exl = 17'd24328;  9'd184: exl = 17'd24196;  9'd185: exl = 17'd24066;
                9'd186: exl = 17'd23936;  9'd187: exl = 17'd23806;  9'd188: exl = 17'd23678;  9'd189: exl = 17'd23550;  9'd190: exl = 17'd23423;  9'd191: exl = 17'd23296;
                9'd192: exl = 17'd23170;  9'd193: exl = 17'd23045;  9'd194: exl = 17'd22921;  9'd195: exl = 17'd22797;  9'd196: exl = 17'd22674;  9'd197: exl = 17'd22552;
                9'd198: exl = 17'd22430;  9'd199: exl = 17'd22309;  9'd200: exl = 17'd22188;  9'd201: exl = 17'd22068;  9'd202: exl = 17'd21949;  9'd203: exl = 17'd21831;
                9'd204: exl = 17'd21713;  9'd205: exl = 17'd21595;  9'd206: exl = 17'd21479;  9'd207: exl = 17'd21363;  9'd208: exl = 17'd21247;  9'd209: exl = 17'd21133;
                9'd210: exl = 17'd21019;  9'd211: exl = 17'd20905;  9'd212: exl = 17'd20792;  9'd213: exl = 17'd20680;  9'd214: exl = 17'd20568;  9'd215: exl = 17'd20457;
                9'd216: exl = 17'd20347;  9'd217: exl = 17'd20237;  9'd218: exl = 17'd20127;  9'd219: exl = 17'd20019;  9'd220: exl = 17'd19911;  9'd221: exl = 17'd19803;
                9'd222: exl = 17'd19696;  9'd223: exl = 17'd19590;  9'd224: exl = 17'd19484;  9'd225: exl = 17'd19379;  9'd226: exl = 17'd19274;  9'd227: exl = 17'd19170;
                9'd228: exl = 17'd19066;  9'd229: exl = 17'd18963;  9'd230: exl = 17'd18861;  9'd231: exl = 17'd18759;  9'd232: exl = 17'd18658;  9'd233: exl = 17'd18557;
                9'd234: exl = 17'd18457;  9'd235: exl = 17'd18357;  9'd236: exl = 17'd18258;  9'd237: exl = 17'd18160;  9'd238: exl = 17'd18061;  9'd239: exl = 17'd17964;
                9'd240: exl = 17'd17867;  9'd241: exl = 17'd17770;  9'd242: exl = 17'd17674;  9'd243: exl = 17'd17579;  9'd244: exl = 17'd17484;  9'd245: exl = 17'd17390;
                9'd246: exl = 17'd17296;  9'd247: exl = 17'd17202;  9'd248: exl = 17'd17109;  9'd249: exl = 17'd17017;  9'd250: exl = 17'd16925;  9'd251: exl = 17'd16834;
                9'd252: exl = 17'd16743;  9'd253: exl = 17'd16652;  9'd254: exl = 17'd16562;  9'd255: exl = 17'd16473;  9'd256: exl = 17'd16384;  9'd257: exl = 17'd16296;
                9'd258: exl = 17'd16208;  9'd259: exl = 17'd16120;  9'd260: exl = 17'd16033;  9'd261: exl = 17'd15946;  9'd262: exl = 17'd15860;  9'd263: exl = 17'd15775;
                9'd264: exl = 17'd15689;  9'd265: exl = 17'd15605;  9'd266: exl = 17'd15520;  9'd267: exl = 17'd15437;  9'd268: exl = 17'd15353;  9'd269: exl = 17'd15270;
                9'd270: exl = 17'd15188;  9'd271: exl = 17'd15106;  9'd272: exl = 17'd15024;  9'd273: exl = 17'd14943;  9'd274: exl = 17'd14862;  9'd275: exl = 17'd14782;
                9'd276: exl = 17'd14702;  9'd277: exl = 17'd14623;  9'd278: exl = 17'd14544;  9'd279: exl = 17'd14465;  9'd280: exl = 17'd14387;  9'd281: exl = 17'd14310;
                9'd282: exl = 17'd14232;  9'd283: exl = 17'd14155;  9'd284: exl = 17'd14079;  9'd285: exl = 17'd14003;  9'd286: exl = 17'd13927;  9'd287: exl = 17'd13852;
                9'd288: exl = 17'd13777;  9'd289: exl = 17'd13703;  9'd290: exl = 17'd13629;  9'd291: exl = 17'd13555;  9'd292: exl = 17'd13482;  9'd293: exl = 17'd13409;
                9'd294: exl = 17'd13337;  9'd295: exl = 17'd13265;  9'd296: exl = 17'd13193;  9'd297: exl = 17'd13122;  9'd298: exl = 17'd13051;  9'd299: exl = 17'd12981;
                9'd300: exl = 17'd12910;  9'd301: exl = 17'd12841;  9'd302: exl = 17'd12771;  9'd303: exl = 17'd12702;  9'd304: exl = 17'd12634;  9'd305: exl = 17'd12566;
                9'd306: exl = 17'd12498;  9'd307: exl = 17'd12430;  9'd308: exl = 17'd12363;  9'd309: exl = 17'd12296;  9'd310: exl = 17'd12230;  9'd311: exl = 17'd12164;
                9'd312: exl = 17'd12098;  9'd313: exl = 17'd12033;  9'd314: exl = 17'd11968;  9'd315: exl = 17'd11903;  9'd316: exl = 17'd11839;  9'd317: exl = 17'd11775;
                9'd318: exl = 17'd11711;  9'd319: exl = 17'd11648;  9'd320: exl = 17'd11585;  9'd321: exl = 17'd11523;  9'd322: exl = 17'd11460;  9'd323: exl = 17'd11399;
                9'd324: exl = 17'd11337;  9'd325: exl = 17'd11276;  9'd326: exl = 17'd11215;  9'd327: exl = 17'd11154;  9'd328: exl = 17'd11094;  9'd329: exl = 17'd11034;
                9'd330: exl = 17'd10975;  9'd331: exl = 17'd10915;  9'd332: exl = 17'd10856;  9'd333: exl = 17'd10798;  9'd334: exl = 17'd10739;  9'd335: exl = 17'd10681;
                9'd336: exl = 17'd10624;  9'd337: exl = 17'd10566;  9'd338: exl = 17'd10509;  9'd339: exl = 17'd10453;  9'd340: exl = 17'd10396;  9'd341: exl = 17'd10340;
                9'd342: exl = 17'd10284;  9'd343: exl = 17'd10229;  9'd344: exl = 17'd10173;  9'd345: exl = 17'd10118;  9'd346: exl = 17'd10064;  9'd347: exl = 17'd10009;
                9'd348: exl = 17'd9955;  9'd349: exl = 17'd9902;  9'd350: exl = 17'd9848;  9'd351: exl = 17'd9795;  9'd352: exl = 17'd9742;  9'd353: exl = 17'd9689;
                9'd354: exl = 17'd9637;  9'd355: exl = 17'd9585;  9'd356: exl = 17'd9533;  9'd357: exl = 17'd9482;  9'd358: exl = 17'd9431;  9'd359: exl = 17'd9380;
                9'd360: exl = 17'd9329;  9'd361: exl = 17'd9279;  9'd362: exl = 17'd9228;  9'd363: exl = 17'd9179;  9'd364: exl = 17'd9129;  9'd365: exl = 17'd9080;
                9'd366: exl = 17'd9031;  9'd367: exl = 17'd8982;  9'd368: exl = 17'd8933;  9'd369: exl = 17'd8885;  9'd370: exl = 17'd8837;  9'd371: exl = 17'd8789;
                9'd372: exl = 17'd8742;  9'd373: exl = 17'd8695;  9'd374: exl = 17'd8648;  9'd375: exl = 17'd8601;  9'd376: exl = 17'd8555;  9'd377: exl = 17'd8508;
                9'd378: exl = 17'd8463;  9'd379: exl = 17'd8417;  9'd380: exl = 17'd8371;  9'd381: exl = 17'd8326;  9'd382: exl = 17'd8281;  9'd383: exl = 17'd8236;
                9'd384: exl = 17'd8192;  9'd385: exl = 17'd8148;  9'd386: exl = 17'd8104;  9'd387: exl = 17'd8060;  9'd388: exl = 17'd8016;  9'd389: exl = 17'd7973;
                9'd390: exl = 17'd7930;  9'd391: exl = 17'd7887;  9'd392: exl = 17'd7845;  9'd393: exl = 17'd7802;  9'd394: exl = 17'd7760;  9'd395: exl = 17'd7718;
                9'd396: exl = 17'd7677;  9'd397: exl = 17'd7635;  9'd398: exl = 17'd7594;  9'd399: exl = 17'd7553;  9'd400: exl = 17'd7512;  9'd401: exl = 17'd7472;
                9'd402: exl = 17'd7431;  9'd403: exl = 17'd7391;  9'd404: exl = 17'd7351;  9'd405: exl = 17'd7311;  9'd406: exl = 17'd7272;  9'd407: exl = 17'd7233;
                9'd408: exl = 17'd7194;  9'd409: exl = 17'd7155;  9'd410: exl = 17'd7116;  9'd411: exl = 17'd7078;  9'd412: exl = 17'd7039;  9'd413: exl = 17'd7001;
                9'd414: exl = 17'd6964;  9'd415: exl = 17'd6926;  9'd416: exl = 17'd6889;  9'd417: exl = 17'd6851;  9'd418: exl = 17'd6814;  9'd419: exl = 17'd6778;
                9'd420: exl = 17'd6741;  9'd421: exl = 17'd6705;  9'd422: exl = 17'd6668;  9'd423: exl = 17'd6632;  9'd424: exl = 17'd6597;  9'd425: exl = 17'd6561;
                9'd426: exl = 17'd6526;  9'd427: exl = 17'd6490;  9'd428: exl = 17'd6455;  9'd429: exl = 17'd6420;  9'd430: exl = 17'd6386;  9'd431: exl = 17'd6351;
                9'd432: exl = 17'd6317;  9'd433: exl = 17'd6283;  9'd434: exl = 17'd6249;  9'd435: exl = 17'd6215;  9'd436: exl = 17'd6182;  9'd437: exl = 17'd6148;
                9'd438: exl = 17'd6115;  9'd439: exl = 17'd6082;  9'd440: exl = 17'd6049;  9'd441: exl = 17'd6016;  9'd442: exl = 17'd5984;  9'd443: exl = 17'd5952;
                9'd444: exl = 17'd5919;  9'd445: exl = 17'd5887;  9'd446: exl = 17'd5856;  9'd447: exl = 17'd5824;  9'd448: exl = 17'd5793;  9'd449: exl = 17'd5761;
                9'd450: exl = 17'd5730;  9'd451: exl = 17'd5699;  9'd452: exl = 17'd5668;  9'd453: exl = 17'd5638;  9'd454: exl = 17'd5607;  9'd455: exl = 17'd5577;
                9'd456: exl = 17'd5547;  9'd457: exl = 17'd5517;  9'd458: exl = 17'd5487;  9'd459: exl = 17'd5458;  9'd460: exl = 17'd5428;  9'd461: exl = 17'd5399;
                9'd462: exl = 17'd5370;  9'd463: exl = 17'd5341;  9'd464: exl = 17'd5312;  9'd465: exl = 17'd5283;  9'd466: exl = 17'd5255;  9'd467: exl = 17'd5226;
                9'd468: exl = 17'd5198;  9'd469: exl = 17'd5170;  9'd470: exl = 17'd5142;  9'd471: exl = 17'd5114;  9'd472: exl = 17'd5087;  9'd473: exl = 17'd5059;
                9'd474: exl = 17'd5032;  9'd475: exl = 17'd5005;  9'd476: exl = 17'd4978;  9'd477: exl = 17'd4951;  9'd478: exl = 17'd4924;  9'd479: exl = 17'd4897;
                9'd480: exl = 17'd4871;  9'd481: exl = 17'd4845;  9'd482: exl = 17'd4819;  9'd483: exl = 17'd4792;  9'd484: exl = 17'd4767;  9'd485: exl = 17'd4741;
                9'd486: exl = 17'd4715;  9'd487: exl = 17'd4690;  9'd488: exl = 17'd4664;  9'd489: exl = 17'd4639;  9'd490: exl = 17'd4614;  9'd491: exl = 17'd4589;
                9'd492: exl = 17'd4565;  9'd493: exl = 17'd4540;  9'd494: exl = 17'd4515;  9'd495: exl = 17'd4491;  9'd496: exl = 17'd4467;  9'd497: exl = 17'd4443;
                9'd498: exl = 17'd4419;  9'd499: exl = 17'd4395;  9'd500: exl = 17'd4371;  9'd501: exl = 17'd4347;  9'd502: exl = 17'd4324;  9'd503: exl = 17'd4301;
                9'd504: exl = 17'd4277;  9'd505: exl = 17'd4254;  9'd506: exl = 17'd4231;  9'd507: exl = 17'd4208;  9'd508: exl = 17'd4186;  9'd509: exl = 17'd4163;
                9'd510: exl = 17'd4141;  9'd511: exl = 17'd4118;
                default: exl = 17'd0;
            endcase
        end
    endfunction
    function [9:0] brev;                  // bit order reversed
        input [9:0] a;
        begin
            brev = {a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9]};
        end
    endfunction

    localparam signed [13:0] C_OS       = 14'sd61;          // quantile to mean, and 3 dB under the mean
    localparam signed [14:0] PL_ALPHA   = -15'sd2;     // 64 x log2(0.98)
    localparam signed [14:0] PL_1MALPHA = -15'sd361;   // 64 x log2(0.02)
    localparam [6:0]         ACQ        = 7'd64;             // pieces of fast learning

    // ---- the sample side -------------------------------------------------------------------
    reg  [9:0]  t        = 10'd0;         // sample counter; a new piece every 256
    reg         play     = 1'b0;          // the half of obuf being played
    reg  [9:0]  obase    = 10'd0;         // where the next piece starts in the overlap-add RAM
    reg  [5:0]  lockhist = 6'd0;          // lock flag at the last six piece starts
    reg         use_q    = 1'b0;          // the noise level comes from the quadrature channel
    reg  [19:0] n0_f     = 20'd0;         // noise level, 8 bits of fraction
    reg  [6:0]  acq      = ACQ;
    reg  [8:0]  obr      = 9'd0;

    wire        hop    = (t[7:0] == 8'd0);
    wire        qhop   = (t[7:0] == 8'd128);
    wire        play_n = hop ? ~play : play;
    wire [5:0]  lh_n   = {lockhist[4:0], locked_i};
    wire        uq_n   = (lh_n == 6'h3F);
    wire [6:0]  acq_n  = big_i ? ACQ : acq;
    wire signed [27:0] rq4 = rq_i[31:4];
    wire signed [21:0] rqs = (rq4 > 28'sd2097151) ? 22'sd2097151 : (rq4 < -28'sd2097152) ? -22'sd2097152 : rq4[21:0];

    always @(posedge clk) begin
        if (tick) ring[t] <= {rqs, x_i};
    end
    always @(posedge clk) y_o <= obuf[obr];

    // ---- the piece side: one small step a clock --------------------------------------------
    localparam [3:0] F_IDLE = 4'd0, F_LOG = 4'd1, F_LOAD = 4'd2, F_FFT = 4'd3, F_BIN = 4'd4, F_ZERO = 4'd5,
                     F_N0 = 4'd6, F_IFFT = 4'd7, F_OLA = 4'd8;
    reg  [3:0]  fs  = F_IDLE;
    reg  [3:0]  p   = 4'd0;               // step inside one operation
    reg  [9:0]  cn  = 10'd0;              // sample, butterfly or cell number
    reg  [3:0]  sg  = 4'd0;               // FFT stage
    reg         qfr = 1'b0;               // this piece is the quadrature one
    reg  [9:0]  fbase = 10'd0;            // the newest sample of the piece
    reg  [31:0] lden_r = 32'd1;
    reg  [7:0]  khi = 8'd68, obs_hi = 8'd56;         // the audio band, taken over when a piece starts
    reg  [8:0]  flm  = 9'd255;            // floor, as 64 x log2(1 / gain^2)
    reg  [11:0] n0_ref = 12'd0;
    reg         acq_f = 1'b0;
    reg  signed [13:0] othr = 14'sd0;     // a cell below this counts as "below the noise level"
    reg  signed [13:0] llam = 14'sd0;     // the noise level in this piece's own units
    reg  [6:0]  nobs = 7'd0, below = 7'd0;

    localparam [7:0] OBS_LO = 8'd5;

    // the work RAM: two ports
    reg  [9:0]  aa = 10'd0, ab = 10'd0;
    reg  [63:0] da = 64'd0, db = 64'd0, qa = 64'd0, qb = 64'd0;
    reg         wea = 1'b0, web = 1'b0;
    always @(posedge clk) begin
        if (wea) fr[aa] <= da;
        qa <= fr[aa];
    end
    always @(posedge clk) begin
        if (web) fr[ab] <= db;
        qb <= fr[ab];
    end
    // the other memories
    reg  [9:0]  rga = 10'd0;
    reg  [43:0] rgq = 44'd0;
    reg  [8:0]  twa = 9'd0;
    reg  [35:0] twq = 36'd0;
    reg  [9:0]  oa = 10'd0;
    reg  [25:0] od = 26'd0, olaq = 26'd0;
    reg         owe = 1'b0;
    reg  [8:0]  obwa = 9'd0;
    reg  [21:0] obd = 22'd0;
    reg         obwe = 1'b0;
    reg  [7:0]  bsa = 8'd0;
    reg  [19:0] bsd = 20'd0;
    reg         bswe = 1'b0;
    always @(posedge clk) begin
        rgq  <= ring[rga];
        twq  <= tw[twa];
        olaq <= ola[oa];
        if (owe)  ola[oa]    <= od;
        if (obwe) obuf[obwa] <= obd;
        if (bswe) bst[bsa]   <= bsd;
    end
    wire [19:0] bsq = bst[bsa];
    wire signed [17:0] wre = twq[17:0];
    wire signed [17:0] wim = twq[35:18];

    // four multipliers shared by everything
    reg  signed [32:0] ma = 33'sd0, mb = 33'sd0;
    reg  signed [17:0] wr = 18'sd0, wi = 18'sd0;
    reg  signed [50:0] p1 = 51'sd0, p2 = 51'sd0, p3 = 51'sd0, p4 = 51'sd0;
    always @(posedge clk) begin
        p1 <= ma * wr;
        p2 <= mb * wi;
        p3 <= ma * wi;
        p4 <= mb * wr;
    end
    wire signed [50:0] r11 = (p1 + 51'sd32768) >>> 16;            // ma x wr, rounded
    wire signed [50:0] r44 = (p4 + 51'sd32768) >>> 16;            // mb x wr
    wire signed [50:0] rfr = (p1 - p2 + 51'sd32768) >>> 16;       // forward: (ma + j mb)(wr + j wi)
    wire signed [50:0] rfi = (p3 + p4 + 51'sd32768) >>> 16;
    wire signed [50:0] rir = (p1 + p2 + 51'sd32768) >>> 16;       // inverse: times the conjugate
    wire signed [50:0] rii = (p4 - p3 + 51'sd32768) >>> 16;

    // 64 x log2 of a number, in two clocks
    reg  [32:0] lg_v = 33'd0;
    reg  [5:0]  lg_e = 6'd0, lg_m = 6'd0;
    reg         lg_z = 1'b1;
    reg  [11:0] lg_l = 12'd0;
    reg  [5:0]  e_c;
    integer     li;
    always @* begin
        e_c = 6'd0;
        for (li = 0; li <= 32; li = li + 1) if (lg_v[li]) e_c = li[5:0];
    end
    wire [32:0] lg_n = lg_v << (6'd32 - e_c);                     // the leading one moved to bit 32
    always @(posedge clk) begin
        lg_e <= e_c;
        lg_m <= lg_n[31:26];
        lg_z <= (lg_v == 33'd0);
        lg_l <= lg_z ? 12'd0 : ({lg_e, 6'd0} + {6'd0, mlut(lg_m)});
    end

    // FFT addresses: butterfly number cn with a zero put in at bit position zp gives the upper
    // element; the lower one has a one there
    wire [3:0]  zp   = (fs == F_FFT) ? (4'd9 - sg) : sg;
    wire [9:0]  lom  = (10'd1 << zp) - 10'd1;
    wire [9:0]  blo  = cn & lom;
    wire [9:0]  bhi  = cn >> zp;
    wire [9:0]  ia   = (bhi << (zp + 4'd1)) | blo;
    wire [9:0]  ib   = ia | (10'd1 << zp);
    wire [9:0]  twx  = (fs == F_FFT) ? (blo << sg) : (blo << (4'd9 - sg));

    // pieces of the data path
    wire signed [31:0] ar = qa[31:0],  ai = qa[63:32];
    wire signed [31:0] br = qb[31:0],  bi = qb[63:32];
    wire signed [32:0] sr = ar + br,   si = ai + bi;
    reg  signed [31:0] ar_r = 32'sd0, ai_r = 32'sd0;
    wire signed [33:0] ar34 = {{2{ar_r[31]}}, ar_r};
    wire signed [33:0] ai34 = {{2{ai_r[31]}}, ai_r};
    wire signed [33:0] tr34 = {rir[32], rir[32:0]};               // t = b x conj(twiddle)
    wire signed [33:0] ti34 = {rii[32], rii[32:0]};
    wire signed [33:0] ur = ar34 + tr34 + 34'sd1;
    wire signed [33:0] ui = ai34 + ti34 + 34'sd1;
    wire signed [33:0] vr = ar34 - tr34 + 34'sd1;
    wire signed [33:0] vi = ai34 - ti34 + 34'sd1;
    wire signed [18:0] wsum = cn[9] ? (19'sd65536 + wre) : (19'sd65536 - wre);
    wire [16:0] win  = wsum[17:1];                                // Hann window from the cosine table
    wire signed [21:0] smp = qfr ? rgq[43:22] : rgq[21:0];
    // size of a cell: the larger part + 3/8 of the smaller
    wire [31:0] mre  = ar[31] ? (32'd0 - ar) : ar;
    wire [31:0] mim  = ai[31] ? (32'd0 - ai) : ai;
    wire [31:0] mmx  = (mre > mim) ? mre : mim;
    wire [31:0] mmn  = (mre > mim) ? mim : mre;
    wire [33:0] mm3  = {2'b00, mmn} + {1'b0, mmn, 1'b0};
    wire [32:0] msz  = {1'b0, mmx} + {2'b00, mm3[33:3]};
    reg  [63:0] xa = 64'd0, xb = 64'd0;
    // the gain rule
    reg  signed [14:0] plg = 15'sd0;      // gamma: this cell's power over the noise, 64 x log2
    reg  signed [14:0] a_r = 15'sd0, b_r = 15'sd0, xi_r = 15'sd0;
    reg                pos_r = 1'b0;
    reg  [8:0]         g2m = 9'd0;        // how far the gain^2 is below 1
    wire signed [14:0] plg_c = ($signed({3'b000, lg_l}) - $signed({llam[13], llam})) <<< 1;
    wire signed [10:0] gamp  = bsq[19:9];
    wire [8:0]         g2p   = bsq[8:0];
    wire signed [14:0] hi_c  = (a_r >= b_r) ? a_r : b_r;
    wire signed [15:0] df_c  = (a_r >= b_r) ? ($signed({a_r[14], a_r}) - $signed({b_r[14], b_r}))
                                            : ($signed({b_r[14], b_r}) - $signed({a_r[14], a_r}));
    wire signed [15:0] j_c   = $signed({xi_r[14], xi_r}) + 16'sd256;
    wire [9:0]         vm_c  = j_c[15] ? gtm(10'd0) : (j_c > 16'sd1023) ? 10'd0 : gtm(j_c[9:0]);
    wire signed [10:0] gclp  = (plg > 15'sd1023) ? 11'sd1023 : (plg < -15'sd1024) ? -11'sd1024 : plg[10:0];
    wire [16:0]        glin  = exl(g2m);
    wire               inobs = (cn[7:0] >= OBS_LO) && (cn[7:0] <= obs_hi) && (cn[9:8] == 2'b00);
    wire               isbel = ($signed({2'b00, lg_l}) < othr);
    // overlap-add
    wire signed [25:0] yw   = r11[25:0];
    wire signed [25:0] osum = (cn < 10'd768) ? ($signed(olaq) + yw) : yw;
    wire signed [50:0] o23  = r11;                                // sum x 2/3
    wire signed [21:0] osat = (o23 > 51'sd2097151) ? 22'sd2097151 : (o23 < -51'sd2097152) ? -22'sd2097152 : o23[21:0];
    // noise level update
    wire signed [10:0] dlt  = $signed({4'b0000, nobs}) - $signed({1'b0, below, 3'b000});       // above - 7 x below
    // learning afresh: big steps, bigger when few cells are watched, so that 64 pieces can climb 1300 u or more
    wire signed [21:0] dlx  = $signed({{11{dlt[10]}}, dlt});
    wire signed [21:0] stp  = !acq_f ? dlx : (nobs < 7'd32) ? (dlx <<< 8) : (nobs < 7'd64) ? (dlx <<< 7) : (dlx <<< 6);
    wire signed [21:0] n0n  = $signed({2'b00, n0_f}) + stp;
    wire signed [21:0] n0c  = (!qfr && !acq_f && (n0n > $signed({2'b00, n0_f}) + 22'sd1)) ? ($signed({2'b00, n0_f}) + 22'sd1) : n0n;
    wire [19:0]        n0x  = n0c[21] ? 20'd0 : (n0c > 22'sd1048575) ? 20'hFFFFF : n0c[19:0];

    always @(posedge clk) begin
        wea <= 1'b0;  web <= 1'b0;  owe <= 1'b0;  obwe <= 1'b0;  bswe <= 1'b0;

        // ---- every sample --------------------------------------------------------------
        if (rst) begin
            t <= 10'd0;  play <= 1'b0;  obase <= 10'd0;  lockhist <= 6'd0;  use_q <= 1'b0;
            n0_f <= 20'd0;  acq <= ACQ;  fs <= F_IDLE;
        end else begin
            if (tick) begin
                t    <= t + 10'd1;
                obr  <= {play_n, t[7:0]};
                acq  <= acq_n;
                if (hop) begin
                    play     <= ~play;
                    lockhist <= lh_n;
                    use_q    <= uq_n;
                    // the audio piece: the newest 1024 samples, this one included
                    fbase  <= t;
                    lden_r <= lden_i;
                    khi    <= khi_i;
                    obs_hi <= ohi_i;
                    flm    <= (fsel_i == 2'd0) ? 9'd255 : (fsel_i == 2'd1) ? 9'd383 : (fsel_i == 2'd2) ? 9'd510 : 9'd128;
                    n0_ref <= n0_f[19:8];
                    qfr    <= 1'b0;
                    nobs   <= 7'd0;
                    below  <= 7'd0;
                    if (!uq_n) begin                              // this piece will measure the noise level
                        acq_f <= (acq_n != 7'd0);
                        if (acq_n != 7'd0) acq <= acq_n - 7'd1;
                    end
                    p  <= 4'd0;
                    fs <= F_LOG;
                end else if (qhop && use_q && locked_i) begin
                    // the quadrature piece: only its levels are needed
                    fbase  <= t;
                    khi    <= khi_i;
                    obs_hi <= ohi_i;
                    n0_ref <= n0_f[19:8];
                    othr   <= $signed({2'b00, n0_f[19:8]}) - 14'sd256;        // it was taken >> 4
                    qfr    <= 1'b1;
                    nobs   <= 7'd0;
                    below  <= 7'd0;
                    acq_f  <= (acq_n != 7'd0);
                    if (acq_n != 7'd0) acq <= acq_n - 7'd1;
                    cn <= 10'd0;
                    p  <= 4'd0;
                    fs <= F_LOAD;
                end
            end

            // ---- the piece -----------------------------------------------------------------
            case (fs)
                // 64 x log2 of the AGC's divisor: it refers this piece's levels to before the AGC
                F_LOG: begin
                    p <= p + 4'd1;
                    if (p == 4'd0) lg_v <= {1'b0, lden_r};
                    if (p == 4'd3) begin
                        othr <= $signed({2'b00, n0_ref}) - $signed({2'b00, lg_l}) + 14'sd960;
                        llam <= $signed({2'b00, n0_ref}) - $signed({2'b00, lg_l}) + 14'sd960 + C_OS;
                        cn   <= 10'd0;
                        p    <= 4'd0;
                        fs   <= F_LOAD;
                    end
                end
                // 1024 samples, oldest first, each times the window, into the work RAM
                F_LOAD: begin
                    p <= p + 4'd1;
                    if (p == 4'd0) begin
                        rga <= fbase + 10'd1 + cn;
                        twa <= cn[8:0];
                    end
                    if (p == 4'd2) begin
                        ma <= {{11{smp[21]}}, smp};
                        mb <= 33'sd0;
                        wr <= {1'b0, win};
                        wi <= 18'sd0;
                    end
                    if (p == 4'd4) begin
                        aa  <= cn;
                        da  <= {32'd0, r11[31:0]};
                        wea <= 1'b1;
                        p   <= 4'd0;
                        cn  <= cn + 10'd1;
                        if (cn == 10'd1023) begin
                            cn <= 10'd0;
                            sg <= 4'd0;
                            fs <= F_FFT;
                        end
                    end
                end
                // forward FFT, ten stages of 512 butterflies: top = a + b, bottom = (a - b) x twiddle
                F_FFT: begin
                    p <= p + 4'd1;
                    if (p == 4'd0) begin
                        aa  <= ia;
                        ab  <= ib;
                        twa <= twx[8:0];
                    end
                    if (p == 4'd2) begin
                        da <= {si[31:0], sr[31:0]};
                        ma <= $signed({ar[31], ar}) - $signed({br[31], br});
                        mb <= $signed({ai[31], ai}) - $signed({bi[31], bi});
                        wr <= wre;
                        wi <= wim;
                    end
                    if (p == 4'd4) begin
                        db  <= {rfi[31:0], rfr[31:0]};
                        wea <= 1'b1;
                        web <= 1'b1;
                        p   <= 4'd0;
                        cn  <= cn + 10'd1;
                        if (cn == 10'd511) begin
                            cn <= 10'd0;
                            sg <= sg + 4'd1;
                            if (sg == 4'd9) begin
                                cn <= qfr ? {2'b00, OBS_LO} : 10'd0;
                                fs <= F_BIN;
                            end
                        end
                    end
                end
                // the cells. Cell k sits at the bit-reversed address, its mirror image at that of -k.
                F_BIN: begin
                    p <= p + 4'd1;
                    case (p)
                        4'd0: begin
                            aa  <= brev(cn);
                            ab  <= brev(10'd0 - cn);
                            bsa <= cn[7:0];
                        end
                        4'd2: begin
                            xa   <= qa;
                            xb   <= qb;
                            lg_v <= msz;
                        end
                        4'd5: begin
                            if (inobs && (qfr || !use_q)) begin
                                nobs <= nobs + 7'd1;
                                if (isbel) below <= below + 7'd1;
                            end
                            plg <= plg_c;
                            a_r <= $signed({{4{gamp[10]}}, gamp}) - $signed({6'd0, g2p}) + PL_ALPHA;
                            if (qfr) begin
                                p  <= 4'd0;
                                cn <= cn + 10'd1;
                                if (cn[7:0] == obs_hi) fs <= F_N0;
                            end
                        end
                        4'd6: begin
                            pos_r <= (plg > 15'sd0);
                            b_r   <= plg - ((plg < 15'sd512) ? $signed({6'd0, c1m(plg[8:0])}) : 15'sd0) + PL_1MALPHA;
                        end
                        4'd7: begin
                            xi_r <= pos_r ? (hi_c + ((df_c < 16'sd512) ? $signed({8'd0, lam(df_c[8:0])}) : 15'sd0)) : a_r;
                        end
                        4'd8: begin
                            g2m <= (vm_c[9] || (vm_c[8:0] > flm)) ? flm : vm_c[8:0];
                        end
                        4'd9: begin
                            ma   <= {xa[31], xa[31:0]};
                            mb   <= {xa[63], xa[63:32]};
                            wr   <= {1'b0, glin};
                            wi   <= 18'sd0;
                            bsd  <= {gclp, g2m};
                            bswe <= 1'b1;
                        end
                        4'd11: begin
                            da  <= {r44[31:0], r11[31:0]};
                            wea <= 1'b1;
                            ma  <= {xb[31], xb[31:0]};
                            mb  <= {xb[63], xb[63:32]};
                        end
                        4'd13: begin
                            db  <= {r44[31:0], r11[31:0]};
                            web <= (cn != 10'd0);
                            p   <= 4'd0;
                            cn  <= cn + 10'd1;
                            if (cn[7:0] == khi) fs <= F_ZERO;
                        end
                        default: ;
                    endcase
                end
                // everything above the audio band: zero, both the cell and its mirror image
                F_ZERO: begin
                    aa  <= brev(cn);
                    ab  <= brev(10'd0 - cn);
                    da  <= 64'd0;
                    db  <= 64'd0;
                    wea <= 1'b1;
                    web <= (cn != 10'd512);
                    cn  <= cn + 10'd1;
                    if (cn == 10'd512) fs <= F_N0;
                end
                // the noise level: one cell in eight is to lie below it
                F_N0: begin
                    if (qfr || !use_q) n0_f <= n0x;
                    cn <= 10'd0;
                    sg <= 4'd0;
                    p  <= 4'd0;
                    fs <= qfr ? F_IDLE : F_IFFT;
                end
                // inverse FFT: t = b x conj(twiddle), top = (a + t) / 2, bottom = (a - t) / 2
                F_IFFT: begin
                    p <= p + 4'd1;
                    if (p == 4'd0) begin
                        aa  <= ia;
                        ab  <= ib;
                        twa <= twx[8:0];
                    end
                    if (p == 4'd2) begin
                        ar_r <= ar;
                        ai_r <= ai;
                        ma   <= {br[31], br};
                        mb   <= {bi[31], bi};
                        wr   <= wre;
                        wi   <= wim;
                    end
                    if (p == 4'd4) begin
                        da  <= {ui[32:1], ur[32:1]};
                        db  <= {vi[32:1], vr[32:1]};
                        wea <= 1'b1;
                        web <= 1'b1;
                        p   <= 4'd0;
                        cn  <= cn + 10'd1;
                        if (cn == 10'd511) begin
                            cn <= 10'd0;
                            sg <= sg + 4'd1;
                            if (sg == 4'd9) fs <= F_OLA;
                        end
                    end
                end
                // window again, add to the pieces before; the oldest 256 samples are finished
                F_OLA: begin
                    p <= p + 4'd1;
                    if (p == 4'd0) begin
                        aa  <= cn;
                        oa  <= obase + cn;
                        twa <= cn[8:0];
                    end
                    if (p == 4'd2) begin
                        ma <= {ar[31], ar};
                        mb <= 33'sd0;
                        wr <= {1'b0, win};
                        wi <= 18'sd0;
                    end
                    if (p == 4'd4) begin
                        od  <= osum;
                        owe <= 1'b1;
                        ma  <= {{7{osum[25]}}, osum};
                        wr  <= 18'sd43691;                        // 2/3: overlapping windows add up to 1.5
                        if (cn >= 10'd256) begin
                            p  <= 4'd0;
                            cn <= cn + 10'd1;
                            if (cn == 10'd1023) begin
                                obase <= obase + 10'd256;
                                fs    <= F_IDLE;
                            end
                        end
                    end
                    if (p == 4'd6) begin
                        obd  <= osat;
                        obwa <= {~play, cn[7:0]};
                        obwe <= 1'b1;
                        p    <= 4'd0;
                        cn   <= cn + 10'd1;
                    end
                end
                default: ;
            endcase
        end
    end
endmodule