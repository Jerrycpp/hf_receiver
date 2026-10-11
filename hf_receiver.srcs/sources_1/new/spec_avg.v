module spec_avg (
  input  wire        aclk,
  input  wire        aresetn,
  input  wire [79:0] s_axis_tdata,
  input  wire [15:0] s_axis_tuser,
  input  wire        s_axis_tvalid,
  input  wire        s_axis_tlast,
  output wire        s_axis_tready,
  input  wire [4:0]  ctrl_i,
  input  wire        fifo_full_i,
  output reg  [63:0] m_axis_tdata,
  output reg         m_axis_tvalid,
  output reg         m_axis_tlast,
  // Display tap: every finished line, whether or
  // not the FIFO has room for it.
  // {local, last, valid, bin[10:0], power[63:0]}
  output reg  [77:0] wf_o = 78'd0
);
  assign s_axis_tready = 1'b1;

  // Settings are latched once per FFT frame, so a
  // line is always sent whole or not at all.
  reg [3:0]  log2m = 0;
  reg        zoom  = 0;
  reg        room  = 0;
  reg [10:0] fc    = 0;        // frame counter
  wire [10:0] m1 = (11'd1 << log2m) - 11'd1;
  wire [11:0] k  = s_axis_tuser[11:0];

  // which bins to keep, and their RAM address
  wire        use_k = zoom ? (k[11] == k[10])
                           : ~k[11];
  wire [10:0] addr  = zoom ? {~k[11], k[9:0]}
                           : k[10:0];
  wire        end_k = zoom ? (k == 12'd4095)
                           : (k == 12'd2047);

  reg signed [25:0] re0, im0;         // stage 0
  reg [10:0] a0; reg v0, u0, f0, l0, e0;
  reg [51:0] re1, im1;                // stage 1
  reg [10:0] a1; reg v1, u1, f1, l1, e1;
  reg [52:0] p2; reg [63:0] q2;       // stage 2
  reg [10:0] a2; reg v2, u2, f2, l2, e2;
  // "last frame of the average", and the view the
  // frame was taken in, for the display
  reg w0 = 0, w1 = 0, w2 = 0;
  reg z0 = 0, z1 = 0, z2 = 0;

  reg  [63:0] mem [0:2047];
  wire [63:0] p64 = {11'd0, p2};
  wire [63:0] sum = f2 ? p64 : q2 + p64;

  always @(posedge aclk) begin
    // stage 0: capture one FFT bin
    re0 <= s_axis_tdata[36:11];
    im0 <= s_axis_tdata[76:51];
    a0  <= addr;
    v0  <= s_axis_tvalid;
    u0  <= use_k;
    f0  <= (fc == 0);
    l0  <= (fc == m1) && room;
    w0  <= (fc == m1);
    z0  <= zoom;
    e0  <= end_k;
    // stage 1: squares
    re1 <= re0 * re0;
    im1 <= im0 * im0;
    a1 <= a0; v1 <= v0; u1 <= u0;
    f1 <= f0; l1 <= l0; e1 <= e0;
    w1 <= w0; z1 <= z0;
    // stage 2: power, read running total
    p2 <= re1 + im1;
    q2 <= mem[a1];
    a2 <= a1; v2 <= v1; u2 <= u1;
    f2 <= f1; l2 <= l1; e2 <= e1;
    w2 <= w1; z2 <= z1;
    // stage 3: store; send on the last frame
    if (v2 && u2) mem[a2] <= sum;
    m_axis_tdata  <= sum;
    m_axis_tvalid <= v2 && u2 && l2;
    m_axis_tlast  <= e2;
    wf_o <= {z2, e2, v2 && u2 && w2, a2, sum};

    if (!aresetn) begin
      fc <= 0; room <= 0;
      log2m <= ctrl_i[3:0];
      zoom  <= ctrl_i[4];
      v0 <= 0; v1 <= 0; v2 <= 0;
      m_axis_tvalid <= 0;
      wf_o[75] <= 1'b0;
    end else if (s_axis_tvalid && s_axis_tlast) begin
      // frame boundary: next frame's settings
      fc    <= (fc >= m1) ? 11'd0 : fc + 11'd1;
      log2m <= ctrl_i[3:0];
      zoom  <= ctrl_i[4];
      room  <= ~fifo_full_i;
    end
  end
endmodule