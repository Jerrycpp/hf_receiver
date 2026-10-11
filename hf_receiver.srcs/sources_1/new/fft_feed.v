module fft_feed #(
  parameter integer N = 4096,
  // bits in a zoom CIC's output word
  parameter integer CIC_BITS = 28,
  // 1 = mirror the local view left to right
  // (does the same as bit 0 of opt_i)
  parameter integer MIRROR = 0
)(
  input  wire        aclk,
  input  wire        aresetn,
  // from the notebook: [3:0] averaging as a power
  // of two, [4] flips the view the switch selects
  input  wire [4:0]  ctrl_i,
  input  wire        sw_i,    // 0 = wide, 1 = local
  input  wire [15:0] opt_i,   // bit 0: mirror local
  input  wire [11:0] sample_i,
  // zoom CIC on the sine product
  input  wire [31:0] s_axis_sin_tdata,
  input  wire        s_axis_sin_tvalid,
  // zoom CIC on the cosine product
  input  wire [31:0] s_axis_cos_tdata,
  input  wire        s_axis_cos_tvalid,
  output wire [47:0] m_axis_tdata,
  output wire        m_axis_tvalid,
  input  wire        m_axis_tready,
  output wire        m_axis_tlast,
  // to spec_avg: {local, averaging}
  output reg  [4:0]  ctrl_o = 5'd0
);
  // The switch: two flip-flops, then it has to
  // hold still for 17 ms before it counts.
  (* ASYNC_REG = "TRUE" *) reg [1:0] sw_ff = 2'b00;
  reg        sw_ok  = 1'b0;
  reg [19:0] sw_cnt = 20'd0;
  always @(posedge aclk) begin
    sw_ff <= {sw_ff[0], sw_i};
    if (sw_ff[1] == sw_ok) begin
      sw_cnt <= 20'd0;
    end else begin
      sw_cnt <= sw_cnt + 20'd1;
      if (&sw_cnt) sw_ok <= sw_ff[1];
    end
  end
  wire want = sw_ok ^ ctrl_i[4];   // 1 = local

  reg        zoom = 1'b0;          // view in force
  reg [31:0] cnt  = 0;
  wire last = (cnt == N - 1);
  wire beat = m_axis_tvalid && m_axis_tready;
  wire flip = (MIRROR != 0) ^ opt_i[0];

  // the FFT takes 24 bits: the top 24 of each CIC
  localparam integer T = CIC_BITS - 1;
  wire [23:0] zs = s_axis_sin_tdata[T -: 24];
  wire [23:0] zc = s_axis_cos_tdata[T -: 24];

  // FFT input word: {imaginary, real}
  wire [47:0] wide_d = {24'd0, sample_i, 12'd0};
  wire [47:0] zoom_d = flip ? {zs, zc} : {zc, zs};

  assign m_axis_tdata  = zoom ? zoom_d : wide_d;
  assign m_axis_tvalid = zoom ? s_axis_sin_tvalid
                              : 1'b1;
  assign m_axis_tlast  = last;

  // The local view gets its samples 256 times more
  // slowly, so it averages 256 times fewer frames.
  // A line then takes the same time in both views.
  wire [3:0] avg_l = (ctrl_i[3:0] > 4'd8)
                   ? ctrl_i[3:0] - 4'd8 : 4'd0;

  always @(posedge aclk) begin
    if (!aresetn) begin
      cnt  <= 0;
      zoom <= 1'b0;
    end else begin
      if (beat) cnt <= last ? 0 : cnt + 1;
      // Into the local view only between two
      // frames. Back to wide at once, because the
      // local stream may have stopped.
      if (!zoom && want && beat && last)
        zoom <= 1'b1;
      else if (zoom && !want)
        zoom <= 1'b0;
    end
    ctrl_o <= {zoom, zoom ? avg_l : ctrl_i[3:0]};
  end
endmodule