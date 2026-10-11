module am_audio #(
  parameter integer K = 10
)(
  input  wire        aclk,
  input  wire        aresetn,
  input  wire [63:0] s_axis_tdata,
  input  wire        s_axis_tvalid,
  input  wire [4:0]  ctrl_i,
  output reg  [23:0] m_axis_tdata,
  output reg         m_axis_tvalid
);
  wire [3:0] gain = ctrl_i[3:0];
  wire       test = ctrl_i[4];

  wire signed [28:0]   x   = {1'b0, s_axis_tdata[27:0]};
  reg  signed [28+K:0] acc = 0;
  wire signed [28:0]   avg = acc >>> K;
  wire signed [29:0]   ac  = x - avg;
  wire signed [44:0]   g   = ac <<< gain;
  wire signed [44:0]   hi  = 45'sd8388607;
  wire signed [44:0]   lo  = -45'sd8388608;
  reg         [5:0]    tc  = 0;

  always @(posedge aclk) begin
    m_axis_tvalid <= 1'b0;
    if (!aresetn) begin
      acc <= 0;
      tc  <= 0;
    end else if (s_axis_tvalid) begin
      acc <= acc + ac;
      tc  <= tc + 6'd1;
      m_axis_tvalid <= 1'b1;
      if (test)
        m_axis_tdata <= tc[5] ? 24'h100000 : 24'hF00000;
      else if (g > hi) m_axis_tdata <= 24'h7FFFFF;
      else if (g < lo) m_axis_tdata <= 24'h800000;
      else             m_axis_tdata <= g[23:0];
    end
  end
endmodule