module axis_tlast_gen #(
  parameter integer W = 32,
  parameter integer N = 16384
)(
  input  wire         aclk,
  input  wire         aresetn,
  input  wire [W-1:0] s_axis_tdata,
  input  wire         s_axis_tvalid,
  output wire         s_axis_tready,
  output wire [W-1:0] m_axis_tdata,
  output wire         m_axis_tvalid,
  input  wire         m_axis_tready,
  output wire         m_axis_tlast
);
  reg [31:0] cnt = 0;
  wire last = (cnt == N - 1);
  wire beat = s_axis_tvalid && m_axis_tready;

  assign m_axis_tdata  = s_axis_tdata;
  assign m_axis_tvalid = s_axis_tvalid;
  assign s_axis_tready = m_axis_tready;
  assign m_axis_tlast  = last;

  always @(posedge aclk) begin
    if (!aresetn)  cnt <= 0;
    else if (beat) cnt <= last ? 0 : cnt + 1;
  end
endmodule