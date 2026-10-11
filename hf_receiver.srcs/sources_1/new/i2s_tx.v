module i2s_tx (
  input  wire        aclk,
  input  wire        aresetn,
  input  wire [23:0] s_axis_tdata,
  input  wire        s_axis_tvalid,
  output reg         bclk,
  output reg         lrclk,
  output reg         sdata
);
  reg [4:0]  p    = 0;   // clocks within a bit
  reg [5:0]  b    = 0;   // bit within a frame
  reg [23:0] hold = 0;
  reg [63:0] sr   = 0;

  always @(posedge aclk) begin
    if (s_axis_tvalid) hold <= s_axis_tdata;

    if (!aresetn) begin
      p <= 0; b <= 0; sr <= 0;
      bclk <= 0; lrclk <= 0; sdata <= 0;
    end else begin
      p <= (p == 19) ? 5'd0 : p + 5'd1;
      if (p == 9) bclk <= 1'b1;
      if (p == 19) begin
        bclk  <= 1'b0;
        b     <= b + 6'd1;
        lrclk <= (b >= 31) && (b != 63);
        if (b == 63) begin
          sdata <= 1'b0;
          sr    <= {hold, 8'b0, hold, 8'b0};
        end else begin
          sdata <= sr[63];
          sr    <= {sr[62:0], 1'b0};
        end
      end
    end
  end
endmodule