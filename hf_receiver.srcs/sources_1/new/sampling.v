`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/06/2026 08:42:05 PM
// Design Name: 
// Module Name: sampling
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module sampling(
    input clk,
    input rst_n,

    input [12:0] sample_i,

    output reg [12:0] sample_o,
    output reg sample_valid_o
    );

    (* IOB = "TRUE" *) reg [12:0] cap;

    always @(negedge clk) cap <= sample_i;

    always@(posedge clk) begin
        if (!rst_n) begin
            sample_o <= 0;
            sample_valid_o <= 0;
        end
        else begin
            sample_o <= {cap[12], ~cap[11], cap[10:0]};
            sample_valid_o <= 1;
        end
    end
endmodule
