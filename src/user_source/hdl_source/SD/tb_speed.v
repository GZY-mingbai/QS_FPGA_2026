`timescale 1ns/1ps
module tb_speed;
reg clk=0; always #10 clk=~clk;
reg rst=1, start=0;
wire wr, rdy; wire [20:0] wo; wire [31:0] wd; wire rd;
integer n=0;
bmp_scale dut(.clk(clk),.rst(rst),.start(start),.src_w(16'd320),.src_h(16'd240),
  .stretch(1'b0),.fmt(1'b1),.pix_vld(1'b0),.pix(24'd0),.pix_rd(rd),
  .we(wr),.woff(wo),.wdata(wd),.busy(),.done());
always @(posedge clk) begin
  n <= n+1;
  if ((n % 50000)==0) $display("cyc=%0d st=%0d oy=%0d", n, dut.st, dut.oy);
  if (n > 1000000) $finish;
end
initial begin
  repeat(5) @(posedge clk); rst=0; repeat(2) @(posedge clk);
  start=1; @(posedge clk); start=0;
end
endmodule
