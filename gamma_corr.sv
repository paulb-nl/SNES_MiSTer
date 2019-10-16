module gamma_corr
(
	input             clk_sys,
	input             clk_vid,
	input             ce_pix,
	input             gamma_en,
	input             gamma_wr,
	input       [7:0] gamma_wr_addr,
	input       [7:0] gamma_value,
	input             HSync,
	input             VSync,
	input             HBlank,
	input             VBlank,
	input      [23:0] RGB_in,
	output reg        HSync_out,
	output reg        VSync_out,
	output reg        HBlank_out,
	output reg        VBlank_out,
	output reg [23:0] RGB_out
);

reg  [7:0] gamma_curve[256];
always @(posedge clk_sys) begin
	if (gamma_wr) begin
		gamma_curve[gamma_wr_addr] <= gamma_value;
	end
end

reg [7:0] gamma_index;
wire [7:0] gamma = gamma_curve[gamma_index];
always @(posedge clk_vid) begin
	reg [7:0] R_in, G_in, B_in;
	reg [7:0] R_gamma, G_gamma, B_gamma;
	reg       hs,vs,hb,vb;
	reg [1:0] ctr = 0;

	if(ce_pix) begin
		{R_in,G_in,B_in} <= RGB_in;
		hs <= HSync; vs <= VSync;
		hb <= HBlank; vb <= VBlank;

		RGB_out  <= gamma_en ? {R_gamma,G_gamma,B_gamma} : {R_in,G_in,B_in};
		HSync_out <= hs; VSync_out <= vs;
		HBlank_out <= hb; VBlank_out <= vb;

		ctr <= 1;
		gamma_index <= RGB_in[23:16];
	end

	if (|ctr) ctr <= ctr + 1'd1;

	case(ctr)
		1: begin R_gamma <= gamma; gamma_index <= G_in; end
		2: begin G_gamma <= gamma; gamma_index <= B_in; end
		3: begin B_gamma <= gamma; end
	endcase
end

endmodule