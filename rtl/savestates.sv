module savestates
(
	input reset_n,
	input clk,

	input             save,
	input             save_sd,
	input             load,
	input       [1:0] slot,

	input             cart_download,
	input       [3:0] ram_size,

	input             sysclkf_ce,
	input             sysclkr_ce,

	input             romsel_n,

	input      [15:0] rom_q,

	input      [23:0] ca,
	input             cpurd_n,
	input             cpuwr_n,

	input       [7:0] pa,
	input             pard_n,
	input             pawr_n,

	input       [7:0] di,
	output reg  [7:0] ss_do,

	output     [23:0] rom_addr,
	output     [15:0] rom_d,
	output            rom_ce_n,
	output            rom_oe_n,
	output            rom_we_n,
	output            rom_word,

	output     [19:0] ext_addr,

	input       [7:0] spc_di,

	input      [15:0] ddr_di,
	output     [15:0] ddr_do,
	input             ddr_ack,
	output     [21:0] ddr_addr,
	output            ddr_we,
	output reg        ddr_req,

	output            aram_sel,
	output            dsp_regs_sel,
	output            smp_regs_sel,

	output            bsram_sel,
	input       [7:0] bsram_di,

	output            ss_ovr,
	output reg        ss_busy
);

reg cpurd_n_old, cpuwr_n_old;
reg pawr_n_old, pard_n_old;
reg save_old, load_old;

always @(posedge clk or negedge reset_n) begin
	if (~reset_n) begin
		cpurd_n_old <= 1'b1;
		cpuwr_n_old <= 1'b1;
		pard_n_old <= 1'b1;
		pawr_n_old <= 1'b1;
		save_old <= 0;
		load_old <= 0;
	end else begin
		cpurd_n_old <= cpurd_n;
		cpuwr_n_old <= cpuwr_n;

		pawr_n_old <= pawr_n;
		pard_n_old <= pard_n;

		save_old <= save;
		load_old <= load;
	end
end

reg save_en;
reg load_en;
reg cpu_pause, cpu_pause_byte, cpu_pause_end;
reg rd_rti;

wire nmi_vect = ({ca[23:1],1'b0} == 24'h00FFEA) || ({ca[23:1],1'b0} == 24'h00FFFA);
wire nmi_vect_l = nmi_vect & ~ca[0];
wire nmi_vect_h = nmi_vect &  ca[0];

wire ss_reg_sel = (ca[23:16] == 8'hC0);

reg [19:0] ss_data_addr;
reg [19:0] ss_data_size;
reg [19:0] ss_ddr_addr;
reg [1:0] ss_slot;
reg ss_data_addr_inc;
wire ss_data_sel = ss_reg_sel & (ca[15:0] == 16'h6000);
wire ss_addr_sel = ss_reg_sel & (ca[15:0] == 16'h6001);
wire ss_ext_addr_sel = ss_reg_sel & (ca[15:0] == 16'h6002);
wire ss_ramsize_sel = ss_reg_sel & (ca[15:0] == 16'h6003);
wire ss_copy_sel = ss_reg_sel & ({ca[15:1],1'b0} == 16'h600E);
wire rti_sel = (ca[23:0] == 24'h008008);

reg [19:0] ss_ext_addr;
reg ss_ext_addr_inc;

wire spc_sel = (aram_sel | dsp_regs_sel | smp_regs_sel);
wire spc_read = spc_sel & ~pard_n;

wire bsram_read = bsram_sel & ~pard_n;

reg [3:0] copy_state;
reg [15:0] copy_data, rom_d_r;
reg sd_ready[4];
reg load_ready;
reg sdr_copy_wr;

wire copy_busy = (copy_state != COPY_IDLE);

localparam COPY_IDLE = 4'd0, COPY_TO_DDR_INIT = 4'd1, COPY_TO_DDR = 4'd2,
			COPY_TO_DDR_WRITE_SIZEL = 4'd3, COPY_TO_DDR_WRITE_SIZEH = 4'd4,
			COPY_TO_DDR_WRITE_CNTL = 4'd5, COPY_TO_DDR_WRITE_CNTH = 4'd6,
			COPY_TO_SDR_CHECK_HEAD = 4'd7, COPY_TO_SDR_READ_HEAD1 = 4'd8,
			COPY_TO_SDR_READ_HEAD2 = 4'd9, COPY_TO_SDR_CHECK_HEAD_END = 4'd10,
			COPY_TO_SDR_INIT = 4'd11, COPY_TO_SDR = 4'd12, COPY_END = 4'd13;


localparam SS_MAX_SIZE = 20'hC0000; // 768KB
reg [31:0] ss_count = 0;

always @(posedge clk) begin
	if (~reset_n) begin
		ss_busy <= 0;
		save_en <= 0;
		load_en <= 0;
		cpu_pause <= 0;
		cpu_pause_byte <= 0;
		cpu_pause_end <= 0;
		rd_rti <= 0;
		ss_data_addr <= 0;
		ss_data_addr_inc <= 0;
		ss_ext_addr <= 0;
		ss_ext_addr_inc <= 0;
		copy_state <= COPY_IDLE;
		sdr_copy_wr <= 0;
		if (cart_download) begin
			sd_ready[0] <= 0;
			sd_ready[1] <= 0;
			sd_ready[2] <= 0;
			sd_ready[3] <= 0;
		end
	end else begin
		if (~(load_en | save_en)) begin
			if (~save_old & save) begin
				save_en <= 1;
				ss_slot <= slot;
			end else if (~load_old & load) begin
				load_en <= 1;
				ss_slot <= slot;
				if (sd_ready[slot]) begin
					load_ready <= 1;
				end else begin
					load_ready <= 0;
					copy_state <= COPY_TO_SDR_CHECK_HEAD;
				end
			end
		end

		if (cpurd_n_old & ~cpurd_n) begin
			if (nmi_vect_l & ~ss_busy & (save_en | (load_en & load_ready))) begin
				ss_busy <= 1; // Override NMI vector
				if (save_en) begin
					ss_count <= ss_count + 1'b1;
				end
			end

			if (cpu_pause) begin
				if (~copy_busy & cpu_pause_byte) begin
					cpu_pause_end <= 1;
				end
			end

			if (ss_busy & rti_sel) begin
				rd_rti <= 1;
			end
		end

		if (~cpurd_n_old & cpurd_n) begin

			if (cpu_pause) begin
				cpu_pause_byte <= ~cpu_pause_byte;
			end

			if (cpu_pause_end) begin
				cpu_pause_end <= 0;
				cpu_pause <= 0;
			end

			if (rd_rti) begin
				ss_busy <= 0;
				rd_rti <= 0;
				load_en <= 0;
				save_en <= 0;
			end
		end

		if (cpuwr_n_old & ~cpuwr_n & ss_busy) begin
			if (ss_addr_sel) begin
				ss_data_addr <= 20'd8;
			end
			if (ss_ext_addr_sel) begin
				ss_ext_addr <= 0;
			end

			if (ss_copy_sel) begin
				cpu_pause <= 1; // Keep CPU branching to the current address during copying
				cpu_pause_byte <= 0;

				if (save_en) begin
					// save done
					sd_ready[ss_slot] <= 1;
					ss_data_size <= ss_data_addr;
					// Copy the state to DDR so HPS can save it to SD card.
					copy_state <= COPY_TO_DDR_INIT;
				end

				if (load_en & ~sd_ready[ss_slot]) begin
					// Need to copy the state from DDR to SDR
					copy_state <= COPY_TO_SDR_INIT;
				end
			end
		end

		if ((cpuwr_n_old & ~cpuwr_n) | (cpurd_n_old & ~cpurd_n)) begin
			if (ss_data_sel) begin
				ss_data_addr_inc <= 1;
			end
		end

		if ((~cpuwr_n_old & cpuwr_n) | (~cpurd_n_old & cpurd_n)) begin
			if (ss_data_addr_inc) begin
				ss_data_addr <= ss_data_addr + 1'b1;
				ss_data_addr_inc <= 0;
			end
		end

		if ((pawr_n_old & ~pawr_n) | (pard_n_old & ~pard_n)) begin
			if (spc_sel | bsram_sel) begin
				ss_ext_addr_inc <= 1;
			end
		end

		if ((~pawr_n_old & pawr_n) | (~pard_n_old & pard_n)) begin
			if (ss_ext_addr_inc) begin
				ss_ext_addr <= ss_ext_addr + 1'b1;
				ss_ext_addr_inc <= 0;
			end
		end

		if (ddr_req == ddr_ack) begin
			case(copy_state)
				COPY_TO_DDR_INIT: begin
					if (sysclkf_ce) begin
						ss_data_addr <= 20'd8;
						copy_state <= COPY_TO_DDR;
					end
				end
				COPY_TO_DDR: begin
					if (sysclkf_ce) begin
						if (ss_data_addr < ss_data_size) begin
							copy_data <= rom_q;
							ss_data_addr <= ss_data_addr + 19'd2;
							ss_ddr_addr <= ss_data_addr;
							ddr_req <= ~ddr_ack;
						end else begin
							copy_state <= COPY_TO_DDR_WRITE_SIZEL;
						end
					end
				end
				COPY_TO_DDR_WRITE_SIZEL: begin
					copy_data <= ss_data_size[17:2];
					ss_ddr_addr <= 20'd4;
					ddr_req <= ~ddr_ack;
					copy_state <= COPY_TO_DDR_WRITE_SIZEH;
				end
				COPY_TO_DDR_WRITE_SIZEH: begin
					copy_data <= {14'd0, ss_data_size[19:18]};
					ss_ddr_addr <= 20'd6;
					ddr_req <= ~ddr_ack;
					copy_state <= save_sd ? COPY_TO_DDR_WRITE_CNTL : COPY_END;
				end
				COPY_TO_DDR_WRITE_CNTL: begin
					copy_data <= ss_count[15:0];
					ss_ddr_addr <= 20'd0;
					ddr_req <= ~ddr_ack;
					copy_state <= COPY_TO_DDR_WRITE_CNTH;
				end
				COPY_TO_DDR_WRITE_CNTH: begin
					copy_data <= ss_count[31:16];
					ss_ddr_addr <= 20'd2;
					ddr_req <= ~ddr_ack;
					copy_state <= COPY_END;
				end
				COPY_TO_SDR_CHECK_HEAD: begin // Dummy read to clear DDR cache
					ss_ddr_addr <= ~20'd0;
					ddr_req <= ~ddr_ack;
					copy_state <= COPY_TO_SDR_READ_HEAD1;
				end
				COPY_TO_SDR_READ_HEAD1: begin
					ss_ddr_addr <= 20'd8;
					ddr_req <= ~ddr_ack;
					copy_state <= COPY_TO_SDR_READ_HEAD2;
				end
				COPY_TO_SDR_READ_HEAD2: begin
					copy_data <= ddr_di;
					ss_ddr_addr <= 20'd10;
					ddr_req <= ~ddr_ack;
					copy_state <= COPY_TO_SDR_CHECK_HEAD_END;
				end
				COPY_TO_SDR_CHECK_HEAD_END: begin
					copy_state <= COPY_END;
					if ({copy_data, ddr_di} == 32'h4E53_5345) begin // "SNES"
						load_ready <= 1;
					end else begin
					   load_en <= 0;
					end
				end
				COPY_TO_SDR_INIT: begin
					ss_ddr_addr <= 20'd8;
					ddr_req <= ~ddr_ack;
					copy_state <= COPY_TO_SDR;
				end
				COPY_TO_SDR: begin
					if (sysclkr_ce) begin
						copy_data <= ddr_di;
						ss_data_addr <= ss_ddr_addr;
						sdr_copy_wr <= 1;
						ss_ddr_addr <= ss_ddr_addr + 20'd2;
						ddr_req <= ~ddr_ack;
					end
					if (sysclkf_ce) begin
						sdr_copy_wr <= 0;
						if (ss_ddr_addr == SS_MAX_SIZE) begin
							sd_ready[ss_slot] <= 1;
							copy_state <= COPY_END;
						end
					end

				end
				COPY_END: begin
					copy_state <= COPY_IDLE;
				end
			endcase

		end
	end
end


wire [15:0] nmi_vect_addr = save_en ? 16'h8000 : 16'h8004;

wire [7:0] pause_opcodes [2] = '{ 'h80, 'hFE }; // Branch to self

wire [7:0] ssr_do;
wire ssr_oe;
savestates_regs ss_regs
(
	.reset_n(reset_n),
	.clk(clk),

	.ss_busy(ss_busy),
	.save_en(save_en),

	.ss_reg_sel(ss_reg_sel),

	.sysclkf_ce(sysclkf_ce),
	.sysclkr_ce(sysclkr_ce),

	.romsel_n(romsel_n),

	.ca(ca),
	.cpurd_ce(cpurd_n_old & ~cpurd_n),
	.cpurd_ce_n(~cpurd_n_old & cpurd_n),
	.cpuwr_ce(cpuwr_n_old & ~cpuwr_n),

	.pa(pa),

	.pard_ce(pard_n_old & ~pard_n),
	.pawr_ce(pawr_n_old & ~pawr_n),

	.di(di),
	.ssr_do(ssr_do),
	.ssr_oe(ssr_oe)
);
always @(posedge clk) begin
	ss_do <= rom_q[7:0];
	if (cpu_pause) ss_do <= pause_opcodes[cpu_pause_byte];
	if (nmi_vect_l) ss_do <= nmi_vect_addr[7:0];
	if (nmi_vect_h) ss_do <= nmi_vect_addr[15:8];
	if (ss_ramsize_sel) ss_do <= { 4'd0, ram_size };
	if (ssr_oe) ss_do <= ssr_do;
end

always @(*) begin
	// savestate.bin ROM. This is at the end of the first 16MB so this
	// overlaps with the end of the last save state slot if it gets too big.
	rom_addr = { 2'b11, 6'b11_1111, ca[16], ca[14:0] };

	// Save state contents
	if (ss_data_sel | copy_busy) rom_addr = { 2'b11, ss_slot[1:0], ss_data_addr[19:0] };
end

// Data to SDRAM
always @(posedge clk) begin
	rom_d_r <= { 8'h00, di };
	if (spc_read) rom_d_r[7:0] <= spc_di;
	if (bsram_read) rom_d_r[7:0] <= bsram_di;
	if (copy_busy) rom_d_r <= copy_data;
end

reg rom_rd;
always @(posedge clk) begin
	rom_rd <= sysclkr_ce | sysclkf_ce;
end

assign ss_ovr = ss_busy & ~romsel_n;
assign aram_sel = ss_busy & (pa == 8'h84);
assign dsp_regs_sel = ss_busy & (pa == 8'h85);
assign smp_regs_sel = ss_busy & (pa == 8'h86);
assign bsram_sel = ss_busy & (pa == 8'h87);
assign ext_addr = ss_ext_addr;

assign rom_word = copy_busy;
assign rom_d = rom_d_r;
assign rom_we_n = ~( (~cpuwr_n & ss_data_sel & sysclkf_ce) | (sdr_copy_wr & sysclkf_ce) );
assign rom_ce_n = romsel_n;
assign rom_oe_n = ~rom_rd;

assign ddr_addr = { ss_slot[1:0], ss_ddr_addr[19:0] };
assign ddr_do = copy_data;
assign ddr_we = copy_busy & save_en;

endmodule