// --------------------------------------------------------------------
//
// --------------------------------------------------------------------

`include "timescale.v"
`include "gpio_defines.v"


module top(
  ////////////////////////  Clock Input     ////////////////////////
  input [1:0]       clock_24,               //  24 MHz
  input [1:0]       clock_27,               //  27 MHz
  input             clock_50,               //  50 MHz
  input             ext_clock,              //  External Clock
  ////////////////////////  Push Button     ////////////////////////
  input [3:0]       key,                    //  Pushbutton[3:0]
  ////////////////////////  DPDT Switch     ////////////////////////
  input [9:0]       sw,                     //  Toggle Switch[9:0]
  ////////////////////////  7-SEG Dispaly   ////////////////////////
  output    [6:0]   hex0,                   //  Seven Segment Digit 0
  output    [6:0]   hex1,                   //  Seven Segment Digit 1
  output    [6:0]   hex2,                   //  Seven Segment Digit 2
  output    [6:0]   hex3,                   //  Seven Segment Digit 3
  ////////////////////////////  LED     ////////////////////////////
  output    [7:0]   ledg,                   //  LED Green[7:0]
  output    [9:0]   ledr,                   //  LED Red[9:0]
  ////////////////////////////  UART    ////////////////////////////
  output            uart_txd,               //  UART Transmitter
  input             uart_rxd,               //  UART Receiver
  ///////////////////////       SDRAM Interface ////////////////////////
  inout [15:0]      dram_dq,                //  SDRAM Data bus 16 Bits
  output    [11:0]  dram_addr,              //  SDRAM Address bus 12 Bits
  output            dram_ldqm,              //  SDRAM Low-byte Data Mask 
  output            dram_udqm,              //  SDRAM High-byte Data Mask
  output            dram_we_n,              //  SDRAM Write Enable
  output            dram_cas_n,             //  SDRAM Column Address Strobe
  output            dram_ras_n,             //  SDRAM Row Address Strobe
  output            dram_cs_n,              //  SDRAM Chip Select
  output            dram_ba_0,              //  SDRAM Bank Address 0
  output            dram_ba_1,              //  SDRAM Bank Address 0
  output            dram_clk,               //  SDRAM Clock
  output            dram_cke,               //  SDRAM Clock Enable
  ////////////////////////  Flash Interface ////////////////////////
  inout [7:0]       fl_dq,                  //  FLASH Data bus 8 Bits
  output    [21:0]  fl_addr,                //  FLASH Address bus 22 Bits
  output            fl_we_n,                //  FLASH Write Enable
  output            fl_rst_n,               //  FLASH Reset
  output            fl_oe_n,                //  FLASH Output Enable
  output            fl_ce_n,                //  FLASH Chip Enable
  ////////////////////////  SRAM Interface  ////////////////////////
  inout   [15:0]    sram_dq,                //  SRAM Data bus 16 Bits
  output  [17:0]    sram_addr,              //  SRAM Address bus 18 Bits
  output            sram_ub_n,              //  SRAM High-byte Data Mask 
  output            sram_lb_n,              //  SRAM Low-byte Data Mask 
  output            sram_we_n,              //  SRAM Write Enable
  output            sram_ce_n,              //  SRAM Chip Enable
  output            sram_oe_n,              //  SRAM Output Enable
  ////////////////////  SD Card Interface   ////////////////////////
  inout             sd_dat,                 //  SD Card Data
  inout             sd_dat3,                //  SD Card Data 3
  inout             sd_cmd,                 //  SD Card Command Signal
  output            sd_clk,                 //  SD Card Clock
  ////////////////////////  I2C     ////////////////////////////////
  inout             i2c_sdat,               //  I2C Data
  output            i2c_sclk,               //  I2C Clock
  ////////////////////////  PS2     ////////////////////////////////
  input             ps2_dat,                //  PS2 Data
  input             ps2_clk,                //  PS2 Clock
  ////////////////////  USB JTAG link   ////////////////////////////
  input             tdi,                    // CPLD -> FPGA (data in)
  input             tck,                    // CPLD -> FPGA (clk)
  input             tcs,                    // CPLD -> FPGA (CS)
  output            tdo,                    // FPGA -> CPLD (data out)
  ////////////////////////  VGA         ////////////////////////////
  output            vga_hs,                 //  VGA H_SYNC
  output            vga_vs,                 //  VGA V_SYNC
  output    [3:0]   vga_r,                  //  VGA Red[3:0]
  output    [3:0]   vga_g,                  //  VGA Green[3:0]
  output    [3:0]   vga_b,                  //  VGA Blue[3:0]
  ////////////////////  Audio CODEC     ////////////////////////////
  inout             aud_adclrck,            //  Audio CODEC ADC LR Clock
  input             aud_adcdat,             //  Audio CODEC ADC Data
  inout             aud_daclrck,            //  Audio CODEC DAC LR Clock
  output            aud_dacdat,             //  Audio CODEC DAC Data
  inout             aud_bclk,               //  Audio CODEC Bit-Stream Clock
  output            aud_xck,                //  Audio CODEC Chip Clock
  ////////////////////////  GPIO    ////////////////////////////////
  inout [35:0]      gpio_0,                 //  GPIO Connection 0
  inout [35:0]      gpio_1                  //  GPIO Connection 1
);

	parameter DW 	= 32; 
	parameter AW 	= 32; 


  //---------------------------------------------------
  // system wires
	wire 				sys_rst;
	wire        sys_clk = clock_24[0];
	
	
//   //---------------------------------------------------
//   // sync reset
//   sync 
//     i_sync_reset( 
//             .async_sig(~key[0]), 
//             .sync_out(sys_rst), 
//             .clk(sys_clk) 
//           );
// 	

  //---------------------------------------------------
  // FLED
	reg [24:0] counter;
	wire [7:0]  fled;

	always @(posedge sys_clk or posedge sys_rst)
	  if(sys_rst)
  		counter <= 25'b0;
  	else
  		counter <= counter + 1;
  
	assign fled[0]  = sw[0];
	assign fled[1]  = sw[1];
	assign fled[2]  = sw[2];
	assign fled[3]  = sw[3];
	assign fled[4]  = sw[4];
	assign fled[5]  = sw[5];
	assign fled[6]  = sw[6];
	assign fled[7]  = counter[24];
	
	
// --------------------------------------------------------------------
//  wb_async_mem_bridge
  wire [31:0] m0_data_i;
  wire [31:0] m0_data_o;
  wire [31:0] m0_addr_o;
  wire [3:0]  m0_sel_o;
  wire        m0_we_o;
  wire        m0_cyc_o;
  wire        m0_stb_o;
  wire        m0_ack_i;
  wire        m0_err_i;
  wire        m0_rty_i;
  
  wb_async_mem_bridge #( .AW(24) )
    i_wb_async_mem_bridge(
      .wb_data_i(m0_data_i),
      .wb_data_o(m0_data_o),
      .wb_addr_o(m0_addr_o[23:0]),
      .wb_sel_o(m0_sel_o),
      .wb_we_o(m0_we_o),
      .wb_cyc_o(m0_cyc_o),
      .wb_stb_o(m0_stb_o),
      .wb_ack_i(m0_ack_i),
      .wb_err_i(m0_err_i),
      .wb_rty_i(m0_rty_i),
      
      .mem_d( gpio_1[31:0] ),
      .mem_a( gpio_0[23:0] ),
      .mem_oe_n( gpio_0[30] ),
      .mem_bls_n( { gpio_0[26], gpio_0[27], gpio_0[28], gpio_0[29] } ),
      .mem_we_n( gpio_0[25] ),
      .mem_cs_n( gpio_0[24] ),
      
      .wb_clk_i(sys_clk),
      .wb_rst_i(sys_rst)
    );
  
  
  //---------------------------------------------------
  // wb_conmax_top

  // Slave 0 Interface

  wire  [DW-1:0]  s0_data_i;
  wire  [DW-1:0]  s0_data_o;
  wire  [AW-1:0]  s0_addr_o;
  wire  [3:0]     s0_sel_o;
  wire            s0_we_o;
  wire            s0_cyc_o;
  wire            s0_stb_o;
  wire            s0_ack_i;
  wire            s0_err_i;
  wire            s0_rty_i;

  wire  [DW-1:0]  s1_data_i;
  wire  [DW-1:0]  s1_data_o;
  wire  [AW-1:0]  s1_addr_o;
  wire  [3:0]     s1_sel_o;
  wire            s1_we_o;
  wire            s1_cyc_o;
  wire            s1_stb_o;
  wire            s1_ack_i;
  wire            s1_err_i;
  wire            s1_rty_i;

  wire  [DW-1:0]  s2_data_i;
  wire  [DW-1:0]  s2_data_o;
  wire  [AW-1:0]  s2_addr_o;
  wire  [3:0]     s2_sel_o;
  wire            s2_we_o;
  wire            s2_cyc_o;
  wire            s2_stb_o;
  wire            s2_ack_i;
  wire            s2_err_i;
  wire            s2_rty_i;

  wire  [DW-1:0]  s3_data_i;
  wire  [DW-1:0]  s3_data_o;
  wire  [AW-1:0]  s3_addr_o;
  wire  [3:0]     s3_sel_o;
  wire            s3_we_o;
  wire            s3_cyc_o;
  wire            s3_stb_o;
  wire            s3_ack_i;
  wire            s3_err_i;
  wire            s3_rty_i;

  wire  [DW-1:0]  s4_data_i;
  wire  [DW-1:0]  s4_data_o;
  wire  [AW-1:0]  s4_addr_o;
  wire  [3:0]     s4_sel_o;
  wire            s4_we_o;
  wire            s4_cyc_o;
  wire            s4_stb_o;
  wire            s4_ack_i;
  wire            s4_err_i;
  wire            s4_rty_i;

  wire  [DW-1:0]  s5_data_i;
  wire  [DW-1:0]  s5_data_o;
  wire  [AW-1:0]  s5_addr_o;
  wire  [3:0]     s5_sel_o;
  wire            s5_we_o;
  wire            s5_cyc_o;
  wire            s5_stb_o;
  wire            s5_ack_i;
  wire            s5_err_i;
  wire            s5_rty_i;

  wire  [DW-1:0]  s6_data_i;
  wire  [DW-1:0]  s6_data_o;
  wire  [AW-1:0]  s6_addr_o;
  wire  [3:0]     s6_sel_o;
  wire            s6_we_o;
  wire            s6_cyc_o;
  wire            s6_stb_o;
  wire            s6_ack_i;
  wire            s6_err_i;
  wire            s6_rty_i;
      
  wb_conmax_top
    i_wb_conmax_top(
      // Master 0 Interface
      .m0_data_i(m0_data_o),
      .m0_data_o(m0_data_i),
      .m0_addr_i( {m0_addr_o[23:20], 8'b0, m0_addr_o[19:0]} ),
      .m0_sel_i(m0_sel_o),
      .m0_we_i(m0_we_o),
      .m0_cyc_i(m0_cyc_o),
      .m0_stb_i(m0_stb_o),
      .m0_ack_o(m0_ack_i),
      .m0_err_o(m0_err_i),
      .m0_rty_o(m0_rty_i),
      // Master 1 Interface 
      .m1_data_i(32'h0000_0000),
      .m1_addr_i(32'h0000_0000),
      .m1_sel_i(4'h0),
      .m1_we_i(1'b0),
      .m1_cyc_i(1'b0),
      .m1_stb_i(1'b0),
      // Master 2 Interface
      .m2_data_i(32'h0000_0000),
      .m2_addr_i(32'h0000_0000),
      .m2_sel_i(4'h0),
      .m2_we_i(1'b0),
      .m2_cyc_i(1'b0),
      .m2_stb_i(1'b0),
      // Master 3 Interface
      .m3_data_i(32'h0000_0000),
      .m3_addr_i(32'h0000_0000),
      .m3_sel_i(4'h0),
      .m3_we_i(1'b0),
      .m3_cyc_i(1'b0),
      .m3_stb_i(1'b0),
      // Master 4 Interface
      .m4_data_i(32'h0000_0000),
      .m4_addr_i(32'h0000_0000),
      .m4_sel_i(4'h0),
      .m4_we_i(1'b0),
      .m4_cyc_i(1'b0),
      .m4_stb_i(1'b0),
      // Master 5 Interface
      .m5_data_i(32'h0000_0000),
      .m5_addr_i(32'h0000_0000),
      .m5_sel_i(4'h0),
      .m5_we_i(1'b0),
      .m5_cyc_i(1'b0),
      .m5_stb_i(1'b0),
      // Master 6 Interface
      .m6_data_i(32'h0000_0000),
      .m6_addr_i(32'h0000_0000),
      .m6_sel_i(4'h0),
      .m6_we_i(1'b0),
      .m6_cyc_i(1'b0),
      .m6_stb_i(1'b0),
      // Master 7 Interface
      .m7_data_i(32'h0000_0000),
      .m7_addr_i(32'h0000_0000),
      .m7_sel_i(4'h0),
      .m7_we_i(1'b0),
      .m7_cyc_i(1'b0),
      .m7_stb_i(1'b0),
      
      // Slave 0 Interface
      .s0_data_i(s0_data_i),
      .s0_data_o(s0_data_o),
      .s0_addr_o(s0_addr_o),
      .s0_sel_o(s0_sel_o),
      .s0_we_o(s0_we_o),
      .s0_cyc_o(s0_cyc_o),
      .s0_stb_o(s0_stb_o),
      .s0_ack_i(s0_ack_i),
      .s0_err_i(s0_err_i),
      .s0_rty_i(s0_rty_i),
      // Slave 1 Interface
      .s1_data_i(s1_data_i),
      .s1_data_o(s1_data_o),
      .s1_addr_o(s1_addr_o),
      .s1_sel_o(s1_sel_o),
      .s1_we_o(s1_we_o),
      .s1_cyc_o(s1_cyc_o),
      .s1_stb_o(s1_stb_o),
      .s1_ack_i(s1_ack_i),
      .s1_err_i(s1_err_i),
      .s1_rty_i(s1_rty_i),
      // Slave 2 Interface
      .s2_data_i(s2_data_i),
      .s2_data_o(s2_data_o),
      .s2_addr_o(s2_addr_o),
      .s2_sel_o(s2_sel_o),
      .s2_we_o(s2_we_o),
      .s2_cyc_o(s2_cyc_o),
      .s2_stb_o(s2_stb_o),
      .s2_ack_i(s2_ack_i),
      .s2_err_i(s2_err_i),
      .s2_rty_i(s2_rty_i),
      // Slave 3 Interface
      .s3_data_i(s3_data_i),
      .s3_data_o(s3_data_o),
      .s3_addr_o(s3_addr_o),
      .s3_sel_o(s3_sel_o),
      .s3_we_o(s3_we_o),
      .s3_cyc_o(s3_cyc_o),
      .s3_stb_o(s3_stb_o),
      .s3_ack_i(s3_ack_i),
      .s3_err_i(s3_err_i),
      .s3_rty_i(s3_rty_i),
      // Slave 4 Interface
      .s4_data_i(32'h0000_0000),
      .s4_ack_i(1'b0),
      .s4_err_i(1'b0),
      .s4_rty_i(1'b0),
      // Slave 5 Interface
      .s5_data_i(32'h0000_0000),
      .s5_ack_i(1'b0),
      .s5_err_i(1'b0),
      .s5_rty_i(1'b0),
      // Slave 6 Interface
      .s6_data_i(32'h0000_0000),
      .s6_ack_i(1'b0),
      .s6_err_i(1'b0),
      .s6_rty_i(1'b0),
      // Slave 7 Interface
      .s7_data_i(32'h0000_0000),
      .s7_ack_i(1'b0),
      .s7_err_i(1'b0),
      .s7_rty_i(1'b0),
      // Slave 8 Interface
      .s8_data_i(32'h0000_0000),
      .s8_ack_i(1'b0),
      .s8_err_i(1'b0),
      .s8_rty_i(1'b0),
      // Slave 9 Interface
      .s9_data_i(32'h0000_0000),
      .s9_ack_i(1'b0),
      .s9_err_i(1'b0),
      .s9_rty_i(1'b0),
      // Slave 10 Interface
      .s10_data_i(32'h0000_0000),
      .s10_ack_i(1'b0),
      .s10_err_i(1'b0),
      .s10_rty_i(1'b0),
      // Slave 11 Interface
      .s11_data_i(32'h0000_0000),
      .s11_ack_i(1'b0),
      .s11_err_i(1'b0),
      .s11_rty_i(1'b0),
      // Slave 12 Interface
      .s12_data_i(32'h0000_0000),
      .s12_ack_i(1'b0),
      .s12_err_i(1'b0),
      .s12_rty_i(1'b0),
      // Slave 13 Interface
      .s13_data_i(32'h0000_0000),
      .s13_ack_i(1'b0),
      .s13_err_i(1'b0),
      .s13_rty_i(1'b0),
      // Slave 14 Interface
      .s14_data_i(32'h0000_0000),
      .s14_ack_i(1'b0),
      .s14_err_i(1'b0),
      .s14_rty_i(1'b0),
      // Slave 15 Interface
      .s15_data_i(32'h0000_0000),
      .s15_ack_i(1'b0),
      .s15_err_i(1'b0),
      .s15_rty_i(1'b0),
      
      .clk_i(sys_clk),
      .rst_i(sys_rst)
    );	
    
    
  //---------------------------------------------------
  // async_mem_if
  assign s0_err_i = 1'b0;
  assign s0_rty_i = 1'b0;
  
  async_mem_if #( .AW(18), .DW(16) )
    i_sram (
      .async_dq(sram_dq),    
      .async_addr(sram_addr),  
      .async_ub_n(sram_ub_n),  
      .async_lb_n(sram_lb_n),  
      .async_we_n(sram_we_n),  
      .async_ce_n(sram_ce_n),  
      .async_oe_n(sram_oe_n),  
      .wb_clk_i(sys_clk),   
      .wb_rst_i(sys_rst),
      .wb_adr_i( {14'h0000, s0_addr_o[17:0]} ),
      .wb_dat_i(s0_data_o),
      .wb_we_i(s0_we_o),
      .wb_stb_i(s0_stb_o),
      .wb_cyc_i(s0_cyc_o),
      .wb_sel_i(s0_sel_o),
      .wb_dat_o(s0_data_i),
      .wb_ack_o(s0_ack_i),
      .ce_setup(4'h0), 
      .op_hold(4'h1), 
      .ce_hold(4'h0),
      .big_endian_if_i(1'b0),
      .lo_byte_if_i(1'b0)
    );
              

  //---------------------------------------------------
  // GPIO a
  assign s1_rty_i = 1'b0;
  
  wire        gpio_a_inta_o;
  wire        gpio_a_clk_i;
  wire [31:0] gpio_a_aux_i;
  wire [31:0] gpio_a_ext_pad_i;
  wire [31:0] gpio_a_ext_pad_o;
  wire [31:0] gpio_a_ext_padoe_o;
  
  gpio_top
    i_gpio_a(
  	          .wb_clk_i(sys_clk),
  	          .wb_rst_i(sys_rst),
  	          .wb_cyc_i(s1_cyc_o),
  	          .wb_adr_i( s1_addr_o[7:0] ),
  	          .wb_dat_i(s1_data_o),
  	          .wb_sel_i(s1_sel_o),
  	          .wb_we_i(s1_we_o),
  	          .wb_stb_i(s1_stb_o),
  	          .wb_dat_o(s1_data_i),
  	          .wb_ack_o(s1_ack_i),
  	          .wb_err_o(s1_err_i),
  	          .wb_inta_o(gpio_a_inta_o),
  	          
`ifdef GPIO_AUX_IMPLEMENT
  	          .aux_i(gpio_a_aux_i),
`endif // GPIO_AUX_IMPLEMENT
  	          
`ifdef GPIO_CLKPAD
              .clk_pad_i(gpio_a_clk_i),
`endif //  GPIO_CLKPAD
              
  	          .ext_pad_i(gpio_a_ext_pad_i),
  	          .ext_pad_o(gpio_a_ext_pad_o),
  	          .ext_padoe_o(gpio_a_ext_padoe_o)
            );
    
    	
  //---------------------------------------------------
  // GPIO b
  assign s2_rty_i = 1'b0;
  
  wire        gpio_b_inta_o;
  wire        gpio_b_clk_i;
  wire [31:0] gpio_b_aux_i;
  wire [31:0] gpio_b_ext_pad_i;
  wire [31:0] gpio_b_ext_pad_o;
  wire [31:0] gpio_b_ext_padoe_o;
  
  gpio_top
    i_gpio_b(
  	          .wb_clk_i(sys_clk),
  	          .wb_rst_i(sys_rst),
  	          .wb_cyc_i(s2_cyc_o),
  	          .wb_adr_i( s2_addr_o[7:0] ),
  	          .wb_dat_i(s2_data_o),
  	          .wb_sel_i(s2_sel_o),
  	          .wb_we_i(s2_we_o),
  	          .wb_stb_i(s2_stb_o),
  	          .wb_dat_o(s2_data_i),
  	          .wb_ack_o(s2_ack_i),
  	          .wb_err_o(s2_err_i),
  	          .wb_inta_o(gpio_b_inta_o),
  	          
`ifdef GPIO_AUX_IMPLEMENT
  	          .aux_i(gpio_b_aux_i),
`endif // GPIO_AUX_IMPLEMENT
  	          
`ifdef GPIO_CLKPAD
              .clk_pad_i(gpio_b_clk_i),
`endif //  GPIO_CLKPAD
              
  	          .ext_pad_i(gpio_b_ext_pad_i),
  	          .ext_pad_o(gpio_b_ext_pad_o),
  	          .ext_padoe_o(gpio_b_ext_padoe_o)
            );
            
    
  //---------------------------------------------------
  // GPIO b
  qaz_system
    i_qaz_system(
                    .sys_data_i(s3_data_o),
                    .sys_data_o(s3_data_i),
                    .sys_addr_i(s3_addr_o),
                    .sys_sel_i(s3_sel_o),
                    .sys_we_i(s3_we_o),
                    .sys_cyc_i(s3_cyc_o),
                    .sys_stb_i(s3_stb_o),
                    .sys_ack_o(s3_ack_i),
                    .sys_err_o(s3_err_i),
                    .sys_rty_o(s3_rty_i),
                    
                    .async_rst_i(~key[0]),
                    
                    .hex0(gpio_a_aux_i[6:0]),
                    .hex1(gpio_a_aux_i[14:8]),
                    .hex2(gpio_a_aux_i[22:16]),
                    .hex3(gpio_a_aux_i[30:24]),
    
                    .sys_clk_i(sys_clk), 
                    .sys_rst_o(sys_rst)
                  );
      
  //---------------------------------------------------
  // IO pads
  genvar i;
  
  // gpio a
  wire [31:0] gpio_a_io_buffer_o;
  
  generate for( i = 0; i < 32; i = i + 1 )
    begin: gpio_a_pads
      assign gpio_a_io_buffer_o[i] = gpio_a_ext_padoe_o[i] ? gpio_a_ext_pad_o[i] : 1'bz;
    end  
  endgenerate  

  // gpio b
  wire [31:0] gpio_b_io_buffer_o;
  
  generate for( i = 0; i < 32; i = i + 1 )
    begin: gpio_b_pads
      assign gpio_b_io_buffer_o[i] = gpio_b_ext_padoe_o[i] ? gpio_b_ext_pad_o[i] : 1'bz;
    end  
  endgenerate  
  
      	
  //---------------------------------------------------
  // outputs
  
  //  Turn off all display
//   assign  hex0        =   7'h7f;
//   assign  hex1        =   7'h7f;
//   assign  hex2        =   7'h7f;
//   assign  hex3        =   7'h7f;
//   assign  ledg        =   8'hff;
//   assign  ledg        =   fled;
//   assign  ledr        =   10'h000;
  
  //  All inout port turn to tri-state
  assign  dram_dq     =   16'hzzzz;
  assign  fl_dq       =   8'hzz;
//   assign  sram_dq     =   16'hzzzz;
  assign  sd_dat      =   1'bz;
  assign  i2c_sdat    =   1'bz;
  assign  aud_adclrck =   1'bz;
  assign  aud_daclrck =   1'bz;
  assign  aud_bclk    =   1'bz;
//   assign  gpio_0      =   36'hzzzzzzzzz;
//   assign  gpio_1      =   36'hzzzzzzzzz;
  
  assign hex0             = gpio_a_io_buffer_o[6:0];
  assign hex1             = gpio_a_io_buffer_o[14:8];
  assign hex2             = gpio_a_io_buffer_o[22:16];
  assign hex3             = gpio_a_io_buffer_o[30:24];
  assign gpio_a_aux_i[7]  = 1'b0;
  assign gpio_a_aux_i[15] = 1'b0;
  assign gpio_a_aux_i[23] = 1'b0;
  assign gpio_a_aux_i[31] = 1'b0;
  assign gpio_a_ext_pad_i = 32'b0;
  
  assign ledg             = gpio_b_io_buffer_o[7:0];
  assign ledr             = gpio_b_io_buffer_o[17:8];
  assign gpio_b_aux_i     = { 24'b0, fled } ;
  assign gpio_b_ext_pad_i = { key, sw, 18'b0 };
  
  assign gpio_1[35]       = ~gpio_b_inta_o;
  
endmodule

