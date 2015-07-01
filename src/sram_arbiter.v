///////////////////////////////////////////////////////////////////////////////
// Module: sram_arbiter.v
// Project: NF2.1 
// Description: SRAM controller
// Author: 
//
// Provides access to the SRAM for lookups.
///////////////////////////////////////////////////////////////////////////////

`timescale  1ns /  10ps

module sram_arbiter  #(parameter SRAM_ADDR_WIDTH = 19,
                       parameter SRAM_DATA_WIDTH = 36)

   (// register interface
    input                            sram_reg_req,
    input                            sram_reg_rd_wr_L,    // 1 = read, 0 = write
    input [`SRAM_REG_ADDR_WIDTH-1:0] sram_reg_addr,
    input [`CPCI_NF2_DATA_WIDTH-1:0] sram_reg_wr_data,

    output reg                             sram_reg_ack,
    output reg [`CPCI_NF2_DATA_WIDTH -1:0] sram_reg_rd_data,

    // --- Requesters (read and/or write)
    input                            wr_req,
    input      [SRAM_ADDR_WIDTH-1:0] wr_addr,
    input      [SRAM_DATA_WIDTH-1:0] wr_data,
    output reg                       wr_ack,

    input                            rd_req,
    input      [SRAM_ADDR_WIDTH-1:0] rd_addr, 
    output reg [SRAM_DATA_WIDTH-1:0] rd_data,
    output reg                       rd_ack,
    output reg                       rd_vld,

    // --- SRAM signals (pins and control)
    output reg [SRAM_ADDR_WIDTH-1:0]   sram_addr1,
    output reg [SRAM_ADDR_WIDTH-1:0]   sram_addr2,
    output reg                         sram_we,
    output reg [SRAM_DATA_WIDTH/9-1:0] sram_bw,
    output reg [SRAM_DATA_WIDTH-1:0]   sram_wr_data,
    input      [SRAM_DATA_WIDTH-1:0]   sram_rd_data,
    output reg                         sram_tri_en,

    // --- Misc

    input reset,
    input clk

    );

   //----------------------- Localparam ----------------------
   localparam  WR_0           =1;
   localparam  RD_0           =2;
   localparam  WR_1           =3;
   localparam  RD_1           =4;
   localparam  REG_INT        =0;
   /*localparam  WAIT           =6;
   localparam  POSWAIT        =7;*/
   localparam  LEN_STATES     =2;
   //----------------------- Localparam ----------------------


   //------------------ Registers/Wires -----------------
   reg                       rd_vld_early2, rd_vld_early1, rd_vld_early3;
   reg [SRAM_DATA_WIDTH-1:0] sram_wr_data_early2, sram_wr_data_early1;
   reg                       sram_tri_en_early2, sram_tri_en_early1;
   reg                       sram_reg_ack_early3, sram_reg_ack_early2, sram_reg_ack_early1;

   reg                       sram_reg_addr_is_high, sram_reg_addr_is_high_d1, sram_reg_addr_is_high_d2;
   reg                       sram_reg_cntr_read;

   wire [SRAM_DATA_WIDTH-1:0] sram_wr_data_early1_shuffled;
   wire [SRAM_DATA_WIDTH-1:0] sram_rd_data_shuffled;
   wire [SRAM_ADDR_WIDTH-1:0] sram_reg_addr_rev;

   //reg                       rd_wr;

   reg                        do_reset;



   //----------------------- Logic ----------------------
   /* The SRAM considers each byte to be 9bits. So
    * in order to use the sram_bw signals correctly
    * we need to shuffled bits around to match our internal
    * view in the User Data Path that the extra bits
    * are all collected at the MSB of the sram data lines */
   
   always @(posedge clk) begin
      if(reset) begin
         {sram_we, sram_bw}    <= -1;           // active low
         sram_addr1             <= 0;
         sram_addr2             <= 0;
         do_reset              <= 1'b1;
	 // synthesis translate_off
         do_reset              <= 0;
	 // synthesis translate_on
	      //rd_wr                <= 1;
         sram_reg_ack         <= 0;
         rd_vld               <= 1'b0;
      end
      else begin
         if(do_reset) begin
            if(sram_addr1 == {SRAM_ADDR_WIDTH{1'b1}}) begin
               do_reset               <= 0;
               {sram_we, sram_bw}     <= -1;           // active low
               rd_ack               <= 1'b0;
               rd_vld               <= 1'b0;
               wr_ack               <= 1'b0;
            end
            else begin
               //each mem have your own addr
               sram_addr1              <= sram_addr1 + 1'b1;
               sram_addr2              <= sram_addr2 + 1'b1;
               {sram_we, sram_bw}     <= 9'h0;
               sram_wr_data_early2    <= 0;
               sram_tri_en_early2     <= 1;
               do_reset               <= 1;
            end // else: !if(sram_addr == {SRAM_ADDR_WIDTH{1'b1}})
         end // if (do_reset)

         else begin
         //first pipeline stage
            sram_reg_addr_is_high <= sram_reg_addr[0];
            if(sram_reg_req) begin
               sram_addr1 <= sram_reg_addr[19:1];
               sram_addr2 <= sram_reg_addr[19:1];
               sram_wr_data_early2 <= sram_reg_addr[0] ? {12'h0,sram_reg_wr_data,36'h0}:{48'h0,sram_reg_wr_data};
               sram_tri_en_early2 <= !sram_reg_rd_wr_L && sram_reg_req;
               if(!sram_reg_rd_wr_L) begin
                  sram_bw <= sram_reg_addr[0] ? 8'h0f : 8'hf0;
                  sram_we <= 1'b0;
               end
               else begin //leitura
                  sram_bw <= 8'hff;
                  sram_we <= 1'b1;
               end 
               rd_ack <= 0;
               wr_ack <= 0;
               rd_vld_early3 <= 0;
               sram_reg_ack_early3 <= sram_reg_req;
            end
            else if(wr_req) begin
               sram_addr1 <= wr_addr;
               sram_addr2 <= wr_addr;
               //$display("wraddr: %x, dta: %x\n",wr_addr, wr_data);
               //sram_wr_data_early2 <= {4'b0,wr_data[63:32],4'b0,wr_data[31:0]};
               sram_wr_data_early2 <= wr_data; //bf use all 72 bits
               sram_tri_en_early2 <= wr_req;
               sram_we <= 1'b0;
               //sram_bw <= 8'hf0;
               sram_bw <= 8'h00; //wr_req write in two memories
               wr_ack <= 1;
               rd_ack <= 0;
               rd_vld_early3 <= 0;
               sram_reg_ack_early3 <= 0;
            end
            else if(rd_req) begin
               sram_bw <= 8'hff;
               sram_we <= 1'b1;
               sram_addr1 <= rd_addr;
               sram_addr2 <= rd_addr;
               //$display("rdaddr: %x\n",rd_addr);
               sram_tri_en_early2 <= 0;
               rd_vld_early3 <= rd_req;// || rd_1_req;
               rd_ack <= rd_req;// || rd_1_req;
               wr_ack <= 0;
               sram_reg_ack_early3 <= 0;
            end
            else begin
               wr_ack <= 1'b0;
               rd_ack <= 1'b0;
               rd_vld <= 1'b0;
               {sram_we,sram_bw} <= 9'h1ff;
               rd_vld_early3 <= 1'b0;
               sram_tri_en_early2 <= 1'b0;
               sram_wr_data_early2 <= sram_wr_data_early2;
               sram_reg_ack_early3 <= 1'b0;
            end
         end // else: !if(do_reset)
         
         //Second pipeline stage
         sram_tri_en_early1 <= sram_tri_en_early2;
         sram_wr_data_early1 <= sram_wr_data_early2;
         rd_vld_early2 <= rd_vld_early3;
         sram_reg_ack_early2 <= sram_reg_ack_early3;
         sram_reg_addr_is_high_d1 <= sram_reg_addr_is_high;

         //third pipeline stage - Coloca dado e seta tri_en depois de 2 clocks
         sram_tri_en <= sram_tri_en_early1;
         sram_wr_data <= sram_wr_data_early1;
         //rd_data <= {sram_rd_data[71:36],sram_rd_data[31:0]};
         rd_vld_early1 <= rd_vld_early2;
         sram_reg_ack_early1 <= sram_reg_ack_early2;
         sram_reg_addr_is_high_d2 <= sram_reg_addr_is_high_d1;

         //forth pipeline stage - Coloca dado e seta tri_en depois de 2 clocks
         rd_vld <= rd_vld_early1;
         sram_reg_ack <= sram_reg_ack_early1;
         //sram_reg_rd_data <= {sram_rd_data[71:36],sram_rd_data[31:0]};
         sram_reg_rd_data <= sram_reg_addr_is_high_d2 ? sram_rd_data[68:36]:sram_rd_data[31:0];
         //rd_data <= {sram_rd_data[71:36],sram_rd_data[31:0]};
         rd_data <= rd_vld_early1?sram_rd_data:rd_data; //bf use all 72 bits
         /*if(rd_vld_early1)
            $display("Dadoslidos: %x\n",sram_rd_data);*/
      end // else: !if(reset)
   end // always @ (posedge clk)

/************************** Debugging *************************/
   // synthesis translate_off

   // Synthesis code to set the we flag to 0 on startup to remove the annoying
   // "Cypress SRAM stores 'h xxxxxxxxx at addr 'h xxxxx" messages at the
   // beginning of the log file until the clock starts running.
   initial
   begin
      {sram_we, sram_bw} = 9'h1ff;
   end

   // Detect when we write sequential addresses using reg iface, and we skip one
   reg [2:0] seq_state;
   reg [SRAM_ADDR_WIDTH-1:0] prev_addr;
   always @(posedge clk) begin
      if(reset) begin
         seq_state <= 0;
      end
      else begin
         case (seq_state)
            // check when we start a new write
            0: begin
               if(sram_reg_req && !sram_reg_rd_wr_L) begin
                  prev_addr <= sram_reg_addr;
                  seq_state <= 1;
               end
            end

            // wait till req goes low
            1: begin
               if(!sram_reg_req) begin
                  seq_state <= 2;
               end
            end

            // wait for new addr, check if sequential
            2: begin
               if(sram_reg_req && !sram_reg_rd_wr_L) begin
                  // if it is not sequential then we check if we have moved to the next
                  // sequence or if this is a mistake
                  if(sram_reg_addr == prev_addr + 2) begin
                     // Oh oh, we've skipped an address
                     $display("%t %m WARNING: SRAM reg write request skipped an address: %05x.", $time, prev_addr + 1'b1);
                     $stop;
                  end
                  seq_state <= 1;
                  prev_addr <= sram_reg_addr;
               end // if (sram_reg_req && !sram_reg_rd_wr_L)
            end // case: 2
         endcase // case(seq_state)
      end // else: !if(reset)
   end // always @ (posedge clk)

   // synthesis translate_on

endmodule // sram_arbiter


