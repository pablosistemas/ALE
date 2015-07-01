`timescale  1ns /  10ps

module bloom_filter
   #(
      parameter DATA_WIDTH = 64,
      parameter CTRL_WIDTH = DATA_WIDTH/8,
      parameter SRAM_ADDR_WIDTH = 19, //created
      parameter BITSBUCKET    = 4, //created
      parameter NUMBUCKET  = 12,
      parameter INDEX_LEN = NUMBUCKET*BITSBUCKET,
      parameter UDP_REG_SRC_WIDTH = 2
   )
   (
      input  [DATA_WIDTH-1:0]             in_data,
      input  [CTRL_WIDTH-1:0]             in_ctrl,
      input                               in_wr,
      output                              in_rdy,

      output [DATA_WIDTH-1:0]             out_data,
      output [CTRL_WIDTH-1:0]             out_ctrl,
      output                              out_wr,
      input                               out_rdy,

      output reg                          wr_req,
      output reg [SRAM_ADDR_WIDTH-1:0]    wr_addr,
      output reg [DATA_WIDTH-1:0]         wr_data,
      input                               wr_ack,

      output reg                          rd_req,
      output reg [SRAM_ADDR_WIDTH-1:0]    rd_addr,
      input [DATA_WIDTH-1:0]              rd_data,
      input                               rd_ack,
      input                               rd_vld,

      // --- Register interface
      input                               reg_req_in,
      input                               reg_ack_in,
      input                               reg_rd_wr_L_in,
      input  [`UDP_REG_ADDR_WIDTH-1:0]    reg_addr_in,
      input  [`CPCI_NF2_DATA_WIDTH-1:0]   reg_data_in,
      input  [UDP_REG_SRC_WIDTH-1:0]      reg_src_in,

      output                              reg_req_out,
      output                              reg_ack_out,
      output                              reg_rd_wr_L_out,
      output  [`UDP_REG_ADDR_WIDTH-1:0]   reg_addr_out,
      output  [`CPCI_NF2_DATA_WIDTH-1:0]  reg_data_out,
      output  [UDP_REG_SRC_WIDTH-1:0]     reg_src_out,

      // misc
      input                                reset,
      input                                clk
   );

   /* ----------Estados----------*/
   localparam SHIFT_RD = 0;
   localparam SHIFT_WR = 1;
   localparam LE_MEM1 = 2;
   localparam LE_MEM2 = 3;
   localparam ATUALIZA_BUCKET_ACK = 4;
   localparam ATUALIZA_BUCKET_DATA = 5;
   localparam BUSCA_BUCKET = 6;
   localparam DECREMENTA_BUCKET = 7;
   localparam INCREMENTA_BUCKET = 8;
   localparam ESCREVE_MEM1 = 9;
   localparam ESCREVE_MEM2 = 10;
   localparam RD_ACK = 11;
   localparam WR_ACK = 12;

   localparam NUM_LEITURAS = 4; //tempo entre req e dado
   localparam NUM_BITS_BF = 4;
   localparam NUM_BITS_RES = 48;
   localparam NUM_BF = NUM_BITS_RES/NUM_BITS_BF;
   localparam ADDR_TO_SHIFT = 2**5;
   localparam LIFE_TIME = 320*10**-3;
   localparam CLK = 8*10**-9;
   // Define the log2 function
   `LOG2_FUNC

   function [11:0] buscaff;
      input [71:0] data;
      reg [11:0]  index;
      begin
      index[11] = data[71:71-BITSBUCKET+1]>4'h0;
      index[10] = (~index[11])&data[71-BITSBUCKET:71-2*BITSBUCKET-1]>4'h0;
      index[9] = (~index[10]&~index[11])&data[71-2*BITSBUCKET:71-3*BITSBUCKET-1]>4'h0;
      index[8] = (~index[9]&~index[10]&~index[11])&data[71-3*BITSBUCKET:71-4*BITSBUCKET-1]>4'h0;
      index[7] = (~index[8]&~index[9]&~index[10]&~index[11])&data[71-4*BITSBUCKET:71-5*BITSBUCKET-1]>4'h0;
      index[6] = (~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-5*BITSBUCKET:71-6*BITSBUCKET-1]>4'h0;
      index[5] = (~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-6*BITSBUCKET:71-7*BITSBUCKET-1]>4'h0;
      index[4] = (~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-7*BITSBUCKET:71-8*BITSBUCKET-1]>4'h0;
      index[3] = (~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-8*BITSBUCKET:71-9*BITSBUCKET-1]>4'h0;
      index[2] = (~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-9*BITSBUCKET:71-10*BITSBUCKET-1]>4'h0;
      index[1] = (~index[2]&~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-10*BITSBUCKET:71-11*BITSBUCKET-1]>4'h0;
      index[0] = (~index[1]&~index[2]&~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-11*BITSBUCKET:71-12*BITSBUCKET-1]>4'h0;
      buscaff = index; 
   end
   endfunction

   //fifo ctrl
   wire                    in_fifo_hash_empty;
   wire                    in_fifo_shift_empty;
   wire                    in_fifo_addr_empty;

   reg                     in_fifo_hash_rd_en;
   reg                     in_fifo_shift_rd_en;
   reg                     in_fifo_addr_rd_en;

   wire [2+2*SRAM_ADDR_WIDTH-1:0]             in_fifo_hash_dout;
   wire [72:0]             in_fifo_shift_dout;
   wire [SRAM_ADDR_WIDTH-1:0]             in_fifo_addr_dout;
   wire                    in_fifo_hash_wr;
   wire                    in_wr_shift_fifo;
   wire                    in_fifo_addr_nearly_full;
   wire                    in_fifo_shift_full;
   wire                    in_fifo_hash_nearly_full;
   reg                     in_wr_addr_fifo;

   reg [21:0]              timer;
   reg [7:0]               t_blommf;
   reg                     data_proc, ack_proc;
   reg                     data_proc_next, ack_proc_next;
   reg [3:0]               state, state_next;
   reg [6:0]               index, index_next;
   reg                     rd_req_next,wr_req_next;
   reg [SRAM_ADDR_WIDTH-1:0]  rd_addr_next, wr_addr_next;
   reg [SRAM_ADDR_WIDTH-1:0]  nxt_addr_to_shift_rd;
   reg [SRAM_ADDR_WIDTH-1:0]  nxt_addr_to_shift_rd_next;
   reg [SRAM_ADDR_WIDTH-1:0]  nxt_addr_to_shift_wr;
   reg [SRAM_ADDR_WIDTH-1:0]  nxt_addr_to_shift_wr_next;

   reg [SRAM_ADDR_WIDTH-1:0]  escreve_mem1_addr, escreve_mem1_addr_next;
   reg [SRAM_ADDR_WIDTH-1:0]  escreve_mem2_addr, escreve_mem2_addr_next;

   wire [SRAM_ADDR_WIDTH-1:0]  hash0, hash1;
   reg [DATA_WIDTH-1:0]        wr_data_next;

   reg [INDEX_LEN-1:0]   indice, indice_next;

   reg  [3:0]                 n_shifts;
   reg  [2:0]                 requisicoes, requisicoes_next;
   wire [47:0]                bfilters_fixed; 
   reg  [1:0]                 clk_cntr_next, clk_cntr;

   fallthrough_small_fifo #(.WIDTH(2+2*SRAM_ADDR_WIDTH), .MAX_DEPTH_BITS(3)) in_fifo_hash 
        (.din ({data_pkt,ack_pkt,hash0,hash1}),     // Data in
         .wr_en (in_fifo_hash_wr),               // Write enable
         .rd_en (in_fifo_hash_rd_en),       // Read the next word 
         .dout ({in_fifo_hash_dout}),
         .full (),
         .nearly_full (in_fifo_hash_nearly_full),
         .empty (in_fifo_hash_empty),
         .reset (reset),
         .clk (clk));

   fallthrough_small_fifo #(.WIDTH(73), .MAX_DEPTH_BITS(3)) in_fifo_shift_module 
        (.din ({rd_vld,rd_data}),     // Data in
         .wr_en (in_wr_shift_fifo),               // Write enable
         .rd_en (in_fifo_shift_rd_en),       // Read the next word 
         .dout ({in_fifo_shift_dout}),
         .full (in_fifo_shift_full),
         .nearly_full (),
         .empty (in_fifo_shift_empty),
         .reset (reset),
         .clk (clk));

   simulacao #(
        .DATA_WIDTH(DATA_WIDTH),
        .CTRL_WIDTH(CTRL_WIDTH),
        .UDP_REG_SRC_WIDTH (UDP_REG_SRC_WIDTH),
        .SRAM_ADDR_WIDTH(SRAM_ADDR_WIDTH)
    ) simulacao (
        /*.out_data              (out_data),
        .out_ctrl              (out_ctrl),
        .out_wr                (out_wr),
        .out_rdy               (out_rdy),*/

        .out_data              (in_evt_data),
        .out_ctrl              (in_evt_ctrl),
        .out_wr                (in_evt_wr),
        .out_rdy               (in_evt_rdy),

        .in_data              (in_data),
        .in_ctrl              (in_ctrl),
        .in_wr                (in_wr),
        .in_rdy               (in_rdy),

        .reg_req_in           (reg_req_in),
        .reg_ack_in           (reg_ack_in),
        .reg_rd_wr_L_in       (reg_rd_wr_L_in),
        .reg_addr_in          (reg_addr_in),
        .reg_data_in          (reg_data_in),
        .reg_src_in           (reg_src_in),

        .reg_req_out           (reg_req_out),
        .reg_ack_out           (reg_ack_out),
        .reg_rd_wr_L_out       (reg_rd_wr_L_out),
        .reg_addr_out          (reg_addr_out),
        .reg_data_out          (reg_data_out),
        .reg_src_out           (reg_src_out),

        .hash_0               (hash0),
        .hash_1               (hash1),
        .data_pkt             (data_pkt),
        .ack_pkt              (ack_pkt),
        .data_proc            (data_proc),
        .ack_proc             (ack_proc),

        .clk              (clk),
        .reset            (reset));

   //synthesis translate_off -- criando criador de pkts
/*
   evt_capture_regs
     #(.DATA_WIDTH (DATA_WIDTH),
       .CTRL_WIDTH (CTRL_WIDTH),
       .UDP_REG_SRC_WIDTH (UDP_REG_SRC_WIDTH),
       .NUM_MONITORED_SIGS (NUM_MONITORED_SIGS),
       .NUM_ABS_REG_PAIRS (NUM_ABS_REG_PAIRS),
       .SIGNAL_ID_SIZE (SIGNAL_ID_SIZE),
       .TIMER_RES_SIZE(TIMER_RES_SIZE),
       .HEADER_LENGTH (HEADER_LENGTH),
       .OP_LUT_STAGE_NUM (OP_LUT_STAGE_NUM),
       .EVT_CAPTURE_VERSION(EVT_CAPTURE_VERSION))
     evt_capture_regs
     (
      .reg_req_in       (reg_req_in),
      .reg_ack_in       (reg_ack_in),
      .reg_rd_wr_L_in   (reg_rd_wr_L_in),
      .reg_addr_in      (reg_addr_in),
      .reg_data_in      (reg_data_in),
      .reg_src_in       (reg_src_in),

      .reg_req_out      (reg_req_out),
      .reg_ack_out      (reg_ack_out),
      .reg_rd_wr_L_out  (reg_rd_wr_L_out),
      .reg_addr_out     (reg_addr_out),
      .reg_data_out     (reg_data_out),
      .reg_src_out      (reg_src_out),

      //.send_pkt            (send_pkt),
      .header_data         (header_data[DATA_WIDTH-1:0]),
      .header_ctrl         (header_ctrl[CTRL_WIDTH-1:0]),
      //.enable_events       (enable_events),
      //.reset_timers        (reset_timers),
      //.monitor_mask        (monitor_mask[NUM_MONITORED_SIGS-1:0]),
      //.signal_id_mask                   (signal_id_mask),
      //.tmr_resolution         (tmr_resolution[TIMER_RES_SIZE-1:0]),

      .header_word_number     (header_word_number[HEADER_LENGTH_SIZE-1:0]),
      //.evt_pkt_sent        (evt_pkt_sent),
      //.num_evts_in_pkt        (num_evts_in_pkt[8:0]),
      //.evts_dropped        (evts_dropped[NUM_MON_SIGS_SIZE-1:0]),
      .clk           (clk),
      .reset            (reset));

      .in_data              (in_evt_data),
      .in_ctrl              (in_evt_ctrl),
      .in_wr                (in_evt_wr),
      .in_rdy               (in_evt_rdy),
 
      .out_data              (out_data),
      .out_ctrl              (out_ctrl),
      .out_wr                (out_wr),
      .out_rdy               (out_rdy),
 

*/
   

   //synthesis translate_on -- criando criador de pkts
   
   always @(*) begin
      //if current bf > last update
      if(t_blommf >= in_fifo_shift_dout[23:16]) begin : cyclic_shift
         n_shifts = t_blommf-in_fifo_shift_dout[23:16];
      end
      else begin
         n_shifts = t_blommf+(NUM_BF-1)-in_fifo_shift_dout[23:16];
      end
   end

   assign bfilters_fixed = {in_fifo_shift_dout[71:24]>>(n_shifts*NUM_BITS_BF)};

   assign in_fifo_hash_wr = !in_fifo_hash_nearly_full && (data_pkt||ack_pkt);
   //assign in_fifo_hash_wr = 0;
   assign in_wr_shift_fifo = !in_fifo_shift_full && rd_vld;

   always@(posedge clk) begin
      if(timer >= LIFE_TIME/NUM_BF/CLK) begin 
         timer <= 0;
         t_blommf <= (t_blommf + 1)%NUM_BF;
      end else begin
         timer <= timer+1;
         t_blommf <= t_blommf;
      end
   end

   always@(reset) begin
         timer = 0;
         wr_req = 0;
         rd_req = 0;
         t_blommf = 0;
         state = SHIFT_RD;
         nxt_addr_to_shift_wr = 0;
         nxt_addr_to_shift_rd = 0;
         escreve_mem1_addr = 0;
         escreve_mem2_addr = 0;
         requisicoes = 0;
   end

   always @(*) begin
      in_fifo_hash_rd_en = 0;
      in_fifo_shift_rd_en = 0;

      state_next = state;
      nxt_addr_to_shift_rd_next = nxt_addr_to_shift_rd;
      nxt_addr_to_shift_wr_next = nxt_addr_to_shift_wr;

      rd_req_next = 0;
      rd_addr_next = rd_addr;

      wr_req_next = 0;
      wr_addr_next = wr_addr;
      wr_data_next = wr_data;

      {data_proc_next,ack_proc_next} = in_fifo_hash_dout[21:20];
      
      escreve_mem1_addr_next = escreve_mem1_addr;
      escreve_mem2_addr_next = escreve_mem2_addr;

      indice_next = indice;
      requisicoes_next = requisicoes;

      clk_cntr_next = clk_cntr;

      case(state) 
         RD_ACK: begin
            if(clk_cntr == 3)
               state_next = SHIFT_RD;
            else if(rd_ack) begin
               state_next = SHIFT_RD;
               nxt_addr_to_shift_rd_next=(nxt_addr_to_shift_rd+'h1)%ADDR_TO_SHIFT;
               requisicoes_next = requisicoes + 'h1;
            end else begin
               clk_cntr_next = clk_cntr+'h1;
               state_next = RD_ACK;
            end
         end
         WR_ACK: begin
            if(clk_cntr == 3)
               state_next = SHIFT_WR;
            else if(wr_ack) begin
               state_next = SHIFT_WR;
               nxt_addr_to_shift_wr_next=(nxt_addr_to_shift_wr+'h1)%ADDR_TO_SHIFT;
               requisicoes_next = requisicoes - 'h1;
               in_fifo_shift_rd_en = 1;
            end else begin
               clk_cntr_next = clk_cntr+'h1;
               state_next = WR_ACK;
            end
         end
         SHIFT_RD: begin
            if(!in_fifo_shift_full && requisicoes < 5) begin
               rd_req_next = 1;
               rd_addr_next = nxt_addr_to_shift_rd;
               clk_cntr_next = 'h0;
               state_next = RD_ACK;//SHIFT_RD;
            end
            // se fifo cheia, esvazia fifo
            else begin
               state_next = SHIFT_WR;
            end
            //if fifo full we don't generate new reqs 
         end
         SHIFT_WR: begin
            if(in_fifo_shift_empty && requisicoes > 0)
               state_next = SHIFT_WR;
            /*if(!in_fifo_shift_empty && in_fifo_shift_dout[72] &&*/            
            else if(requisicoes > 0) begin
               wr_req_next = in_fifo_shift_dout[72];
               wr_addr_next = nxt_addr_to_shift_wr;
               wr_data_next = {4'b0,in_fifo_shift_dout[72-1:72-NUM_BITS_RES+NUM_BITS_BF],t_blommf,{nxt_addr_to_shift_wr[15:0]}};
               clk_cntr_next = 'h0;
               state_next = WR_ACK; //SHIFT_WR;
            end else begin
               //$display("Saindo SHIFT_WR: %x\n",nxt_addr_to_shift_wr);
               if(!in_fifo_hash_empty)
                  state_next = LE_MEM1;
               else
                  state_next = SHIFT_RD;
            end
         end
         LE_MEM1 : begin
            $display("LE_MEM1\n");
            indice_next = 48'h0;
            if(!in_fifo_hash_empty) begin
               rd_req_next = 1;
               rd_addr_next = in_fifo_hash_dout[SRAM_ADDR_WIDTH-1:0];
               escreve_mem1_addr_next = in_fifo_hash_dout[SRAM_ADDR_WIDTH-1:0];
               state_next = LE_MEM2;
            end else 
               state_next = SHIFT_RD;
         end
         LE_MEM2 : begin
            $display("LE_MEM2\n");
            in_fifo_hash_rd_en = 1;
            rd_req_next = 1;
            rd_addr_next = in_fifo_hash_dout[2*SRAM_ADDR_WIDTH-1:SRAM_ADDR_WIDTH];
            escreve_mem2_addr_next = in_fifo_hash_dout[2*SRAM_ADDR_WIDTH-1:SRAM_ADDR_WIDTH];

            if(in_fifo_hash_dout[2+2*SRAM_ADDR_WIDTH-1]) begin
               $display("pacote de dados\n");
               state_next = ATUALIZA_BUCKET_DATA;
            end else begin
               $display("pacote de ack\n");
               state_next = ATUALIZA_BUCKET_ACK;
            end
         end
         ATUALIZA_BUCKET_DATA : begin
            $display("ATUALIZA_BUCKET_DATA\n");
            if(!in_fifo_shift_empty && in_fifo_shift_dout[72]) begin
               in_fifo_shift_rd_en = 1;
               $display("Dadosatualizabucket: %x\n",in_fifo_shift_dout[71:0]);
               wr_data_next = {bfilters_fixed,t_blommf,in_fifo_shift_dout[15:0]};
               state_next = INCREMENTA_BUCKET;
            end
            else
               state_next = ATUALIZA_BUCKET_DATA;
         end
         ATUALIZA_BUCKET_ACK : begin
            $display("ATUALIZA_BUCKET_ACK\n");
            if(!in_fifo_shift_empty && in_fifo_shift_dout[72]) begin
               in_fifo_shift_rd_en = 1;
               $display("bfilters: %x\n",bfilters_fixed);
               wr_data_next = {bfilters_fixed,t_blommf,in_fifo_shift_dout[15:0]};
               state_next = BUSCA_BUCKET;
            end
            else
               state_next = ATUALIZA_BUCKET_ACK;
         end
         BUSCA_BUCKET : begin
            $display("BUSCA_BUCKET\n");
            {indice_next[44],indice_next[40],indice_next[36],
            indice_next[32],indice_next[28],indice_next[24],
            indice_next[20],indice_next[16],indice_next[12],
            indice_next[8],indice_next[4],indice_next[0]} = buscaff(wr_data);
            state_next = DECREMENTA_BUCKET;
         end
         DECREMENTA_BUCKET : begin
            $display("DECREMENTA_BUCKET\n");
            wr_data_next = {wr_data[71:24]-indice,wr_data[23:0]};
            state_next = ESCREVE_MEM1;
         end
         INCREMENTA_BUCKET : begin
            $display("INCREMENTA_BUCKET\n");
            wr_data_next = {wr_data[71:68]+4'h1,wr_data[67:0]};
            state_next = ESCREVE_MEM1;
         end
         ESCREVE_MEM1 : begin
            $display("ESCREVE_MEM1: %x\n",wr_data);
            wr_addr_next = escreve_mem1_addr;
            wr_req_next = 1;
            state_next = ESCREVE_MEM2;
         end
         ESCREVE_MEM2 : begin
            $display("ESCREVE_MEM2\n");
            wr_addr_next = escreve_mem2_addr;
            wr_req_next = 1;
            state_next = SHIFT_RD;
         end
         default : begin
            //$display("DEFAULT\n");
            state_next = SHIFT_RD;
         end
      endcase
   end

   always@(posedge clk) begin
      state <= state_next;
      nxt_addr_to_shift_rd <= nxt_addr_to_shift_rd_next;
      nxt_addr_to_shift_wr <= nxt_addr_to_shift_wr_next;

      rd_req <= rd_req_next%ADDR_TO_SHIFT;
      rd_addr <= rd_addr_next;

      data_proc <= data_proc_next;
      ack_proc <= ack_proc_next;

      wr_data <= wr_data_next;
      wr_req <= wr_req_next;
      wr_addr <= wr_addr_next;

      escreve_mem1_addr <= escreve_mem1_addr_next;
      escreve_mem2_addr <= escreve_mem2_addr_next;

      indice <= indice_next;
      requisicoes <= requisicoes_next;
      clk_cntr <= clk_cntr_next;

      //synthesis translate_off
      if(state_next == SHIFT_WR && in_fifo_shift_dout[72])
         $display("dadoemshift: %x\n",in_fifo_shift_dout[71:0]);
      if(state_next == SHIFT_RD)
         $display("SHIFT_RD: %x \n",nxt_addr_to_shift_rd);
      else if (state_next == SHIFT_WR)
         $display("SHIFT_WR: %x, empty: %d, requisicoes: %d\n",
            nxt_addr_to_shift_wr,in_fifo_shift_empty,requisicoes);
      if(state_next == LE_MEM1) begin
         if(in_fifo_hash_dout[SRAM_ADDR_WIDTH-1:0] >= nxt_addr_to_shift_wr)
            $display("endereconaoatualizado: %x\n",in_fifo_hash_dout[SRAM_ADDR_WIDTH-1:0]);
         else
            $display("enderecoatualizado: %x\n",in_fifo_hash_dout[SRAM_ADDR_WIDTH-1:0]);
      end
      if(state_next == LE_MEM2) begin
         if(in_fifo_hash_dout[SRAM_ADDR_WIDTH-1:0] >= nxt_addr_to_shift_wr)
            $display("endereconaoatualizado2: %x\n",in_fifo_hash_dout[2*SRAM_ADDR_WIDTH-1:SRAM_ADDR_WIDTH]);
         else
            $display("enderecoatualizado2: %x\n",in_fifo_hash_dout[2*SRAM_ADDR_WIDTH-1:SRAM_ADDR_WIDTH]);
      end
      if(state_next == DECREMENTA_BUCKET)
         $display("indice: %x", indice_next);
      //synthesis translate_on

   end //always
endmodule
