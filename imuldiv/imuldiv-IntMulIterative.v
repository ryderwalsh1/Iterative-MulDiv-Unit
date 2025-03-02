//========================================================================
// Lab 1 - Iterative Mul Unit
//========================================================================

`ifndef RISCV_INT_MUL_ITERATIVE_V
`define RISCV_INT_MUL_ITERATIVE_V

module imuldiv_IntMulIterative
(
  input                clk,
  input                reset,

  input  [31:0] mulreq_msg_a,
  input  [31:0] mulreq_msg_b,
  input         mulreq_val,
  output        mulreq_rdy,

  output [63:0] mulresp_msg_result,
  output        mulresp_val,
  input         mulresp_rdy
);

// declare interconnect wires
wire a_mux_sel;
wire b_mux_sel;
wire result_mux_sel;
wire result_en;
wire add_mux_sel;
wire cntr_mux_sel;
wire sign_en;
wire sign_mux_sel;
wire b_reg0;
wire [4:0] counter;
wire sign;

  imuldiv_IntMulIterativeDpath dpath
  (
    .clk                (clk),
    .reset              (reset),

    .mulreq_msg_a       (mulreq_msg_a),
    .mulreq_msg_b       (mulreq_msg_b),
    .mulresp_msg_result (mulresp_msg_result),

    .a_mux_sel          (a_mux_sel),
    .b_mux_sel          (b_mux_sel),
    .result_mux_sel     (result_mux_sel),
    .result_en          (result_en),
    .add_mux_sel        (add_mux_sel),
    .cntr_mux_sel       (cntr_mux_sel),
    .sign_en            (sign_en),
    .sign_mux_sel       (sign_mux_sel),

    .b_reg0             (b_reg0),
    .counter            (counter),
    .sign               (sign)
  );

  imuldiv_IntMulIterativeCtrl ctrl
  (
    .clk                (clk),
    .reset              (reset),

    .mulreq_val         (mulreq_val),
    .mulreq_rdy         (mulreq_rdy),
    .mulresp_val        (mulresp_val),
    .mulresp_rdy        (mulresp_rdy),

    .b_reg0             (b_reg0),
    .counter            (counter),
    .sign               (sign),

    .a_mux_sel          (a_mux_sel),
    .b_mux_sel          (b_mux_sel),
    .result_mux_sel     (result_mux_sel),
    .result_en          (result_en),
    .add_mux_sel        (add_mux_sel),
    .cntr_mux_sel       (cntr_mux_sel),
    .sign_en            (sign_en),
    .sign_mux_sel       (sign_mux_sel)
  );

endmodule

//------------------------------------------------------------------------
// Datapath
//------------------------------------------------------------------------

module imuldiv_IntMulIterativeDpath
(
  input         clk,
  input         reset,

  input  [31:0] mulreq_msg_a,       // Operand A
  input  [31:0] mulreq_msg_b,       // Operand B

  output [63:0] mulresp_msg_result, // Result of operation

  // intputs from control
  input         a_mux_sel,
  input         b_mux_sel,
  input         result_mux_sel,
  input         result_en,
  input         add_mux_sel,
  input         cntr_mux_sel,
  input         sign_en,
  input         sign_mux_sel,
  // outputs to control
  output        b_reg0,
  output [4:0]  counter,
  output        sign
);


  //----------------------------------------------------------------------
  // Sequential Logic
  //----------------------------------------------------------------------

  reg  [63:0] a_reg;       // Register for storing operand A
  reg  [31:0] b_reg;       // Register for storing operand B
  reg  [63:0] result_reg;  // Register for storing accumulating result
  reg  [4:0]  counter_reg; // Register for storing counter
  reg         sign_reg;    // Register for storing sign bit

  // synchronous registers
  always @( posedge clk ) begin
    a_reg <= a_mux;
    b_reg <= b_mux;
    counter_reg <= counter_mux;
    result_reg <= result_mux;
    
    if (sign_en) begin
    sign_reg <= sign_bit_a ^ sign_bit_b;
    end
  end


  //----------------------------------------------------------------------
  // Combinational Logic
  //----------------------------------------------------------------------

  // declare muxes & wires
  wire [63:0] a_mux;
  wire [31:0] b_mux;
  wire [63:0] result_mux;
  wire [63:0] add_mux;
  wire [4:0] counter_mux;
  wire [63:0] a_shift_out;
  wire [31:0] b_shift_out;
  wire sign;

  // Extract sign bits

  wire sign_bit_a = mulreq_msg_a[31];
  wire sign_bit_b = mulreq_msg_b[31];

  // Unsign operands if necessary

  wire [31:0] unsigned_a = ( sign_bit_a ) ? (~mulreq_msg_a + 1'b1) : mulreq_msg_a;
  wire [31:0] unsigned_b = ( sign_bit_b ) ? (~mulreq_msg_b + 1'b1) : mulreq_msg_b;

  // Computation logic
  assign a_shift_out = a_reg << 1;
  assign b_shift_out = b_reg >> 1;
  assign b_reg0 = b_reg[0];
  assign counter = counter_reg;
  assign sign = sign_reg;

  // mux logic
  // a_mux
  assign a_mux = (a_mux_sel) ? a_shift_out : {32'b0, unsigned_a};
  // b_mux
  assign b_mux = (b_mux_sel) ? b_shift_out : unsigned_b;
  // result_mux
  assign result_mux = (result_mux_sel) ? add_mux : 64'b0;
  // add mux
  assign add_mux = (add_mux_sel) ? (a_reg + result_reg) : result_reg;
  // counter mux
  assign counter_mux = (cntr_mux_sel) ? (counter_reg - 1) : 5'd31;
  // sign mux
  assign mulresp_msg_result = (sign_mux_sel) ? (~result_reg + 1'b1) : result_reg;

endmodule

//------------------------------------------------------------------------
// Control Logic
//------------------------------------------------------------------------

module imuldiv_IntMulIterativeCtrl
(
    // External Inputs
    input  wire        clk,
    input  wire        reset,

    input         mulreq_val,         // Request val Signal
    output        mulreq_rdy,         // Request rdy Signal
    output        mulresp_val,        // Response val Signal
    input         mulresp_rdy,        // Response rdy Signal

    // inputs from datapath
    input         b_reg0,
    input [4:0]   counter,
    input         sign,
    // outputs to datapath
    output         a_mux_sel,
    output         b_mux_sel,
    output         result_mux_sel,
    output         result_en,
    output         add_mux_sel,
    output         cntr_mux_sel,
    output         sign_en,
    output         sign_mux_sel
);

localparam READY = 0;
localparam COMPUTE = 1;
localparam VALID = 2;

// state, next state registers
reg [1:0] state, next_state;
// mux select bits
reg       a_mux_sel, b_mux_sel, result_mux_sel, add_mux_sel, cntr_mux_sel, sign_mux_sel;

reg       result_en;
reg       sign_en;
reg       mulreq_rdy;
reg       mulresp_val;

// output combinational logic
always @( * ) begin
  // default values here
  a_mux_sel = 0;
  b_mux_sel = 0;
  result_mux_sel = 0;
  result_en = 0;
  add_mux_sel = 0;
  cntr_mux_sel = 0;
  sign_mux_sel = 0;
  sign_en = 0;
  mulreq_rdy = 0;
  mulresp_val = 0;

  case (state)
  READY: begin
    a_mux_sel = 0;
    b_mux_sel = 0;
    result_mux_sel = 0;
    result_en = 0;
    cntr_mux_sel = 0;
    mulreq_rdy = 1;
  end
  COMPUTE: begin
    a_mux_sel = 1;
    b_mux_sel = 1;
    result_mux_sel = 1;
    result_en = b_reg0;
    add_mux_sel = b_reg0;
    cntr_mux_sel = 1;
    sign_en = 0;
    sign_mux_sel = sign;
  end
  VALID: begin
    mulresp_val = 1;
    sign_mux_sel = sign;
  end
  endcase
end

// next state combinational logic
always @( * ) begin
  // default value
  next_state = READY;
  // next state logic
  case (state)
  READY: begin
    if (mulreq_rdy && mulreq_val) begin
      next_state = COMPUTE;
      sign_en = 1; // only on edge from READY to COMPUTE
    end
  end
  COMPUTE: begin
    if (|counter) begin
      next_state = COMPUTE;
    end
    else begin
      next_state = VALID;
    end
  end
  VALID: begin
    if (mulresp_rdy && mulresp_val) begin
      next_state = READY;
    end
  end
  endcase
end

// State Update Sequential Logic
always @(posedge clk) begin
  if (reset) begin
    // Add your initial state here
    state <= READY;
  end
  else begin
    // Add your next state here
    state <= next_state;
  end
end

endmodule

`endif
