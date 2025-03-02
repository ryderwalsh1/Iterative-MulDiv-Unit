//========================================================================
// Lab 1 - Iterative Div Unit
//========================================================================

`ifndef RISCV_INT_DIV_ITERATIVE_V
`define RISCV_INT_DIV_ITERATIVE_V

`include "imuldiv-DivReqMsg.v"

module imuldiv_IntDivIterative
(

  input         clk,
  input         reset,

  input         divreq_msg_fn,
  input  [31:0] divreq_msg_a,
  input  [31:0] divreq_msg_b,
  input         divreq_val,
  output        divreq_rdy,

  output [63:0] divresp_msg_result,
  output        divresp_val,
  input         divresp_rdy
);

// declare interconnect wires
wire        a_mux_sel;
wire        a_en;
wire        b_en;
wire        fn_en;
wire        cntr_mux_sel;
wire        sign_en;
wire        is_op_signed;
wire        rem_sign_mux_sel;
wire        div_sign_mux_sel;
wire        sub_mux_sel;
wire  [4:0] counter;
wire        sub_out64;
wire        div_sign;
wire        rem_sign;
wire        fn;

  imuldiv_IntDivIterativeDpath dpath
  (
    .clk                (clk),
    .reset              (reset),
    .divreq_msg_fn      (divreq_msg_fn),
    .divreq_msg_a       (divreq_msg_a),
    .divreq_msg_b       (divreq_msg_b),
    .divresp_msg_result (divresp_msg_result),

    .a_mux_sel          (a_mux_sel),
    .a_en               (a_en),
    .b_en               (b_en),
    .fn_en              (fn_en),
    .cntr_mux_sel       (cntr_mux_sel),
    .sign_en            (sign_en),
    .is_op_signed       (is_op_signed),
    .rem_sign_mux_sel   (rem_sign_mux_sel),
    .div_sign_mux_sel   (div_sign_mux_sel),
    .sub_mux_sel        (sub_mux_sel),

    .counter            (counter),
    .sub_out64          (sub_out64),
    .div_sign           (div_sign),
    .rem_sign           (rem_sign),
    .fn                 (fn)
  );

  imuldiv_IntDivIterativeCtrl ctrl
  (
    .clk                (clk),
    .reset              (reset),

    .divreq_val         (divreq_val),
    .divreq_rdy         (divreq_rdy),
    .divresp_val        (divresp_val),
    .divresp_rdy        (divresp_rdy),

    .counter            (counter),
    .sub_out64          (sub_out64),
    .div_sign           (div_sign),
    .rem_sign           (rem_sign),
    .fn                 (fn),

    .a_mux_sel          (a_mux_sel),
    .a_en               (a_en),
    .b_en               (b_en),
    .fn_en              (fn_en),
    .cntr_mux_sel       (cntr_mux_sel),
    .sign_en            (sign_en),
    .is_op_signed       (is_op_signed),
    .rem_sign_mux_sel   (rem_sign_mux_sel),
    .div_sign_mux_sel   (div_sign_mux_sel),
    .sub_mux_sel        (sub_mux_sel)
  );

endmodule

//------------------------------------------------------------------------
// Datapath
//------------------------------------------------------------------------

module imuldiv_IntDivIterativeDpath
(
  input         clk,
  input         reset,

  input         divreq_msg_fn,      // Function of MulDiv Unit
  input  [31:0] divreq_msg_a,       // Operand A
  input  [31:0] divreq_msg_b,       // Operand B

  output [63:0] divresp_msg_result, // Result of operation

  // inputs from control
  input         a_mux_sel,
  input         a_en,
  input         b_en,
  input         fn_en,
  input         cntr_mux_sel,
  input         sign_en,
  input         is_op_signed,
  input         rem_sign_mux_sel,
  input         div_sign_mux_sel,
  input         sub_mux_sel,

  // outputs to control
  output  [4:0] counter,
  output        sub_out64,
  output        div_sign,
  output        rem_sign,
  output        fn

);

  //----------------------------------------------------------------------
  // Sequential Logic
  //----------------------------------------------------------------------

  reg         fn_reg;       // Register for storing function
  reg  [64:0] a_reg;        // Register for storing operand A
  reg  [64:0] b_reg;        // Register for storing operand B
  reg  [4:0]  counter_reg;  // Register for storing counter
  reg         div_sign_reg; // Register for storing quotient sign
  reg         rem_sign_reg; // Register for storing remainder sign

  always @( posedge clk ) begin
    if (reset) begin
    a_reg <= 0;
    b_reg <= 0;
    counter_reg <= 5'd31;
    div_sign_reg <= 0;
    rem_sign_reg <= 0;
    fn_reg <= 0;
    end
    if (a_en) begin
      a_reg <= a_mux;
    end
    if (b_en) begin
      b_reg <= {1'b0, b_handled_sign, 32'b0};
    end
    counter_reg <= counter_mux;
    if (sign_en) begin
      div_sign_reg <= divreq_msg_a[31] ^ divreq_msg_b[31];
      rem_sign_reg <= divreq_msg_a[31];
    end
    if (fn_en) begin
      fn_reg <= divreq_msg_fn;
    end
  end

  //----------------------------------------------------------------------
  // Combinational Logic
  //----------------------------------------------------------------------

  // declare muxes & wires
  wire [64:0] a_mux;
  wire [31:0] rem_sign_mux;
  wire [31:0] div_sign_mux;
  wire [64:0] sub_mux;
  wire [4:0]  counter_mux;
  wire [64:0] a_shift_out;
  wire [64:0] sub_out;
  wire sign_bit_a;
  wire sign_bit_b;
  wire [31:0] unsigned_a;
  wire [31:0] unsigned_b;
  wire [31:0] a_handled_sign;
  wire [31:0] b_handled_sign;
  
  
  // Extract sign bits

  assign sign_bit_a = divreq_msg_a[31];
  assign sign_bit_b = divreq_msg_b[31];

  // Unsign operands if necessary

  assign unsigned_a = ( sign_bit_a ) ? (~divreq_msg_a + 1'b1) : divreq_msg_a;
  assign unsigned_b = ( sign_bit_b ) ? (~divreq_msg_b + 1'b1) : divreq_msg_b;
  assign a_handled_sign = (is_op_signed) ? unsigned_a : divreq_msg_a;
  assign b_handled_sign = (is_op_signed) ? unsigned_b : divreq_msg_b;

  // Computation logic
  assign a_shift_out = a_reg << 1;
  assign sub_out = a_shift_out - b_reg;
  assign counter = counter_reg;
  assign sub_out64 = sub_out[64];
  assign div_sign = div_sign_reg;
  assign rem_sign = rem_sign_reg;
  assign divresp_msg_result = {rem_sign_mux, div_sign_mux};
  assign fn = fn_reg;

  // mux logic
  // a_mux
  assign a_mux = (a_mux_sel) ? sub_mux : {33'b0, a_handled_sign};
  // remainder sign mux
  assign rem_sign_mux = (rem_sign_mux_sel) ? (~a_reg[63:32] + 1'b1) : a_reg[63:32];
  // quotient sign mux
  assign div_sign_mux = (div_sign_mux_sel) ? (~a_reg[31:0] + 1'b1) : a_reg[31:0];
  // subtract mux
  assign sub_mux = (sub_mux_sel) ? a_shift_out : {sub_out[64:1], 1'b1};
  // counter mux
  assign counter_mux = (cntr_mux_sel) ? (counter_reg - 1'b1) : 5'd31;



endmodule

//------------------------------------------------------------------------
// Control Logic
//------------------------------------------------------------------------

module imuldiv_IntDivIterativeCtrl
(
  input         clk,
  input         reset,

  input         divreq_val,         // Request val Signal
  output        divreq_rdy,         // Request rdy Signal
  output        divresp_val,        // Response val Signal
  input         divresp_rdy,        // Response rdy Signal

  // inputs from datapath
  input   [4:0] counter,
  input         sub_out64,
  input         div_sign,
  input         rem_sign,
  input         fn,
  // outputs to datapath
  output        a_mux_sel,
  output        a_en,
  output        b_en,
  output        fn_en,
  output        cntr_mux_sel,
  output        sign_en,
  output        is_op_signed,
  output        rem_sign_mux_sel,
  output        div_sign_mux_sel,
  output        sub_mux_sel
);

localparam READY = 0;
localparam COMPUTE = 1;
localparam VALID = 2;

// state, next state registers
reg [1:0] state, next_state;
// mux select bits
reg       a_mux_sel, sub_mux_sel, cntr_mux_sel, rem_sign_mux_sel, div_sign_mux_sel;
// enable bits
reg       a_en, b_en, fn_en, sign_en;

reg       divreq_rdy;
reg       divresp_val;

// output combinational logic
assign is_op_signed = fn;
always @( * ) begin
  // default values here
  a_mux_sel = 0;
  sub_mux_sel = 0;
  cntr_mux_sel = 0;
  rem_sign_mux_sel = 0;
  div_sign_mux_sel = 0;
  a_en = 0;
  b_en = 0;
  fn_en = 0;
  sign_en = 0;
  divreq_rdy = 0;
  divresp_val = 0;

  case (state)
  READY: begin
    // enables high, muxes low
    a_en = 1;
    b_en = 1;
    fn_en = 1;
    sign_en = 1;
    a_mux_sel = 0;
    cntr_mux_sel = 0;
    divreq_rdy = 1;
  end
  COMPUTE: begin
    a_en = 1;
    b_en = 0;
    fn_en = 0;
    sign_en = 0;
    a_mux_sel = 1;
    sub_mux_sel = sub_out64;
    cntr_mux_sel = 1;
  end
  VALID: begin
    divresp_val = 1;
    fn_en = 1;
    // handle signage
    if (is_op_signed) begin
      rem_sign_mux_sel = rem_sign;
      div_sign_mux_sel = div_sign;
    end
    else begin
      rem_sign_mux_sel = 0;
      div_sign_mux_sel = 0;
    end
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
    if (divreq_rdy && divreq_val) begin
      next_state = COMPUTE;
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
    if (divresp_rdy && divresp_val) begin
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
