`timescale 1ns/1ps
// Isolated product transaction engine; not a relaxation of frozen diagnostic master.
module d2e_mdio_transaction #(
 parameter integer CLOCK_HZ=25000000,MDC_HZ=500000
)(
 input wire clk,rst_n,request,write_selector,write_config,config_authorized,
 input wire[15:0]config_before,
 input wire[4:0]phy_address,register_address,
 input wire[15:0]selector_address,
 input wire mdio_i,
 output wire mdio_o,output wire mdio_oe,output reg mdc,
 output reg busy,done,acknowledged,blocked_write,
 output reg[15:0]read_data
);
 localparam integer HALF_DIV=(CLOCK_HZ+2*MDC_HZ-1)/(2*MDC_HZ);
 localparam integer DW=HALF_DIV<2?1:$clog2(HALF_DIV);
 reg[DW-1:0]divider;
 reg[5:0]bit_index;
 reg[4:0]phy_q,reg_q;
 reg[15:0]selector_q,rx_shift;
 reg write_q,ta_good;
 reg selector_context_valid;reg[15:0]last_selector;reg[4:0]last_phy;
 wire write_requested=write_selector||write_config;
 (* async_reg="true" *)reg mdio_s1,mdio_s2;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin mdio_s1<=1;mdio_s2<=1;end
  else begin mdio_s1<=mdio_i;mdio_s2<=mdio_s1;end
 reg outgoing_bit;
 always @*begin
  outgoing_bit=1;
  if(bit_index==32)outgoing_bit=0;
  else if(bit_index==33)outgoing_bit=1;
  else if(bit_index==34)outgoing_bit=!write_q;
  else if(bit_index==35)outgoing_bit=write_q;
  else if(bit_index>=36&&bit_index<=40)outgoing_bit=phy_q[40-bit_index];
  else if(bit_index>=41&&bit_index<=45)outgoing_bit=reg_q[45-bit_index];
  else if(write_q&&bit_index==47)outgoing_bit=0;
  else if(write_q&&bit_index>=48)outgoing_bit=selector_q[63-bit_index];
 end
 assign mdio_o=1'b0;
 assign mdio_oe=busy&&!outgoing_bit&&(write_q||bit_index<46);
 wire selector_allowed=write_selector&&!write_config&&register_address==5'h1e&&
   (selector_address==16'ha000||selector_address==16'ha001||selector_address==16'ha003);
 wire config_allowed=write_config&&!write_selector&&config_authorized&&register_address==5'h1f&&
  selector_context_valid&&last_selector==16'ha003&&last_phy==phy_address&&
  selector_address[3:0]==4'hd&&selector_address[15:4]==config_before[15:4];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   divider<=0;bit_index<=0;phy_q<=0;reg_q<=0;selector_q<=0;
   selector_context_valid<=0;last_selector<=0;last_phy<=0;
   rx_shift<=0;write_q<=0;ta_good<=0;mdc<=0;busy<=0;
   done<=0;acknowledged<=0;blocked_write<=0;read_data<=0;
  end else begin
   done<=0;blocked_write<=0;
   if(!busy)begin
    mdc<=0;divider<=0;
    if(request)begin
     acknowledged<=0;
     if(write_requested&&!(selector_allowed||config_allowed))begin done<=1;blocked_write<=1;end
     else begin
      phy_q<=phy_address;reg_q<=register_address;selector_q<=selector_address;
      write_q<=write_requested;bit_index<=0;rx_shift<=0;ta_good<=0;busy<=1;
     end
    end
   end else if(divider==HALF_DIV-1)begin
    divider<=0;
    if(!mdc)begin
     mdc<=1;
     if(!write_q&&bit_index==47)ta_good<=!mdio_s2;
     if(!write_q&&bit_index>=48)rx_shift<={rx_shift[14:0],mdio_s2};
    end else begin
     mdc<=0;
     if(bit_index==63)begin
      busy<=0;done<=1;read_data<=rx_shift;
      // Clause22 writes have no acknowledgement phase. For selector writes
      // this flag denotes completed emission, not proof of PHY acceptance.
      acknowledged<=write_q||ta_good;
      if(write_q&&reg_q==5'h1e)begin selector_context_valid<=1;last_selector<=selector_q;last_phy<=phy_q;end
     end else bit_index<=bit_index+1'b1;
    end
   end else divider<=divider+1'b1;
  end
 end
endmodule
