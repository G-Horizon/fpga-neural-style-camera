`timescale 1ns/1ps
// Startup-only restricted GE TX-delay RMW, then atomic read-only polling.
// There is one MDIO station. Configuration data can only be emitted at step12
// before startup_done/session_seen; an error requires the shared reset.
module d2e_phy_startup #(
 parameter integer CLOCK_HZ=100000000,MDC_HZ=500000,ROLE=0,
 parameter integer POLL_WAIT_CYCLES=CLOCK_HZ/10,
 parameter integer TRANSACTION_TIMEOUT=128*((CLOCK_HZ+2*MDC_HZ-1)/(2*MDC_HZ))+64
)(input wire clk,rst_n,session_active,mdio_i,
 output wire mdc,mdio_o,mdio_oe,
 output wire qualified_phy,output wire[415:0]status_words,
 output wire[31:0]config_status,
 output reg[15:0]config_before,config_desired,config_readback);
 localparam SI=0,SW=1,SC=2,EI=3,EW=4,COMMIT=5,PAUSE=6,FAULT=7;
 localparam integer WAIT_W=POLL_WAIT_CYCLES<2?1:$clog2(POLL_WAIT_CYCLES);
 localparam[7:0]ROLE8=ROLE;
 localparam[15:0]DIV16=(CLOCK_HZ+2*MDC_HZ-1)/(2*MDC_HZ);
 localparam[31:0]CLOCK32=CLOCK_HZ;
 reg[2:0]state;reg[5:0]step;
 reg[4:0]scan_phy,selected_phy,found_phy;reg scan_low,found,id_hi_ok;
 reg[15:0]id_hi;reg[31:0]presence_work,presence,cycles,sweeps,read_ok,no_ta,timeout_count;
 reg[WAIT_W-1:0]pause_count;
 reg startup_done,session_seen,halt,link_hold,have_poll;
 reg id_verified,utp_verified,rx_verified,before_valid,selector_verified;
 reg[7:0]failure_code;
 reg[15:0]t_id_hi,t_a000,t_a001,t_a003,t_bmcr,t_bmsr1,t_bmsr2,t_psr;
 reg[15:0]a000,a001,a003,bmcr,bmsr1,bmsr2,psr;
 reg[31:0]valid_bits;
 reg request,write_selector,write_config;
 reg[4:0]reg_address,phy_address;reg[15:0]argument;
 wire busy,done,ack,blocked;wire[15:0]data;
 wire authorize=(state==EW&&step==12&&!startup_done&&!session_seen&&!session_active&&
  id_verified&&utp_verified&&rx_verified&&before_valid&&selector_verified&&!halt);
 wire engine_rst_n=rst_n&&!halt&&!(session_active&&!startup_done);
 d2e_mdio_transaction #(.CLOCK_HZ(CLOCK_HZ),.MDC_HZ(MDC_HZ))u_mdio(
 .clk(clk),.rst_n(engine_rst_n),.request(request),.write_selector(write_selector),.write_config(write_config),
 .config_authorized(authorize),.config_before(config_before),.phy_address(phy_address),
 .register_address(reg_address),.selector_address(argument),.mdio_i(mdio_i),.mdio_o(mdio_o),.mdio_oe(mdio_oe),
 .mdc(mdc),.busy(busy),.done(done),.acknowledged(ack),.blocked_write(blocked),.read_data(data));
 assign qualified_phy=startup_done&&link_hold&&have_poll&&!halt;
 wire[7:0]flags={1'b0,link_hold,link_hold,1'b1,state==PAUSE,1'b1,sweeps!=0,found};
 wire[31:0]metadata={3'd0,selected_phy,flags,ROLE8,8'd1};
 assign status_words={valid_bits,DIV16,a000,CLOCK32,sweeps,no_ta,read_ok,a003,a001,psr,bmsr2,bmsr1,bmcr,
  (found?32'h4f51e91a:32'd0),presence,cycles,metadata};
 assign config_status={failure_code,8'd0,step,2'd0,state,halt,session_seen,startup_done,have_poll,link_hold};
 task fail;
 input[7:0]code;
 begin failure_code<=code;halt<=1;link_hold<=0;have_poll<=0;state<=FAULT;request<=0;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=SI;step<=0;scan_phy<=0;selected_phy<=0;found_phy<=0;scan_low<=0;found<=0;id_hi_ok<=0;id_hi<=0;
   presence_work<=0;presence<=0;cycles<=0;sweeps<=0;read_ok<=0;no_ta<=0;timeout_count<=0;pause_count<=0;
   startup_done<=0;session_seen<=0;halt<=0;link_hold<=0;have_poll<=0;
   id_verified<=0;utp_verified<=0;rx_verified<=0;before_valid<=0;selector_verified<=0;failure_code<=0;
   t_id_hi<=0;t_a000<=0;t_a001<=0;t_a003<=0;t_bmcr<=0;t_bmsr1<=0;t_bmsr2<=0;t_psr<=0;
   a000<=0;a001<=0;a003<=0;bmcr<=0;bmsr1<=0;bmsr2<=0;psr<=0;valid_bits<=0;
   config_before<=0;config_desired<=0;config_readback<=0;
   request<=0;write_selector<=0;write_config<=0;reg_address<=0;phy_address<=0;argument<=0;
  end else begin
   cycles<=cycles+1'b1;request<=0;
   if(session_active)session_seen<=1;
   if(done&&!write_selector&&!write_config)begin if(ack)read_ok<=read_ok+1'b1;else no_ta<=no_ta+1'b1;end
   if(state==SW||state==EW)timeout_count<=timeout_count+1'b1;else timeout_count<=0;
   if(session_active&&!startup_done)fail(8'd8);
   else if((state==SW||state==EW)&&!done&&timeout_count>=TRANSACTION_TIMEOUT)fail(8'd7);
   else case(state)
    SI:if(!busy)begin
     phy_address<=scan_phy;reg_address<=scan_low?5'd3:5'd2;argument<=0;write_selector<=0;write_config<=0;request<=1;state<=SW;
    end
    SW:if(done)begin
     if(!scan_low)begin id_hi<=data;id_hi_ok<=ack;scan_low<=1;state<=SI;end
     else begin
      if(id_hi_ok&&ack&&id_hi!=0&&id_hi!=16'hffff&&data!=0&&data!=16'hffff)begin
       presence_work[scan_phy]<=1;
       if({id_hi,data}==32'h4f51e91a&&(!found||scan_phy==3))begin found<=1;found_phy<=scan_phy;end
      end
      scan_low<=0;if(scan_phy==31)state<=SC;else begin scan_phy<=scan_phy+1'b1;state<=SI;end
     end
    end
    SC:begin
     presence<=presence_work;sweeps<=sweeps+1'b1;selected_phy<=found_phy;step<=0;
     id_verified<=0;utp_verified<=0;rx_verified<=0;before_valid<=0;selector_verified<=0;
     if(found)state<=EI;else begin pause_count<=0;state<=PAUSE;end
    end
    EI:if(!busy)begin
     if(step==12&&(startup_done||config_before[3:0]==4'hd))begin step<=13;end
     else begin
      phy_address<=selected_phy;write_selector<=0;write_config<=0;argument<=0;
      case(step)
       0,22:reg_address<=2;
       1,23:reg_address<=3;
       2,16:begin reg_address<=30;write_selector<=1;argument<=16'ha000;end
       5,19:begin reg_address<=30;write_selector<=1;argument<=16'ha001;end
       8,13:begin reg_address<=30;write_selector<=1;argument<=16'ha003;end
       3,6,9,11,14,17,20:reg_address<=30;
       4,7,10,15,18,21:reg_address<=31;
       12:begin reg_address<=31;write_config<=1;argument<=config_desired;end
       24:reg_address<=0;
       25,26:reg_address<=1;
       27:reg_address<=17;
       default:reg_address<=0;
      endcase
      request<=1;state<=EW;
     end
    end
    EW:if(done)begin
     if(!ack||blocked)fail(8'd2);
     else begin
      state<=EI;step<=step+1'b1;
      case(step)
       0,22:t_id_hi<=data;
       1,23:if({t_id_hi,data}!=32'h4f51e91a)fail(8'd3);else id_verified<=1;
       3,17:if(data!=16'ha000)fail(8'd4);
       6,20:if(data!=16'ha001)fail(8'd4);
       9,14:if(data!=16'ha003)fail(8'd4);
       11:if(data!=16'ha003)fail(8'd4);else selector_verified<=1;
       4,18:begin t_a000<=data;if(data[1])fail(8'd5);else utp_verified<=1;end
       7:begin t_a001<=data;if(data[8])fail(8'd5);else rx_verified<=1;end
       10:begin
        t_a003<=data;
        if((data&16'h7c00)!=0)fail(8'd5);
        else if(startup_done&&data!=config_desired)fail(8'd6);
        else if(!startup_done)begin config_before<=data;config_desired<=(data&16'hfff0)|16'h000d;before_valid<=1;end
       end
       15:begin config_readback<=data;t_a003<=data;if(data!=config_desired)fail(8'd6);end
       21:if(data!=t_a001)fail(8'd6);
       24:t_bmcr<=data;
       25:t_bmsr1<=data;
       26:t_bmsr2<=data;
       27:begin t_psr<=data;state<=COMMIT;end
      endcase
     end
    end
    COMMIT:begin
     // No raw clear between polls: every public word is one completed poll.
     a000<=t_a000;a001<=t_a001;a003<=t_a003;bmcr<=t_bmcr;bmsr1<=t_bmsr1;bmsr2<=t_bmsr2;psr<=t_psr;
     valid_bits<=32'hfff;startup_done<=1;have_poll<=1;
     link_hold<=id_verified&&utp_verified&&rx_verified&&before_valid&&config_readback==config_desired&&
       t_bmsr2[2]&&t_psr[10]&&t_psr[11]&&t_psr[13]&&((t_psr&16'hc200)==16'h8000);
     pause_count<=0;state<=PAUSE;
    end
    PAUSE:if(pause_count==POLL_WAIT_CYCLES-1)begin
     if(startup_done)begin step<=0;state<=EI;end
     else begin scan_phy<=0;scan_low<=0;found<=0;found_phy<=0;presence_work<=0;state<=SI;end
    end else pause_count<=pause_count+1'b1;
    FAULT:begin request<=0;link_hold<=0;end
    default:fail(8'd9);
   endcase
  end
 end
endmodule
