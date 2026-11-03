`timescale 1ns/1ps
module tb_palm_fist;
  reg clk=0,rst_n=0,cap_valid=0,cap_ready=1,cap_sof=0,cap_eof=0,source_bad=0;
  reg [383:0] cap_rgb=0;
  reg [7:0] cap_x0=0,cap_y=0;
  reg [31:0] cap_fid=0;
  wire effect_trigger,primed,frame_good;
  integer triggers=0,good_frames=0,checks=0;
  integer nextfid=1;
  reg check_reference=1;
  d64_wave_detector dut(.*);
  always #5 clk=~clk;
  always @(posedge clk) begin
    #1;
    if(effect_trigger)triggers=triggers+1;
    if(frame_good)good_frames=good_frames+1;
  end
  function integer bg(input integer c,input integer r);
    bg=30+(c*7+r*11)%23;
  endfunction
  function integer occupied(input integer shape,input integer c,input integer r);
    begin
      case(shape)
        1:occupied=(c>=6&&c<=9&&r>=3&&r<=5)||
          ((c==4||c==11)&&r==3)||((c==5||c==10)&&(r==2||r==3))||
          (c>=6&&c<=9&&(r==2||r==6));
        2:occupied=c>=6&&c<=9&&r>=3&&r<=5;
        3:occupied=c>=2&&c<=5&&r>=2&&r<=6;
        4:occupied=c>=10&&c<=13&&r>=2&&r<=6;
        5:occupied=1;
        7:occupied=c>=5&&c<=10&&r>=3&&r<=5;
        8:occupied=c>=4&&c<=11&&r>=2&&r<=6;
        default:occupied=0;
      endcase
    end
  endfunction
  task expect_count(input integer expected,input [255:0] labelname);
    begin
      checks=checks+1;
      if(triggers!==expected)$fatal(1,"%0s: expected %0d triggers, got %0d",labelname,expected,triggers);
      $display("PASS %0s: triggers=%0d",labelname,triggers);
    end
  endtask
  task reset_scene;
    begin
      @(negedge clk);rst_n=0;cap_valid=0;source_bad=0;
      repeat(3)@(negedge clk);
      rst_n=1;triggers=0;good_frames=0;nextfid=1;check_reference=1;
    end
  endtask
  // mode 1=coordinate violation; 2=source_bad; 3=early EOF;
  // mode 4=missing EOF; 5=FID change within a frame. Every fifth packet may
  // stall for one clock: rejected cycles must not rotate the background ring.
  task frame(input integer shape,input integer mode,input integer stalls);
    integer y,c,g;
    begin
      for(y=0;y<144;y=y+1)for(c=0;c<16;c=c+1)begin
        @(negedge clk);
        cap_valid=1;cap_ready=1;cap_x0=c*16;cap_y=y;
        cap_fid=nextfid;cap_sof=(c==0&&y==0);cap_eof=(c==15&&y==143);
        source_bad=0;
        g=bg(c,y/16);
        if(occupied(shape,c,y/16))g=(shape==8?0:190);
        cap_rgb=0;cap_rgb[207:200]=g;
        if(mode==1&&y==40&&c==5)cap_x0=96;
        if(mode==2&&y==40&&c==5)source_bad=1;
        if(mode==3&&y==40&&c==5)cap_eof=1;
        if(mode==4&&y==143&&c==15)cap_eof=0;
        if(mode==5&&y==40&&c==5)cap_fid=nextfid+10;
        if(stalls&&((c+y)%5)==0)begin
          cap_ready=0;
          @(negedge clk);cap_ready=1;
        end
        if(mode==0&&check_reference&&dut.compare_frame&&y%16==8)begin
          #1;
          if(dut.background!==bg(c,y/16))$fatal(1,"frozen background mismatch c=%0d r=%0d got=%0d expected=%0d",c,y/16,dut.background,bg(c,y/16));
        end
      end
      @(negedge clk);cap_valid=0;cap_sof=0;cap_eof=0;source_bad=0;
      nextfid=nextfid+1;
      repeat(2)@(negedge clk);
    end
  endtask
  integer i,m;
  initial begin
    reset_scene;frame(0,0,0);
    if(!primed)$fatal(1,"complete empty frame did not prime background");
    frame(1,0,1);frame(1,0,1);frame(2,0,1);frame(2,0,1);
    expect_count(1,"open/open/fist/fist stalled");
    // Held fist and rapid repetitions cannot retrigger during cooldown.
    repeat(5)frame(2,0,0);
    frame(1,0,0);frame(1,0,0);frame(2,0,0);frame(2,0,0);
    expect_count(1,"cooldown prevents retrigger");
    repeat(60)frame(0,0,0);
    frame(1,0,0);frame(1,0,0);frame(2,0,0);frame(2,0,0);
    expect_count(2,"cooldown expiry new gesture");
    reset_scene;frame(0,0,0);repeat(4)frame(2,0,0);
    expect_count(0,"direct fist rejected");
    reset_scene;frame(0,0,0);repeat(3)begin frame(3,0,0);frame(4,0,0);end
    frame(2,0,0);frame(2,0,0);
    expect_count(0,"left/right translation rejected");
    reset_scene;frame(0,0,0);frame(1,0,0);frame(2,0,0);frame(2,0,0);
    expect_count(0,"single open frame rejected");
    reset_scene;frame(0,0,0);frame(1,0,0);frame(1,0,0);
    frame(2,0,0);frame(7,0,0);frame(2,0,0);
    expect_count(0,"fist requires consecutive frames");
    frame(2,0,0);expect_count(1,"fist confirmation succeeds");
    reset_scene;frame(0,0,0);frame(1,0,0);frame(1,0,0);
    repeat(25)frame(7,0,0);frame(2,0,0);frame(2,0,0);
    expect_count(0,"gesture timeout rejected");
    reset_scene;frame(0,0,0);frame(1,0,0);frame(1,0,0);
    frame(5,0,0);frame(2,0,0);frame(2,0,0);
    expect_count(0,"whole-frame flash rejected");
    reset_scene;frame(0,0,0);repeat(2)frame(8,0,0);repeat(2)frame(2,0,0);
    expect_count(0,"wrong foreground polarity rejected");
    reset_scene;frame(0,0,0);frame(1,0,0);frame(1,0,0);
    check_reference=0;nextfid=nextfid+1;frame(2,0,0);frame(2,0,0);
    expect_count(0,"FID gap cancels armed gesture");
    // Each malformed frame cancels a partial gesture; one complete empty
    // frame re-primes the reference, then a fresh gesture must work again.
    for(m=1;m<=5;m=m+1)begin
      reset_scene;frame(0,0,0);frame(1,0,0);frame(1,0,0);
      frame(2,m,0);expect_count(0,"bad frame does not trigger");
      if(primed)$fatal(1,"bad frame did not unprime mode=%0d",m);
      frame(0,0,0);frame(2,0,0);frame(2,0,0);
      expect_count(0,"bad frame cancels previous open");
      frame(1,0,0);frame(1,0,0);frame(2,0,0);frame(2,0,0);
      expect_count(1,"recalibration restores detector");
    end
    $display("ALL_PASS checks=%0d",checks);$finish;
  end
  initial begin #200000000; $fatal(1,"simulation timeout"); end
endmodule

