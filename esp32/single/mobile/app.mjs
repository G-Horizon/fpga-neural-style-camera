import {parseFrame,makeAck,FrameGate} from './protocol.mjs';
const $=s=>document.getElementById(s),canvas=$('view');
const gl=canvas.getContext('webgl2',{alpha:false,antialias:false,premultipliedAlpha:false});
let ws,gate=new FrameGate(),generation=0,lastTime=0,times=[],retry,lastPicture=null,receivedFrames=0;
function status(s,error=false){$('status').textContent=s;$('status').style.color=error?'#b42318':'#0f766e';}
function updateNetworkFps(){
  const value=lastTime && performance.now()-lastTime<=1500 && times.length?1000*times.length/times.reduce((a,b)=>a+b,0):0;
  $('networkFps').textContent=value.toFixed(1);
}
function compile(type,src){const s=gl.createShader(type);gl.shaderSource(s,src);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;}
let program,texture,effect;
function gpuCompleted(){
  const fence=gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE,0);gl.flush();
  return new Promise((resolve,reject)=>{const deadline=performance.now()+2000;
    function poll(){const result=gl.clientWaitSync(fence,0,0);
      if(result===gl.ALREADY_SIGNALED || result===gl.CONDITION_SATISFIED){gl.deleteSync(fence);resolve();}
      else if(result===gl.WAIT_FAILED || performance.now()>deadline){gl.deleteSync(fence);reject(Error('GPU 渲染超时'));}
      else setTimeout(poll,8);
    }poll();
  });
}
if(gl){
  program=gl.createProgram();
  gl.attachShader(program,compile(gl.VERTEX_SHADER,`#version 300 es
  void main(){vec2 p=vec2(float((gl_VertexID<<1)&2),float(gl_VertexID&2));gl_Position=vec4(p*2.0-1.0,0,1);}`));
  gl.attachShader(program,compile(gl.FRAGMENT_SHADER,`#version 300 es
  precision highp float;precision highp int;uniform highp usampler2D image;uniform int effect;out vec4 color;
  ivec3 coord(int p,int n){int q=2*p+6;int i=q/10-1;int w=(i<0||i>=n-1)?0:((q%10)*256+5)/10;return ivec3(clamp(i,0,n-1),clamp(i+1,0,n-1),w);}
  void main(){ivec3 x=coord(int(gl_FragCoord.x),256),y=coord(719-int(gl_FragCoord.y),144);
    uvec3 a=texelFetch(image,ivec2(x.x,y.x),0).rgb,b=texelFetch(image,ivec2(x.y,y.x),0).rgb;
    uvec3 c=texelFetch(image,ivec2(x.x,y.y),0).rgb,d=texelFetch(image,ivec2(x.y,y.y),0).rgb;
    uint wx=uint(x.z),wy=uint(y.z);uvec3 z=((a*(256u-wx)+b*wx)*(256u-wy)+(c*(256u-wx)+d*wx)*wy+32768u)>>16u;
    if(effect==1)z=255u-z;color=vec4(vec3(z)/255.0,1);}`));
  gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));
  gl.useProgram(program);gl.disable(gl.DITHER);gl.disable(gl.BLEND);gl.viewport(0,0,1280,720);
  texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,texture);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
  gl.pixelStorei(gl.UNPACK_ALIGNMENT,1);gl.texStorage2D(gl.TEXTURE_2D,1,gl.RGB8UI,256,144);
  gl.uniform1i(gl.getUniformLocation(program,'image'),0);effect=gl.getUniformLocation(program,'effect');
}else{$('notice').textContent='此浏览器未提供 WebGL2，请使用较新的 Chrome。';status('无法渲染',true);}
function connect(){
  clearTimeout(retry);generation++;const mine=generation;gate=new FrameGate();times=[];lastTime=0;lastPicture=null;receivedFrames=0;
  $('networkFps').textContent='—';$('sourceFps').textContent='—';$('receivedFrames').textContent='0';
  if(ws){ws.onclose=null;ws.close();}if(!gl)return;
  status('连接中');$('notice').hidden=false;$('notice').textContent='正在连接相机…';
  ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/stream`);ws.binaryType='arraybuffer';
  ws.onopen=()=>{if(mine===generation)status('已连接 · 等待帧');};
  ws.onmessage=async e=>{
    if(mine!==generation)return;
    try{const f=parseFrame(e.data);
      if(gate.accept(f)){
        gl.bindTexture(gl.TEXTURE_2D,texture);gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,256,144,gl.RGB_INTEGER,gl.UNSIGNED_BYTE,f.pixels);
        gl.uniform1i(effect,f.effect);gl.drawArrays(gl.TRIANGLES,0,3);$('notice').hidden=true;
        $('style').textContent=['梵高','浮世绘','水墨'][f.style]+(f.effect?' · 反相':'');
        if(!lastPicture || f.id!==lastPicture.id || f.epoch!==lastPicture.epoch){
          const now=performance.now();if(lastTime)times.push(now-lastTime);lastTime=now;if(times.length>20)times.shift();
          lastPicture={id:f.id,epoch:f.epoch};receivedFrames++;$('receivedFrames').textContent=String(receivedFrames);updateNetworkFps();
        }
        $('sourceFps').textContent=f.fps.toFixed(1);$('sourceLabel').textContent=(f.flags&4)?'模拟输入 · 手机仅放大':'FPGA 风格画面 · 无融合';status((f.flags&4)?'模拟画面':'实时画面');
        await gpuCompleted();
      }
      // One credit only after the complete frame has been checked and GPU work completed.
      if(mine===generation && ws.readyState===WebSocket.OPEN)ws.send(makeAck(f));
    }catch(err){status(err.message,true);ws.close(1003,'bad frame');}
  };
  ws.onerror=()=>{if(mine===generation)status('网络连接失败',true);};
  ws.onclose=()=>{if(mine!==generation)return;status('已断开，自动重连',true);$('notice').hidden=false;$('notice').textContent='连接相机热点后等待重连';retry=setTimeout(connect,2000);};
}
$('connect').onclick=connect;$('fpsToggle').onchange=()=>{$('fps').hidden=!$('fpsToggle').checked;};
$('fullscreen').onclick=async()=>{try{if(document.fullscreenElement)await document.exitFullscreen();else await document.documentElement.requestFullscreen();}catch{status('请使用 Chrome 全屏按钮',true);}};
setInterval(updateNetworkFps,1000);
connect();
