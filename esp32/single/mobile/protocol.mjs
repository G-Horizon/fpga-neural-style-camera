export const MAGIC = 0x31525346; // ASCII FSR1, little endian
export const HEADER_BYTES = 48;
export const ACK_MAGIC = 0x314b4341; // ACK1
export function crc32(bytes) {
  let c = 0xffffffff;
  for (const b of bytes) { c ^= b; for (let i=0;i<8;i++) c=(c>>>1)^((c&1)?0xedb88320:0); }
  return (c ^ 0xffffffff) >>> 0;
}
export function parseFrame(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < HEADER_BYTES) throw Error('帧头不完整');
  const v = new DataView(buffer);
  if (v.getUint32(0,true)!==MAGIC || v.getUint16(4,true)!==1 || v.getUint16(6,true)!==HEADER_BYTES) throw Error('帧协议不兼容');
  const f={id:v.getUint32(8,true),epoch:v.getUint32(12,true),timestamp:v.getUint32(16,true),
    style:v.getUint8(20),effect:v.getUint8(21),flags:v.getUint16(22,true),width:v.getUint16(24,true),
    height:v.getUint16(26,true),stride:v.getUint16(28,true),format:v.getUint8(30),
    length:v.getUint32(32,true),fps:v.getUint16(36,true)/100,sequence:v.getUint32(44,true)};
  if (f.width!==256 || f.height!==144 || f.stride!==768 || f.format!==1 || f.length!==110592 || buffer.byteLength!==HEADER_BYTES+f.length || !(f.flags&1) || f.style>2 || f.effect>1) throw Error('帧格式或元数据无效');
  f.pixels=new Uint8Array(buffer,HEADER_BYTES,f.length);
  if ((f.flags&2) && crc32(f.pixels)!==v.getUint32(40,true)) throw Error('帧 CRC 错误');
  return f;
}
export function makeAck(f) {
  const a=new ArrayBuffer(16),v=new DataView(a);v.setUint32(0,ACK_MAGIC,true);v.setUint32(4,f.id,true);v.setUint32(8,f.epoch,true);v.setUint32(12,f.sequence,true);return a;
}
export function isNewer(a,b) {return a!==b && ((a-b)>>>0)<0x80000000;}
export class FrameGate {
  last=null;
  accept(f) {
    if (this.last) {
      if(f.epoch!==this.last.epoch){if(!isNewer(f.epoch,this.last.epoch))return false;}
      else if(f.id!==this.last.id){if(!isNewer(f.id,this.last.id))return false;}
      else if(!isNewer(f.sequence,this.last.sequence) || f.style!==this.last.style ||
          (f.effect===this.last.effect && f.fps===this.last.fps))return false;
    }
    this.last={id:f.id,epoch:f.epoch,sequence:f.sequence,effect:f.effect,fps:f.fps,style:f.style};return true;
  }
}
// Reference for GLSL and FPGA verification: centre-aligned 15/2 scale,
// Rounded Q8 phase table and one final rounding, matching d64_bilinear_scaler.v.
export function sourceCoord(p,n) {
  const q=4*p+17,i=Math.floor(q/30)-1;
  const w=i<0 || i>=n-1?0:Math.floor(((q%30)*256+15)/30);
  return [Math.max(0,Math.min(n-1,i)),Math.max(0,Math.min(n-1,i+1)),w];
}
export function bilinearPixel(pixels,x,y,effect=0) {
  const [x0,x1,wx]=sourceCoord(x,256),[y0,y1,wy]=sourceCoord(y,144);
  return [0,1,2].map(c=>{
    const a=pixels[(y0*256+x0)*3+c],b=pixels[(y0*256+x1)*3+c],d=pixels[(y1*256+x0)*3+c],e=pixels[(y1*256+x1)*3+c];
    const z=Math.floor(((a*(256-wx)+b*wx)*(256-wy)+(d*(256-wx)+e*wx)*wy+32768)/65536);
    return effect===1?255-z:z;
  });
}
