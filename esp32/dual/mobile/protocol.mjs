export const MAGIC=0x32463244,CHUNK_MAGIC=0x32433244,END_MAGIC=0x32453244,ACK_MAGIC=0x324b4341,HEADER_BYTES=64;
export const STYLE_BYTES=110592,GUIDE_BYTES=2073600,CHUNK_BYTES=8192;
const table=Uint32Array.from({length:256},(_,i)=>{let c=i;for(let j=0;j<8;j++)c=(c>>>1)^((c&1)?0xedb88320:0);return c>>>0;});
export function crc32(bytes,seed=0xffffffff,final=true){let c=seed;for(const b of bytes)c=(c>>>8)^table[(c^b)&255];return final?(c^0xffffffff)>>>0:c>>>0;}
function view(buffer){if(!(buffer instanceof ArrayBuffer))throw Error('需要完整二进制消息');return new DataView(buffer);}
function check(v,bytes,n,at){if(v.getUint32(at,true)!==crc32(bytes.subarray(0,n)))throw Error('消息头 CRC 错误');}
export function parseHeader(buffer){
 const v=view(buffer),p=new Uint8Array(buffer);if(buffer.byteLength!==64||v.getUint32(0,true)!==MAGIC||v.getUint16(4,true)!==2||v.getUint16(6,true)!==64)throw Error('双板帧协议不兼容');check(v,p,60,60);
 const f={id:v.getUint32(8,true),epoch:v.getUint32(12,true),sequence:v.getUint32(16,true),timestamp:v.getUint32(20,true),style:v.getUint8(24),effect:v.getUint8(25),quality:v.getUint8(26),flags:v.getUint8(27),fps:v.getUint16(28,true)/100,width:v.getUint16(30,true),height:v.getUint16(32,true),guideWidth:v.getUint16(34,true),guideHeight:v.getUint16(36,true),algorithm:v.getUint16(38,true),styleBytes:v.getUint32(40,true),guideBytes:v.getUint32(44,true),styleCRC:v.getUint32(48,true),guideCRC:v.getUint32(52,true),session:v.getUint32(56,true)};
 if(f.style>2||f.effect>1||f.quality>1||(f.flags&~7)||!(f.flags&1)||!(f.flags&2)||f.width!==256||f.height!==144||f.styleBytes!==STYLE_BYTES||!f.session||f.algorithm!==(f.quality?1:0)||f.guideBytes!==(f.quality?GUIDE_BYTES:0)||f.guideWidth!==(f.quality?1920:0)||f.guideHeight!==(f.quality?1080:0)||(!f.quality&&f.guideCRC))throw Error('双板帧几何、算法或元数据无效');
 return f;
}
export class FrameAssembler{
 current=null;
 accept(buffer){
  const v=view(buffer),p=new Uint8Array(buffer);if(p.length<4)throw Error('消息不完整');const magic=v.getUint32(0,true);
  if(magic===MAGIC){if(this.current)throw Error('上一帧尚未完整');const f=parseHeader(buffer);this.current={...f,pixels:new Uint8Array(STYLE_BYTES),guide:f.quality?new Uint8Array(GUIDE_BYTES):null,offset:[0,0],crc:[0xffffffff,0xffffffff]};return null;}
  const f=this.current;if(!f)throw Error('缺少本帧描述符');if(p.length<32||v.getUint32(4,true)!==f.sequence||v.getUint32(8,true)!==f.id||v.getUint32(12,true)!==f.epoch)throw Error('分块帧号不匹配');check(v,p,28,28);
  if(magic===CHUNK_MAGIC){const plane=v.getUint8(16),n=v.getUint16(18,true),offset=v.getUint32(20,true),target=plane===0?f.pixels:plane===1?f.guide:null;
   if(v.getUint8(17)!==0||!target||!n||n>CHUNK_BYTES||p.length!==32+n||offset!==f.offset[plane]||n>target.length-offset||(plane===1&&f.offset[0]!==STYLE_BYTES))throw Error('分块顺序、范围或长度无效');
   const data=p.subarray(32);if(crc32(data)!==v.getUint32(24,true))throw Error('分块 CRC 错误');target.set(data,offset);f.offset[plane]+=n;f.crc[plane]=crc32(data,f.crc[plane],false);return null;
  }
  if(magic!==END_MAGIC||p.length!==32||v.getUint32(16,true)!==f.styleCRC||v.getUint32(20,true)!==f.guideCRC||v.getUint32(24,true)!==f.styleBytes+f.guideBytes||f.offset[0]!==f.styleBytes||f.offset[1]!==f.guideBytes||((f.crc[0]^0xffffffff)>>>0)!==f.styleCRC||(f.quality&&((f.crc[1]^0xffffffff)>>>0)!==f.guideCRC))throw Error('帧尚未完整或整帧 CRC 错误');
  this.current=null;return f;
 }
}
export function makeAck(f){const a=new ArrayBuffer(16),v=new DataView(a);[ACK_MAGIC,f.id,f.epoch,f.sequence].forEach((x,i)=>v.setUint32(i*4,x,true));return a;}
export function isNewer(a,b){return a!==b&&((a-b)>>>0)<0x80000000;}
export class FrameGate{last=null;accept(f){const p=this.last;if(p&&p.session===f.session){if(f.epoch!==p.epoch){if(!isNewer(f.epoch,p.epoch))return false;}else if(f.id!==p.id){if(!isNewer(f.id,p.id))return false;}else if(!isNewer(f.sequence,p.sequence))return false;}this.last={id:f.id,epoch:f.epoch,sequence:f.sequence,session:f.session};return true;}}
export function sourceCoord(p,n){const q=4*p+17,i=Math.floor(q/30)-1,w=i<0||i>=n-1?0:Math.floor(((q%30)*256+15)/30);return [Math.max(0,Math.min(n-1,i)),Math.max(0,Math.min(n-1,i+1)),w];}
export function bilinearPixel(pixels,x,y){const[x0,x1,wx]=sourceCoord(x,256),[y0,y1,wy]=sourceCoord(y,144);return [0,1,2].map(c=>{const a=pixels[(y0*256+x0)*3+c],b=pixels[(y0*256+x1)*3+c],d=pixels[(y1*256+x0)*3+c],e=pixels[(y1*256+x1)*3+c];return Math.floor(((a*(256-wx)+b*wx)*(256-wy)+(d*(256-wx)+e*wx)*wy+32768)/65536);});}
export function restorePixel(pixels,guide,x,y,effect=0){let rgb=bilinearPixel(pixels,x,y);if(guide){const i=y*1920+x,g=guide[i],left=guide[y*1920+Math.max(0,x-1)],right=guide[y*1920+Math.min(1919,x+1)],low=(left+2*g+right+2)>>2,delta=g-low,amount=(Math.min(Math.abs(delta),32)+1)>>1;rgb=rgb.map(c=>Math.max(0,Math.min(255,c+(delta<0?-amount:amount))));}return effect?rgb.map(c=>255-c):rgb;}
