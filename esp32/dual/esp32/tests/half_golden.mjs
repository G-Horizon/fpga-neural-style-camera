import fs from'node:fs';import path from'node:path';import{fileURLToPath}from'node:url';import{restorePixel,crc32}from'../../mobile/protocol.mjs';
const out=path.join(path.dirname(fileURLToPath(import.meta.url)),'results');const rgb=new Uint8Array(110592),guide=new Uint8Array(2073600),restored=new Uint8Array(1920*1080*3);
for(let i=0;i<rgb.length;i++)rgb[i]=(i*13+7)&255;for(let i=0;i<guide.length;i++)guide[i]=(i*17+(i>>>10)*31+37)&255;
let off=0;for(let y=0;y<1080;y++)for(let x=0;x<1920;x++){const p=restorePixel(rgb,guide,x,y,1);for(const c of p)restored[off++]=c;}
const result={pixels:2073600,effect:1,crc32:crc32(restored).toString(16).padStart(8,'0'),scope:'Production JS integer bilinear+sameframehorizontalclippedhalf+finaleffect;compared separately to actual unchanged RTL'};fs.writeFileSync(path.join(out,'HALF_JS.json'),JSON.stringify(result,null,2));console.log(JSON.stringify(result));
