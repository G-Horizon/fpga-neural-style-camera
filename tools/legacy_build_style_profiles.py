"""Package retained whole-network profiles without renumbering their ABI IDs.

This does not recalibrate a checkpoint. It consumes the immutable 24-image
derived contracts and tensors, checks all fields before encoding, reconstructs
all weight tensors from the packed physical image, and binds every input hash.
"""
from pathlib import Path
import argparse, hashlib, json, math, struct, zlib
import numpy as np

STYLES = ('vangogh', 'ukiyoe', 'inkwash')
STYLE_IDS = {'vangogh': 1, 'ukiyoe': 2, 'inkwash': 3}
MAGIC = 0x31545344
TRUNK_CFG_BIAS_W = 20
DEC_CFG_BIAS_W = 13

def check_cfg_bias_domain(style, layer, channel, bias):
    """Validate the hardware storage domain before emitting unchanged ABI64.

    ValueError deliberately survives python -O. A new checkpoint outside this
    domain must widen the hardware parameter and rerun the physical gates.
    """
    width = (TRUNK_CFG_BIAS_W if layer.startswith('trunk.') else
             DEC_CFG_BIAS_W if layer in ('ups.0','ups.1','tail') else 32)
    minimum, maximum = -(1 << (width-1)), (1 << (width-1))-1
    if not minimum <= bias <= maximum:
        raise ValueError(f'{style}/{layer}/channel {channel}: bias {bias} '
                         f'outside signed{width} [{minimum},{maximum}]')
    field = bias & ((1 << width)-1)
    decoded = field - (1 << width) if field & (1 << (width-1)) else field
    if decoded != bias or (decoded & 0xffffffff) != (bias & 0xffffffff):
        raise ValueError(f'{style}/{layer}/channel {channel}: signed{width} '
                         f'bias roundtrip changed {bias} into {decoded}')
    return width

def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()

def build(source, out, style, sid, reference, compute_lanes=128):
    # Historical direct callers retain128; the production CLI selects64.
    if compute_lanes not in (64,128):
        raise ValueError('compute_lanes must be64 or128')
    word_bytes=compute_lanes;abi=2 if compute_lanes==64 else 1
    p = source / style
    doc = json.loads((p/'derived_contract.json').read_text(encoding='utf-8'))
    arrays = np.load(p/'derived_weights.npz', allow_pickle=False)
    assert doc['residual'] == 'shared' and doc['weight_bits'] == 8
    zero = int(doc['formats']['trunk']['zero'])
    assert 0 <= zero <= 1023
    for n,f in doc['formats'].items():
        if f['qmax'] == 1023:
            assert f['qmin'] == 0 and f['zero'] == zero, (n,f)
        else:
            assert (f['qmin'],f['qmax'],f['zero']) == (0,255,0), (n,f)
    words, config, layers = [], [], []
    bounds=[]
    for L in doc['layers']:
        name, kind, cin, cout, k = L['name'], L['kind'], L['cin'], L['cout'], L['k']
        w = arrays[name+'/w_q'].astype(np.int64)
        want = (cin,cout,k,k) if kind == 'tconv' else (cout,cin,k,k)
        assert w.shape == want and np.all((w>=-128)&(w<=127))
        ic = min(8,cin); oc = 4 if name=='tail' else min(16,cout)
        if compute_lanes==64:oc//=2
        ig,og = math.ceil(cin/ic),math.ceil(cout/oc)
        base = len(words)
        input_major = name.startswith('trunk.') or name in ('ups.0','ups.1','tail')
        for kh in range(k):
            for kw in range(k):
                for i in range(ig):
                    for o in range(og):
                        b = bytearray(word_bytes)
                        for ii in range(ic):
                            for oo in range(oc):
                                ci,co = i*ic+ii,o*oc+oo
                                if ci>=cin or co>=cout: continue
                                v = int(w[ci,co,kh,kw] if kind=='tconv' else w[co,ci,kh,kw])
                                b[(ii*oc+oo) if input_major else (oo*ic+ii)] = v&255
                        words.append(bytes(b))
        # Independent tensor reconstruction follows the declared source axes.
        rec = np.zeros(w.shape,dtype=np.int64)
        for ci in range(cin):
            for co in range(cout):
                for kh in range(k):
                    for kw in range(k):
                        ti = base+((kh*k+kw)*ig+ci//ic)*og+co//oc
                        pos = (ci%ic)*oc+co%oc if input_major else (co%oc)*ic+ci%ic
                        v=words[ti][pos]; v=v-256 if v>=128 else v
                        if kind=='tconv':rec[ci,co,kh,kw]=v
                        else:rec[co,ci,kh,kw]=v
        assert np.array_equal(rec,w), name
        bias = arrays[name+'/b_q'].astype(np.int64)
        if kind=='conv':
            zin = int(doc['formats'][L['in_tensor']]['zero'])
            bias = bias-zin*w.sum(axis=(1,2,3))
        f=doc['formats'][L['in_tensor']]
        if kind=='conv':
            matrices=[('all',w.reshape(cout,-1))]
            xlo,xhi=int(f['qmin']),int(f['qmax'])
        else:
            matrices=[]
            for phase,kh,kws in [('EE',[1],[1]),('EO',[1],[0,2]),
                                  ('OE',[0,2],[1]),('OO',[0,2],[0,2])]:
                matrices.append((phase,np.concatenate([w[:,:,a,b].T for a in kh for b in kws],axis=1)))
            xlo,xhi=int(f['qmin'])-int(f['zero']),int(f['qmax'])-int(f['zero'])
        # Bound every partial accumulation, including bias preload, rather than
        # just checking the final sum of the current reference picture.
        for phase,mat in matrices:
            lower=bias+np.minimum(np.minimum(mat*xlo,mat*xhi),0).sum(axis=1)
            upper=bias+np.maximum(np.maximum(mat*xlo,mat*xhi),0).sum(axis=1)
            assert np.all(lower>=-(1<<24)) and np.all(upper<(1<<24)), (name,phase,'ACC25 prefix overflow')
            bounds.append(dict(layer=name,phase=phase,min=int(lower.min()),max=int(upper.max())))
        off = len(config)
        for co in range(cout):
            b,m,s=int(bias[co]),int(L['requant_M'][co]),int(L['requant_shift'][co])
            check_cfg_bias_domain(style, name, co, b)
            assert -(1<<24)<=b<(1<<24), (name,co,b,'ACC25 preload')
            assert 0<m<=65535 and 0<=s<=31, (name,co,m,s)
            packed=(b&0xffffffff)|(m<<32)|(s<<48)
            assert (packed&0xffffffff)==(b&0xffffffff)
            assert ((packed>>32)&65535)==m and ((packed>>48)&31)==s
            config.append(packed)
        layers.append(dict(name=name,weight_base=base,weight_tiles=len(words)-base,
                           config_base=off,config_entries=cout,input_major=input_major,
                           ic=ic,oc=oc,ic_groups=ig,oc_groups=og))
    assert len(words)==(494 if compute_lanes==64 else 247) and len(config)==403
    image=b''.join(words)
    payload=image+b''.join(struct.pack('<Q',c) for c in config)
    crc=zlib.crc32(payload)&0xffffffff
    header=MAGIC|(abi<<32)|(sid<<40)|(len(words)<<48)|(403<<64)|(zero<<80)|(crc<<96)
    binary=(header.to_bytes(16,'little')+payload).ljust((16+len(payload)+15)//16*16,b'\0')
    d=out/style; d.mkdir(parents=True,exist_ok=True)
    (d/'profile.bin').write_bytes(binary)
    beats=[binary[i:i+16].ljust(16,b'\0') for i in range(0,len(binary),16)]
    (d/'profile.mem').write_text(''.join(f'{int.from_bytes(b,"little"):032x}\n' for b in beats))
    (d/'weights.mem').write_text(''.join(w[::-1].hex()+'\n' for w in words))
    (d/'config.mem').write_text(''.join(f'{c:016x}\n' for c in config))
    result=dict(style=style,style_id=sid,ddr_base=hex(0x700000+sid*0x10000),
                abi=abi,compute_lanes=compute_lanes,weight_tiles=len(words),weight_bits=word_bytes*8,
                weight_address_bits=9 if compute_lanes==64 else 8,
                trunk_zero=zero,bytes=len(binary),beats=len(beats),payload_crc32=f'{crc:08x}',
                source_contract_sha256=digest(p/'derived_contract.json'),
                source_weights_sha256=digest(p/'derived_weights.npz'),
                metadata=json.loads((p/'checkpoint_metadata.json').read_text()),
                layers=layers,outputs={n:digest(d/n) for n in ('profile.bin','profile.mem','weights.mem','config.mem')},
                accumulator_prefix_bounds=bounds,
                cfg_bias_storage_domain=dict(trunk_signed_bits=TRUNK_CFG_BIAS_W,
                                             decoder_signed_bits=DEC_CFG_BIAS_W,
                                             sign_extension_roundtrip='PASS'),
                hardware_status='NOT_VERIFIED',integer_weight_roundtrip='PASS')
    (d/'manifest.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return result

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'models/legacy_profiles')
    ap.add_argument('--compute-lanes',type=int,choices=[64,128],default=64)
    args=ap.parse_args()
    out=[build(args.source,args.output,s,STYLE_IDS[s],Path(__file__).resolve().parents[1]/'golden',args.compute_lanes) for s in STYLES]
    (args.output/'PROFILES.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    text=['`timescale 1ns/1ps',
          '// Generated by tools/build_style_profiles.py from retained profile packages (IDs 1/2/3).',
          'module d64_front_profile_rom(input wire [1:0] style,input wire [1:0] page,output reg [1023:0] words);',
          'always @* begin','case({style,page})']
    for style in STYLES:
        sid=STYLE_IDS[style]
        w=[int(x,16) for x in (args.output/style/'config.mem').read_text().split()]
        for page,a in enumerate((w[:8]+[0]*8,w[8:24],w[24:40],w[40:56])):
            value=sum(c<<(64*i) for i,c in enumerate(a))
            text.append(f"4'd{sid*4+page}: words=1024'h{value:0256x}; // {style} page {page}")
    text += ["default:words=1024'd0;",'endcase','end','endmodule']
    target=args.output/'generated/d64_front_profile_rom.v'
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text('\n'.join(text)+'\n',encoding='utf-8')
    text=['`timescale 1ns/1ps','// Generated from PROFILES.json; a header cannot relabel another payload.',
          'module d64_style_profile_manifest(input wire [1:0] style,output reg [31:0] crc,output reg [9:0] trunk_zero);',
          'always @* begin case(style)']
    for item in out:
        text.append(f"2'd{item['style_id']}:begin crc=32'h{item['payload_crc32']};trunk_zero=10'd{item['trunk_zero']};end")
    text+=['default:begin crc=0;trunk_zero=0;end','endcase end','endmodule']
    (args.output/'generated/d64_style_profile_manifest.v').write_text('\n'.join(text)+'\n',encoding='utf-8')
    print(f'PASS: three retained profiles, IDs1/2/3, ABI{out[0]["abi"]}, {out[0]["weight_tiles"]} tiles /403 config entries; integer tensors reconstructed.')
if __name__=='__main__':main()
