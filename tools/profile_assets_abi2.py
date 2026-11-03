"""Read-only validation of current three-style ABI2/64-MAC boot assets.

Checks binary headers, payload CRC, zero padding, every .mem representation,
manifest hashes, Flash slot placement, and the actual programmer HEX bytes.
This module never invokes the programmer or writes production assets.
"""
from pathlib import Path
import argparse, hashlib, json, re, struct, zlib

ROOT=Path(__file__).resolve().parents[1]
STYLES=((1,'vangogh'),(2,'ukiyoe'),(3,'inkwash'))
MAGIC=0x31545344
ABI=2
COMPUTE_LANES=64
WEIGHT_TILES=494
CONFIG_ENTRIES=403
WEIGHT_BYTES=WEIGHT_TILES*COMPUTE_LANES
PAYLOAD_BYTES=WEIGHT_BYTES+CONFIG_ENTRIES*8
PROFILE_BYTES=34864
PROFILE_BEATS=2179
FLASH_BASE=0x700000
PROGRAM_OFFSET=0x710000
SLOT_BYTES=0x10000
IMAGE_BYTES=3*SLOT_BYTES
PARTITION_BYTES=4*SLOT_BYTES

def sha(data):return hashlib.sha256(data).hexdigest()

def require(condition,message):
    if not condition:raise ValueError(message)

def number(value,label):
    if type(value)is int:return value
    if isinstance(value,str):
        try:return int(value,0)
        except ValueError:pass
    raise ValueError(label+': expected literal integer/hex address')

def plain_hex(path,word_bytes):
    """Official Efinity plain HEX/.mem: one fixed-width hex word per line."""
    words=path.read_text(encoding='ascii').splitlines()
    require(bool(words),str(path)+': empty hex image')
    require(all(re.fullmatch('[0-9a-fA-F]{'+str(word_bytes*2)+'}',line)for line in words),
            str(path)+': expected one '+str(word_bytes*2)+'-digit word per line')
    return [int(line,16).to_bytes(word_bytes,'little')for line in words]

def validate_profile_bytes(data,sid,label='profile'):
    require(len(data)==PROFILE_BYTES,label+': expected2179 aligned128-bit beats /34864 bytes')
    magic,abi,style,tiles,configs,zero,crc=struct.unpack('<IBBHHHI',data[:16])
    require(magic==MAGIC,label+': bad DST1 magic')
    require(abi==ABI,label+': expectedABI2 for64 MAC; ABI1/128-MAC packages are incompatible')
    require(style==sid and sid in(1,2,3),label+': retained physical style ID must be1/2/3')
    require(tiles==WEIGHT_TILES,label+': expected494 x512-bit weight tiles')
    require(configs==CONFIG_ENTRIES,label+': expected403 config entries')
    require(zero<=1023,label+': reserved trunk-zero header bits are nonzero')
    payload=data[16:16+PAYLOAD_BYTES]
    require((zlib.crc32(payload)&0xffffffff)==crc,label+': payload CRC32 mismatch')
    require(data[16+PAYLOAD_BYTES:]==bytes(8),label+': final high64 padding must be zero')
    return dict(style_id=sid,abi=abi,weight_tiles=tiles,config_entries=configs,
                trunk_zero=zero,payload_crc32=f'{crc:08x}',bytes=len(data),beats=PROFILE_BEATS,sha256=sha(data))

def validate_packages(profiles):
    profiles=Path(profiles)
    aggregate=json.loads((profiles/'PROFILES.json').read_text(encoding='utf-8'))
    require(isinstance(aggregate,list)and[(r.get('style_id'),r.get('style'))for r in aggregate]==list(STYLES),
            'PROFILES.json: exactly retained style IDs1/2/3 required')
    packages=[]
    for sid,style in STYLES:
        folder=profiles/style;manifest=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
        data=(folder/'profile.bin').read_bytes()
        info=validate_profile_bytes(data,sid,style)
        weights=plain_hex(folder/'weights.mem',64)
        config=plain_hex(folder/'config.mem',8)
        beats=plain_hex(folder/'profile.mem',16)
        require(len(weights)==494 and b''.join(weights)==data[16:16+WEIGHT_BYTES],style+': weight image differs from profile payload')
        require(len(config)==403 and b''.join(config)==data[16+WEIGHT_BYTES:16+PAYLOAD_BYTES],style+': config image differs from profile payload')
        require(len(beats)==2179 and b''.join(beats)==data,style+': profile.mem differs from profile.bin')
        expected=dict(style=style,style_id=sid,abi=2,compute_lanes=64,weight_bits=512,
                      weight_tiles=494,weight_address_bits=9,trunk_zero=info['trunk_zero'],
                      bytes=34864,beats=2179,payload_crc32=info['payload_crc32'])
        for doc,label in [(manifest,style+'/manifest.json'),(aggregate[sid-1],'PROFILES.json/'+style)]:
            for key,value in expected.items():require(doc.get(key)==value,label+': stale '+key+' metadata')
            require(number(doc.get('ddr_base'),label+'/ddr_base')==FLASH_BASE+sid*SLOT_BYTES,label+': DDR profile slot differs')
            for filename in('profile.bin','profile.mem','weights.mem','config.mem'):
                require(doc.get('outputs',{}).get(filename)==sha((folder/filename).read_bytes()),label+': stale '+filename+' hash')
        packages.append(dict(info,style=style,data=data,flash_address=FLASH_BASE+sid*SLOT_BYTES))
    return packages

def validate_assets(profiles=ROOT/'models/legacy_abi2_profiles',require_programmer_hex=True):
    profiles=Path(profiles);packages=validate_packages(profiles)
    path=profiles/'FLASH_MANIFEST.json';manifest=json.loads(path.read_text(encoding='utf-8'))
    expected={'abi':2,'compute_lanes':64,'weight_bits':512,'weight_tiles':494,
              'retained_style_ids':[1,2,3],'image_bytes':IMAGE_BYTES}
    for key,value in expected.items():require(manifest.get(key)==value,'FLASH_MANIFEST.json: stale '+key)
    require(number(manifest.get('program_offset'),'program_offset')==PROGRAM_OFFSET,'Flash image must start at0x710000')
    require(number(manifest.get('hardware_address_base'),'hardware_address_base')==FLASH_BASE,'Hardware profile base must remain0x700000')
    require(number(manifest.get('required_asset_mask'),'required_asset_mask')==14,'Asset-ready mask must be0x0e')
    capacity=number(manifest.get('flash_capacity'),'flash_capacity')
    require(PROGRAM_OFFSET+IMAGE_BYTES<=capacity,'Flash asset span exceeds capacity')
    image=(profiles/'flash_assets.bin').read_bytes()
    require(len(image)==IMAGE_BYTES and sha(image)==manifest.get('sha256'),'Flash BIN size/SHA mismatch')
    hex_matches=b''.join(plain_hex(profiles/'flash_assets.hex',1))==image
    if require_programmer_hex:require(hex_matches,'Programmer HEX bytes differ from checked BIN')
    records=manifest.get('profiles',[])
    require([(r.get('style_id'),r.get('style'))for r in records]==list(STYLES),'Flash profile inventory must be exactly IDs1/2/3')
    expected_image=bytearray(b'\xff'*IMAGE_BYTES)
    for package,record in zip(packages,records):
        for key in ('style','style_id','bytes','sha256'):require(record.get(key)==package[key],'Flash record differs: '+key)
        for key in ('flash_address','ddr_address'):
            require(number(record.get(key),key)==package['flash_address'],'Flash/DDR slot differs for '+package['style'])
        off=(package['style_id']-1)*SLOT_BYTES
        expected_image[off:off+PROFILE_BYTES]=package['data']
    require(bytes(expected_image)==image,'Flash slot data/padding differs from the three checked profiles')
    return dict(abi=2,compute_lanes=64,weight_bits=512,weight_tiles=494,weight_address_bits=9,
                config_entries=403,profile_bytes=34864,profile_beats=2179,
                program_offset='0x710000',hardware_address_base='0x700000',image_bytes=IMAGE_BYTES,
                image_sha256=sha(image),hex_sha256=sha((profiles/'flash_assets.hex').read_bytes()),
                programmer_hex_matches_bin=hex_matches,
                flash_manifest_sha256=sha(path.read_bytes()),required_asset_mask='0x0e',
                retained_style_ids=[1,2,3],profiles=[{k:v for k,v in p.items()if k!='data'}for p in packages])

def validate_project_assets(project,require_programmer_hex=True):
    project=Path(project);summary=validate_assets(project/'profiles',require_programmer_hex)
    text=(project/'rtl/style/d64_style_profile_manifest.v').read_text(encoding='utf-8')
    rows=re.findall(r"2'd([1-3])\s*:\s*begin\s+crc\s*=\s*32'h([0-9a-fA-F]{8})\s*;\s*trunk_zero\s*=\s*10'd([0-9]+)\s*;\s*end",text)
    require(len(rows)==3,'Compiled style manifest must contain exactly three retained CRC/zero entries')
    for sid,crc,zero in rows:
        info=summary['profiles'][int(sid)-1]
        require(crc.lower()==info['payload_crc32']and int(zero)==info['trunk_zero'],'Compiled CRC/zero differs from ABI2 profile '+sid)
    loader=(project/'rtl/style/d64_style_loader.v').read_text(encoding='utf-8')
    require(re.search(r"stream_data\[39:32\]\s*!=\s*8'd2\b",loader)is not None,'Compiled loader does not explicitly requireABI2')
    require(re.search(r"stream_data\[63:48\]\s*!=\s*16'd494\b",loader)is not None,'Compiled loader does not explicitly require494 tiles')
    require(re.search(r"stream_data\[79:64\]\s*!=\s*16'd403\b",loader)is not None,'Compiled loader config count differs')
    summary['compiled_profile_manifest_sha256']=sha((project/'rtl/style/d64_style_profile_manifest.v').read_bytes())
    summary['compiled_loader_sha256']=sha((project/'rtl/style/d64_style_loader.v').read_bytes())
    return summary

def verify_readback(path,address,profiles=ROOT/'models/legacy_abi2_profiles'):
    path=Path(path);summary=validate_assets(profiles,require_programmer_hex=False)
    data=b''.join(plain_hex(path,1))if path.suffix.lower()=='.hex'else path.read_bytes()
    off=PROGRAM_OFFSET-address
    require(off>=0 and off+IMAGE_BYTES<=len(data),'Readback does not cover the complete0x710000..0x73ffff asset span')
    require(data[off:off+IMAGE_BYTES]==(Path(profiles)/'flash_assets.bin').read_bytes(),'Readback ABI2 assets differ from checked local image')
    return dict(address=hex(address),readback_bytes=len(data),sha256=sha(data),asset_bytes=IMAGE_BYTES,
                asset_sha256=summary['image_sha256'],matched=True)

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--profiles',type=Path,default=ROOT/'models/legacy_abi2_profiles')
    ap.add_argument('--project',type=Path);ap.add_argument('--readback',type=Path)
    ap.add_argument('--binary-only',action='store_true',help='Check all ABI2 BIN/MEM/manifest fields, report programmer HEX mismatch separately; never approve that HEX for programming')
    ap.add_argument('--readback-address',type=lambda s:int(s,0),default=PROGRAM_OFFSET)
    a=ap.parse_args();summary=validate_project_assets(a.project,not a.binary_only)if a.project else validate_assets(a.profiles,not a.binary_only)
    if a.readback:summary['readback']=verify_readback(a.readback,a.readback_address,a.project/'profiles'if a.project else a.profiles)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
