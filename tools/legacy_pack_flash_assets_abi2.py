"""Package only the current three ABI2/64-MAC profiles; no hardware access.

--output may point at a review directory. No RTL or profile package is generated.
An ABI1 or mismatched package is refused before any output file is written.
"""
from pathlib import Path
import argparse,json
from profile_assets_abi2 import ROOT,validate_packages,validate_assets,sha,IMAGE_BYTES,SLOT_BYTES,PROGRAM_OFFSET,FLASH_BASE

def pack(profiles,output):
    packages=validate_packages(profiles)
    image=bytearray(b'\xff'*IMAGE_BYTES);records=[]
    for package in packages:
        offset=(package['style_id']-1)*SLOT_BYTES
        image[offset:offset+len(package['data'])]=package['data']
        records.append({k:package[k]for k in('style','style_id','bytes','sha256')})
        records[-1].update(flash_address=hex(package['flash_address']),ddr_address=hex(package['flash_address']))
    manifest=dict(abi=2,compute_lanes=64,weight_bits=512,weight_tiles=494,
                  program_offset=hex(PROGRAM_OFFSET),hardware_address_base=hex(FLASH_BASE),
                  retained_style_ids=[1,2,3],required_asset_mask='0x0e',flash_capacity=8388608,
                  image_bytes=len(image),sha256=sha(image),profiles=records)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    (output/'flash_assets.bin').write_bytes(image)
    (output/'flash_assets.hex').write_bytes(''.join(f'{byte:02x}\n'for byte in image).encode('ascii'))
    (output/'FLASH_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profiles',type=Path,default=ROOT/'models/legacy_abi2_profiles')
    ap.add_argument('--output',type=Path)
    ap.add_argument('--check',action='store_true',help='Read-onlystrict existingBIN/HEX/ABI2 consistency check')
    a=ap.parse_args()
    result=validate_assets(a.profiles)if a.check else pack(a.profiles,a.output or a.profiles)
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
