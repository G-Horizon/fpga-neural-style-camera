"""Independently rebuild each laser DXF and compare it with its STEP."""
from pathlib import Path
import json
import math

import cadquery as cq
import ezdxf
from build_plates import polygon_shape, cut_circle, bounds, THICKNESS

HERE=Path(__file__).resolve().parent
reports=[]
for path in sorted(HERE.glob('*R2*.dxf')):
    doc=ezdxf.readfile(path)
    audit=doc.audit()
    ms=doc.modelspace()
    outer=list(ms.query('LWPOLYLINE[layer=="OUTLINE"]'))
    assert len(outer)==1 and outer[0].closed
    rebuilt=polygon_shape(list(outer[0].get_points('xyb')))
    for poly in ms.query('LWPOLYLINE'):
        if poly is not outer[0]:
            assert poly.closed
            rebuilt=rebuilt.cut(polygon_shape(list(poly.get_points('xyb'))))
    for circle in ms.query('CIRCLE'):
        rebuilt=cut_circle(rebuilt,circle.dxf.center.x,circle.dxf.center.y,2*circle.dxf.radius)
    rebuilt=rebuilt.clean()
    wp=cq.importers.importStep(str(path.with_suffix('.step')))
    step=cq.Compound.makeCompound(wp.vals())
    assert rebuilt.isValid() and step.isValid()
    assert len(rebuilt.Solids())==len(step.Solids())==1
    delta_volume=abs(rebuilt.Volume()-step.Volume())
    assert delta_volume<1e-6,(path.name,delta_volume)
    delta_bounds=max(abs(a-b) for a,b in zip(bounds(rebuilt),bounds(step)))
    assert delta_bounds<1e-6
    difference=rebuilt.cut(step).Volume()+step.cut(rebuilt).Volume()
    assert difference<1e-6,(path.name,difference)
    center_checks=[]
    for circle in ms.query('CIRCLE'):
        p=cq.Vector(circle.dxf.center.x,circle.dxf.center.y,THICKNESS/2)
        assert not rebuilt.isInside(p,1e-7)
        center_checks.append([p.x,p.y])
    assert not audit.errors and not audit.fixes
    reports.append({'part':path.stem,'status':'PASS','dxf_units_mm':doc.header['$INSUNITS']==4,
                    'dxf_errors':len(audit.errors),'dxf_fixes':len(audit.fixes),
                    'closed_outer_polylines':1,'solid_count':1,'reconstructed_volume_mm3':rebuilt.Volume(),
                    'step_volume_mm3':step.Volume(),'volume_abs_error_mm3':delta_volume,
                    'bounds_abs_max_error_mm':delta_bounds,
                    'symmetric_BREP_difference_volume_mm3':difference,
                    'hole_center_void_checks':len(center_checks)})
camera=next(r for r in reports if r['part']=='02R2_camera_plate')
cam_doc=ezdxf.readfile(HERE/'02R2_camera_plate.dxf')
cam_points=list(next(iter(cam_doc.modelspace().query('LWPOLYLINE[layer=="OUTLINE"]'))).get_points('xy'))
body_height=max(p[1] for p in cam_points)-5
gap=50+2*0.5
clearance=gap-body_height
top_projection=5-clearance-THICKNESS
assert abs(body_height-50.7)<1e-7 and abs(clearance-.3)<1e-7 and abs(top_projection-2)<1e-7
loaded_parts={}
for name in ['01R2_mid_plate','02R2_camera_plate','03R2_base_plate','04R2_upper_carrier','05R2_s3_retainer']:
    loaded_parts[name]=cq.Compound.makeCompound(cq.importers.importStep(str(HERE/(name+'.step'))).vals())
assert not loaded_parts['01R2_mid_plate'].isInside(cq.Vector(105,6,THICKNESS/2),1e-7)
assert loaded_parts['03R2_base_plate'].isInside(cq.Vector(105,6,THICKNESS/2),1e-7)
assert not loaded_parts['04R2_upper_carrier'].isInside(cq.Vector(105,6,THICKNESS/2),1e-7)
assert loaded_parts['01R2_mid_plate'].isInside(cq.Vector(94,94,THICKNESS/2),1e-7)
assert not loaded_parts['04R2_upper_carrier'].isInside(cq.Vector(39.5,35.1,THICKNESS/2),1e-7)
old_root=HERE.parents[2]/'ti60_stack_assembly'/'components'/'plates'
unchanged=[]
for new,old in [('02R2_camera_plate','02R1_camera_plate'),('05R2_s3_retainer','05R1_s3_retainer')]:
    old_path=old_root/(old+'.step')
    if old_path.exists():
        old_shape=cq.Compound.makeCompound(cq.importers.importStep(str(old_path)).vals())
        new_shape=loaded_parts[new]
        difference=old_shape.cut(new_shape).Volume()+new_shape.cut(old_shape).Volume()
        assert difference<1e-6
        unchanged.append({'part':new,'R1_reference_read_only':str(old_path),'symmetric_difference_volume_mm3':difference})
out={'status':'PASS','date':'2026-10-22','method':'Read DXF from disk, rebuild every exact line/arc/circle, compare independent shape to STEP readback and check all circle centers are void.',
     'R2_architecture_checks':{'camera_slot_in01':True,'camera_slot_absent03':True,'camera_slot_in04':True,'rear_FPC_window_absent01':True,'microphone_acoustic_window_preserved04':True},
     'unchanged_R1_geometry':unchanged,
     'camera_axial_fit':{'body_height_measured_from_DXF_mm':body_height,
         'nominal_frame_inside_gap_mm':gap,'upper_shoulder_clearance_mm':clearance,
         'upper_tongue_projection_mm':top_projection,'lower_tongue_projection_mm':5-THICKNESS},
     'parts':reports,'manufacturing_notes':[
      'All four main plates are2.7mm. No countersinks, tapped holes or bottom-face engraving.',
      '01R2 and04R2 camera slot26.4x3.0mm is a prototype fit; confirm material thickness and kerf with a coupon.03R2 has no camera slot.01R2 has no rear FPC through-window.',
      '04R2 antenna notchY48.5..71.5 joins both pin-row windows; the former0.4mm ligaments are removed. Carrier tongue tip corners haveR0.8 fillets.',
      '05R2 edge ligament besideM2 holes is1.3mm; tighten gently and use nylon washers.'
     ]}
(HERE/'PLATE_VALIDATION.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2))
