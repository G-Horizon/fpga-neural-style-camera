"""Exact millimetre geometry for the independent R2 acrylic plate set.

This generator never opens or changes the user's original SolidWorks parts.
Run with the project's CAD Python environment. STEP and DXF share the exact
same B-rep geometry. The DXF outer contour is taken after the edge-connected
cutouts have been resolved, so a laser cutter receives one closed outline.
"""
from __future__ import annotations

from pathlib import Path
import hashlib
import json
import math

import cadquery as cq
import ezdxf
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.BRep import BRep_Tool
from OCP.GC import GC_MakeArcOfCircle
from OCP.TopAbs import TopAbs_REVERSED
from OCP.TopExp import TopExp
from OCP.gp import gp_Pnt, gp_Vec

HERE = Path(__file__).resolve().parent
THICKNESS = 2.7
Q = math.tan(math.pi / 8)


def bounds(shape):
    b = shape.BoundingBox()
    return [b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax]


def rr_points(cx, cy, width, height, radius):
    if radius == 0:
        return [(cx-width/2, cy-height/2, 0), (cx+width/2, cy-height/2, 0),
                (cx+width/2, cy+height/2, 0), (cx-width/2, cy+height/2, 0)]
    assert 0 < radius <= min(width, height) / 2
    x0, x1, y0, y1 = cx-width/2, cx+width/2, cy-height/2, cy+height/2
    # bulge belongs to the arc departing from this vertex.
    return [(x0+radius, y0, 0), (x1-radius, y0, Q),
            (x1, y0+radius, 0), (x1, y1-radius, Q),
            (x1-radius, y1, 0), (x0+radius, y1, Q),
            (x0, y1-radius, 0), (x0, y0+radius, Q)]


def polygon_shape(points, z0=0, thickness=THICKNESS):
    wire = BRepBuilderAPI_MakeWire()
    for a, b in zip(points, points[1:]+points[:1]):
        x0, y0, bulge = a
        x1, y1, _ = b
        p0, p1 = gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z0)
        if math.dist((x0, y0), (x1, y1)) < 1e-10:
            continue
        if abs(bulge) < 1e-12:
            edge = BRepBuilderAPI_MakeEdge(p0, p1).Edge()
        else:
            dx, dy = x1-x0, y1-y0
            factor = (1-bulge*bulge)/(4*bulge)
            cx, cy = (x0+x1)/2-dy*factor, (y0+y1)/2+dx*factor
            r = math.hypot(x0-cx, y0-cy)
            ang = math.atan2(y0-cy, x0-cx) + 2*math.atan(bulge)
            pm = gp_Pnt(cx+r*math.cos(ang), cy+r*math.sin(ang), z0)
            edge = BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(p0, pm, p1).Value()).Edge()
        wire.Add(edge)
    assert wire.IsDone()
    face = BRepBuilderAPI_MakeFace(wire.Wire()).Face()
    return cq.Shape.cast(BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, thickness)).Shape())


def rr(cx, cy, w, h, r, z0=0, thickness=THICKNESS):
    return polygon_shape(rr_points(cx, cy, w, h, r), z0, thickness)


def cut_rr(shape, spec):
    return shape.cut(rr(*spec, z0=-0.1, thickness=THICKNESS+0.2))


def cut_circle(shape, x, y, diameter):
    tool = cq.Solid.makeCylinder(diameter/2, THICKNESS+0.2, cq.Vector(x,y,-0.1))
    return shape.cut(tool)


LOWER_FRAME = [(5,5), (5,95), (115,95), (115,15)]
UPPER_FRAME = [(5,5), (5,60), (115,95), (115,15)]
FPGA_HOLES = [(13,13,2.2),(13,48.814,2.2),(106.98,13,2.2),
              (106.98,87,2.2),(18.5,83,3.1),(48.5,83,3.1)]
CAMERA_SLOT = [105,6,26.4,3.0,0]
CAMERA_FPC_SLOT = [15,18.019043021308992,20,3,0]


def wire_points(wire):
    """Exact ordered DXF bulges, respecting OCCT edge/axis orientation."""
    exp = BRepTools_WireExplorer(wire.wrapped)
    points = []
    while exp.More():
        edge = exp.Current()
        start = BRep_Tool.Pnt_s(TopExp.FirstVertex_s(edge, True))
        end = BRep_Tool.Pnt_s(TopExp.LastVertex_s(edge, True))
        curve = BRepAdaptor_Curve(edge)
        wrapped = cq.Edge.cast(edge)
        bulge = 0.0
        if wrapped.geomType() == 'CIRCLE':
            circle = curve.Circle()
            center = circle.Location()
            a0 = math.atan2(start.Y()-center.Y(), start.X()-center.X())
            a1 = math.atan2(end.Y()-center.Y(), end.X()-center.X())
            sign = ( -1 if edge.Orientation()==TopAbs_REVERSED else 1) * circle.Axis().Direction().Z()
            delta = ((a1-a0) % (2*math.pi)) if sign > 0 else -((a0-a1) % (2*math.pi))
            bulge = math.tan(delta/4)
        else:
            assert wrapped.geomType() == 'LINE', wrapped.geomType()
        points.append((start.X(), start.Y(), bulge))
        exp.Next()
    return points


def export_dxf(shape, path, hole_circles):
    doc = ezdxf.new('R2010')
    doc.units = ezdxf.units.MM
    doc.header['$INSUNITS'] = 4
    doc.header['$MEASUREMENT'] = 1
    for name, color in [('OUTLINE',7),('HOLES',1),('SLOT',3),('CUTOUT',4)]:
        doc.layers.new(name, dxfattribs={'color':color})
    ms = doc.modelspace()
    faces = [f for f in shape.Faces() if f.geomType()=='PLANE' and abs(f.Center().z)<1e-6]
    face = max(faces, key=lambda f:f.Area())
    outer = face.outerWire()
    ms.add_lwpolyline(wire_points(outer),format='xyb',close=True,dxfattribs={'layer':'OUTLINE'})
    # Direct circles are exact and simplest for native SolidWorks hole sketches.
    for x,y,d in hole_circles:
        ms.add_circle((x,y),d/2,dxfattribs={'layer':'HOLES'})
    inner = face.innerWires()
    circular_count = 0
    noncircular = 0
    for wire in inner:
        if all(e.geomType()=='CIRCLE' for e in wire.Edges()):
            circular_count += 1
            continue
        noncircular += 1
        ms.add_lwpolyline(wire_points(wire),format='xyb',close=True,
                         dxfattribs={'layer':'SLOT' if noncircular%2 else 'CUTOUT'})
    assert circular_count == len(hole_circles),(path,circular_count,len(hole_circles))
    doc.saveas(path)
    back = ezdxf.readfile(path)
    audit = back.audit()
    assert not audit.errors and not audit.fixes
    outlines = list(back.modelspace().query('LWPOLYLINE[layer=="OUTLINE"]'))
    assert len(outlines)==1 and outlines[0].closed
    assert all(p.closed for p in back.modelspace().query('LWPOLYLINE'))
    return {'closed_outer_contours':1,'circular_holes':circular_count,
            'closed_inner_profiles':noncircular,'audit_errors':0,'audit_fixes':0,
            'outer_points_xyb_mm':wire_points(outer),
            'inner_points_xyb_mm':[wire_points(w) for w in inner if not all(e.geomType()=='CIRCLE' for e in w.Edges())]}


def mesh(shape, name):
    verts, triangles = shape.tessellate(0.045, 0.12)
    flat, normals = [], []
    for tri in triangles:
        a,b,c = [verts[i] for i in tri]
        n = (b-a).cross(c-a).normalized()
        for v in (a,b,c):
            flat.extend([round(v.x,6),round(v.y,6),round(v.z,6)])
            normals.extend([round(n.x,6),round(n.y,6),round(n.z,6)])
    return {'name':name,'category':'acrylic','color':[0.47,0.75,0.83],
            'vertices':flat,'normals':normals}


def build():
    all_specs=[]
    for name in ['01R2_mid_plate','02R2_camera_plate','03R2_base_plate',
                 '04R2_upper_carrier','05R2_s3_retainer']:
        spec={'name':name,'units':'mm','thickness_mm':THICKNESS,'holes':[],
              'rounded_rectangle_cuts':[],'outline_vertices_xyb_mm':[]}
        if name=='02R2_camera_plate':
            points=[(0,0,0),(2,0,0),(2,-5,0),(28,-5,0),(28,0,0),(30,0,0),
                    (30,50.7,0),(28,50.7,0),(28,55.7,0),(2,55.7,0),(2,50.7,0),(0,50.7,0)]
            shape=polygon_shape(points)
            spec['holes']=[(4,22.5,2.2),(26,22.5,2.2),(4,37,2.2),(26,37,2.2)]
            spec['rounded_rectangle_cuts']=[CAMERA_FPC_SLOT]
            spec['body_height_mm']=50.7
            spec['tongue_width_length_mm']=[26,5]
            spec['total_height_mm']=60.7
            spec['frame_nominal_inside_gap_mm']=51
            spec['upper_shoulder_clearance_mm']=0.3
            spec['upper_tongue_projection_beyond_upper_plate_mm']=2.0
            spec['lower_tongue_projection_beyond_middle_plate_mm']=2.3
            spec['installation']={'rotation_axis':'X','rotation_degrees':90,
               'translation_mm':[90,7.35,14.4],'lower_shoulder_global_Z_mm':14.4,
               'upper_shoulder_global_Z_mm':65.1,'lower_support_plate':'01R2_mid_plate',
               'upper_support_plate':'04R2_upper_carrier'}
            spec['axial_fit_revision_reason']='A51mm rigid shoulder span against a nominal51mm standoff stack has no tolerance for actual acrylic, washer, pillar or cutting variation. Body50.7 allows0.3mm axial clearance while the5mm upper tongue still traverses the2.7mm plate and projects2.0mm.'
        elif name=='05R2_s3_retainer':
            shape=rr(0,0,5,46,2)
            spec['holes']=[(0,-20,2.4),(0,20,2.4)]
            spec['installation']={'translate_xy_mm':[78,60],
                                  'bottom_above_upper_carrier_top_mm':4.0,
                                  'nylon_washers_each_end':8}
            spec['silicone_contact_pad']={'center_global_xy_mm':[78,69],
                   'footprint_xy_mm':[3,1.5],'free_thickness_mm':2.5,
                   'compressed_model_thickness_mm':2.4,'compression_mm':0.1,
                   'reason':'Previous point78,61.3 intersects resistorR8. New point78,69 is clear of component envelopes in the existing S3 model.'}
        else:
            shape=rr(60,50,120,100,4)
            spec['external_size_mm']=[120,100]
            spec['corner_radius_mm']=4
            if name=='01R2_mid_plate':
                union_frames=list(dict.fromkeys(LOWER_FRAME+UPPER_FRAME))
                spec['holes']=FPGA_HOLES+[(x,y,2.4) for x,y in union_frames]
                spec['rounded_rectangle_cuts']=[CAMERA_SLOT]
                spec['R2_revision_reason']='Camera now sits between FPGA middle and S3 upper layers. Keep the lower camera tongue slot in01, remove the obsolete rear FPC through-window because the FPC remains in the same upper stage.'
                spec['lower_frame_centers_mm']=LOWER_FRAME
                spec['upper_frame_centers_mm']=UPPER_FRAME
            elif name=='03R2_base_plate':
                spec['holes']=[(x,y,2.4) for x,y in LOWER_FRAME]
                spec['rounded_rectangle_cuts']=[]
                spec['R2_revision_reason']='Base-to-middle layer uses shortFF8 pillars. Camera is no longer in this lower stage, so remove its former base-plate tongue slot.'
                spec['adhesive_feet']={'diameter_mm':10,'height_mm':5,
                       'centers_mm':[(12,12),(12,88),(108,88),(108,22)],'drill_holes':False}
            else:
                spec['holes']=[(x,y,2.4) for x,y in UPPER_FRAME]+[(78,40,2.4),(78,80,2.4)]
                spec['rounded_rectangle_cuts']=[
                    [82.5,47.3,58,3.6,1.2],[82.5,72.7,58,3.6,1.2],
                    [114.55,60,11.1,23,0],
                    [39.9,29,5,2.4,1.2],[39.9,41,5,2.4,1.2],
                    [39.5,35.1,6,6,0.8],[31.25,35,3.5,16,0.8],
                    CAMERA_SLOT]
                spec['R2_revision_reason']='Add26.4x3.0 slot at105,6 for the upper camera tongue. Other microphone/S3 features and allXY hole centers remain unchanged.'
                spec['S3_origin_xy_mm']=[54,60]
                spec['microphone_center_xy_mm']=[36,35]
                spec['edge_connected_cutout']='antenna notch reaches outer +X edge and joins both pin-row windows'
                spec['antenna_notch_bounds_mm']=[109,48.5,120.1,71.5]
                spec['design_revision_reason']='The initial Y49.5..70.5 notch left two fragile0.4mm acrylic ligaments. Expanding to48.5..71.5 overlaps each row window by0.6mm, giving an open-ended U-shaped carrier with a single connected left root.'
                spec['carrier_tongue_tip_corner_radius_mm']=0.8
        for hole in spec['holes']:
            shape=cut_circle(shape,*hole)
        for cut in spec['rounded_rectangle_cuts']:
            shape=cut_rr(shape,cut)
        if name=='04R2_upper_carrier':
            targets=[]
            for edge in shape.Edges():
                a,b=edge.startPoint(),edge.endPoint()
                if edge.geomType()=='LINE' and abs(a.x-b.x)<1e-7 and abs(a.y-b.y)<1e-7 and abs(abs(a.z-b.z)-THICKNESS)<1e-7:
                    if abs(a.x-109)<1e-7 and min(abs(a.y-49.1),abs(a.y-70.9))<1e-7:
                        targets.append(edge)
            assert len(targets)==2,(name,len(targets))
            shape=shape.Solids()[0].fillet(0.8,targets)
        shape=shape.clean()
        assert shape.isValid(), name
        assert len(shape.Solids())==1,(name,len(shape.Solids()))
        path=HERE/(name+'.step')
        cq.exporters.export(shape,str(path),exportType='STEP')
        loaded=cq.importers.importStep(str(path))
        again=cq.Compound.makeCompound(loaded.vals())
        assert again.isValid() and len(again.Solids())==1
        assert abs(again.Volume()-shape.Volume())<1e-6,(name,again.Volume(),shape.Volume())
        dxf=export_dxf(shape,HERE/(name+'.dxf'),spec['holes'])
        spec['geometry_validation']={'valid':True,'solid_count':1,'volume_mm3':shape.Volume(),
                                    'bounds_mm':bounds(shape),'step_readback_valid':True,
                                    'step_readback_volume_abs_error_mm3':abs(again.Volume()-shape.Volume()),
                                    'step_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        spec['dxf_geometry']=dxf
        (HERE/(name+'.json')).write_text(json.dumps(spec,ensure_ascii=False,indent=2),encoding='utf-8')
        all_specs.append(spec)
        yield shape,spec
    (HERE/'parameters.json').write_text(json.dumps({'revision':'R2','units':'mm','parts':all_specs},ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':
    result=list(build())
    meshes=[mesh(s,spec['name']) for s,spec in result]
    (HERE/'preview_geometry.json').write_text(json.dumps({'parts':meshes,'units':'mm',
                 'native_part_coordinates':True},separators=(',',':')),encoding='utf-8')
    print(json.dumps([{'name':spec['name'],**spec['geometry_validation']} for _,spec in result],indent=2))
