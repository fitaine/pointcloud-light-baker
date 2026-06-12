"""
export_volume.py — export Blender volume objects as density grids for the
web viewer's ray-marcher. Read-only: the .blend is never modified or saved.

Handles two volume recipes:
  - VOLUME objects built with [Mesh to Volume] + optional [Volume Displace]
    modifiers (the "Anneau de nuage" recipe)
  - MESH objects with a Volume Scatter / Principled Volume material —
    uniform density inside the mesh (node-driven Density networks are NOT
    reconstructed; the socket default value is used)

Blender exposes no API to read modifier-generated grids, so the script
reconstructs them exactly:

  - the undisplaced volume is rebuilt in Geometry Nodes (Mesh to Density
    Grid on the modifier's source object, world space) and sampled with
    Sample Grid
  - Volume Displace is replicated from Blender's own source semantics:
    grid is sampled at  p - (texture(p) - mid_level) * strength,
    evaluating the SAME texture datablock via tex.evaluate()

Output per volume object, next to this script (or --out):
  <slug>_density.npy   float32 (NX, NY, NZ) grid
  <slug>_meta.json     world bbox, shape, shader params, displace recipe

Usage:
  blender --background scene.blend --python export_volume.py -- [--out DIR]
      [--res 224] [--object "Name"]
"""

import bpy
import json
import os
import re
import sys
import time
import numpy as np

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
def arg(flag, default=None):
    return argv[argv.index(flag) + 1] if flag in argv else default

OUT_DIR  = arg('--out', os.path.dirname(os.path.abspath(__file__)))
LONG_RES = int(arg('--res', 224))
ONLY     = arg('--object')
PAD_FACTOR = 1.3   # bbox padding for displacement overshoot

os.makedirs(OUT_DIR, exist_ok=True)


def build_sampler(source_obj, density, voxel_size):
    """GN rig: source mesh → density grid → sampled at probe positions."""
    ng = bpy.data.node_groups.new('_VOLEXPORT_NG', 'GeometryNodeTree')
    ng.interface.new_socket('Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
    ng.interface.new_socket('Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')
    n_in, n_out = ng.nodes.new('NodeGroupInput'), ng.nodes.new('NodeGroupOutput')
    n_obj = ng.nodes.new('GeometryNodeObjectInfo')
    n_obj.inputs['Object'].default_value = source_obj
    n_obj.transform_space = 'RELATIVE'      # world space — probes are world
    n_m2g = ng.nodes.new('GeometryNodeMeshToDensityGrid')
    n_m2g.inputs['Density'].default_value = density
    n_m2g.inputs['Voxel Size'].default_value = voxel_size
    n_m2g.inputs['Gradient Width'].default_value = voxel_size * 0.1
    n_smp = ng.nodes.new('GeometryNodeSampleGrid')
    n_smp.data_type = 'FLOAT'
    n_pos = ng.nodes.new('GeometryNodeInputPosition')
    n_sto = ng.nodes.new('GeometryNodeStoreNamedAttribute')
    n_sto.data_type, n_sto.domain = 'FLOAT', 'POINT'
    n_sto.inputs['Name'].default_value = 'd'
    lk = ng.links.new
    lk(n_obj.outputs['Geometry'], n_m2g.inputs['Mesh'])
    lk(n_m2g.outputs[0], n_smp.inputs['Grid'])
    lk(n_pos.outputs['Position'], n_smp.inputs['Position'])
    lk(n_in.outputs['Geometry'], n_sto.inputs['Geometry'])
    lk(n_smp.outputs['Value'], n_sto.inputs['Value'])
    lk(n_sto.outputs['Geometry'], n_out.inputs['Geometry'])
    return ng


def world_bbox(obj):
    from mathutils import Vector
    pts = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    mn = np.array([min(p[i] for p in pts) for i in range(3)])
    mx = np.array([max(p[i] for p in pts) for i in range(3)])
    return mn, mx


def volume_shader_params(obj):
    """Read volume shader params — Principled Volume or plain Volume Scatter."""
    for slot in obj.material_slots:
        mat = slot.material
        if not (mat and mat.node_tree):
            continue
        pv = next((n for n in mat.node_tree.nodes
                   if n.type == 'PRINCIPLED_VOLUME'), None)
        if pv:
            g = lambda nm: pv.inputs[nm].default_value
            return {'color': list(g('Color'))[:3],
                    'density_input': g('Density'),
                    'anisotropy': g('Anisotropy'),
                    'emission_strength': g('Emission Strength'),
                    'emission_color': list(g('Emission Color'))[:3]}
        vs = next((n for n in mat.node_tree.nodes
                   if n.type == 'VOLUME_SCATTER'), None)
        if vs:
            if vs.inputs['Density'].is_linked:
                print("  WARNING: Volume Scatter Density is node-driven — "
                      "using the socket default value (texture networks "
                      "are not reconstructed)")
            g = lambda nm: vs.inputs[nm].default_value
            return {'color': list(g('Color'))[:3],
                    'density_input': g('Density'),
                    'anisotropy': g('Anisotropy'),
                    'emission_strength': 0.0,
                    'emission_color': [1.0, 1.0, 1.0]}
    return None


def has_volume_material(obj):
    """MESH whose material plugs a volume shader into the output's Volume."""
    if obj.type != 'MESH':
        return False
    for slot in obj.material_slots:
        mat = slot.material
        if not (mat and mat.node_tree):
            continue
        if any(n.type in ('VOLUME_SCATTER', 'PRINCIPLED_VOLUME')
               for n in mat.node_tree.nodes):
            return True
    return False


def export_volume(obj):
    slug = re.sub(r'[^a-z0-9]+', '-', obj.name.lower()).strip('-')
    print(f"\n[VolExport] {obj.name} → {slug}")
    m2v = next((m for m in obj.modifiers if m.type == 'MESH_TO_VOLUME'), None)
    if obj.type == 'MESH':
        # Mesh with a Volume Scatter / Principled Volume material: the volume
        # is "uniform density inside the mesh" — the mesh itself is the source.
        m2v, disp, src = None, None, obj
    elif m2v is None or m2v.object is None:
        print("  no Mesh to Volume modifier — skipped (unsupported volume type)")
        return
    else:
        disp = next((m for m in obj.modifiers if m.type == 'VOLUME_DISPLACE'), None)
        src = m2v.object
    mn, mx = world_bbox(src)
    pad = (disp.strength * PAD_FACTOR) if disp else (mx - mn).max() * 0.05
    mn, mx = mn - pad, mx + pad
    ext = mx - mn
    long_axis = ext.max()
    res = (np.maximum(8, np.round(LONG_RES * ext / long_axis))).astype(int)
    NX, NY, NZ = (int(v) for v in res)
    # source voxel size: replicate VOXEL_AMOUNT semantics on the source bbox
    src_ext = world_bbox(src)[1] - world_bbox(src)[0]
    if m2v is None:
        vsize = float(long_axis / LONG_RES)   # mesh volume: match export cell
    elif m2v.resolution_mode == 'VOXEL_AMOUNT':
        vsize = float(src_ext.max() / max(1, m2v.voxel_amount))
    else:
        vsize = m2v.voxel_size
    vsize = max(vsize, long_axis / 2048)   # don't build absurdly fine grids
    print(f"  grid {NX}x{NY}x{NZ}  voxel {((mx-mn)/res).round(1).tolist()} m  "
          f"src voxel {vsize:.2f} m")

    ng = build_sampler(src, m2v.density if m2v else 1.0, vsize)
    mesh = bpy.data.meshes.new('_VOLEXPORT_M')
    mesh.vertices.add(NX * NY)
    probe = bpy.data.objects.new('_VOLEXPORT_PROBE', mesh)
    bpy.context.scene.collection.objects.link(probe)
    probe.modifiers.new('smp', 'NODES').node_group = ng
    dg = bpy.context.evaluated_depsgraph_get()

    xs = np.linspace(mn[0], mx[0], NX)
    ys = np.linspace(mn[1], mx[1], NY)
    zs = np.linspace(mn[2], mx[2], NZ)
    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    flatx, flaty = gx.ravel(), gy.ravel()
    n = NX * NY
    co = np.empty((n, 3), dtype=np.float32)
    dens = np.empty((NX, NY, NZ), dtype=np.float32)

    tex = disp.texture if disp else None
    mid = np.array(disp.texture_mid_level) if disp else None
    strength = disp.strength if disp else 0.0
    # NOTE: displacement positions use the VOLUME object's local space
    # (texture_map_mode LOCAL). If the volume object has a non-identity
    # transform, convert here. Anneau de nuage is identity.
    if disp and tuple(obj.matrix_world.translation) != (0.0, 0.0, 0.0):
        print("  WARNING: volume object has a transform — LOCAL texture "
              "mapping not converted (extend export_volume.py)")

    t0 = time.time()
    for k, z in enumerate(zs):
        if tex is not None:
            ev = tex.evaluate
            for i in range(n):
                ti = ev((flatx[i], flaty[i], z))[3]
                co[i, 0] = flatx[i] - (ti - mid[0]) * strength
                co[i, 1] = flaty[i] - (ti - mid[1]) * strength
                co[i, 2] = z        - (ti - mid[2]) * strength
        else:
            co[:, 0], co[:, 1], co[:, 2] = flatx, flaty, z
        mesh.vertices.foreach_set('co', co.ravel())
        mesh.update()
        dg.update()
        vals = np.empty(n, dtype=np.float32)
        probe.evaluated_get(dg).data.attributes['d'].data.foreach_get('value', vals)
        dens[:, :, k] = vals.reshape(NX, NY)
    print(f"  sampled in {time.time()-t0:.0f}s  max {dens.max():.3f}  "
          f"occupancy {(dens > 0.005).mean()*100:.1f}%")

    np.save(os.path.join(OUT_DIR, f'{slug}_density.npy'), dens)
    meta = {'object': obj.name, 'min': mn.tolist(), 'max': mx.tolist(),
            'shape': [NX, NY, NZ], 'frame': 'blend-world',
            'shader': volume_shader_params(obj)}
    if disp:
        meta['displace'] = {'strength': strength, 'mid_level': mid.tolist(),
                            'texture': f'{tex.type} {getattr(tex, "noise_basis", "")}'}
    with open(os.path.join(OUT_DIR, f'{slug}_meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    print(f"  → {slug}_density.npy / {slug}_meta.json")

    bpy.data.objects.remove(probe, do_unlink=True)
    bpy.data.node_groups.remove(ng)


def export_lights():
    """Scene lights for the volume lighting bake — generic, any .blend."""
    from mathutils import Vector
    lights = []
    for o in bpy.context.scene.objects:
        if o.type != 'LIGHT' or o.hide_render:
            continue
        L = o.data
        d = {'name': o.name, 'type': L.type, 'energy': L.energy,
             'color': list(L.color),
             'location': list(o.matrix_world.translation)}
        if L.type in ('SPOT', 'SUN', 'AREA'):
            d['direction'] = list(o.matrix_world.to_3x3() @ Vector((0, 0, -1)))
        if L.type == 'SPOT':
            d['spot_size'], d['spot_blend'] = L.spot_size, L.spot_blend
        lights.append(d)
    w = bpy.context.scene.world
    if w and w.node_tree:
        bg = next((n for n in w.node_tree.nodes if n.type == 'BACKGROUND'), None)
        if bg:
            lights.append({'name': '__world__', 'type': 'WORLD',
                           'color': list(bg.inputs['Color'].default_value)[:3],
                           'energy': bg.inputs['Strength'].default_value})
    path = os.path.join(OUT_DIR, 'scene_lights.json')
    with open(path, 'w') as f:
        json.dump(lights, f, indent=1)
    print(f"[VolExport] {len(lights)} lights → {path}")


export_lights()

targets = [o for o in bpy.context.scene.objects
           if (o.type == 'VOLUME' or has_volume_material(o))
           and (ONLY is None or o.name == ONLY)]
if not targets:
    print("[VolExport] no VOLUME objects or volume-material meshes found" +
          (f" named '{ONLY}'" if ONLY else ""))
for obj in targets:
    export_volume(obj)
print("\n[VolExport] done — .blend NOT modified or saved.")
