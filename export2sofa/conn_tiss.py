import bpy
import math
import numpy as np
from mathutils import Vector

class ConnectiveTissue(bpy.types.Operator):
    bl_idname = "mesh.construct_con_tissue"
    bl_label = "Construct Connective Tissue"
    bl_description = "Works with SQUARE Grid meshes, appropriately placed between 2 objects as to avoid shrinkwrap wraparounds (overlapping vertices/degenerate tetra/hexahedra)"

    def obj_list_cb(self, context):  
        return [(obj.name, obj.name, obj.name) for obj in bpy.data.objects]  
    
    object1 = bpy.props.EnumProperty(items=obj_list_cb, name = "Object 1", description = "Choose Object 1 here")   
    object2 = bpy.props.EnumProperty(items=obj_list_cb, name = "Object 2", description = "Choose Object 2 here")  

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        col = layout.column_flow(align=True, columns=1)
        col.prop(self, "object1")
        col.prop(self, "object2")
    
    @classmethod
    def poll(self, context):
        return (context.object is not None and context.object.type == 'MESH')
    
    def execute(self, context):
        construct(context, self)
        return {'FINISHED'}
    
    def cancel(self,context):
        return {'CANCELLED'}

def construct(context,options):

    o1 = bpy.data.objects[options.object1]  # cache the objects as dictionary indexing will change
    o2 = bpy.data.objects[options.object2]
    
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    plane_top = context.selected_objects[0]
    bpy.ops.object.duplicate()
    plane_bot = context.selected_objects[0]

    # create connective tissue object (mesh to be filled in later)
    bpy.ops.object.add(type='MESH')
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    ct = bpy.context.object
    ct.name = 'ConnectiveTissue'
    ct['annotated_type'] = 'VOLUMETRIC'
    ct['carvable'] = 1
    bpy.ops.object.select_all(action='DESELECT')
    
    # shrinkwrap object 1
    context.scene.objects.active = plane_top
    bpy.ops.object.modifier_add(type='SHRINKWRAP')
    context.object.modifiers["Shrinkwrap"].use_keep_above_surface = True
    #context.object.modifiers["Shrinkwrap"].target = bpy.data.objects["adrenal gland"]   # later replaced by "o1"
    context.object.modifiers["Shrinkwrap"].target = o1
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Shrinkwrap")

    # shrinkwrap object 2
    context.scene.objects.active = plane_bot
    bpy.ops.object.modifier_add(type='SHRINKWRAP')
    context.object.modifiers["Shrinkwrap"].use_keep_above_surface = True
    #context.object.modifiers["Shrinkwrap"].target = bpy.data.objects["kidney_hollow"]   # later replaced by "o2"
    context.object.modifiers["Shrinkwrap"].target = o2
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Shrinkwrap")

  #------ create connective tissue
    #-- create mid plane as average of top and bottom
    plane_bot.select = True
    bpy.ops.object.duplicate()
    plane_mid = context.selected_objects[0]
    for i in range(len(plane_top.data.vertices)):
        plane_mid.data.vertices[i].co = (plane_top.data.vertices[i].co + plane_bot.data.vertices[i].co)/2
    
    ctac_parent = bpy.data.objects.new("ConnTissAttConstr", None)
    context.scene.objects.link(ctac_parent)
    
    #-- create attach constraints
    num_vert = int(math.sqrt(len(plane_top.data.vertices)))
    indices = np.linspace(0, num_vert-1,int(num_vert/1.5), dtype=int)
    for i in indices:
        for j in indices:
            pt_top = plane_top.data.vertices[i*num_vert + j].co
            pt_mid = plane_mid.data.vertices[i*num_vert + j].co
            pt_bot = plane_bot.data.vertices[i*num_vert + j].co
            maxradius_top = (pt_top-pt_mid).length
            maxradius_bot = (pt_mid-pt_bot).length

            bpy.ops.mesh.primitive_ico_sphere_add(size=1.0, location=pt_top)
            sph_top = context.selected_objects[0]
            sph_top.scale = ((maxradius_top*0.99),(maxradius_top*0.99),(maxradius_top*0.99))
            sph_top.hide = True
            #sph_top.parent = ctac_parent
            sph_top['annotated_type'] = 'ATTACHCONSTRAINT'
            sph_top['object1'] = ct.name
            sph_top['object2'] = o1.name#"adrenal gland"
            sph_top['alwaysMatchFor'] = 2   # expand search space (sphere radius) for object 2 until vertex is found 
            
            bpy.ops.mesh.primitive_ico_sphere_add(size=1.0, location=pt_bot)
            sph_bot = context.selected_objects[0]
            sph_bot.scale = ((maxradius_bot*0.99),(maxradius_bot*0.99),(maxradius_bot*0.99))
            sph_bot.hide = True
            #sph_bot.parent = ctac_parent
            sph_bot['annotated_type'] = 'ATTACHCONSTRAINT'
            sph_bot['object1'] = ct.name
            sph_bot['object2'] = o2.name#"kidney_hollow"
            sph_bot['alwaysMatchFor'] = 2  
    
    #-- join the three planes. NOTE: polygon indexing: middle 0..w^2-1, bottom w^2..2*(w^2)-1, top 2*(w^2)..3*(w^2)-1)
    bpy.ops.object.select_all(action='DESELECT')
    context.scene.objects.active = plane_mid
    plane_mid.select = True
    plane_bot.select = True
    plane_top.select = True
    bpy.ops.object.join()
    
    #-- construct tetrahedra
    three_planes = context.scene.objects.active
    M = bpy.data.meshes.new(name = "tet_mesh")
    M.vertices.add(len(three_planes.data.vertices))
    for i, v in enumerate(three_planes.data.vertices):
        M.vertices[i].co = v.co

    w = num_vert - 1
    for i in range(w*w):
        top_quad = three_planes.data.polygons[2*w*w + i]
        mid_quad = three_planes.data.polygons[i]
        bot_quad = three_planes.data.polygons[w*w + i]
        
        createTets(M, (bot_quad, mid_quad),i)
        createTets(M, (mid_quad, top_quad),i+1)
    
    make_outer_surface(M)
    ct.data = M
    
    bpy.ops.object.select_all(action='DESELECT')
    three_planes.select = True
    bpy.ops.object.delete()
    
def createTets(m, quad_tuple, iteration):
    def isOdd(x): return (x % 2 != 0)
    
    q_idx = isOdd(iteration)
    
    tet = m.tetrahedra.add()
    tet.vertices[0] = quad_tuple[int(q_idx)].vertices[0]
    tet.vertices[1] = quad_tuple[int(q_idx)].vertices[1]
    tet.vertices[2] = quad_tuple[int(q_idx)].vertices[2]
    tet.vertices[3] = quad_tuple[int(not q_idx)].vertices[1]
    if isOdd(iteration):
        tet.vertices[0], tet.vertices[2] = tet.vertices[2], tet.vertices[0]
    
    tet = m.tetrahedra.add()
    tet.vertices[0] = quad_tuple[int(not q_idx)].vertices[3]
    tet.vertices[1] = quad_tuple[int(not q_idx)].vertices[2]
    tet.vertices[2] = quad_tuple[int(not q_idx)].vertices[1]
    tet.vertices[3] = quad_tuple[int(q_idx)].vertices[2]
    if isOdd(iteration):
        tet.vertices[0], tet.vertices[2] = tet.vertices[2], tet.vertices[0]
    
    tet = m.tetrahedra.add()
    tet.vertices[0] = quad_tuple[int(q_idx)].vertices[2]
    tet.vertices[1] = quad_tuple[int(q_idx)].vertices[3]
    tet.vertices[2] = quad_tuple[int(q_idx)].vertices[0]
    tet.vertices[3] = quad_tuple[int(not q_idx)].vertices[3]
    if isOdd(iteration):
        tet.vertices[0], tet.vertices[2] = tet.vertices[2], tet.vertices[0]
    
    tet = m.tetrahedra.add()
    tet.vertices[0] = quad_tuple[int(not q_idx)].vertices[1]
    tet.vertices[1] = quad_tuple[int(not q_idx)].vertices[0]
    tet.vertices[2] = quad_tuple[int(not q_idx)].vertices[3]
    tet.vertices[3] = quad_tuple[int(q_idx)].vertices[0]
    if isOdd(iteration):
        tet.vertices[0], tet.vertices[2] = tet.vertices[2], tet.vertices[0]
    
    tet = m.tetrahedra.add()
    tet.vertices[0] = quad_tuple[int(q_idx)].vertices[0]
    tet.vertices[1] = quad_tuple[int(q_idx)].vertices[2]
    tet.vertices[2] = quad_tuple[int(not q_idx)].vertices[3]
    tet.vertices[3] = quad_tuple[int(not q_idx)].vertices[1]
    if isOdd(iteration):
        tet.vertices[0], tet.vertices[1] = tet.vertices[1], tet.vertices[0]

def encodeFacet(a, b, c):
  if a < b:
    if a < c:
      i,j,k = a,b,c
    else:
      i,j,k = c,a,b
  else:
    if b < c:
      i,j,k = b,c,a
    else:
      i,j,k = c,a,b
  return i << 40 | j << 20 | k

def decodeFacet(f):
  a = f >> 40 & ( (1 << 20) - 1 ) 
  b = f >> 20 & ( (1 << 20) - 1 ) 
  c = f       & ( (1 << 20) - 1 ) 
  return a,b,c

LU = [ [1,3,2], [0,2,3], [0,3,1], [0,1,2] ]
def make_outer_surface(M):
  faceSet = set()
  for t in M.tetrahedra:
    for l in LU:
      f = encodeFacet(t.vertices[l[0]],t.vertices[l[1]],t.vertices[l[2]])
      rf = encodeFacet(t.vertices[l[0]],t.vertices[l[2]],t.vertices[l[1]])
      if rf in faceSet:
        faceSet.remove(rf)
      else:
        faceSet.add(f)
  
  M.tessfaces.add(len(faceSet))
  for i,f in enumerate(faceSet):
    a, b, c = decodeFacet(f)
    M.tessfaces[i].vertices =  (int(a), int(c), int(b))
    
  M.update(calc_edges=True)           
  M.calc_normals()  

def register():
    bpy.utils.register_class(ConnectiveTissue)
    
def unregister():
    bpy.utils.unregister_class(ConnectiveTissue)

if __name__ == "__main__":
    register()
    #bpy.ops.mesh.construct_con_tissue()