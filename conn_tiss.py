import bpy
import math
import numpy as np
from mathutils import Vector

class ConnectiveTissue(bpy.types.Operator):
    bl_idname = "mesh.construct_con_tissue"
    bl_label = "Construct Connective Tissue"
    bl_options = { 'UNDO' }
    bl_description = "Works with SQUARE Grid meshes, appropriately placed between 2 objects as to avoid shrinkwrap wraparounds (overlapping vertices/degenerate tetra/hexahedra)"

    object1 = bpy.props.StringProperty(name = "Object 1", description = "Choose Object 1 here")   
    object2 = bpy.props.StringProperty(name = "Object 2", description = "Choose Object 2 here")  

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        col = layout.column_flow(align=True, columns=1)
        col.prop_search(self, "object1", context.scene, "objects")
        col.prop_search(self, "object2", context.scene, "objects")
    
    @classmethod
    def poll(self, context):
        return (context.object is not None and context.object.type == 'MESH')
    
    def execute(self, context):
        construct(context, self)
        return {'FINISHED'}
    
    def cancel(self,context):
        return {'CANCELLED'}

def construct(context,options):

    autoDefinePlane = False
    defineSpringDirectly = True
    maxDim = 1e+16

    if True:
        o1 = bpy.data.objects[options.object1]  # cache the objects as dictionary indexing will change
        o2 = bpy.data.objects[options.object2]
    else:
        o1 = bpy.data.objects['Sphere']; o2 = bpy.data.objects['Sphere.001']
        # o1 = bpy.data.objects['Spleen']; o2 = bpy.data.objects['fundus']
    
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)	
    if autoDefinePlane:           
        bpy.ops.object.select_all(action='DESELECT') 
        o1.select = True
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY') 
        center1t = o1.location 
        center1 = Vector((center1t[0],center1t[1],center1t[2]))    
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        o1.select = False
        o2.select = True 
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY') 
        center2t = o2.location 
        center2 = Vector((center2t[0],center2t[1],center2t[2]))
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        o2.select = False
                       
        footprint1 = o1.closest_point_on_mesh(center2,maxDim)
        dualfp1 = o2.closest_point_on_mesh(footprint1[0],maxDim)
        footprint2 = o2.closest_point_on_mesh(center1,maxDim)
        dualfp2 = o1.closest_point_on_mesh(footprint2[0],maxDim)
  
        center = footprint2[0]/2 + dualfp2[0]/2       
        dim1 = min(o1.dimensions.x,o1.dimensions.y,o1.dimensions.z) 
        dim2 = min(o2.dimensions.x,o2.dimensions.y,o2.dimensions.z)                               
        bpy.ops.mesh.primitive_grid_add(radius=min(dim1,dim2)/4, location=center)
        plane_top = context.selected_objects[0]
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    else:
        plane_top = context.selected_objects[0]        
    
    if False:
        # re-mesh the grid with uniform quads
        print('remeshing ------------------')   
        tol_corners = 1e-6; tol_normal = 1e-16
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY')     
        grid_center = plane_top.location   
        # - look for the 4 corners: maximum distance to center 
        # bpy.ops.mesh.primitive_uv_sphere_add(size=.2, location=grid_center)
        gdata = plane_top.data
        nv = len(gdata.vertices)
        diagDis = -1; grid_corners = []   
        for i in range(nv):
            # bpy.ops.mesh.primitive_uv_sphere_add(size=.1, location=plane_top.matrix_world * gdata.vertices[i].co)
            tDis = (plane_top.matrix_world * gdata.vertices[i].co - grid_center).length
            if tDis >= diagDis: diagDis = tDis; 
        print(diagDis)
        for i in range(nv):
            tDis = (plane_top.matrix_world * gdata.vertices[i].co - grid_center).length
            if abs(tDis - diagDis) <= tol_corners: grid_corners.append(i)
        # * grid_verts: rearrange grid_corners so that they agree with grid's normal    
        # bpy.ops.mesh.primitive_uv_sphere_add(size=.3, location=plane_top.matrix_world * gdata.vertices[grid_corners[0]].co)
        # bpy.ops.mesh.primitive_uv_sphere_add(size=.3, location=plane_top.matrix_world * gdata.vertices[grid_corners[1]].co)
        # bpy.ops.mesh.primitive_uv_sphere_add(size=.3, location=plane_top.matrix_world * gdata.vertices[grid_corners[2]].co)
        # bpy.ops.mesh.primitive_uv_sphere_add(size=.3, location=plane_top.matrix_world * gdata.vertices[grid_corners[3]].co)
        grid_normal = gdata.polygons[0].normal
        point_out = [] # 4 vectors pointing from grid_center to grid_corners
        for i in range(4): point_out.append(plane_top.matrix_world*gdata.vertices[grid_corners[i]].co-grid_center)
        grid_vert0 = 0 # starting vertex, now look for next one 
        grid_vert2 = -1    
        for i in range(1,4):         
            tDis = (plane_top.matrix_world*gdata.vertices[grid_corners[i]].co - plane_top.matrix_world*gdata.vertices[grid_vert0].co).length        
            if abs(tDis-2*diagDis) <= 2*tol_corners: grid_vert2 = i; break             
        if grid_vert2 == -1: print('error: conn_tiss.py: opposite vertex not found, perhaps change tol_corners'); return 
        print(grid_vert2)
        remainVerts = [0,1,2,3]; remainVerts.remove(grid_vert2); remainVerts.remove(grid_vert0)
        grid_vert1 = -1
        print(remainVerts)
        print(point_out)
        for i in remainVerts:   
            print(i)
            print(crossProd(point_out[grid_vert0],point_out[i]))
            print(dotProd(crossProd(point_out[grid_vert0],point_out[i]),grid_normal))
            if dotProd(crossProd(point_out[grid_vert0],point_out[i]),grid_normal) > tol_normal: grid_vert1 = i;break 
        if grid_vert1 == -1: print('error: conn_tiss.py: next vertex not found, perhaps change tol_corners'); return 
        remainVerts.remove(grid_vert1)
        grid_vert3 = remainVerts[0]
        grid_vert0 = grid_corners[grid_vert0]
        grid_vert1 = grid_corners[grid_vert1]
        grid_vert2 = grid_corners[grid_vert2]
        grid_vert3 = grid_corners[grid_vert3]
        # bpy.ops.mesh.primitive_uv_sphere_add(size=.3, location=plane_top.matrix_world * gdata.vertices[grid_vert0].co)
        # bpy.ops.mesh.primitive_uv_sphere_add(size=.4, location=plane_top.matrix_world * gdata.vertices[grid_vert1].co)
        # bpy.ops.mesh.primitive_uv_sphere_add(size=.5, location=plane_top.matrix_world * gdata.vertices[grid_vert2].co)
        # bpy.ops.mesh.primitive_uv_sphere_add(size=.6, location=plane_top.matrix_world * gdata.vertices[grid_vert3].co)    
        
        print(grid_corners)
        print(grid_normal)
        
        if len(grid_corners) != 4:
           print('Error: conn_tiss.py: did not detect 4 corners of the grid, perhaps change tol_corners'); return 
             
        # 
        # grid_center = plane_top.data.vertices[nv-1].co/nv          
        # for i in range(nv-1):
            # grid_center = grid_center + plane_top.data.vertices[i].co/nv          
        
        return 
    
    # context.scene.objects.link(plane_top)
    bpy.ops.object.duplicate()
    plane_bot = context.selected_objects[0]

    # create connective tissue object (mesh to be filled in later)
    bpy.ops.object.add(type='MESH')
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    ct = bpy.context.object
    #ct.name = 'ConnectiveTissue'
    ct['annotated_type'] = 'VOLUMETRIC'
    ct['carvable'] = 1
    bpy.ops.object.select_all(action='DESELECT')
    
    # shrinkwrap object 1
    context.scene.objects.active = plane_top
    bpy.ops.object.modifier_add(type='SHRINKWRAP')
    context.object.modifiers["Shrinkwrap"].use_keep_above_surface = True
    context.object.modifiers["Shrinkwrap"].target = o1
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Shrinkwrap")

    # shrinkwrap object 2
    context.scene.objects.active = plane_bot
    bpy.ops.object.modifier_add(type='SHRINKWRAP')
    context.object.modifiers["Shrinkwrap"].use_keep_above_surface = True
    context.object.modifiers["Shrinkwrap"].target = o2
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Shrinkwrap")

  #------ create connective tissue
    #-- create mid plane as average of top and bottom
    plane_bot.select = True
    bpy.ops.object.duplicate()
    plane_mid = context.selected_objects[0]
    for i in range(len(plane_top.data.vertices)):
        plane_mid.data.vertices[i].co = (plane_top.data.vertices[i].co + plane_bot.data.vertices[i].co)/2
    
    if not(defineSpringDirectly):
        ctac_parent = bpy.data.objects.new("ConnTissAttConstr", None)
        ctac_parent['annotated_type'] = 'ATTACHCONSTRAINTGROUP'
        ctac_parent.hide = True
        context.scene.objects.link(ctac_parent)   
    
    #-- create attach constraints
    num_vert = int(math.sqrt(len(plane_top.data.vertices)))
        
    if not(defineSpringDirectly):
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
                sph_top.parent = ctac_parent
                sph_top['annotated_type'] = 'ATTACHCONSTRAINT'
                sph_top['object1'] = ct.name
                sph_top['object2'] = o1.name#"adrenal gland"
                sph_top['alwaysMatchFor'] = 2   # expand search space (sphere radius) for object 2 until vertex is found 
                
                bpy.ops.mesh.primitive_ico_sphere_add(size=1.0, location=pt_bot)
                sph_bot = context.selected_objects[0]
                sph_bot.scale = ((maxradius_bot*0.99),(maxradius_bot*0.99),(maxradius_bot*0.99))
                sph_bot.hide = True
                sph_bot.parent = ctac_parent
                sph_bot['annotated_type'] = 'ATTACHCONSTRAINT'
                sph_bot['object1'] = ct.name
                sph_bot['object2'] = o2.name#"kidney_hollow"
                sph_bot['alwaysMatchFor'] = 2  
    
    #-- join the three planes. NOTE: polygon indexing: middle 0..w^2-1, bottom w^2..2*(w^2)-1, top 2*(w^2)..3*(w^2)-1)
    nPlaneVert = len(plane_top.data.vertices)
    nMvert = 3*nPlaneVert 
    
    #-- construct tetrahedra
    M = bpy.data.meshes.new(name = "tet_mesh")
    M.vertices.add(nMvert) 
    for i in range(0,nPlaneVert):
        M.vertices[i].co = plane_mid.data.vertices[i].co   
        M.vertices[nPlaneVert + i].co = plane_bot.data.vertices[i].co   
        M.vertices[2*nPlaneVert + i].co = plane_top.data.vertices[i].co   

    botVertices = [i + nPlaneVert for i in range(nPlaneVert)]      
    topVertices = [i + 2*nPlaneVert for i in range(nPlaneVert)]
    w = num_vert - 1
    for i in range(w*w): 
        mid_quad = plane_mid.data.polygons[i]
        bot_quad = plane_bot.data.polygons[i]        
        top_quad = plane_top.data.polygons[i]  
        for j in range(4):
            bot_quad.vertices[j] = bot_quad.vertices[j] + nPlaneVert
            top_quad.vertices[j] = top_quad.vertices[j] + 2*nPlaneVert         
        
        createTets(M, (bot_quad, mid_quad),i)
        createTets(M, (mid_quad, top_quad),i+1)    
       
    make_outer_surface(M)    
    ct.data = M
    
    if defineSpringDirectly:
        ct['annotated_type'] = 'CONNECTIVETISSUE'
        ct['topObject'] = o1.name 
        ct['botObject'] = o2.name
        ct['topVertices'] = topVertices
        ct['botVertices'] = botVertices
        
    # bpy.ops.object.select_all(action='DESELECT')
    plane_mid.select = True; bpy.context.object.hide_render = True; bpy.context.object.hide = True
    plane_top.select = True; bpy.ops.object.delete()
    plane_bot.select = True; bpy.ops.object.delete()    
    
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
  
def crossProd(v1,v2):
    return [v1[1]*v2[2] - v1[2]*v2[1],-(v1[0]*v2[2] - v1[2]*v2[0]),v1[0]*v2[1] - v1[1]*v2[0]]     
    
def dotProd(v1,v2):
    return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]

def register():
    bpy.utils.register_class(ConnectiveTissue)
    
def unregister():
    bpy.utils.unregister_class(ConnectiveTissue)
