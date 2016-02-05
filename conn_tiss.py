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
    maxDim = 1e+16    
    meshType = 8 # 8 = hex, 4 = tet 

    if True:
        o1 = bpy.data.objects[options.object1]  # cache the objects as dictionary indexing will change
        o2 = bpy.data.objects[options.object2]
    else:
        o1 = bpy.data.objects['a2']; o2 = bpy.data.objects['a1']        
    
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
    
    # context.scene.objects.link(plane_top)
    bpy.ops.object.duplicate()
    plane_bot = context.selected_objects[0]

    # create connective tissue object (mesh to be filled in later)
    bpy.ops.object.add(type='MESH')
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    ct = bpy.context.object
    #ct.name = 'ConnectiveTissue'
    ct.name = 'ct'
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
             
    nPlaneVert = len(plane_top.data.vertices)
    nMvert = 3*nPlaneVert 
    
    #-- construct hexahedra
    if meshType==4:
      M = bpy.data.meshes.new(name = "tet_mesh")    
    elif meshType==8:
      M = bpy.data.meshes.new(name = "hex_mesh")
    else:
      print("unexpected meshType indicator");return 
    M.vertices.add(nMvert) 
    for i in range(0,nPlaneVert):
        M.vertices[i].co = plane_mid.data.vertices[i].co   
        M.vertices[nPlaneVert + i].co = plane_bot.data.vertices[i].co   
        M.vertices[2*nPlaneVert + i].co = plane_top.data.vertices[i].co                  

    botVertices = [i + nPlaneVert for i in range(nPlaneVert)]      
    topVertices = [i + 2*nPlaneVert for i in range(nPlaneVert)]
    nquad = len(plane_mid.data.polygons) 
    for i in range(nquad): 
        mid_quad = plane_mid.data.polygons[i]
        bot_quad = plane_bot.data.polygons[i]        
        top_quad = plane_top.data.polygons[i]  
        for j in range(4):
            bot_quad.vertices[j] = bot_quad.vertices[j] + nPlaneVert
            top_quad.vertices[j] = top_quad.vertices[j] + 2*nPlaneVert    
        if meshType==4:
          createTets(M, (bot_quad, mid_quad),i)
          createTets(M, (mid_quad, top_quad),i+1)  
        elif meshType==8:
          # determine if orientation by 4 vertices of one quad agrees with the direction pointing into the hex
          if i == 0:                     
            # direction of the 0123 orientation
            vec1 = plane_mid.data.vertices[mid_quad.vertices[1]].co
            vec0 = plane_mid.data.vertices[mid_quad.vertices[0]].co
            vec3 = plane_mid.data.vertices[mid_quad.vertices[3]].co            
            vec01 = Vector((vec1[0]-vec0[0],vec1[1]-vec0[1],vec1[2]-vec0[2]))
            vec03 = Vector((vec3[0]-vec0[0],vec3[1]-vec0[1],vec3[2]-vec0[2]))            
            dir0123 =  crossProd(vec01,vec03)
            # bottom to top direction of a hex
            midv0 = plane_mid.data.vertices[mid_quad.vertices[0]].co
            topv0 = plane_top.data.vertices[mid_quad.vertices[0]].co # index of mid_quad since this is the index for plane, not for mesh M
            bot2top = Vector((topv0[0]-midv0[0],topv0[1]-midv0[1],topv0[2]-midv0[2]))            
            dotp = dotProd(dir0123,bot2top)
            if dotp>0:
              coDirection  = True 
            elif dotp<0:
              coDirection = False 
            else:
              print("three planes are too close"); return 
            print("codirection---------------------------------------")
            print(coDirection)
            print(dotp)
            print(dir0123)
            print(bot2top)
            print(vec01)
            print(vec03)
          hex = M.hexahedra.add()
          for j in range(4):
              # bot-mid hex
              if coDirection:
                hex.vertices[j] = bot_quad.vertices[j]
                hex.vertices[4+j] = mid_quad.vertices[j]     
              else:
                hex.vertices[j] = bot_quad.vertices[3-j]
                hex.vertices[4+j] = mid_quad.vertices[3-j]     
          hex = M.hexahedra.add()
          for j in range(4):
              # mid-top hex
              if coDirection:
                hex.vertices[j] = mid_quad.vertices[j]
                hex.vertices[4+j] = top_quad.vertices[j]  
              else:
                hex.vertices[j] = mid_quad.vertices[3-j]
                hex.vertices[4+j] = top_quad.vertices[3-j]               
     
    if meshType==4:
      make_outer_surface(M)    
    elif meshType==8:
      make_hex_outer_surface(M)
    ct.data = M
    
    ct['annotated_type'] = 'CONNECTIVETISSUE'
    ct['topObject'] = o1.name 
    ct['botObject'] = o2.name
    ct['topVertices'] = topVertices
    ct['botVertices'] = botVertices    
        
    bpy.ops.object.select_all(action='DESELECT')
    plane_top.select = True; bpy.ops.object.delete()    
    plane_mid.select = True; bpy.ops.object.delete()
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
  
def encodeHexFacet(a, b, c, d):
  # rearrange the vertices so that: the first is the smallest, orientation is preserved
  vt = [a,b,c,d]
  vf = map(float,[a,b,c,d])
  mv = min(vf)
  minIdx = -1 
  for vi,v in enumerate(vt):
    if abs(mv-v)<.1:
        minIdx = vi; break 
  if minIdx == 0:
    i,j,k,l = a,b,c,d
  elif minIdx == 1:
    i,j,k,l = b,c,d,a
  elif minIdx == 2:
    i,j,k,l = c,d,a,b 
  elif minIdx == 3: 
    i,j,k,l = d,a,b,c
  # assert(max(vf) < 32768) # max index < 2^15
  return i << 45 | j << 30 | k << 15 | l    
  
def decodeHexFacet(f):
  a = f >> 45 & ( (1 << 15) - 1 ) 
  b = f >> 30 & ( (1 << 15) - 1 ) 
  c = f >> 15 & ( (1 << 15) - 1 ) 
  d = f       & ( (1 << 15) - 1 ) 
  return a,b,c,d    
  
# hex_faces = [ [0,1,2,3],[4,7,6,5],[0,4,5,1],[1,5,6,2],[3,2,6,7],[0,3,7,4] ]
# def make_hex_outer_surface(H):
  # faceSet = set()
  # for t in H.hexahedra:
    # for l in hex_faces:
      # f = encodeHexFacet(t.vertices[l[0]],t.vertices[l[1]],t.vertices[l[2]],t.vertices[l[3]])
      # rf = encodeHexFacet(t.vertices[l[0]],t.vertices[l[3]],t.vertices[l[2]],t.vertices[l[1]])
      # if rf in faceSet:
        # faceSet.remove(rf)
      # else:
        # faceSet.add(f)
  
  # H.tessfaces.add(len(faceSet))
  # for i,f in enumerate(faceSet):
    # a, b, c, d = decodeHexFacet(f)
    # H.tessfaces[i].vertices =  (int(a), int(d), int(c), int(b))
    
  # H.update(calc_edges=True)           
  # H.calc_normals()    
  
hex_faces = [ [0,1,2,3],[4,7,6,5],[0,4,5,1],[1,5,6,2],[3,2,6,7],[0,3,7,4] ]
def make_hex_outer_surface(H):
  faceSet = set()
  for t in H.hexahedra:
    for l in hex_faces:
      f = encodeHexFacet(t.vertices[l[0]],t.vertices[l[1]],t.vertices[l[2]],t.vertices[l[3]])
      rf = encodeHexFacet(t.vertices[l[0]],t.vertices[l[3]],t.vertices[l[2]],t.vertices[l[1]])
      if rf in faceSet:
        faceSet.remove(rf)
      else:
        faceSet.add(f)
  
  H.tessfaces.add(len(faceSet))
  for i,f in enumerate(faceSet):
    a, b, c, d = decodeHexFacet(f)
    H.tessfaces[i].vertices =  (int(a), int(d), int(c), int(b))
    
  H.update(calc_edges=True)           
  H.calc_normals()    
  
def crossProd(v1,v2):
    return [v1[1]*v2[2] - v1[2]*v2[1],-(v1[0]*v2[2] - v1[2]*v2[0]),v1[0]*v2[1] - v1[1]*v2[0]]     
    
def dotProd(v1,v2):
    return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]

def register():
    bpy.utils.register_class(ConnectiveTissue)
    
def unregister():
    bpy.utils.unregister_class(ConnectiveTissue)
