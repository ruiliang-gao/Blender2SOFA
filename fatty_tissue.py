import bpy
import math
import numpy as np
from mathutils import Vector

class Fatty_Tissue(bpy.types.Operator):
    bl_idname = "mesh.construct_fatty_tissue"
    bl_label = "Construct a fatty tissue"
    bl_options = { 'UNDO' }
    bl_description = "Create a fatty tissue around a given object from a given cube"    
    subd_num = bpy.props.IntProperty(name="Number of subdivision (default = 10)", description = "Number of subdivisions")    
    out_radius = bpy.props.FloatProperty(name="Radius of outer tissue (default = cube_edge/2)", description = "Desired distance from the outer suface of the tissue to the object")    
    
    object = bpy.props.StringProperty(name = "Object", description = "Choose an object")   
    
    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):        
        layout = self.layout
        col = layout.column_flow(align=True, columns=1)        
        col.prop(self,"subd_num")
        rad = layout.column_flow(align=True, columns=1)        
        rhad.prop(self,"out_radius")
        sch = layout.column_flow(align=True, columns=1)        
        sch.prop_search(self, "object", context.scene, "objects")
        
    @classmethod
    def poll(self, context):
        # return (context.object is not None and context.object.type == 'MESH')
        return (context.object is not None)
    
    def execute(self, context):
        construct(context, self)
        return {'FINISHED'}
    
    def cancel(self,context):
        return {'CANCELLED'}

def construct(context, options):

  # default values 
  if options.subd_num==0:
    options.subd_num = 10
  if options.out_radius==0:
    options.out_radius==.2

  if options.object=="":
    options.object='Sphere'
  
  cube = context.selected_objects[0]
  organ = bpy.data.objects[options.object]
  
  organ.select = True
  bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
  bpy.ops.object.select_all(action='DESELECT')

  cubeverts = cube.data.vertices
  #for i in range(8):
    #bpy.ops.mesh.primitive_uv_sphere_add(size=0.05*(i+1), location=cubeverts[i].co)
    
  #TODO how to fix the bug of randomly chosen vertices order?
  e1 = (cubeverts[1].co - cubeverts[0].co).normalized()
  e2 = (cubeverts[2].co - cubeverts[0].co).normalized()
  e3 = (cubeverts[4].co - cubeverts[0].co).normalized()
  #print('e1,e2,e3',e1,e2,e3)
  order = vectorEq(crossProd(e1,e2),e3)
  #print('answer',vectorEq(crossProd(e1,e2),e3))
  if order == True:
    cubeCornerB = cube.matrix_world * cubeverts[1].co
    cubeCornerA = cube.matrix_world * cubeverts[0].co
    cubeCornerC = cube.matrix_world * cubeverts[2].co
    cubeCornerD = cube.matrix_world * cubeverts[3].co
    cubeCornerF = cube.matrix_world * cubeverts[5].co
    cubeCornerE = cube.matrix_world * cubeverts[4].co
    cubeCornerG = cube.matrix_world * cubeverts[6].co
    cubeCornerH = cube.matrix_world * cubeverts[7].co
  else:
    cubeCornerB = cube.matrix_world * cubeverts[2].co
    cubeCornerA = cube.matrix_world * cubeverts[0].co
    cubeCornerC = cube.matrix_world * cubeverts[1].co
    cubeCornerD = cube.matrix_world * cubeverts[3].co
    cubeCornerF = cube.matrix_world * cubeverts[6].co
    cubeCornerE = cube.matrix_world * cubeverts[4].co
    cubeCornerG = cube.matrix_world * cubeverts[5].co
    cubeCornerH = cube.matrix_world * cubeverts[7].co
  edgeLength = (cubeCornerB - cubeCornerA).magnitude
  level = options.subd_num
  nEdgeVerts = level+1
  nMVerts = pow(nEdgeVerts,3)
  nPlaneVerts = pow(nEdgeVerts,2)
  unitLength = edgeLength / level
 
  # build the vertices list for the hexmesh
  Mesh = bpy.data.meshes.new(name = "hex_mesh_temp")
  Mesh.vertices.add(nMVerts)
  bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
  #print('cubeCornerA',cubeCornerA)
  for i in range(0,nMVerts):
    z = int(i/nPlaneVerts)
    xy = i%nPlaneVerts
    y = xy%nEdgeVerts
    x = int(xy/nEdgeVerts)
    x = x/level
    y = y/level
    z = z/level
    #Mesh.vertices[i].co = cubeCornerA + Vector((-unitLength*x,unitLength*y,unitLength*z))
    Mesh.vertices[i].co = cubeCornerA*(1-x)*(1-y)*(1-z) + cubeCornerB*(1-x)*y*(1-z) + cubeCornerC*x*(1-y)*(1-z) + cubeCornerD*x*y*(1-z) + cubeCornerE*(1-x)*(1-y)*z + cubeCornerF*(1-x)*y*z + cubeCornerG*x*(1-y)*z + cubeCornerH*x*y*z
    #print(crossProd(cubeCornerA - cubeCornerB,cubeCornerA - cubeCornerC))
  
  #build the subcubes list [[],[],...], each subcube has 8 indexs
  hexes = [] # list of indices of each hex 
  nHexes = pow(level,3)
  for i in range(0,nHexes):
    hz = int(i/level/level)
    ixy = i%(level*level)
    hy = ixy%level
    hx = int(ixy/level)
    ii = hz*nPlaneVerts + hx*nEdgeVerts + hy
    #indexList should be in the clockwise order
    indexList = [ii, ii+1, ii+1+nEdgeVerts, ii+nEdgeVerts, ii+nPlaneVerts, ii+1+nPlaneVerts, ii+1+nEdgeVerts+nPlaneVerts, ii+nEdgeVerts+nPlaneVerts]
    hexes.append(indexList)
  
  # change the following:
  num_verts = nMVerts
  # list that contains coordinates of all vertices  
  
  # build a hex mesh for the fatty tissue 
  M = bpy.data.meshes.new(name = "hex_mesh")
  M.vertices.add(num_verts)
  bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
  
  for ii in range(num_verts):
    M.vertices[ii].co =  Mesh.vertices[ii].co 
   
  #Build Boolean Table T by computing the dist between hex center and the given organ
  nhex = len(hexes) 
  T = []
  Old2New = [-1]*num_verts
  New2Old = []
  nNewVerts = 0
  topVertices = []
  botVertices = []
  for i in range(nhex):
    ahex = hexes[i]
    ct = computeCenter(ahex,M)
    location,normal,index = organ.closest_point_on_mesh(ct)
    diff = ct - location
    dist = diff.magnitude
    orient = dotProd(diff,normal)
    #print(i,'th hex, loc, dist, ori', location, dist, orient)
    
    #TODO update new hexmesh vertices
    if dist < options.out_radius and orient > 0:
      #bpy.ops.mesh.primitive_uv_sphere_add(size=0.08, location=ct)
      for j in range(8):
        if Old2New[hexes[i][j]] == -1:
          Old2New[hexes[i][j]] = nNewVerts
          nNewVerts = nNewVerts + 1
          New2Old.append(hexes[i][j])
      T.append(1)
      if dist < 1 * unitLength:
        topVertices.append(index)
        botVertices.append(nNewVerts - 1)
        #print('found one closest point')
        
    else:
      T.append(0)

  newM = bpy.data.meshes.new(name = "new_hex_mesh")
  bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
  newM.vertices.add(nNewVerts)
  print(nNewVerts,'num')
  for i in range(nNewVerts):
    newM.vertices[i].co =  Mesh.vertices[New2Old[i]].co
    #print('newM.vertices[i].co',newM.vertices[i].co)
  # Add hexahedras to newM
  for i in range(nhex):
    if T[i] == 1:
      index = []
      for j in range(8):
        index.append(Old2New[hexes[i][j]])
      hex = newM.hexahedra.add()
      hex.vertices = index  
   
  # # add hexahedra to M 
  # for ii in range(nhex):
    # if T[ii] == 1:
      # hex = M.hexahedra.add()
      # hex.vertices = hexes[ii]
    

  # make_hex_outer_surface(M)
  
  # create the outcome fatty_tissue 
  bpy.ops.object.add(type='MESH')
  bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
  fatty_tissue = bpy.context.object  
  fatty_tissue.name = 'fatty_tissue'
  fatty_tissue.data = newM
  #fatty_tissue['annotated_type'] = 'VOLUMETRIC'
  fatty_tissue['annotated_type'] = 'CONNECTIVETISSUE'
  fatty_tissue['topObject'] = organ.name 
  fatty_tissue['botObject'] = organ.name
  print('ftname, ogname',fatty_tissue.name,organ.name)
  #fatty_tissue['topVertices'] = topVertices
  fatty_tissue['botVertices'] = botVertices
  print('len of bot',len(botVertices))
  fatty_tissue['carvable'] = 1       
  fatty_tissue['collisionGroup'] = 1       
  fatty_tissue['contactFriction'] = 0.010  
  fatty_tissue['contactStiffness'] = 50000
  fatty_tissue['damping'] = 0.100
  fatty_tissue['poissonRatio'] = 0.250
  fatty_tissue['precomputeConstraints'] = 0
  fatty_tissue['selfCollision'] = 0
  fatty_tissue['suture'] = 0
  fatty_tissue['youngModulus'] = 30000
  fatty_tissue['color'] = "yellow"

  bpy.ops.object.select_all(action='DESELECT')
  cube.hide = True; cube.hide_render = True 
  # bpy.ops.mesh.primitive_uv_sphere_add(size=1, location=cube.data.vertices[0].co)
  
  

def crossProd(v1,v2):
    return [v1[1]*v2[2] - v1[2]*v2[1],-(v1[0]*v2[2] - v1[2]*v2[0]),v1[0]*v2[1] - v1[1]*v2[0]]    
        
def dotProd(v1,v2):
    return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]
    
def vectorEq(v1,v2):
    d = (v1[0]-v2[0])*(v1[0]-v2[0]) + (v1[1]-v2[1])*(v1[1]-v2[1]) + (v1[2]-v2[2])*(v1[2]-v2[2]) 
    if d<0.01:
    
      return True
    else:
      return False
 
def computeCenter(ahex, M):
    v0 = M.vertices[ahex[0]].co
    v7 = M.vertices[ahex[6]].co
    return (v0+v7)/2
