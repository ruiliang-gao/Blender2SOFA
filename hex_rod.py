import bpy
import math
import numpy as np
from mathutils import Vector

class HexRod(bpy.types.Operator):
    bl_idname = "mesh.construct_hex_rod"
    bl_label = "Construct Hex Rod"
    bl_options = { 'UNDO' }
    bl_description = "Create a rod made of hexahedra (one per unit length) along the input curve"    
    hex_number = bpy.props.IntProperty(name="Number of hexahedra (default = 20)", description = "Number of hexahedra to construct")    
    rod_radius = bpy.props.FloatProperty(name="Radius of the rod (default = curve length/30)", description = "Radius of the rod")    
    
    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):        
        layout = self.layout
        col = layout.column_flow(align=True, columns=1)        
        col.prop(self,"hex_number")
        rad = layout.column_flow(align=True, columns=1)        
        rad.prop(self,"rod_radius")        
    
    @classmethod
    def poll(self, context):
        # return (context.object is not None and context.object.type == 'MESH')
        return (context.object is not None)
    
    def execute(self, context):
        construct(context, self)
        return {'FINISHED'}
    
    def cancel(self,context):
        return {'CANCELLED'}

def constructSegments(context,options):
  # return a piecewise linear curve from the original curve 
  curve = context.selected_objects[0]     
  m = curve.to_mesh(context.scene, True, 'PREVIEW')  
  return m

def construct(context, options):
  # m = constructSegments(context, options)
  curve = context.selected_objects[0]   
  
  # duplicate to just calculate length and default values 
  bpy.ops.object.duplicate()
  curve1 = context.selected_objects[0]     
  curve1.data.splines[0].resolution_u = 20
  m = curve1.to_mesh(context.scene,True,"PREVIEW")
  nE = len(m.edges)
  curveLen = 0
  for i in range(nE):
    curveLen = curveLen + (m.vertices[m.edges[i].vertices[0]].co - m.vertices[m.edges[i].vertices[1]].co).length
  if options.rod_radius==0:
    options.rod_radius = curveLen/30
  if options.hex_number==0:
    options.hex_number = 20
    
  curve.data.splines[0].resolution_u = options.hex_number
  
  # create a square 
  bpy.ops.curve.primitive_bezier_circle_add(radius=options.rod_radius)  
  square = context.selected_objects[0]
  square.data.splines[0].resolution_u = 1
  
  curve.data.bevel_object = bpy.data.objects[square.name]
  # m.data.bevel_object = bpy.data.objects[square.name]
  
  # outcome mesh from the bevel object 
  tube = curve.to_mesh(context.scene,True,"PREVIEW")
  nMvert = len(tube.vertices)
  nHex = int(len(tube.polygons)/4)
  # now construct the hex rod 
  M = bpy.data.meshes.new(name = "hex_mesh")
  bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
  M.vertices.add(nMvert)
  for i in range(nMvert):
    M.vertices[i].co = curve.matrix_world * tube.vertices[i].co
    # bpy.ops.mesh.primitive_ico_sphere_add(size=.1,location=M.vertices[i].co)    

  # check if the orientation defined by 4 first vertices of the first 4 quads 
  # is the same as the direction pointing into the volume
  vec1 = tube.vertices[tube.polygons[1].vertices[0]].co 
  vec0 = tube.vertices[tube.polygons[0].vertices[0]].co
  vec3 = tube.vertices[tube.polygons[3].vertices[0]].co         
  vec01 = Vector((vec1[0]-vec0[0],vec1[1]-vec0[1],vec1[2]-vec0[2]))
  vec03 = Vector((vec3[0]-vec0[0],vec3[1]-vec0[1],vec3[2]-vec0[2]))            
  dir0123 =  crossProd(vec01,vec03)  
  out0 = tube.vertices[tube.polygons[0].vertices[0]].co
  in0  = tube.vertices[tube.polygons[4].vertices[0]].co
  out2in = Vector((in0[0]-out0[0],in0[1]-out0[1],in0[2]-out0[2]))            
  dotp = dotProd(dir0123,out2in)  
  if dotp>0:
    pointIn  = True 
  elif dotp<0:
    pointIn = False 
  else:
    print("the test hex is too distorted"); assert(False)
     
  # here assume that tube.polygons[0].vertices[3] = tube.polygons[4].vertices[0]
  if (tube.vertices[tube.polygons[0].vertices[1]].co-tube.vertices[tube.polygons[4].vertices[0]].co).length > 1e-9:
    print("hex_rod.py: convention not satisfied"); assert(False)
              
  for i in range(nHex):
    hex = M.hexahedra.add()
    for j in range(4):
      hex.vertices[j] = tube.polygons[4*i + j].vertices[0]
      hex.vertices[4+j] = tube.polygons[4*i + j].vertices[1]
    
  # make_hex_outer_surface(M)
  
  # create the outcome rod 
  bpy.ops.object.add(type='MESH')
  bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
  rod = bpy.context.object  
  rod.name = 'rod'
  rod.data = M
  rod['annotated_type'] = 'VOLUMETRIC'
  rod['carvable'] = 1       
  rod['collisionGroup'] = 1       
  rod['contactFriction'] = 0.010  
  rod['contactStiffness'] = 500
  rod['damping'] = 0.100
  rod['poissonRatio'] = 0.450
  rod['precomputeConstraints'] = 0
  rod['selfCollision'] = 0
  rod['suture'] = 0
  rod['youngModulus'] = 300
  rod['color'] = "white"

  bpy.ops.object.select_all(action='DESELECT')
  square.select = True; curve.select = True; 
  curve1.select = True
  bpy.ops.object.delete()
  

def crossProd(v1,v2):
    return [v1[1]*v2[2] - v1[2]*v2[1],-(v1[0]*v2[2] - v1[2]*v2[0]),v1[0]*v2[1] - v1[1]*v2[0]]    
        
def dotProd(v1,v2):
    return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]
        
def register():
    bpy.utils.register_class(HexRod)
    
def unregister():
    bpy.utils.unregister_class(HexRod)
