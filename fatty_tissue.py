import bpy
import math
import numpy as np
from mathutils import Vector

class Fatty_Tissue(bpy.types.Operator):
    bl_idname = "mesh.construct_fatty_tissue"
    bl_label = "Construct a fatty tissue"
    bl_options = { 'UNDO' }
    bl_description = "Create a fatty tissue around a given object from a given cube"    
    subd_num = bpy.props.IntProperty(name="Number of subdivision (default = 20)", description = "Number of subdivisions")    
    out_radius = bpy.props.FloatProperty(name="Radius of outer tissue (default = cube_edge/2)", description = "Desired distance from the outer suface of the tissue to the object")    
    
    object = bpy.props.StringProperty(name = "Object", description = "Choose an object")   
    
    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):        
        layout = self.layout
        col = layout.column_flow(align=True, columns=1)        
        col.prop(self,"subd_num")
        rad = layout.column_flow(align=True, columns=1)        
        rad.prop(self,"out_radius")       
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

  if options.object=="":
    options.object='Sphere'
    
  organ = bpy.data.objects[options.object]; 
  cube = context.selected_objects[0]   
  # make sure that origin is at vertex 0
  
  # default values 
  if options.subd_num==0:
    options.subd_num = 20
  if options.out_radius==0:
    options.out_radius==.2 
  
  # change the following:
  num_verts = 8
  vert_coord = cube.data.vertices 
  hexes = []
  # ahex is a test hex ( = original cube)
  ahex = []
  for ii in range(8):
    ahex.append(ii)
  hexes.append(ahex)
  
  # build a hex mesh for the fatty tissue 
  M = bpy.data.meshes.new(name = "hex_mesh")
  bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
  M.vertices.add(num_verts)
  for ii in range(num_verts):
    M.vertices[ii].co = cube.matrix_world * vert_coord[ii].co 
    
  nhex = len(hexes)
  hex = M.hexahedra.add()
  for ii in range(nhex):
    hex.vertices = hexes[ii]
    
  # make_hex_outer_surface(M)
  
  # create the outcome fatty_tissue 
  bpy.ops.object.add(type='MESH')
  bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
  fatty_tissue = bpy.context.object  
  fatty_tissue.name = 'fatty_tissue'
  fatty_tissue.data = M
  fatty_tissue['annotated_type'] = 'VOLUMETRIC'
  fatty_tissue['carvable'] = 1       
  fatty_tissue['collisionGroup'] = 1       
  fatty_tissue['contactFriction'] = 0.010  
  fatty_tissue['contactStiffness'] = 500
  fatty_tissue['damping'] = 0.100
  fatty_tissue['poissonRatio'] = 0.450
  fatty_tissue['precomputeConstraints'] = 0
  fatty_tissue['selfCollision'] = 0
  fatty_tissue['suture'] = 0
  fatty_tissue['youngModulus'] = 300
  fatty_tissue['color'] = "white"

  bpy.ops.object.select_all(action='DESELECT')
  cube.hide = True; cube.hide_render = True 
  # bpy.ops.mesh.primitive_uv_sphere_add(size=1, location=cube.data.vertices[0].co)
  
  

def crossProd(v1,v2):
    return [v1[1]*v2[2] - v1[2]*v2[1],-(v1[0]*v2[2] - v1[2]*v2[0]),v1[0]*v2[1] - v1[1]*v2[0]]    
        
def dotProd(v1,v2):
    return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]
        
def register():
    bpy.utils.register_class(Fatty_Tissue)
    
def unregister():
    bpy.utils.unregister_class(Fatty_Tissue)
