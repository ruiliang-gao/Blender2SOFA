import bpy
import math
import numpy as np
from mathutils import Vector

class HexRod(bpy.types.Operator):
    bl_idname = "mesh.construct_hex_rod"
    bl_label = "Construct Hex Rod"
    bl_options = { 'UNDO' }
    bl_description = "Create a rod made of hexahedra (one per unit length) along the input curve"    
    hex_number = bpy.props.IntProperty(name="Number of hexahedra", description = "Number of hexahedra to construct")    
    rod_radius = bpy.props.FloatProperty(name="Radius of the rod", description = "Radius of the rod")    
    
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
  print(curveLen)
  if options.rod_radius==0:
    options.rod_radius = curveLen/30
  if options.hex_number==0:
    options.hex_number = 30
  curve1.hide = True; curve1.hide_render = True 
  # bpy.ops.object.delete(use_global=False) 
    
  curve.data.splines[0].resolution_u = options.hex_number
  
  # default values of hex_number and rod_radius
  # bpy.ops.mesh.primitive_uv_sphere_add(size=1,location=curve.data.splines[0].bezier_points[0].co)
  # bpy.ops.mesh.primitive_uv_sphere_add(size=1,location=curve.data.splines[0].bezier_points[1].co)
  # return 
  
  # create a square 
  bpy.ops.curve.primitive_bezier_circle_add(radius=options.rod_radius)  
  square = context.selected_objects[0]
  square.data.splines[0].resolution_u = 1
  
  curve.data.bevel_object = bpy.data.objects[square.name]
  # m.data.bevel_object = bpy.data.objects[square.name]
  square.hide = True; square.hide_render = True 
  
  # outcome mesh from the bevel object 
  tube = curve.to_mesh(context.scene,True,"PREVIEW")
  nMvert = len(tube.vertices)
  # now construct the hex rod 
  M = bpy.data.meshes.new(name = "hex_mesh")
  M.vertices.add(nMvert)
  for i in range(nMvert):
    M.vertices[i].co = tube.vertices[i].co
  
  
  print('sadff ddddddddddddddddddddddddddddddd')  
  print(len(M.vertices))  
  return         

def register():
    bpy.utils.register_class(HexRod)
    
def unregister():
    bpy.utils.unregister_class(HexRod)
