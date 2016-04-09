import bpy
import math
import numpy as np
from mathutils import Vector
from .io_msh import recalc_outer_surface

class ConnectiveTissue(bpy.types.Operator):
    bl_idname = "mesh.construct_con_tissue"
    bl_label = "Construct Connective Tissue"
    bl_options = { 'UNDO' }
    bl_description = "Works with SQUARE Grid meshes, appropriately placed between 2 objects as to avoid shrinkwrap wraparounds (overlapping vertices/degenerate tetra/hexahedra)"

    plane   = bpy.props.StringProperty(name = "Dividing Plane", description= "The plane that defines the connection")
    object1 = bpy.props.StringProperty(name = "Object 1", description = "Choose Object 1 here")
    object2 = bpy.props.StringProperty(name = "Object 2", description = "Choose Object 2 here")
    layerCount = bpy.props.IntProperty(name = "Layer Count", description = "Number of planar layers between the two objects",default=2,min=1,max=100)

    def invoke(self, context, event):
        self.plane = context.active_object.name
        oo = [ o.name for o in context.selected_objects if o.name != self.plane ]
        if len(oo) >= 1: self.object1 = oo[0]
        if len(oo) >= 2: self.object2 = oo[1]
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        col = layout.column_flow(columns=1)
        col.prop_search(self, "plane", context.scene, "objects")
        col.prop_search(self, "object1", context.scene, "objects")
        col.prop_search(self, "object2", context.scene, "objects")
        col.prop(self,'layerCount')

    @classmethod
    def poll(self, context):
        return context.object is not None and context.object.type == 'MESH'

    def shrinkwrapTo(self,context,plane, o):
        sh = plane.modifiers.new('Shrinkwrap-' + o.name,'SHRINKWRAP')
        sh.use_keep_above_surface = True
        sh.wrap_method = 'NEAREST_VERTEX'
        sh.target = o
        M = plane.to_mesh(context.scene, True, 'PREVIEW')
        plane.modifiers.remove(sh)
        return M



    def execute(self, context):
        L = self.layerCount
        # Get the plane dividing the two
        plane = bpy.data.objects[self.plane]
        # Get the two objects from the properties
        o1 = bpy.data.objects[self.object1]
        o2 = bpy.data.objects[self.object2]

        # Shrinkwrap the plane to the respective objects and get the mesh
        shm1 = self.shrinkwrapTo(context, plane, o1)
        shm2 = self.shrinkwrapTo(context, plane, o2)

        # construct a new mesh
        # interpolate the shrink wrapped planes to create layers of the new mesh object
        N = len(shm1.vertices)
        M = bpy.data.meshes.new('Mesh Connecting %s and %s' % (o1.name,o2.name))
        M.vertices.add(N * (L+1))
        for l in range(0, L+1):
            for i in range(0, N):
                u = l / float(L)
                M.vertices[l * N + i].co = (1-u) * shm1.vertices[i].co + u * shm2.vertices[i].co

        # Create hexas in the new mesh object
        for l in range(0, L):
            for p in shm1.polygons:
                h = M.hexahedra.add()
                for j in range(0, 4):
                    h.vertices[0+j] = p.vertices[j] + (l+0)*N
                    h.vertices[4+j] = p.vertices[j] + (l+1)*N

        # Create the outer surface
        recalc_outer_surface(M)

        # Convert the plane into our connective tissue and annotate it as volumetric
        ct = plane
        ct.data = M
        ct.name = 'Tissue Connecting %s and %s' % (o1.name,o2.name)
        ct.sofaprops.template = 'VOLUMETRIC'
        ct.sofaprops.object1 = o1.name
        ct.sofaprops.object2 = o2.name
        ct.sofaprops.carvable = True
        ct.sofaprops.suture = True
        return {'FINISHED'}

    def cancel(self,context):
        return {'CANCELLED'}


