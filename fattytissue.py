import bpy
import numpy as np
from .io_msh import recalc_outer_surface
from mathutils import Vector

# Relative vertex indices of a hexahedron
HEX_VERTICES = [ (0,0,0), (1,0,0), (1,1,0), (0,1,0), (0,0,1), (1,0,1), (1,1,1), (0,1,1) ]

class FattyTissue(bpy.types.Operator):
    bl_idname = "mesh.construct_fatty_tissue"
    bl_label = "Construt Fatty Tissue"
    bl_description = "Construct a fatty tissue in a prescribed cube (empty object) that encompasses an organ. Needs two selected object, the empty cube should be the active one"
    bl_options = { 'UNDO' }

    cube = bpy.props.StringProperty(name = 'Sampling Cube', description = 'The cube used for sampling')
    organ = bpy.props.StringProperty(name = 'Organ', description = 'The organ on which the fat is wrapped around')
    thickness_from_surface = bpy.props.FloatProperty(name = 'Distance from Surface',description='Distance of fatty tissue from organ surface',default=0.5,min=0,step=0.01)
    resolution = bpy.props.IntProperty(name = 'Resolution', description = 'Number of subdivisions along each edge of the cube. Determines the number of hexahedra generated',default=8,min=2,max=20)
    smoothness = bpy.props.IntProperty(name = 'Smoothness', description = 'How smooth the fatty tissue should be. Ideal is 2.',default=2,min=0,max=3)
    project_to_surface = bpy.props.BoolProperty(name = 'Project points to surface',default=False,description='If set, the grid points are moved to the surface, may procedue some degenerate hexahedra')
    keep_the_cube = bpy.props.BoolProperty(name = 'Keep the Cube',default=False, description='If set, the input cube will not be removed after the mesh is generated')

    @classmethod
    def poll(self, context):
        return context.object is not None and context.object.type == 'EMPTY' and context.object.empty_draw_type == 'CUBE' and len(context.selected_objects) == 2

    def check(self, context):
        return True

    def invoke(self, context, event):
        self.cube = context.object.name
        if context.object == context.selected_objects[0]:
            self.organ = context.selected_objects[1].name
        else:
            self.organ = context.selected_objects[0].name
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        l = self.layout
        l.prop_search(self, 'cube', context.scene, 'objects')
        l.prop_search(self, 'organ', context.scene, 'objects')
        l.prop(self, 'resolution')
        l.prop(self, 'project_to_surface')
        if not self.project_to_surface:
          l.prop(self, 'thickness_from_surface')
        l.prop(self, 'smoothness')
        l.prop(self, 'keep_the_cube')

    def execute(self, context):
        L = self.resolution
        o = bpy.data.objects[self.organ]
        c = bpy.data.objects[self.cube]
        project = self.project_to_surface
        if project:
          D = 0
        else:
          D = self.thickness_from_surface

        # Create a new mesh to be associated with the empty object
        M = bpy.data.meshes.new(name = 'Fatty tissue around %s' % o.name)

        # Flags of which vertices on the grid are outside
        isNearParentOrgan = np.zeros([L+1,L+1,L+1],dtype=bool)
        # The index of the grid vertex in the mesh
        vertexIndex = np.zeros([L+1,L+1,L+1],dtype=int)
        # Generate vertices for the points, test each one against the
        # organ, if outside then flag them and add them to the vertex list
        oinv = o.matrix_world.inverted()
        cinv = c.matrix_world.inverted()
        radius = c.empty_draw_size * (c.scale[0] + c.scale[1] + c.scale[2]) / 3 / L
        #radius = c.empty_draw_size *  Vector(c.scale).length / L
        for x in range(L+1):
         for y in range(L+1):
          for z in range(L+1):
            co = c.empty_draw_size * ( 2.0 * Vector((x,y,z)) / L - Vector((1,1,1)) )
            v = oinv * c.matrix_world * co
            # version_string is a string composed of Blender version + "(sub 0)". E.g. "2.76 (sub 0)"
            # blenderVer stores the first 4 digits of the string, that is the version number.
            blenderVer = bpy.app.version_string[0:4]
            if (float(blenderVer) >= 2.77):
                result,location,normal,_ = o.closest_point_on_mesh(v)
            else:
                location,normal,_ = o.closest_point_on_mesh(v)
            d = (o.matrix_world*v - o.matrix_world*location).length
            
            if normal.dot(v - location) > 0:
                if d < D: #or project and d < radius:
                    isNearParentOrgan[x,y,z] = True
                    vertexIndex[x,y,z] = len(M.vertices) 
                    M.vertices.add(1)
                    M.vertices[-1].co = co
                else:
                    vertexIndex[x,y,z] = -1
                    isNearParentOrgan[x,y,z] = False
            else:
                if d < 1.5 * D: #Make sure all inside vertices of a hex are in the M list
                    isNearParentOrgan[x,y,z] = True
                    vertexIndex[x,y,z] = len(M.vertices) 
                    M.vertices.add(1)
                    M.vertices[-1].co = co
                else:
                    vertexIndex[x,y,z] = -1
                    isNearParentOrgan[x,y,z] = False

        for x in range(L):
         for y in range(L):
          for z in range(L):
            # Check that all the vertices required for this hexa are available and outside
            # the surface
            verticesAvailable = all([ isNearParentOrgan[x+i,y+j,z+k] for i,j,k in HEX_VERTICES ])
            # Build the hexa if all the vertices are available
            if verticesAvailable:
                h = M.hexahedra.add()
                h.vertices = [ int(vertexIndex[x+i,y+j,z+k]) for i,j,k in HEX_VERTICES ]

        # Finish up the mesh and calculate the outer surface
        M.update()
        recalc_outer_surface(M)
        M.update()

        # Create the fatty tissue object that will contain the mesh
        O = bpy.data.objects.new(name = 'Fatty tissue around %s' % o.name, object_data = M)
        O.location = c.location
        O.rotation_mode = c.rotation_mode
        O.rotation_euler = c.rotation_euler
        O.rotation_quaternion = c.rotation_quaternion
        O.rotation_axis_angle = c.rotation_axis_angle
        O.scale = c.scale

        # Add the object to the scene
        context.scene.objects.link(O)

        # Deselect all objects (Note: Only c and o could  be selected at this time)
        #bpy.ops.object.select_all(action='DESELECT')
        c.select = False
        o.select = False
        #O.select = True
        
        # Remove the cube
        if not self.keep_the_cube:
            context.scene.objects.unlink(c)
            bpy.data.objects.remove(c)
            
        # Select the fatty tissue object and runs the smooth function the number of times specified by the user in Blender
        # The smooth function can only be executed in Edit Mode
        O.select = True
        bpy.context.scene.objects.active = O
        bpy.ops.object.mode_set(mode = 'EDIT')
        for i in range (0, self.smoothness):
          bpy.ops.mesh.vertices_smooth()
        bpy.ops.object.mode_set(mode = 'OBJECT')
        
        # Set the default values for the fatty tissue
        O.template = 'VOLUMETRIC'
        O.youngModulus = 3000
        O.rayleighStiffness = 0.1
        O.carvable = True
        # Attaches the fatty tissue to the organ
        O.object1 = o.name

        return { 'FINISHED' }
