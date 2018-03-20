import bpy
import numpy as np
import random
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
    # resolution = bpy.props.IntProperty(name = 'Resolution', description = 'Number of subdivisions along each edge of the cube. Determines the number of hexahedra generated',default=6,min=2,max=20)
    resolutionX = bpy.props.IntProperty(name = 'ResolutionX', description = 'Number of subdivisions along X edge of the cube. Determines the number of hexahedra generated',default=6,min=1,max=20)
    resolutionY = bpy.props.IntProperty(name = 'ResolutionY', description = 'Number of subdivisions along Y edge of the cube. Determines the number of hexahedra generated',default=6,min=1,max=20)
    resolutionZ = bpy.props.IntProperty(name = 'ResolutionZ', description = 'Number of subdivisions along Z edge of the cube. Determines the number of hexahedra generated',default=6,min=1,max=20)
    smoothness = bpy.props.IntProperty(name = 'Smoothness', description = 'How smooth the fatty tissue should be. Ideal is 2.',default=2,min=0,max=3)
    project_to_surface = bpy.props.BoolProperty(name = 'Project points to surface',default=False,description='If set, the grid points are moved to the surface, may procedue some degenerate hexahedra')
    keep_the_cube = bpy.props.BoolProperty(name = 'Keep the Cube',default=False, description='If set, the input cube will not be removed after the mesh is generated')
    add_perturbations = bpy.props.BoolProperty(name = 'Add perturbations',default=False, description='The output hex mesh will have random perturbations on its vertices')
    preserve_interior = bpy.props.BoolProperty(name = 'Preserve interior volume',default=True, description='The output hex mesh will preserve the volume that are inside the organ')
    map_to_boundary = bpy.props.BoolProperty(name = 'Map to boundary',default=False, description='Map to ')
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
        # l.prop(self, 'resolution')
        l.prop(self, 'resolutionX')
        l.prop(self, 'resolutionY')
        l.prop(self, 'resolutionZ')
        l.prop(self, 'project_to_surface')
        if not self.project_to_surface:
          l.prop(self, 'thickness_from_surface')
        l.prop(self,'add_perturbations')
        l.prop(self,'preserve_interior')
        l.prop(self, 'smoothness')
        l.prop(self, 'map_to_boundary')
        l.prop(self, 'keep_the_cube')

    def execute(self, context):
        # L = self.resolution
        LX = self.resolutionX
        LY = self.resolutionY
        LZ = self.resolutionZ
        meanL = (LX + LY + LZ)/3
        listL = [LX, LY, LZ]
        maxL = max(listL)
        organ = bpy.data.objects[self.organ]    # formerly o
        cube = bpy.data.objects[self.cube]      # formerly c
        project = self.project_to_surface
        if project:
          D = 0
        else:
          D = self.thickness_from_surface

        # Create a new mesh to be associated with the empty object
        M = bpy.data.meshes.new(name = 'Fatty tissue around %s' % organ.name)

        # Flags of which vertices on the grid are outside
        isNearParentOrgan = np.zeros([LX+1,LY+1,LZ+1],dtype=bool)
        # The index of the grid vertex in the mesh
        vertexIndex = np.zeros([LX+1,LY+1,LZ+1],dtype=int)
        # Generate vertices for the points, test each one against the
        # organ, if outside then flag them and add them to the vertex list
        organInv = organ.matrix_world.inverted()    # formerly oinv
        cubeInv = cube.matrix_world.inverted()      # formerly cinv
        radius = cube.empty_draw_size * (cube.scale[0] + cube.scale[1] + cube.scale[2]) / meanL / 2.0
        print("radius = ", radius)
        #radius = c.empty_draw_size *  Vector(c.scale).length / L
        for x in range(LX+1):
         for y in range(LY+1):
          for z in range(LZ+1):
            if self.add_perturbations:# dx,dy,dz are perturbations
                dx, dy, dz = random.random()/3.0, random.random()/3.0, random.random()/3.0 # random floating point number in range [0.0, 1.0).
                co = cube.empty_draw_size * ( (2.0 * Vector(((x+dx)/LX,(y+dy)/LY,(z+dz)/LZ))) - Vector((1,1,1)) )
            else:
                co = cube.empty_draw_size * ( (2.0 * Vector((x/LX,y/LY,z/LZ))) - Vector((1,1,1)) ) # coord that been centered at the origin
            v = organInv * cube.matrix_world * co
            # version_string is a string composed of Blender version + "(sub 0)". E.g. "2.76 (sub 0)"
            # blenderVer stores the first 4 digits of the string, that is the version number.
            blenderVer = bpy.app.version_string[0:4]
            if (float(blenderVer) >= 2.77):
                result,location,normal,_ = organ.closest_point_on_mesh(v)
            else:
                location,normal,_ = organ.closest_point_on_mesh(v)
            d = (organ.matrix_world*v - organ.matrix_world*location).length #distance of v to the nearest vertex on organ
            if normal.dot(v - location) > 0: # v is outside
                if d < D or d < radius: #or project and d < radius:
                    isNearParentOrgan[x,y,z] = True
                    vertexIndex[x,y,z] = len(M.vertices) 
                    M.vertices.add(1)
                    if self.map_to_boundary:
                        #co = co + (organ.matrix_world*location - organ.matrix_world*v) * 0.5
                        co = co - normal / normal.length / 2.0 * d  
                    M.vertices[-1].co = co
                else:
                    vertexIndex[x,y,z] = -1
                    isNearParentOrgan[x,y,z] = False
            else: # v is inside
                if self.preserve_interior: #Make sure all inside vertices of a hex are in the M list
                    isNearParentOrgan[x,y,z] = True
                    vertexIndex[x,y,z] = len(M.vertices) 
                    M.vertices.add(1)
                    M.vertices[-1].co = co
                elif d < 1.5 * D or d < radius: 
                    isNearParentOrgan[x,y,z] = True
                    vertexIndex[x,y,z] = len(M.vertices) 
                    M.vertices.add(1)
                    M.vertices[-1].co = co
                else:
                    vertexIndex[x,y,z] = -1
                    isNearParentOrgan[x,y,z] = False

        for x in range(LX):
         for y in range(LY):
          for z in range(LZ):
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
        fatObj = bpy.data.objects.new(name = 'Fatty tissue around %s' % organ.name, object_data = M)
        fatObj.location = cube.location
        fatObj.rotation_mode = cube.rotation_mode
        fatObj.rotation_euler = cube.rotation_euler
        fatObj.rotation_quaternion = cube.rotation_quaternion
        fatObj.rotation_axis_angle = cube.rotation_axis_angle
        fatObj.scale = cube.scale

        # Add the object to the scene
        context.scene.objects.link(fatObj)

        # Deselect all objects (Note: Only c and o could  be selected at this time)
        #bpy.ops.object.select_all(action='DESELECT')
        cube.select = False
        organ.select = False
        #O.select = True
        
        # Remove the cube
        if not self.keep_the_cube:
            context.scene.objects.unlink(cube)
            bpy.data.objects.remove(cube)
            
        # Select the fatty tissue object and runs the smooth function the number of times specified by the user in Blender
        # The smooth function can only be executed in Edit Mode
        fatObj.select = True
        bpy.context.scene.objects.active = fatObj
        bpy.ops.object.mode_set(mode = 'EDIT')
        for i in range (0, self.smoothness):
          bpy.ops.mesh.vertices_smooth()
        bpy.ops.object.mode_set(mode = 'OBJECT')
        
        # Set the default values for the fatty tissue
        fatObj.template = 'VOLUMETRIC'
        fatObj.youngModulus = 3000
        fatObj.rayleighStiffness = 0.1
        fatObj.carvable = True
        # Attaches the fatty tissue to the organ
        fatObj.object1 = organ.name

        return { 'FINISHED' }
