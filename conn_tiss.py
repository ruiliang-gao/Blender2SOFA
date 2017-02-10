import bpy
from bpy.props import *
from .io_msh import recalc_outer_surface, remove_degenerate_hexahedra

# DATA OBJECTS

bpy.types.Scene.map1 = StringProperty(
    name = "Map 1",
    description = "Choose Map 1 here",
    default = "")
bpy.types.Scene.map2 = StringProperty(
    name = "Map 2",
    description = "Choose Map 2 here",
    default = "")
bpy.types.Scene.object1 = StringProperty(
    name = "Object 1",
    description = "Choose Object 1 here",
    default = "")
bpy.types.Scene.object2 = StringProperty(
    name = "Object 2",
    description = "Choose Object 2 here",
    default = "")
bpy.types.Scene.layerCount = IntProperty(
    name = "Layer Count",
    description = "Number of planar layers between the two objects",
    default = 3,
    min = 1,
    max = 100)

#   GUI PANEL OBJECT

class OBJECT_PT_ConnectingTissuePanel(bpy.types.Panel):
    bl_label = "Connecting Tissue"
    bl_space_type = "VIEW_3D"
    bl_region_type = "TOOLS"

    @classmethod
    def poll(self, context):
        return context.scene is not None

    def draw(self, context):
        layout = self.layout
        scn = context.scene
        col = layout.column()
        col.prop_search(scn, "map1", context.scene, "objects")
        col.prop_search(scn, "object1", context.scene, "objects")
        col.prop_search(scn, "map2", context.scene, "objects")
        col.prop_search(scn, "object2", context.scene, "objects")
        col.prop(scn, 'layerCount')
        col.operator("mesh.construct_connecting_tissue")

#   CONNECTING TISSUE CONSTRUCTOR

class OBJECT_OT_ConnectingTissue(bpy.types.Operator):
    bl_idname = "mesh.construct_connecting_tissue"
    bl_label = "Construct Connecting Tissue"
    bl_options = { 'UNDO' }
    bl_description = "Convert the active selected grid object into a volumetric hexahedral connecting tissue by shrink-wrapping it to the other two selected objects. Requires 3 objects to be selected, the grid has to be the active one"

    '''@classmethod
    def poll(self, context):
        return context.object is not None and context.object.type == 'MESH' and len(context.selected_objects) <= 5'''


    def execute(self, context):
        # Get the objects from the properties

        scn = context.scene
        L = scn.layerCount

        # Map objects
        m1 = bpy.data.objects[scn.map1]
        m2 = bpy.data.objects[scn.map2]
        # Meshes of the map objects
        shm1 = m1.data
        shm2 = m2.data
        # object to world matricies
        mat1 = m1.matrix_world
        mat2 = m2.matrix_world

        # Connecting objects
        c1 = bpy.data.objects[scn.object1]
        c2 = bpy.data.objects[scn.object2]

        # Check that the mapping objects have the same number of vertices
        if len(shm1.vertices) != len(shm2.vertices):
            self.report({'ERROR_INVALID_INPUT'}, 'Mapping faces must have the same number of vertices')
            return{'CANCELLED'}
        
        # construct a new mesh
        # interpolate the shrink wrapped planes to create layers of the new mesh object
        N = len(shm1.vertices)
        M = bpy.data.meshes.new('Mesh Connecting %s and %s' % (m1.name,m2.name))
        M.vertices.add(N * (L+1))
        for l in range(0, L+1): #L+1 replaced by 2 for testing peritonim generation, but failed <- some bugs unfixed 
            for i in range(0, N):
                u = l / float(L)
                M.vertices[l * N + i].co = (1-u) * mat1 * shm1.vertices[i].co + u * mat2 * shm2.vertices[i].co

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
        tissue = bpy.data.meshes.new('connectingtissue')
        ct = bpy.data.objects.new('Tissue Connecting %s and %s' % (c1.name,c2.name), tissue)
        scn = bpy.context.scene
        scn.objects.link(ct)
        ct.data = M
        ct.template = 'VOLUMETRIC'
        ct.object1 = c1.name
        ct.object2 = c2.name
        ct.carvable = True
        ct.suture = True

        # Select the newly created object
        m1.select = False
        m2.select = False
        c1.select = False
        c2.select = False
        ct.select = True
        scn.objects.active = ct
        return {'FINISHED'}
