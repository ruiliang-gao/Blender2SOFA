import bpy
from .io_msh import recalc_outer_surface, remove_degenerate_hexahedra

class ConnectingTissue(bpy.types.Operator):
    bl_idname = "mesh.construct_connecting_tissue"
    bl_label = "Construct Connecting Tissue"
    bl_options = { 'UNDO' }
    bl_description = "Convert the active selected grid object into a volumetric hexahedral connecting tissue by shrink-wrapping it to the other two selected objects. Requires 3 objects to be selected, the grid has to be the active one"

    #plane   = bpy.props.StringProperty(name = "Dividing Grid", description= "The grid object that defines the area of connection")
    map1 = bpy.props.StringProperty(name = "Map 1", description = "Choose Map 1 here")
    map2 = bpy.props.StringProperty(name = "Map 2", description = "Choose Map 2 here")
    connection1 = bpy.props.StringProperty(name = "Connection 1", description = "Choose Connecting Object 1 here")
    connection2 = bpy.props.StringProperty(name = "Connection 2", description = "Choose Connecting Object 2 2 here")
    removeDegenerateHexahedra = bpy.props.BoolProperty(name = "Remove Degenerate Hexahedra", description = "Removes hexahedra with very small volume", default = True)
    shrinkwrapMethod = bpy.props.EnumProperty(items = (('NEAREST_SURFACEPOINT', 'Nearest Surface Point', ''), ('PROJECT', 'Project', '')), name = "Shrinkwrap Method")
    layerCount = bpy.props.IntProperty(name = "Layer Count", description = "Number of planar layers between the two objects",default=2,min=1,max=100)

    def invoke(self, context, event):
        #self.plane = context.active_object.name
        oo = [ o.name for o in context.selected_objects]
        if len(oo) >= 1: self.map1 = oo[0]
        if len(oo) >= 2: self.map2 = oo[2]
        if len(oo) >= 3: self.connection1 = oo[1]
        if len(oo) >= 4: self.connection2 = oo[3]
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        col = layout.column_flow(columns=1)
        #col.prop_search(self, "plane", context.scene, "objects")
        col.prop_search(self, "map1", context.scene, "objects")
        col.prop_search(self, "connection1", context.scene, "objects")
        col.prop_search(self, "map2", context.scene, "objects")
        col.prop_search(self, "connection2", context.scene, "objects")
        col.prop(self, "removeDegenerateHexahedra")
        col.prop(self, 'shrinkwrapMethod')
        col.prop(self,'layerCount')

    @classmethod
    def poll(self, context):
        return context.object is not None and context.object.type == 'MESH' and len(context.selected_objects) <= 5


    def execute(self, context):
        # TODO: Improve UI
        L = self.layerCount

        # Get the objects from the properties

        # Map objects
        m1 = bpy.data.objects[self.map1]
        m2 = bpy.data.objects[self.map2]
        # Meshes of the map objects
        shm1 = m1.data
        shm2 = m2.data
        # object to world matricies
        mat1 = m1.matrix_world
        mat2 = m2.matrix_world
        # Connecting objects
        c1 = bpy.data.objects[self.connection1]
        c2 = bpy.data.objects[self.connection2]
        
        # construct a new mesh
        # interpolate the shrink wrapped planes to create layers of the new mesh object
        N = len(shm1.vertices)
        M = bpy.data.meshes.new('Mesh Connecting %s and %s' % (m1.name,m2.name))
        M.vertices.add(N * (L+1))
        for l in range(0, L+1):#L+1 replaced by 2 for testing peritonim generation, but failed <- some bugs unfixed 
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

        # Remove degenerate hexahedra
        if(self.removeDegenerateHexahedra):
            remove_degenerate_hexahedra(M)

        # Convert the plane into our connective tissue and annotate it as volumetric
        #m = plane.data
        #ct = plane
        tissue = bpy.data.meshes.new('connectingtissue')
        ct = bpy.data.objects.new('Tissue Connecting %s and %s' % (c1.name,c2.name), tissue)
        scn = bpy.context.scene
        scn.objects.link(ct)
        ct.data = M
        #ct.name = 'Tissue Connecting %s and %s' % (c1.name,c2.name)
        ct.template = 'VOLUMETRIC'
        ct.object1 = c1.name
        ct.object2 = c2.name
        ct.carvable = True
        ct.suture = True
        #bpy.data.meshes.remove(m)



        # Select the newly created object
        m1.select = False
        m2.select = False
        c1.select = False
        c2.select = False
        ct.select = True
        return {'FINISHED'}
