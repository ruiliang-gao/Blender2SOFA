bl_info = {
    'name': "Tetrahedral mesh support plugin",
    'author': "Saleh Dindar",
    'version': (0, 0,  0),
    'blender': (2, 74, 0),
    'location': "",
    'warning': "",
    'description': "Store tetrahedral meshes in Blender and conver regular meshes to tetrahedral meshes",
    'wiki_url': "https://bitbucket.org/salehqt/blender-tetrahedral/wiki",
    'tracker_url': "https://bitbucket.org/salehqt/blender-tetrahedral",
    "category": "Mesh"
}

import bpy
import bmesh
from bpy.props import FloatVectorProperty, EnumProperty, PointerProperty
from tetrahedral import cgaltetrahedralize as cgal


import numpy as N

class TetException(Exception):
    def __init__(self, message):
        Exception.__init__(self, message)
        self.message = message


        
def invoke_CGALTetrahedralize(m, options):
    tet = cgal.TetrahedralMesh()
    tri = cgal.TriangleMesh()
    par = cgal.TetrahedralizeParameters()
    par.cell_size = options.cell_size;
    par.facet_angle = options.facet_angle;
    par.facet_size = options.facet_size;
    par.facet_distance = options.facet_distance;
    par.cell_radius_edge_ratio = options.cell_radius_edge_ratio;

    position =  N.empty((len(m.vertices),3))
    for i, v in enumerate(m.vertices):
        position[i][0] = v.co[0]
        position[i][1] = v.co[1]
        position[i][2] = v.co[2]
    tri.vertices = position
    faces = N.empty([len(m.polygons), 3],dtype=int)
    for i, f in enumerate(m.polygons):
        faces[i][0] = f.vertices[0]
        faces[i][1] = f.vertices[1]
        faces[i][2] = f.vertices[2]
        
    tri.triangles = faces
    print('calling tetrahedralize')
    retval = cgal.tetrahedralize(tri, tet, par)
    print('done calling')
    if retval != 0:
        raise TetException("CGALTetrahedralize error code %d" % retval)
    if tet.tetrahedronCount == 0:
        raise TetException("CGALTetrahedralize returned empty tetrahedralization")
    else:
        return ( tet.vertices, tet.tetrahedra, tet.triangles )

    
def tetrahedralize_object(self, context):
    m = context.object.data
    points, tetrahedra, trifaces = invoke_CGALTetrahedralize(m, self)
    
    M = bpy.data.meshes.new(name = m.name + '.tetra')
    

    for t in tetrahedra:
        mt = M.tetrahedral.tetrahedra.add()
        for j in range(0,4):
            mt.vertices[j] = int(t[j])
            
    for t in trifaces:
        mt = M.tetrahedral.faces.add()
        for j in range(0,3):
            mt.vertices[j] = int(t[j])         
           
    M.vertices.add(len(points))
    for i in range(0,len(points)):
        M.vertices[i].co = points[i]

    M.tessfaces.add(len(trifaces))
    for i, (a,c,b) in enumerate(trifaces):                
        M.tessfaces[i].vertices = (int(a),int(b),int(c))

    M.update(calc_edges=True)           
    M.calc_normals()          
    context.object.data = M

    self.report({'INFO'}, 'Generated %d tetrahedra and %d nodes' % (len(tetrahedra),len(points)))

class MeshTetrahedron(bpy.types.PropertyGroup):
    """Represent a tetrahedron in a mesh"""
    vertices = bpy.props.IntVectorProperty(size=4)
    is_surface = bpy.props.BoolProperty(default=False)
    
class MeshTriFace(bpy.types.PropertyGroup):
    vertices = bpy.props.IntVectorProperty(size=3)    
    
    
# A property group of all the settings we need for a tetrahedral information
# namely we only need the index array
class TetrahedralMeshSettings(bpy.types.PropertyGroup):
    def get_is_tetrahedral(self):
        return len(self.tetrahedra) > 0
    
    is_tetrahedral = bpy.props.BoolProperty(name="Is Tetrahedral?",get=get_is_tetrahedral)
    tetrahedra = bpy.props.CollectionProperty(name="Tetrahedra", type=MeshTetrahedron)
    faces = bpy.props.CollectionProperty(name="Triangle faces", type=MeshTriFace)
        
# A panel will give us a permanent place in the data tab for meshes
class TetrahedralMeshSettingsPanel(bpy.types.Panel):
    """A panel to edit tetrahedral properties"""
    bl_label = "Tetrahedral Mesh"
    bl_idname = "MESH_PT_Tetrahedral"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"
    
    @classmethod
    def poll(self, context):
        return (context.object is not None and context.object.type == 'MESH')

    def draw_header(self, context):
        layout = self.layout
        mesh = context.object.data
        layout.prop(mesh.tetrahedral, "is_tetrahedral", text="")

    def check(self, context):
        return True
    def draw(self, context):
        layout = self.layout
        mesh = context.object.data

        if mesh.tetrahedral.is_tetrahedral:
            # Create a simple row.
            row = layout.row()
 
class TetrahedralizeMesh(bpy.types.Operator):
    bl_idname = "mesh.tetrahedralize"
    bl_label = "Tetrahedralize Mesh"

    cell_size = bpy.props.FloatProperty(name="Maximum Cell Size", default=0.1, min=0.05)
    facet_angle = bpy.props.FloatProperty(name="Minimum dihedral angle", default=25, min=10, max=45)
    facet_size = bpy.props.FloatProperty(name="Maximum facet size", default=0.1, min=0.05)
    facet_distance = bpy.props.FloatProperty(name="Distance between facets", default=0.1,min=0.01)
    cell_radius_edge_ratio = bpy.props.FloatProperty(name="Cell radius-edge ratio", default=3.0, min=1.0)


    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        col = layout.column_flow(align=True, columns=1)
        col.prop(self, "cell_size")
        col.prop(self, "facet_angle")
        col.prop(self, "facet_size")
        col.prop(self, "facet_distance")
        col.prop(self, "cell_radius_edge_ratio")
    
    @classmethod
    def poll(self, context):
        return (context.object is not None and context.object.type == 'MESH')
    
    def execute(self, context):
        self.report({'INFO'}, "Calling CGAL Tetrahedralize")
        try:
            tetrahedralize_object(self, context)
        except TetException as et:
            self.report({'ERROR'}, "Operator failed: %s" % et.message)
            return { 'CANCELLED' }

        return {'FINISHED'}
    
    def cancel(self,context):
        return {'CANCELLED'}
        
            
def register():
    bpy.utils.register_class(MeshTriFace)
    bpy.utils.register_class(MeshTetrahedron)
    bpy.utils.register_class(TetrahedralMeshSettings)    
    bpy.utils.register_class(TetrahedralMeshSettingsPanel)    
    bpy.utils.register_class(TetrahedralizeMesh)
    bpy.types.Mesh.tetrahedral = PointerProperty(type=TetrahedralMeshSettings)

def unregister():
    del bpy.types.Mesh.tetrahedral
    bpy.utils.unregister_class(TetrahedralizeMesh)
    bpy.utils.unregister_class(TetrahedralMeshSettingsPanel)    
    bpy.utils.unregister_class(TetrahedralMeshSettings)    
    bpy.utils.unregister_class(MeshTetrahedron)
    bpy.utils.unregister_class(MeshTriFace)
    
if __name__ == "__main__":
    register()


