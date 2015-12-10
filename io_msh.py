bl_info = { 
    'name': "GMSH Import/Export plugin",
    'author': "Saleh Dindar",
    'version': (0, 0, 0),
    'blender': (2, 75, 0),
    'location': "",
    'warning': "",
    'description': "Import/export tetrahedral meshes from/to GMSH files",
    'wiki_url': "",
    'tracker_url': "",
    'category': "Import-Export",
}
import bpy
import bpy_extras
import re
import os

def encodeFacet(a, b, c):
  if a < b:
    if a < c:
      i,j,k = a,b,c
    else:
      i,j,k = c,a,b
  else:
    if b < c:
      i,j,k = b,c,a
    else:
      i,j,k = c,a,b
  return i << 40 | j << 20 | k
def decodeFacet(f):
  a = f >> 40 & ( (1 << 20) - 1 ) 
  b = f >> 20 & ( (1 << 20) - 1 ) 
  c = f       & ( (1 << 20) - 1 ) 
  return a,b,c

LU = [ [1,3,2], [0,2,3], [0,3,1], [0,1,2] ]
def make_outer_surface(M):
  faceSet = set()
  for t in M.tetrahedra:
    for l in LU:
      f = encodeFacet(t.vertices[l[0]],t.vertices[l[1]],t.vertices[l[2]])
      rf = encodeFacet(t.vertices[l[0]],t.vertices[l[2]],t.vertices[l[1]])
      if rf in faceSet:
        faceSet.remove(rf)
      else:
        faceSet.add(f)
  
  M.tessfaces.add(len(faceSet))
  for i,f in enumerate(faceSet):
    a, b, c = decodeFacet(f)
    M.tessfaces[i].vertices =  (int(a), int(c), int(b))
    
  M.update(calc_edges=True)           
  M.calc_normals()          
    


class MeshTetrahedron(bpy.types.PropertyGroup):
    """Represent a tetrahedron in a mesh"""
    vertices = bpy.props.IntVectorProperty(size=4)
    
# A panel will give us a permanent place in the data tab for meshes
class TetrahedralMeshPanel(bpy.types.Panel):
    """A panel to edit tetrahedral properties"""
    bl_label = "Tetrahedral Mesh"
    bl_idname = "MESH_PT_Tetrahedral"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"
    
    @classmethod
    def poll(self, context):
        return (context.object is not None and context.object.type == 'MESH')

    def check(self, context):
        return True
    def draw(self, context):
        layout = self.layout
        M = context.object.data
        row = layout.row()
        row.label("Tetrahedron count: %d" % len(M.tetrahedra));


class ExportMSHOperator(bpy.types.Operator):
  @classmethod
  def poll(cls, context):
      """
      Enabled only for tetrahedral meshes
      """
      return context.object is not None and context.object.type == 'MESH' \
        and len(context.object.tetrahedra) > 0


class ImportMSH(bpy.types.Operator, bpy_extras.io_utils.ImportHelper):
  """Load a tetrahedral mesh from GMSH file"""
  bl_idname = "import_mesh.msh"
  bl_label = "Import MSH"
  bl_options = {'UNDO'}
  
  filename_ext = ".msh"
  filter_glob = bpy.props.StringProperty(default="*.msh", options={'HIDDEN'})
  
  def execute(self, context):
    objName = bpy.path.display_name(os.path.basename(self.filepath))
    M = bpy.data.meshes.new(name = objName)
    lineno = 0; mode = ''
    f = open(self.filepath, 'r')
    for line in f:
      lineno += 1
      line = line.strip()
      if mode == '':
        if line == '$NOD':
          mode = 'NOD-first'
        elif line == '$ELM':
          mode = 'ELM-first'
        else:
          self.report({'ERROR'}, "%s:%d: Unexpected %s" % (objName,line, lineno))
          return {'CANCELLED'}
      elif mode == 'NOD-first':
        if re.match('^\d+\s*$', line):
          M.vertices.add(int(line))
          mode = 'NOD'
        else:
          self.report({'ERROR'}, "%s:%d: Expected number of nodes here got '%s'" %(objName,lineno, line))
          return {'CANCELLED'}
      elif mode == 'NOD':
        if re.match('^\d+(\s+[+-]?\d+(\.\d*([eE][+-]?\d+)?)?){3}\s*$', line):
          i, x, y, z = line.split()
          M.vertices[int(i)-1].co[0] = float(x)
          M.vertices[int(i)-1].co[1] = float(y)
          M.vertices[int(i)-1].co[2] = float(z)
        elif line == '$ENDNOD':
          mode = ''
        else:
          self.report({'ERROR'}, "%s:%d: Line of node def expected here '%s'" % (objName, lineno, line))
          return {'CANCELLED'}
      elif mode == 'ELM-first':
        if re.match('^\d+\s*$', line):
          # Tthere is no way to resize the tetrahedra collection 
          mode = 'ELM'
        else:
          self.report({'ERROR'}, "%s:%d: Expected number of elements here got '%s'" %(objName, lineno, line))
          return {'CANCELLED'}
      elif mode == 'ELM':
        if re.match('^\d+(\s+\d+)+\s*$', line):
          # TODO: what about triangle elements
          idx, count, _, _, _, a, b, c, d = line.split()
          i = int(idx)
          t = M.tetrahedra.add()
          if i != len(M.tetrahedra):
            self.report({'ERROR'}, "%s:%d: Elements must come in proper order, %d" %(objName,lineno,i))
            return {'CANCELLED'}
          t.vertices[0] = int(a) - 1
          t.vertices[1] = int(b) - 1
          t.vertices[2] = int(c) - 1
          t.vertices[3] = int(d) - 1
        elif line == '$ENDELM':
          mode = ''
        else:
          self.report({'ERROR'}, "%s:%d: Line of element def expected here '%s'" % (objName, lineno,line))
          return {'CANCELLED'}
      else:
        assert("This line" == "Never reached")
    f.close()
    make_outer_surface(M)
    o = bpy.data.objects.new(objName, M)
    context.scene.objects.link(o)
    return { 'FINISHED' }
    
  
      
      

def menu_func_import(self, context):
    self.layout.operator(ImportMSH.bl_idname, text="GMSH (.msh)")

def register():
    bpy.utils.register_class(ImportMSH)
    bpy.utils.register_class(MeshTetrahedron)
    bpy.utils.register_class(TetrahedralMeshPanel)    
    bpy.types.INFO_MT_file_import.append(menu_func_import)
    bpy.types.Mesh.tetrahedra = bpy.props.CollectionProperty(name="Tetrahedra", type=MeshTetrahedron)

def unregister():
    del bpy.types.Mesh.tetrahedra
    bpy.types.INFO_MT_file_import.remove(menu_func_import)
    bpy.utils.unregister_class(ImportMSH)
    bpy.utils.unregister_class(TetrahedralMeshPanel)    
    bpy.utils.unregister_class(MeshTetrahedron)