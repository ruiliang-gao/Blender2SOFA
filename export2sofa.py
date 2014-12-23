bl_info = { 
    'name': "SOFA Export plugin",
    'author': "Saleh Dindar, Di Xie",
    'version': (0, 0,  0),
    'blender': (2, 69, 0),
    'location': "",
    'warning': "",
    'description': "Export Blender scenes into SOFA scene files",
    'wiki_url': "https://bitbucket.org/salehqt/blender2sofa/wiki/",
    'tracker_url': "https://bitbucket.org/salehqt/blender2sofa/",
    'category': 'Mesh'
}

import bpy
import xml.etree.ElementTree as ET
import os
from mathutils import Vector
from math import degrees

def vector_to_string(v):
    t = ""
    for i in v :
        t += str(i) + " "
    return t


def vector_degrees(v):
    return Vector(map(degrees,v))

def createMechanicalObject(o):
    o.rotation_mode = "ZYX"
    t = ET.Element("MechanicalObject",template="Vec3d",name="MO")
    t.set("translation", vector_to_string(o.location))
    t.set("rotation", vector_to_string(vector_degrees(o.rotation_euler)))
    t.set("scale3d", vector_to_string(o.scale))
    return t
    
def exportSoftBody(o, scn):
    t = ET.Element("Node",name=o.name)
    t.append(ET.Element("EulerImplicitSolver"))
    t.append(ET.Element("CGLinearSolver",template="GraphScattered"))
    
    t.append(createMechanicalObject(o))
    t.append(ET.Element("UniformMass",template="Vec3d"))
 
    v = ET.Element("Node",name="Visual")
    og = exportVisual(o, scn,name = 'Visual', with_transform = False)
    og.set('template', 'ExtVec3f')
    v.append(og)
    v.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",object1="../MO",object2="Visual"))
    t.append(v)

    # set n later
    t.append(ET.Element("SparseGridTopology",position="@Visual/Visual.position",quads="@Visual/Visual.quads",triangles="@Visual/Visual.triangles",n="10 10 10"))
    # set young modulus later
    t.append(ET.Element("HexahedronFEMForceField",template="Vec3d",youngModulus=str(o.get('youngModulus')),poissonRatio=str(o.get('poissonRatio'))))

    for q in o.children:
        if q.name.startswith('BoxConstraint'):
            tl = q.matrix_world * Vector(q.bound_box[0])
            br = q.matrix_world * Vector(q.bound_box[6])
            t.append(ET.Element("BoxConstraint",box=vector_to_string(tl)+ ' ' + vector_to_string(br)))
        elif q.name.startswith('SphereConstraint'):
            n = q.name.replace('.', '_')
            t.append(ET.Element("SphereROI",name=n,centers=vector_to_string(q.location),radii=str(max(q.scale))))
            t.append(ET.Element("FixedConstraint", indices="@%s.indices" % n))
            
    c = ET.Element("Node",name="Collision")    
    c.append(exportTopology(o,scn))
    c.append(ET.Element("MechanicalObject",template="Vec3d",name="MOC"))
    c.extend([ ET.Element("PointModel",selfCollision='0'), ET.Element("LineModel",selfCollision='0'), ET.Element("TriangleModel",selfCollision='1') ])
    c.append(ET.Element("BarycentricMapping",input="@../",output="@./"))
    t.append(c)
    
    return t

def exportHaptic(o, scn):
    t = ET.Element("Node",name=o.name)
    t.append(ET.Element("RequiredPlugin",name="Sensable Plugin",pluginName="Sensable"))
    t.append(ET.Element("NewOmniDriver",name="Omni Driver",deviceName="Phantom 1",listening="true",tags="Omni",forceScale="0.5",scale="500", permanent="true", printLog="1"))
    t.append(ET.Element("GraspingManager",name="graspingManager0",listening="1"))
    #Mechanical Object
    momain = createMechanicalObject(o)
    momain.set('template', 'Rigid')
    momain.set('name', 'instrumentstate')
    momain.set('tags', 'Omni')
    momain.set('position', '1 0 0 0 0 0 1')
    t.append(momain)   
    
    t.append(ET.Element("UniformMass", template="Rigid", name="mass", totalmass="0.05"))
    #Visual Model
    t.append(exportVisual(o, scn, name = 'Visual', with_transform = False))
    t.append(ET.Element("RigidMapping", template = "Rigid,ExtVec3f", object1="instrumentstate", object2="Visual"))
    #Collision Model
    c = ET.Element("Node",name="Collision")    
    c.append(exportTopology(o,scn))
      
    mo = createMechanicalObject(o)
    mo.set('template','Vec3d')
    c.append(mo)
    
    c.append(ET.Element("PointModel", template= "Vec3d",name="ParticleModel", contactStiffness="0.1", contactFriction="0.01" ,contactResponse = "stick"))
    c.append(ET.Element("RigidMapping",template = "Rigid,ExtVec3f", object1="instrumentstate", object2="MO"))
    t.append(c)  
    return t

def exportEmptyHaptic(o,scn):
    t = ET.Element("Node", name = o.name)
    t.append(ET.Element("RequiredPlugin",name="Sensable Plugin",pluginName="Sensable"))
    t.append(ET.Element("NewOmniDriver",name="Omni Driver",deviceName="Phantom 1",listening="true",tags="Omni",forceScale="0.5",scale="500", permanent="true", printLog="1"))
    t.append(ET.Element("GraspingManager",name="graspingManager0",listening="1"))
    
    #Mechanical Object for Articulation
    t.append(ET.Element("MechanicalObject", name = "Articulations", template="Vec1d", position="0 0 0 0"))
    #
    ct = ET.Element("Node", name = "Tool")
    ct.append(ET.Element("MechanicalObject", template="Rigid3d", name = "instrumentState", tags="Omni1", position="0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 "))
    ct.append(ET.Element("UniformMass", template = "Rigid3d", name="mass", totolmass="0.05"))
    ct.append(ET.Element("ArticulatedSystemMapping", input1="@../Articulations", output="@instrumentState"))
    
    #Collision Model
    cm = ET.Element("Node", name = "CM")
    mo =  createMechanicalObject(o)
    mo.set('name','Particle')
    mo.set('force', '0 0 0')
    mo.set('externalForce','0 0 0')
    mo.set('derivX', '0 0 0')
    mo.set('resetScale','1')
    cm.append(mo)
    cm.append(ET.Element("TPointModel", template="Vec3d", name="GraspingToolModel", contactStiffness="2", contactResponse="stick"))
    cm.append(ET.Element("RigidMapping", template="Rigid3d,Vec3d", input="@../instrumentState", output="@Particle"))
    ct.append(cm)
    
    ct.append(ET.Element("ArticulatedHierarchyContainer"))
    #Articulation Hierarchy Containers
    ctns = ET.Element("Node", name="articulationCenters")
    #Container 1
    ctn1 = ET.Element("Node", name="articulationCenter1")
    ctn1.append(ET.Element("ArticulationCenter", parentIndex="0", childIndex="2", posOnParent="0 0 0", posOnChild="0 0 0"))
    a = ET.Element("Node", name="articulations")
    a.append(ET.Element("Articulation", translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="0"))
    ctn1.append(a)
    ctns.append(ctn1)   
    #Container 2
    ctn2 = ET.Element("Node", name="articulationCenter2")
    ctn2.append(ET.Element("ArticulationCenter", parentIndex="0", childIndex="3", posOnParent="0 0 0", posOnChild="0 0 0"))
    a = ET.Element("Node", name="articulations")
    a.append(ET.Element("Articulation", translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="1"))
    ctn2.append(a)
    ctns.append(ctn2)
    #Container 3
    ctn3 = ET.Element("Node", name="articulationCenter3")
    ctn3.append(ET.Element("ArticulationCenter", parentIndex="0", childIndex="1", posOnParent="0 0 0", posOnChild="0 0 0"))
    a = ET.Element("Node", name="articulations")
    a.append(ET.Element("Articulation", translation="0", rotation="0", rotationAxis="1 0 0", articulationIndex="2"))
    ctn3.append(a)
    ctns.append(ctn3)
    
    ct.append(ctns)
    
    #Children start here
    #index is a custom property of a child object if index is missing, then set index=1
    for i in o.children:
        child =  ET.Element("Node", name = i.name)
        ci = i.get('index') or 1
        child.append(exportVisual(i, scn, name = 'Visual', with_transform = True))
        child.append(ET.Element("RigidMapping", input="@../instrumentState", output="@Visual", index=str(ci)))
        ct.append(child)
        
    t.append(ct)
    
    return t
     
def exportObstacle(o, scn):
    t = ET.Element("Node",name=o.name)
    t.append(exportVisual(o, scn, name = 'Visual', with_transform = True))
    t.append(exportTopology(o,scn))
    t.append(createMechanicalObject(o))
    t.extend([ ET.Element("PointModel",moving='0',simulated='0')
        , ET.Element("LineModel",moving='0',simulated='0')
        , ET.Element("TTriangleModel",moving='0',simulated='0') ])
    t.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",object1="MO",object2="Visual"))
    return t
    
def exportRigid(o, scn):
    t = ET.Element("Node",name=o.name)
    t.append(exportVisual(o, scn, name = 'Visual', with_transform = False))
    mo = createMechanicalObject(o)
    mo.set('template','Rigid')
    t.append(mo)
    t.append(ET.Element("RigidMapping",template='Rigid,ExtVec3f',object1="MO",object2="Visual"))
    return t

from array import array
    
def exportTopology(o,scn):
    m = o.to_mesh(scn, True, 'PREVIEW')
    
    t = ET.Element("MeshTopology",name='Topology')
    position = [ vector_to_string(v.co) for v in m.vertices]
    triangles = [ vector_to_string(f.vertices) for f in m.polygons if len(f.vertices) == 3 ]
    quads     = [ vector_to_string(f.vertices) for f in m.polygons if len(f.vertices) == 4 ]

    t.set("position", ' '.join(position))
    t.set("triangles", ' '.join(triangles))
    t.set("quads", ' '.join(quads))    
    return t    
    
def exportVisual(o, scn, name = None,with_transform = True):

    m = o.to_mesh(scn, True, 'RENDER')
    
    o.rotation_mode = "ZYX"
    
    t = ET.Element("OglModel",name=name or o.name)
    
    if with_transform :
        t.set("translation", vector_to_string(o.location))
        t.set("rotation", vector_to_string(vector_degrees(o.rotation_euler)))
        t.set("scale3d", vector_to_string(o.scale))

    position = [ vector_to_string(v.co) for v in m.vertices]
    t.set("position", ' '.join(position))
    normal   = [ vector_to_string(v.normal) for v in m.vertices]
    t.set("normal", ' '.join(normal))

    triangles = [ vector_to_string(f.vertices) for f in m.polygons if len(f.vertices) == 3 ]
    quads     = [ vector_to_string(f.vertices) for f in m.polygons if len(f.vertices) == 4 ]
    t.set("triangles", ' '.join(triangles))
    t.set("quads", ' '.join(quads))    

    if len(m.uv_layers) >= 1 :
        uvl = m.uv_layers[0].data
        ## allocate a mapping between vertex indices and loop indices
        mapping = array('I',[ 0 for i in range(0,len(m.vertices)) ])
        for l in m.loops: mapping[l.vertex_index] = l.index     
        texcoords = [ vector_to_string(uvl[mapping[i]].uv) for i in range(0,len(m.vertices))]
        t.set("texcoords", ' '.join(texcoords))

    if len(m.materials) >= 1 :
        mat = m.materials[0]
        
        d = vector_to_string(mat.diffuse_color*mat.diffuse_intensity) 
        a = vector_to_string(mat.diffuse_color*mat.ambient) 
        s = vector_to_string(mat.specular_color*mat.specular_intensity)  
        e = vector_to_string(mat.diffuse_color*mat.emit) 
        ss = mat.specular_hardness
        text = "Default Diffuse 1 %s 1 Ambient 1 %s 1 Specular 1 %s 1 Emissive 1 %s 1 Shininess 1 %d " % (d,a,s,e,ss)
        
        t.set("material", text)
        if len(mat.texture_slots) >= 1 and mat.texture_slots[0] != None :
            tex = mat.texture_slots[0].texture
            if tex.type == 'IMAGE' :
                t.set("texturename", bpy.path.abspath(tex.image.filepath))
                t.set("material","")
    return t
    

def has_modifier(o,name_of_modifier):
    for i in o.modifiers:
        if i.type == name_of_modifier: 
            return True    
    return False

def exportScene(scene,dir):
    root= ET.Element("Node")
    root.set("name", "root")
    if scene.use_gravity :
        root.set("gravity",vector_to_string(scene.gravity))
    else:
        root.set("gravity","0 0 0")
    
    if scene.get('displayFlags') != None :
        root.append(ET.Element("VisualStyle",displayFlags=scene['displayFlags']))
    if scene.get('includes') != None :
        for i in scene['includes'].split(';') :
            root.append(ET.Element("include", href=i))
            
    # for late alarmDistance="0.1"  contactDistance="0.0005"  attractDistance="0.01"
    root.append(ET.Element("DefaultPipeline"))
    root.append(ET.Element("BruteForceDetection"))
    root.append(ET.Element("MinProximityIntersection",useSurfaceNormals="1",contactDistance="0.001",alarmDistance="1"))
    root.append(ET.Element("DefaultContactManager"))
    
    root.append(ET.Element("LightManager"))
    root.append(ET.Element("OglSceneFrame"))
    for o in scene.objects: 
        if not o.hide_render and o.parent == None:
            annotated_type = o.get('annotated_type')
            if o.type == 'MESH' or o.type == 'SURFACE':
                if has_modifier(o,'SOFT_BODY') or annotated_type == 'SOFT_BODY':
                    t = exportSoftBody(o, scene)
                elif has_modifier(o,'COLLISION') or annotated_type == 'COLLISION':
                    t = exportObstacle(o, scene)
                elif has_modifier(o,'HAPTIC') or annotated_type == 'HAPTIC':
                    t = exportHaptic(o, scene)
                elif o.rigid_body != None and o.rigid_body.enabled:
                    t = exportRigid(o, scene)
                else:
                    t = exportVisual(o, scene)
                
                root.append(t) 
            elif o.type == "LAMP":
                if o.data.type == 'SPOT':
                    t = ET.Element("SpotLight", name=o.name)
                    o.rotation_mode = "QUATERNION"
                    t.set("position", vector_to_string(o.location))
                    t.set("color", vector_to_string(o.data.color))
                    direction = o.rotation_quaternion * Vector((0,0,-1))
                    t.set("direction",vector_to_string(direction))
                    root.append(t)
                elif o.data.type == 'POINT':
                    t = ET.Element("PositionalLight", name=o.name)
                    t.set("position", vector_to_string(o.location))
                    t.set("color", vector_to_string(o.data.color))
                    root.append(t)
            elif o.type == "EMPTY":
                if has_modifier(o,'HAPTIC') or annotated_type == 'HAPTIC':
                    t = exportEmptyHaptic(o, scene)
                root.append(t)
    return root    

def exportSceneToFile(C, filepath):
    dir = os.path.dirname(filepath)
    
    root = exportScene(C.scene, dir)                    
            
    ET.ElementTree(root).write(filepath)

    return {'FINISHED'}


# ExportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator



class ExportToSofa(Operator, ExportHelper):
    """Export to Sofa XML scene format"""
    bl_idname = "export.tosofa"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Export To Sofa XML"

    # ExportHelper mixin class uses this
    filename_ext = ".scn"

    filter_glob = StringProperty(
            default="*.scn",
            options={'HIDDEN'},
            )


    @classmethod
    def poll(cls, context):
        return context.scene is not None
 
    def execute(self, context):
        return exportSceneToFile(context, self.filepath)




from subprocess import Popen
from tempfile import mktemp

class RunSofaOperator(bpy.types.Operator):
    bl_idname = "scene.runsofa"
    bl_label = "Run Simulation in Sofa"

    @classmethod
    def poll(cls, context):
        return context.scene is not None

    def execute(self, context):
        if bpy.data.filepath == '':
            fn = mktemp(suffix='.scn')
        else:
            fn = bpy.data.filepath + '.scn'
        exportSceneToFile(context, fn)
        Popen(fn,shell=True)
        return {'FINISHED'}


############## Register/Unregister add-on ###########################################

# Only needed if you want to add into a dynamic menu
def menu_func_export(self, context):
    self.layout.operator(ExportToSofa.bl_idname, text="To Sofa XML Scene")

addon_keymaps = []

def register():
    bpy.utils.register_class(ExportToSofa)
    bpy.types.INFO_MT_file_export.append(menu_func_export)

    bpy.utils.register_class(RunSofaOperator)

    # handle the keymap
    wm = bpy.context.window_manager
    km = wm.keyconfigs.addon.keymaps.new(name='Object Mode', space_type='EMPTY')

    kmi = km.keymap_items.new(RunSofaOperator.bl_idname, 'F5', 'PRESS')
    addon_keymaps.append((km, kmi))
    #kmi.properties.total = 4


def unregister():
    bpy.utils.unregister_class(ExportToSofa)
    bpy.types.INFO_MT_file_export.remove(menu_func_export)

    # handle the keymap
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()
        
    bpy.utils.unregister_class(RunSofaOperator)
    
    
if __name__ == "__main__":
    register()
    bpy.ops.export.tosofa('INVOKE_DEFAULT')
    #bpy.ops.scene.runsofa('INVOKE_DEFAULT')









