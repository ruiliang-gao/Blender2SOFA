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
from mathutils import Vector, Euler, Quaternion
from math import degrees
from array import array
from io import StringIO
from .mesh2tetra import convert as convertMesh2Tetra
from .ui import register as uiRegister
from .ui import unregister as uiUnregister

def ndarray_to_flat_string(a):
    b = StringIO()
    f = a.reshape(a.size)
    for i in f:
        b.write(str(i))
        b.write(' ')
    s = b.getvalue()
    b.close()
    return s

def vector_to_string(v):
    t = ""
    for i in v :
        t += str(i) + " "
    return t


def rotation_to_XYZ_euler(o):
    if o.rotation_mode == 'XYZ':
        v = o.rotation_euler
    else:
        if o.rotation_mode == 'QUATERNION':
            q = o.rotation_quaternion
        else:
            q = Euler(o.rotation_euler, o.rotation_mode).to_quaternion()
        v = q.to_euler('XYZ')
    return Vector(map(degrees,v))

def createMechanicalObject(o):
    t = ET.Element("MechanicalObject",template="Vec3d",name="MO")
    t.set("translation", vector_to_string(o.location))
    t.set("rotation", vector_to_string(rotation_to_XYZ_euler(o)))
    t.set("scale3d", vector_to_string(o.scale))
    return t

def addSolvers(t):
    t.append(ET.Element("EulerImplicitSolver", vdamping = "0.0"))
    t.append(ET.Element("CGLinearSolver",template="GraphScattered"))

def exportVolumetric(o, scn):
    points, tetrahedra = convertMesh2Tetra(o, scn)
    t = ET.Element("Node", name = fixName(o.name))

    addSolvers(t)

    c =  ET.Element('TetrahedronSetTopologyContainer', name="topo")
    c.set('points', ndarray_to_flat_string(points))
    c.set('tetrahedra', ndarray_to_flat_string(tetrahedra))
    
    mo = createMechanicalObject(o)
    
    #mo.set('position','@topo.points')
    t.append(c)
    t.append(mo)
    t.append(ET.Element('TetrahedronSetTopologyModifier'))
    t.append(ET.Element('TetrahedronSetTopologyAlgorithms', template = 'Vec3d'))
    t.append(ET.Element('TetrahedronSetGeometryAlgorithms', template = 'Vec3d'))
    
    # set massDensity later
    t.append(ET.Element("DiagonalMass"))
    # set youngModulus and poissonRatio later, and method=large
    t.append(ET.Element('TetrahedralCorotationalFEMForceField'))
    t.append(ET.fromstring('<UncoupledConstraintCorrection />'))

    t.append(ET.Element("TriangleSet"))
    t.append(ET.Element("TTriangleModel", template="Vec3d"))
    t.append(ET.Element("TPointModel", template="Vec3d"))
    t.append(ET.Element("TLineModel", template="Vec3d"))
    
    t.append(exportVisual(o, scn, name = "Visual"))
    t.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",object1="MO",object2="Visual"))
    return t

def exportSoftBody(o, scn):
    t = ET.Element("Node",name=fixName(o.name)) 
    addSolvers(t)
    t.append(createMechanicalObject(o))
    t.append(ET.Element("UniformMass",template="Vec3d", mass=str(o.get('mass') or 1)))
    v = ET.Element("Node",name="Visual")
    og = exportVisual(o, scn,name = 'Visual', with_transform = False)
    og.set('template', 'ExtVec3f')
    v.append(og)
    v.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",object1="../MO",object2="Visual"))
    t.append(v)

    # set n later
    t.append(ET.Element("SparseGridTopology",position="@Visual/Visual.position",quads="@Visual/Visual.quads",triangles="@Visual/Visual.triangles",n="10 10 10"))
    # set young modulus later
    #t.append(ET.Element("HexahedronFEMForceField",template="Vec3d",youngModulus=str(o.get('youngModulus')),poissonRatio=str(o.get('poissonRatio'))))
    h = ET.Element("HexahedronFEMForceField",template="Vec3d", method="large")
    generateYoungModulus(o,h)
    generatePoissonRatio(o,h)
    t.append(h)
    t.append(ET.fromstring('<UncoupledConstraintCorrection />'))
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
    t = ET.Element("Node",name=fixName(o.name))
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
    t = ET.Element("Node", name = fixName(o.name))
    t.append(ET.Element("RequiredPlugin",name="Sensable Plugin",pluginName="Sensable"))
    t.append(ET.Element("NewOmniDriver",name="Omni Driver",deviceName="Phantom 1",listening="true",tags="Omni1",forceScale="0.5",scale="500", permanent="true", printLog="1"))
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
    
    t.append(ET.Element("ArticulatedHierarchyContainer"))
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
    
    t.append(ctns)
    
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

def exportCM(o,scn):
    """
    This function generates a XML hierarchy for a simple obstacle
    collision model.
    """
    t = ET.Element("Node",name= fixName(o.name))
    momain = createMechanicalObject(o)
    t.append(momain)
    for i in o.children:
        if not i.hide_render:
            annotated_type = i.get('annotated_type')
            if annotated_type == 'COLLISIONMODEL':
                c = ET.Element("Node",name = i.name)    
                c.append(exportTopology(i,scn))
                c.append(createMechanicalObject(i))
                c.extend([ ET.Element("PointModel",selfCollision='0'), ET.Element("LineModel",selfCollision='0'), ET.Element("TriangleModel",selfCollision='1') ])
                #c.append(ET.Element("BarycentricMapping",input="@../",output="@./"))
                t.append(c)
            elif annotated_type == 'SPARSEGRID':
                # set n later
                s = ET.Element("SparseGridTopology",name = i.name)
                generateTopology(i,s, scn)
                # set young modulus later
                t.append(s)
            
                h = ET.Element("HexahedronFEMForceField",template="Vec3d")
                generateYoungModulus(i,h)
                generatePoissonRatio(i,h)
                t.append(h)
            else:
                v = ET.Element("Node",name = i.name)
                og = exportVisual(i, scn,name = 'Visual', with_transform = False)
                og.set('template', 'ExtVec3f')
                v.append(og)
                #v.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",object1="../MO",object2="Visual"))
                t.append(v)
    return t

def exportCloth(o, scn):
    t = ET.Element("Node",name=fixName(o.name))
    t.append(ET.Element("EulerImplicitSolver", printLog="0"))
    t.append(ET.Element("CGLinearSolver", template="GraphScattered", iterations="25",  tolerance="1e-009",  threshold="1e-009"))
    
    tp = ET.Element("TriangleSetTopologyContainer")
    generateTopologyContainer(o,tp,scn)
    t.append(tp)
    
    t.append(ET.Element("TriangleSetTopologyModifier",))
    t.append(ET.Element("TriangleSetTopologyAlgorithms", template="Vec3d" ))
    t.append(ET.Element("TriangleSetGeometryAlgorithms", template="Vec3d"))
    
    momain = createMechanicalObject(o)
    t.append(momain)
    
    t.append(ET.Element("DiagonalMass", template="Vec3d", massDensity="0.15"))
    
    tfff=ET.Element("TriangularFEMForceField", template="Vec3d",  method="large" )
    generatePoissonRatio(o,tfff)
    generateYoungModulus(o,tfff)
    t.append(tfff)
    
    t.append(ET.Element("TriangularBendingSprings", template="Vec3d",  stiffness="300",  damping="1"))
    t.append(ET.Element("TriangleSet"))
    t.append(ET.Element("TTriangleModel", template="Vec3d"))
    t.append(ET.Element("TPointModel", template="Vec3d"))
    t.append(ET.Element("TLineModel", template="Vec3d"))

    t.append(ET.fromstring('<UncoupledConstraintCorrection />'))

    og = exportVisual(o, scn,name = 'Visual', with_transform = True)
    og.set('template', 'ExtVec3f')
    t.append(og)
    t.append(ET.Element("IdentityMapping",template="Vec3d,ExtVec3f",object1="MO",object2="Visual"))
    return t

def pointInsideSphere(v,s):
    center = s.location
    radius = max(s.scale)
    distance = (v - center).length
    if (distance < radius):
        return True
    else:
        return False
    
def verticesInsideSphere(o, s, scn):
    m = o.to_mesh(scn, True, 'PREVIEW')
    vindex = []
    for v in m.vertices:
        if pointInsideSphere((o.matrix_world*v.co), s):
            vindex.append(v.index)
    #print(vindex)
    return vindex
    
def matchVertices(o1, o2, s, scn):
    v1 = verticesInsideSphere(o1, s, scn)
    v2 = verticesInsideSphere(o2, s, scn)
    #print(list(o1.data.vertices))
    v3 = []   
    for i in v1:
        distance = []
        minindex = -1
        for j in v2:
            #print(i)
            #print(j)
            dist = (o1.matrix_world*o1.data.vertices[i].co - o2.matrix_world*o2.data.vertices[j].co).length
            distance.append(dist)
            if dist == min(distance):
                minindex = j
        v3.append(minindex)
    return v3
                
def exportAttachConstraint(o, o1, o2, scn):
    
    t = ET.Element("AttachConstraint", object1=fixName(o1.name), object2=fixName(o2.name), twoWay="true", radius="0.1", indices1=vector_to_string(verticesInsideSphere(o1, o, scn)), indices2=vector_to_string(matchVertices(o1,o2,o, scn)))  
    return t
    
def generateTopologyContainer(o, t, scn):
    m = o.to_mesh(scn, True, 'PREVIEW')
    position = [ vector_to_string(v.co) for v in m.vertices ]
    edges = [ vector_to_string(e.vertices) for e in m.edges ]
    triangles = [ vector_to_string(f.vertices) for f in m.polygons if len(f.vertices) == 3 ]
    t.set("position", ' '.join(position))
    t.set("edges", ' '.join(edges))
    t.set("triangles", ' '.join(triangles))   
    return t
    
def generateTopology(o, t, scn):
    
    m = o.to_mesh(scn, True, 'PREVIEW')
    
    position = [ vector_to_string(v.co) for v in m.vertices]
    triangles = [ vector_to_string(f.vertices) for f in m.polygons if len(f.vertices) == 3 ]
    quads     = [ vector_to_string(f.vertices) for f in m.polygons if len(f.vertices) == 4 ]

    t.set("position", ' '.join(position))
    t.set("triangles", ' '.join(triangles))
    t.set("quads", ' '.join(quads))
    
    return t

def generateYoungModulus(o, t):
    if o.get('youngModulus') != None :
        t.set("youngModulus", str(o.get('youngModulus')))
    return t

def generatePoissonRatio(o, t):
    if o.get('poissonRatio') != None :
        t.set("poissonRatio", str(o.get('poissonRatio')))
    return t
 
def exportObstacle(o, scn):
    t = ET.Element("Node",name=fixName(o.name))
    t.append(exportVisual(o, scn, name = 'Visual', with_transform = True))
    t.append(exportTopology(o,scn))
    t.append(createMechanicalObject(o))
    t.extend([ ET.Element("PointModel",moving='0',simulated='0')
        , ET.Element("LineModel",moving='0',simulated='0')
        , ET.Element("TTriangleModel",moving='0',simulated='0') ])
    t.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",object1="MO",object2="Visual"))
    t.append(ET.fromstring('<UncoupledConstraintCorrection />'))
    return t
    
def exportRigid(o, scn):
    t = ET.Element("Node",name=fixName(o.name))
    t.append(exportVisual(o, scn, name = 'Visual', with_transform = False))
    mo = createMechanicalObject(o)
    mo.set('template','Rigid')
    t.append(mo)
    t.append(ET.Element("RigidMapping",template='Rigid,ExtVec3f',object1="MO",object2="Visual"))
    return t
    
def exportTopology(o,scn):
    t = ET.Element("MeshTopology",name='Topology')
    generateTopology(o,t,scn)
    return t    
    
def fixName(name):
    return name.replace(".","_")    
    
def exportVisual(o, scn, name = None,with_transform = True):

    m = o.to_mesh(scn, True, 'RENDER')
    t = ET.Element("OglModel",name=name or fixName(o.name))
    
    if with_transform :
        t.set("translation", vector_to_string(o.location))
        t.set("rotation", vector_to_string(rotation_to_XYZ_euler(o)))
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
 
    # parameters required for it to run cholesystectomy
    # alarmDistance   
    # constactDistance
    # mu
    
            
    # for late alarmDistance="0.1"  contactDistance="0.0005"  attractDistance="0.01"
    lcp = ET.Element("LCPConstraintSolver", tolerance="1e-3", initial_guess="false", build_lcp="0",  printLog="0" )
    if scene.get('mu') != None :
        lcp.set("mu",str(scene.get('mu')))
    root.append(lcp)
    
    root.append(ET.fromstring('<FreeMotionAnimationLoop printLog = "0"/>'))
 
    root.append(ET.Element("CollisionPipeline", depth="15"))
    root.append(ET.Element("BruteForceDetection"))
    
    mpi = ET.Element("MinProximityIntersection",useSurfaceNormals="1")
    if scene.get('alarmDistance'):
        mpi.set("alarmDistance",str(scene.get('alarmDistance')))
    if scene.get('constactDistance'):
        mpi.set("constactDistance", str(scene.get('constactDistance')))
    root.append(mpi)
    
    #root.append(ET.Element("DefaultContactManager"))    
    root.append(ET.fromstring('<CollisionResponse name="Response" response="FrictionContact"  printLog="1"/>'))
    
    addSolvers(root)
    root.append(ET.Element("LightManager"))
    root.append(ET.Element("OglSceneFrame"))
    l = list(scene.objects)
    l.reverse()
    for o in l: 
        if not o.hide_render and o.parent == None:
            annotated_type = o.get('annotated_type')
            print(fixName(o.name))
            print(annotated_type)
            if o.type == 'MESH' or o.type == 'SURFACE':
                if has_modifier(o,'SOFT_BODY') or annotated_type == 'SOFT_BODY':
                    t = exportSoftBody(o, scene)
                elif has_modifier(o,'COLLISION') or annotated_type == 'COLLISION':
                    t = exportObstacle(o, scene)
                elif has_modifier(o,'HAPTIC') or annotated_type == 'HAPTIC':
                    t = exportHaptic(o, scene)
                elif has_modifier(o,'CLOTH') or annotated_type == 'CLOTH':
                    t = exportCloth(o, scene)
                elif o.rigid_body != None and o.rigid_body.enabled or annotated_type == 'RIGID':
                    t = exportRigid(o, scene)
                elif has_modifier(o,'ATTACHCONSTRAINT') or annotated_type == 'ATTACHCONSTRAINT':
                    o1 = bpy.data.objects[o.get('object1')]
                    o2 = bpy.data.objects[o.get('object2')]
                    t = exportAttachConstraint(o, o1, o2, scene)
                elif annotated_type == 'VOLUMETRIC':
                    t = exportVolumetric(o, scene)
                else:
                    t = exportVisual(o, scene)
                
                root.append(t) 
            elif o.type == "LAMP":
                if o.data.type == 'SPOT':
                    t = ET.Element("SpotLight", name=fixName(o.name))
                    o.rotation_mode = "QUATERNION"
                    t.set("position", vector_to_string(o.location))
                    t.set("color", vector_to_string(o.data.color))
                    direction = o.rotation_quaternion * Vector((0,0,-1))
                    t.set("direction",vector_to_string(direction))
                    root.append(t)
                elif o.data.type == 'POINT':
                    t = ET.Element("PositionalLight", name=fixName(o.name))
                    t.set("position", vector_to_string(o.location))
                    t.set("color", vector_to_string(o.data.color))
                    root.append(t)
            elif o.type == "EMPTY":
                if has_modifier(o,'HAPTIC') or annotated_type == 'HAPTIC':
                    t = exportEmptyHaptic(o, scene)
                    root.append(t)
                elif has_modifier(o,'CM') or annotated_type == 'CM':
                    t = exportCM(o,scene)
                    root.append(t)
            elif o.type == "META":
                if  has_modifier(o,'ATTACHCONSTRAINT') or annotated_type == 'ATTACHCONSTRAINT':
                    o1 = bpy.data.objects[o.get('object1')]
                    o2 = bpy.data.objects[o.get('object2')]
                    t = exportAttachConstraint(o, o1, o2, scene)
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

    uiRegister()


def unregister():
    bpy.utils.unregister_class(ExportToSofa)
    bpy.types.INFO_MT_file_export.remove(menu_func_export)

    # handle the keymap
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()
        
    bpy.utils.unregister_class(RunSofaOperator)

    uiUnregister()    
    
if __name__ == "__main__":
    register()
    #bpy.ops.export.tosofa('INVOKE_DEFAULT')
    bpy.ops.scene.runsofa('INVOKE_DEFAULT')

