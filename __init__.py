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
from export2sofa.mesh2tetra import convert as convertMesh2Tetra
from export2sofa.ui import register as uiRegister
from export2sofa.ui import unregister as uiUnregister

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

    c =  ET.Element('TetrahedronSetTopologyContainer', name="topotetra")
    c.set('points', ndarray_to_flat_string(points))
    c.set('tetrahedra', ndarray_to_flat_string(tetrahedra))
    
    mo = createMechanicalObject(o)
    
    mo.set('position','@topotetra.position')
    t.append(c)
    t.append(mo)
    t.append(ET.Element('TetrahedronSetTopologyModifier'))
    t.append(ET.Element('TetrahedronSetTopologyAlgorithms', template = 'Vec3d'))
    t.append(ET.Element('TetrahedronSetGeometryAlgorithms', template = 'Vec3d'))
    
    # set massDensity later
    t.append(ET.Element("DiagonalMass"))
    
    # set youngModulus and poissonRatio later, and method=large
    tetrahedralCorotationalFEMForceField = ET.Element('TetrahedralCorotationalFEMForceField')
    generateYoungModulus(o,tetrahedralCorotationalFEMForceField)
    generatePoissonRatio(o,tetrahedralCorotationalFEMForceField)
    tetrahedralCorotationalFEMForceField.set("damping", str(o.get('damping')))
    t.append(tetrahedralCorotationalFEMForceField)
    
    t.append(ET.fromstring('<UncoupledConstraintCorrection compliance="0.001   0.00003 0 0   0.00003 0   0.00003" />'))

    n = ET.Element('Node', name="triangle-surface")
    t.append(n)
    ts = ET.Element("TriangleSet", tags="SuturingSurface", group= "2") 

    addConstraints(o, t)

    if o.get('carvable'):

        n.append(ET.Element("TriangleSetTopologyContainer",name="topotri"))
        n.append(ET.Element("TriangleSetTopologyModifier",))
        n.append(ET.Element("TriangleSetTopologyAlgorithms", template="Vec3d" ))
        n.append(ET.Element("TriangleSetGeometryAlgorithms", template="Vec3d"))

        n.append(ET.Element('Tetra2TriangleTopologicalMapping', object1="../../topotetra", object2="topotri"))

        ogl = ET.Element("OglModel", name="Visual", genTex3d = "1");
        addMaterial(o.data, ogl);
        n.append(ogl)
        n.append(ET.Element("IdentityMapping",object1="../MO",object2="Visual"))
        n.append(ts)
        
        n.extend(collisionModelParts(o))
        
    else:
        n.append(exportVisual(o, scn, name = "Visual"))
        n.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",object1="../MO",object2="Visual"))
        t.append(ts)
        
        n.extend(collisionModelParts(o))

    return t

def addConstraints(o, t):
    for q in o.children:
        if q.name.startswith('BoxConstraint'):
            tl = q.matrix_world * Vector(q.bound_box[0])
            br = q.matrix_world * Vector(q.bound_box[6])
            t.append(ET.Element("BoxConstraint",box=vector_to_string(tl)+ ' ' + vector_to_string(br)))
        elif q.name.startswith('SphereConstraint'):
            n = q.name.replace('.', '_')
            t.append(ET.Element("SphereROI",name=n,centers=vector_to_string(q.location),radii=str(max(q.scale))))
            t.append(ET.Element("FixedConstraint", indices="@%s.indices" % n))

def collisionModelParts(o, obstacle = False):
    if obstacle:
        M = "0"
    else:
        M = "1"

    return [ 
        ET.Element("PointModel",selfCollision='0', contactFriction = str(o.get('contactFriction')), contactStiffness = str(o.get('contactStiffness')), group=str(o.get('collisionGroup','1')), moving = M, simulated = M ), 
        ET.Element("LineModel",selfCollision='0', contactFriction = str(o.get('contactFriction')), contactStiffness = str(o.get('contactStiffness')), group=str(o.get('collisionGroup','1')), moving = M, simulated = M), 
        ET.Element("TriangleModel",selfCollision='0', contactFriction = str(o.get('contactFriction')), contactStiffness = str(o.get('contactStiffness')), group=str(o.get('collisionGroup','1')), moving = M, simulated = M) 
    ]

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
    sparseGridTopology = ET.Element("SparseGridTopology",position="@Visual/Visual.position",quads="@Visual/Visual.quads",triangles="@Visual/Visual.triangles",n="10 10 10")
    sparseGridTopology.set("n",str(o.get('resX')) + ' ' + str(o.get('resY')) + ' ' + str(o.get('resZ')) )
    t.append(sparseGridTopology)
   
   # set young modulus later
    #t.append(ET.Element("HexahedronFEMForceField",template="Vec3d",youngModulus=str(o.get('youngModulus')),poissonRatio=str(o.get('poissonRatio'))))
    h = ET.Element("HexahedronFEMForceField",template="Vec3d", method="large")
    generateYoungModulus(o,h)
    generatePoissonRatio(o,h)
    h.set("rayleighStiffness", str(o.get('rayleighStiffness')))
    t.append(h)
    
    t.append(ET.fromstring('<UncoupledConstraintCorrection />'))
    addConstraints(o, t)
                
    c = ET.Element("Node",name="Collision")    
    c.append(exportTopology(o,scn))
    c.append(ET.Element("MechanicalObject",template="Vec3d",name="MOC"))
    c.extend(collisionModelParts(o))
    c.append(ET.Element("BarycentricMapping",input="@../",output="@./"))
    t.append(c)
    return t

def exportHaptic(o, scn):
    t = ET.Element("Node",name=fixName(o.name))
    t.append(ET.Element("RequiredPlugin",name="Sensable Plugin",pluginName="Sensable"))
    newOmniDriver = ET.Element("NewOmniDriver",name="Omni Driver",deviceName=o.get('deviceName',''),listening="true",tags="Omni", permanent="true", printLog="1")
    newOmniDriver.set("forceScale", str(o.get('forceScale')))
    newOmniDriver.set("scale", str(o.get('scale')))
    t.append(newOmniDriver)
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
    n = fixName(o.name)
    t = ET.Element("Node", name = n)
    omniTag = n + "__omni"

    ## Omni driver wrapper
    rl = ET.Element("Node", name="RigidLayer")
    rl.append(ET.Element("RequiredPlugin",name="Sensable Plugin",pluginName="Sensable"))
    rl.append(ET.Element("MechanicalObject", name="ToolRealPosition", tags=omniTag, template="Rigid"))
    rl.append(ET.Element("NewOmniDriver",
                         deviceName = str(o.get('deviceName','')), 
                         tags= omniTag, scale = str(o.get("scale", 300)),
                         permanent="true", listening="true", alignOmniWithCamera="true",
                         forceScale = str(o.get("forceScale", 0.01))));
    nt = ET.Element("Node",name = "Tool");
    nt.append(ET.Element("MechanicalObject", template="Rigid", name="RealPosition"))
    nt.append(ET.Element("SubsetMapping", indices="0"));
    rl.append(nt);
    t.append(rl)

    addSolvers(t);

    # State of the tool
    t.append(ET.Element("MechanicalObject", name = "instrumentState", 
                        template="Rigid3d",
                        position="0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1"))
    t.append(ET.Element("UniformMass", template = "Rigid3d", name="mass", totalmass="0.05"))
    t.append(ET.Element("LCPForceFeedback", activate=str(o.get('forceFeedback',"false")), tags=omniTag, forceCoef="0.1"))

    t.append(ET.Element("RestShapeSpringsForceField", 
            template="Rigid",stiffness="10000000",angularStiffness="2000000",
            external_rest_shape="RigidLayer/Tool/RealPosition", points = "0"))

    
    #Collision Model
    #cm = ET.Element("Node", name = "CM")
    #mo =  createMechanicalObject(o)
    #mo.set('name','Particle')
    #cm.append(mo)
    #pm = ET.Element("TPointModel",
    #                     template="Vec3d",  
    #                     contactStiffness="0.01", bothSide="true",
    #                     #contactResponse="stick"
    #                     )
    
    #toolFunction = o.get('toolFunction', 'Grasp');
    #if toolFunction == 'Carve': pm.set('tags', 'CravingTool')
    #elif toolFunction == 'Suture': pm.set('tags', 'SuturingTool')

    #cm.append(pm)
    #cm.append(ET.Element("RigidMapping", template="Rigid3d,Vec3d", input="@../instrumentState", output="@Particle"))
    #t.append(cm)
    
    for i in o.children:
        if(i.get('index', 0) != 0): 
            child = ET.Element("Node", name= fixName(i.name) + "__CM")
            #child.append(exportTopology(i, scn))
            mo = createMechanicalObject(i)
            mo.set('name', 'CM');
            child.append(mo)
            pm = ET.Element("TPointModel",
                                 template="Vec3d",  
                                 contactStiffness="0.01", bothSide="true",
                                 group="0"
                                 )
    
            toolFunction = o.get('toolFunction', 'Grasp');
            if toolFunction == 'Carve': pm.set('tags', 'CarvingTool')
            elif toolFunction == 'Suture': pm.set('tags', 'SuturingTool')
            elif toolFunction == 'Grasp': pm.set('contactResponse', 'stick')
            child.append(pm)
            child.append(ET.Element("RigidMapping", input="@../instrumentState",output="@CM",index=str(i.get('index', 0))))
            t.append(child)
    
    #Children start here
    #index is a custom property of a child object if index is missing, then set index=1
    for i in o.children:
        child =  ET.Element("Node", name = fixName(i.name))
        child.append(exportVisual(i, scn, name = 'Visual', with_transform = True))
        child.append(ET.Element("RigidMapping", input="@../instrumentState", output="@Visual", index=str(i.get('index', 0))))
        t.append(child)
 
    t.append(ET.Element("UncoupleConstraintCorrection"))   
    #t.append(ET.fromstring('<UncoupledConstraintCorrection compliance="0.001   0.00003 0 0   0.00003 0   0.00003" />'))
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
                c.extend(collisionModelParts(o))
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
    tfff.set("damping", str(o.get('stretchDamping')))
    t.append(tfff)
    
    triangularBendingSprings = ET.Element("TriangularBendingSprings", template="Vec3d")
    triangularBendingSprings.set("stiffness", str(o.get('bendingStiffness')))
    triangularBendingSprings.set("damping", str(o.get('bendingDamping')))
    t.append(triangularBendingSprings)

    t.extend(collisionModelParts(o))

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
    stiffness = o.get('stiffness', 500)
    springs = [
        vector_to_string([i, j, stiffness, 5, 3.5]) for (i,j) in zip(verticesInsideSphere(o1, o, scn), matchVertices(o1,o2,o, scn))
        ]
    ff = ET.Element("StiffSpringForceField", object1=fixName(o1.name), object2=fixName(o2.name), 
                    spring = vector_to_string(springs))  

    return ff
    
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
    if True or len(o.data.vertices) < 200:
        t.append(exportTopology(o,scn))
        t.append(createMechanicalObject(o))
        t.extend(collisionModelParts(o,obstacle = True))
    else:
        t.append(ET.Element("SparseGridTopology",position="@Visual.position",quads="@Visual.quads",triangles="@Visual.triangles",n="10 10 10"))
        t.append(createMechanicalObject(o))
        t.append(ET.Element('TSphereModel'))
    #t.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",object1="MO",object2="Visual"))
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

def addMaterial(m, t):
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

    addMaterial(m, t);
    return t
    

def has_modifier(o,name_of_modifier):
    for i in o.modifiers:
        if i.type == name_of_modifier: 
            return True    
    return False

def exportScene(scene,dir, selection, separate):
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
             
    lcp = ET.Element("LCPConstraintSolver", tolerance="1e-3", initial_guess="false", build_lcp="0",  printLog="0" )
    if scene.get('mu') != None :
        lcp.set("mu",str(scene.get('mu')))
    root.append(lcp)
    
    root.append(ET.fromstring('<FreeMotionAnimationLoop printLog = "0"/>'))
 
    root.append(ET.Element("CollisionPipeline", depth="15"))
    root.append(ET.Element("BruteForceDetection"))
    
    mpi = ET.Element("MinProximityIntersection",useSurfaceNormals="0")
    if scene.get('alarmDistance'):
        mpi.set("alarmDistance",str(scene.get('alarmDistance')))
    if scene.get('contactDistance'):
        mpi.set("contactDistance", str(scene.get('contactDistance')))
    root.append(mpi)
    
    root.append(ET.Element("CollisionGroup"))

    #root.append(ET.Element("DefaultContactManager"))    
    root.append(ET.fromstring('<CollisionResponse name="Response" response="FrictionContact"  printLog="1"/>'))
    root.append(ET.Element("GraspingManager",name="graspingManager0",listening="1"))
    root.append(ET.Element("RequiredPlugin", pluginName="SofaSuturing"))
    root.append(ET.Element("SuturingManager", attachStiffness="200000", sutureKey="["))
    root.append(ET.Element("RequiredPlugin", pluginName="SofaCarving"))
    root.append(ET.Element("CarvingManager"))

  
    #addSolvers(root)
    root.append(ET.Element("LightManager"))
    root.append(ET.Element("OglSceneFrame"))
    if (selection == True):
        print("Use Selected")
        print(scene)
        l = list(bpy.context.selected_objects)
    else:
        l = list(scene.objects)
    l.reverse()
    for o in l: 
        if not o.hide_render and o.parent == None:
            annotated_type = o.get('annotated_type')
            name = fixName(o.name)
            print(fixName(o.name))
            print(annotated_type)
            t = None
            if o.type == 'MESH' or o.type == 'SURFACE' or o.type == 'CURVE':
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
                elif  has_modifier(o,'ATTACHCONSTRAINT') or annotated_type == 'ATTACHCONSTRAINT':
                    if (isinstance(o.get('object1'),str) and isinstance(o.get('object2'),str)):
                        o1 = bpy.data.objects[o.get('object1')]
                        o2 = bpy.data.objects[o.get('object2')]
                    else:
                        continue
                    t = exportAttachConstraint(o, o1, o2, scene)
                elif annotated_type == 'VOLUMETRIC':
                    t = exportVolumetric(o, scene)
                else:
                    t = exportVisual(o, scene)
                 
            elif o.type == "LAMP":
                if o.data.type == 'SPOT':
                    t = ET.Element("SpotLight", name=fixName(o.name))
                    o.rotation_mode = "QUATERNION"
                    t.set("position", vector_to_string(o.location))
                    t.set("color", vector_to_string(o.data.color))
                    direction = o.rotation_quaternion * Vector((0,0,-1))
                    t.set("direction",vector_to_string(direction))
                elif o.data.type == 'POINT':
                    t = ET.Element("PositionalLight", name=fixName(o.name))
                    t.set("position", vector_to_string(o.location))
                    t.set("color", vector_to_string(o.data.color))
            elif o.type == "EMPTY":
                if has_modifier(o,'HAPTIC') or annotated_type == 'HAPTIC':
                    t = exportEmptyHaptic(o, scene)
                elif has_modifier(o,'CM') or annotated_type == 'CM':
                    t = exportCM(o,scene)
            elif o.type == "META":
                if  has_modifier(o,'ATTACHCONSTRAINT') or annotated_type == 'ATTACHCONSTRAINT':
                    if (isinstance(o.get('object1'),str) and isinstance(o.get('object2'),str)):
                        o1 = bpy.data.objects[o.get('object1')]
                        o2 = bpy.data.objects[o.get('object2')]
                    else:
                        continue
                    t = exportAttachConstraint(o, o1, o2, scene)
        
            if (t != None):
                if (separate):
                    ET.ElementTree(t).write(dir+"/"+name+".xml")
                    root.append(ET.Element("include", href=name+".xml"))
                else:
                    root.append(t)
        
    return root    


def exportSceneToFile(C, filepath, selection, separate):
    dir = os.path.dirname(filepath)
    
    root = exportScene(C.scene, dir, selection, separate)
    
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
            
    use_selection = BoolProperty(
            name="Selection Only",
            description="Export Selected Objects Only",
            default=False,
            )
    
    export_separate = BoolProperty(
            name="Export to Separate Files",
            description="Export Objects into Separate Files and Include all in one *.scn File",
            default=False,
            )

    @classmethod
    def poll(cls, context):
        return context.scene is not None
 
    def execute(self, context):
        return exportSceneToFile(context, self.filepath, self.use_selection, self.export_separate)


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
        exportSceneToFile(context, fn, False, False)
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

    kmi = km.keymap_items.new(RunSofaOperator.bl_idname, 'M', 'PRESS', ctrl=True,shift=True)
    addon_keymaps.append((km, kmi))
    kmi = km.keymap_items.new(RunSofaOperator.bl_idname, 'F5', 'PRESS')
    addon_keymaps.append((km, kmi))

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
    bpy.ops.export.tosofa('INVOKE_DEFAULT')
    #bpy.ops.scene.runsofa('INVOKE_DEFAULT')

