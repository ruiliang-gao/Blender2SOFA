import bpy
import os
import xml.etree.ElementTree as ET
from mathutils import Vector, Euler, Quaternion
from math import degrees
from array import array
from io import StringIO
from .lua_format import *
import numpy as np

class ExportException(Exception):
  def __init__(self, message):
    Exception.__init__(self, message)
    self.message = message

def ndarray_to_flat_string(a):
    b = StringIO()
    f = a.reshape(a.size)
    for i in f:
        b.write(str(i))
        b.write(' ')
    s = b.getvalue()
    b.close()
    return s

def iterable(o):
    return hasattr(o, '__getitem__') and hasattr(o, '__len__')

def vector_to_string(v):
    t = ""
    for i in v :
        if iterable(i):
            t += vector_to_string(i) + ' '
        else:
            t += str(i) + ' '
    return t

def stringify_etree(node):
    for a in node.attrib:
        o = node.get(a)
        if isinstance(o, str):
            pass
        elif isinstance(o, np.ndarray):
            node.set(a, ndarray_to_flat_string(o))
        elif iterable(o):
            node.set(a, vector_to_string(o))
        else:
            node.set(a, str(o))
    for c in node:
        stringify_etree(c)

def exportSeparateFile(opt, t, name):
  base = os.path.join(opt.directory, name)
  if opt.file_format == '.salua':
    r = ET.Element("require", href=name)
    writeSubTreeToLua(t, base + '.lua')
  else:
    ext = '.xml'
    r = ET.Element("include", href=name + ext)
    stringify_etree(t)
    ET.ElementTree(t).write(base + ext)
  return r

def geometryNode(opt, t):
    """
    Special handling for geometry nodes when needed
    Most of the time this is identity function. But when isolate_geometry
    is enabled it will put the geometry node into a separate file
    and return the node.
    """
    if opt.isolate_geometry and t.get('name') != None:
        return exportSeparateFile(opt, t, t.get('name') + "-geometry")
    else:
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
    t.set("translation", (o.location))
    t.set("rotation", (rotation_to_XYZ_euler(o)))
    t.set("scale3d", (o.scale))
    return t

def addSolvers(t):
    t.append(ET.Element("EulerImplicitSolver", rayleighMass="0.1", rayleighStiffness="0.1"))
    t.append(ET.Element("CGLinearSolver",iterations="40", tolerance="1.0e-10", threshold="1.0e-6"))

def exportTetrahedralTopology(o, opt, name):
    if o.type == 'MESH' and hasattr(o.data,'tetrahedra') and len(o.data.tetrahedra) > 0:
      m = o.data
    else:
      raise ExportException("While processing %s: Tetrahedral mesh expected!" % o.name)

    points =  np.empty((len(m.vertices),3))
    for i, v in enumerate(m.vertices):
        points[i][0] = v.co[0]
        points[i][1] = v.co[1]
        points[i][2] = v.co[2]

    tetrahedra = np.empty([len(m.tetrahedra), 4],dtype=int)
    for i, f in enumerate(m.tetrahedra):
        tetrahedra[i][0] = f.vertices[0]
        tetrahedra[i][1] = f.vertices[1]
        tetrahedra[i][2] = f.vertices[2]
        tetrahedra[i][3] = f.vertices[3]

    c =  ET.Element('TetrahedronSetTopologyContainer', name= name, createTriangleArray='1')
    c.set('points', points)
    c.set('tetrahedra', tetrahedra)
    return geometryNode(opt,c)

def exportHexahedralTopology(o, opt, name):
    if o.type == 'MESH' and hasattr(o.data,'hexahedra') and len(o.data.hexahedra) > 0:
      m = o.data
    else:
      raise ExportException("While processing %s: Hexahedral mesh expected!" % o.name)

    points =  np.empty((len(m.vertices),3))
    for i, v in enumerate(m.vertices):
      for j in range(3):
        points[i][j] = v.co[j]

    hexahedra = np.empty([len(m.hexahedra), 8],dtype=int)
    for i, f in enumerate(m.hexahedra):
      for j in range(8):
        hexahedra[i][j] = f.vertices[j]

    # c =  ET.Element('HexahedronSetTopologyContainer', name= name, createTriangleArray='1')
    c =  ET.Element('HexahedronSetTopologyContainer', name= name)
    c.set('points', points)
    c.set('hexahedra', hexahedra)
    return geometryNode(opt,c)

# Export a HexahedronSetTopology container with the topology of a
# thick shell. The object is supposed to be convertible to a quad mesh.
# The object can have two custom attributes:
#   - thickness: total thickness of the shell multiplied by normal
#   - layerCount: total number of layers generated. 3 means 4 layers of surfaces and 3 layers of hexahedral elements.
#
# Return value is three nodes,
#   - Volumetric topology for physical model
#   - Outer shell topology
#   - Inner shell topology
def exportThickShellTopologies(o, opt, name):
    m = o.to_mesh(opt.scene, True, 'PREVIEW')
    thickness = o.get('thickness', 0.1)
    layerCount    = o.get('layerCount', 1)
    if layerCount < 1: raise ExportException("Object '%s': Number of layers has to be a positive number" % o.name)
    V = len(m.vertices)
    #r = (layerCount+1)//2
    #rj = list(range(-r,r+1))
    #if (layerCount%2==1) : del rj[r]
    rj = list(range(-layerCount,1))
    points =  np.empty([V * (layerCount+1),3])
    for i, v in enumerate(m.vertices):
      for j, offset in enumerate(rj):
        vn = v.co + v.normal * offset * thickness
        points[i+V*j][0] = vn[0]
        points[i+V*j][1] = vn[1]
        points[i+V*j][2] = vn[2]

    quads = list(filter(lambda f: len(f.vertices) == 4, m.polygons))
    quadCount = len(quads)

    if quadCount == 0 : raise ExportException("Object '%s' has to be a quad mesh for a thick shell topology" % o.name)

    hexahedra = np.empty([quadCount * layerCount, 8], dtype=int)
    for i, f in enumerate(quads):
      for l in range(0, layerCount):
        hexahedra[l*quadCount+i] = [f.vertices[0]+l*V,f.vertices[1]+l*V,f.vertices[2]+l*V,f.vertices[3]+l*V,f.vertices[0]+(l+1)*V,f.vertices[1]+(l+1)*V,f.vertices[2]+(l+1)*V,f.vertices[3]+(l+1)*V]


    # first one is inner, second one is outer shell
    shell = [ np.empty([quadCount*2, 3], dtype=int), np.empty([quadCount*2, 3], dtype=int) ]

    for i, f in enumerate(quads):
      shell[1][i*2] = [f.vertices[0],f.vertices[1],f.vertices[2]]
      shell[1][i*2+1] = [f.vertices[0],f.vertices[2],f.vertices[3]]
      shell[0][i*2] = [f.vertices[0],f.vertices[2],f.vertices[1]]
      shell[0][i*2+1] = [f.vertices[0],f.vertices[3],f.vertices[2]]

    oshell = ET.Element('MeshTopology', name = name + "-outer", triangles = shell[1], points = points[V*layerCount:V*(layerCount+1), ...])
    ishell = ET.Element('MeshTopology', name = name + "-inner", triangles = shell[0], points = points[0:V, ...])
    c =  ET.Element('HexahedronSetTopologyContainer', name= name)
    c.set('points', points)
    c.set('hexahedra', hexahedra)
    return geometryNode(opt,c), geometryNode(opt, oshell), geometryNode(opt, ishell)

def exportThickQuadShell(o, opt):
    name = fixName(o.name)
    t = ET.Element("Node", name = name)
    t.set('author-parent' , 'SolverNode')
    t.set('author-order', 1)

    topo = name + '-hexahedral-topology'
    c, oshell, ishell = exportThickShellTopologies(o, opt, topo)
    t.append(c)

    mo = createMechanicalObject(o)

    #mo.set('position','@'+topo+'.position')
    t.append(mo)

    t.append(ET.Element('HexahedronSetTopologyModifier'))
    t.append(ET.Element('HexahedronSetTopologyAlgorithms', template = 'Vec3d'))
    t.append(ET.Element('HexahedronSetGeometryAlgorithms', template = 'Vec3d'))

    # set massDensity later
    t.append(ET.Element("DiagonalMass"))

    h = ET.Element("HexahedronFEMForceField",template="Vec3d", method="large")

    addElasticityParameters(o,h)
    #h.set("rayleighStiffness", (o.get('rayleighStiffness')))
    t.append(h)

    if o.get('precomputeConstraints') == True:
        t.append(ET.Element('PrecomputedConstraintCorrection', rotations="true", recompute="0"))
    else:
        t.append(ET.Element('UncoupledConstraintCorrection'))


    addConstraints(o, t)

    collisionGroup = int(o.get('collisionGroup', 1))

    for i, tp in enumerate([ oshell, ishell ]):
      n = ET.Element('Node')
      if i == 0:
        n.set('name', 'Collision Outer')
      else:
        n.set('name', 'Collision Inner')
      n.append(tp)
      n.append(ET.Element('TriangleSetTopologyContainer', src = "@" + tp.get('name')))
      n.append(ET.Element('EdgeSetTopologyModifier'))
      moc = createMechanicalObject(o)
      moc.set('name', 'MOC')
      n.append(moc)
      n.extend(collisionModelParts(o, group = collisionGroup + i, bothSide = 1))
      n.append(ET.Element("BarycentricMapping",input="@../MO",output="@MOC"))
      t.append(n)

    v = ET.Element('Node', name="Visual")
    v.append(exportVisual(o, opt, name = name + "-visual"))
    v.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",input="@../MO",output='@' + name + "-visual"))
    t.append(v)


    return t


# Export a TriCubicBezier with the topology of a thick shell.
# In order to test our TriCubicBezier construction
# The object can have two custom attributes:
#   - thickness: total thickness of the shell multiplied by normal
#   - layerCount: total number of layers generated. 3 means 4 layers of surfaces and 3 layers of hexahedral elements.
#
# Returns volumetric topology for physical model
def export3BThickShellTopology(o, opt, name):
    m = o.to_mesh(opt.scene, True, 'PREVIEW')
    thickness = o.get('thickness', 0.1)
    layerCount    = o.get('layerCount', 1)
    if layerCount < 1: raise ExportException("Object '%s': Number of layers has to be a positive number" % o.name)
    quads = list(filter(lambda f: len(f.vertices) == 4, m.polygons))
    quadCount = len(quads)
    if quadCount == 0 or quadCount != len(m.polygons) : raise ExportException("Object '%s' has to be a quad mesh for a thick shell topology" % o.name)

    V = len(m.vertices)
    rj = list(range(-layerCount,1))
    points =  np.empty([V * (layerCount+1),3])
    for i, v in enumerate(m.vertices):
      for j, offset in enumerate(rj):
        vn = v.co + v.normal * offset * thickness
        points[i+V*j][0] = vn[0]
        points[i+V*j][1] = vn[1]
        points[i+V*j][2] = vn[2]


    hexahedra = np.empty([quadCount * layerCount, 8], dtype=int)
    for i, f in enumerate(quads):
      for l in range(0, layerCount):
            hexahedra[l*quadCount+i] = [
                f.vertices[0]+ l   *V,f.vertices[1]+ l   *V,f.vertices[3]+ l   *V,f.vertices[2]+ l   *V,
                f.vertices[0]+(l+1)*V,f.vertices[1]+(l+1)*V,f.vertices[3]+(l+1)*V,f.vertices[2]+(l+1)*V]


    c =  ET.Element('TricubicBezierSetTopologyContainer', name= name)
    c.set('points', points)
    c.set('hexahedra', hexahedra)
    return geometryNode(opt,c)


def export3BThickQuadShell(o, opt):
    name = fixName(o.name)
    t = ET.Element("Node", name = name)

    t.append(ET.Element('RequiredPlugin', name = 'SurfLabHexahedralIGA'));

    t.append(export3BThickShellTopology(o, opt, name + '-hexahedral-topology'))
    t.append(createMechanicalObject(o))
    #t.append(ET.Element("DiagonalMass"))
    t.append(addElasticityParameters(o,ET.Element("TricubicBezierForceField")))

    #if o.get('precomputeConstraints') == True:
    #    t.append(ET.Element('PrecomputedConstraintCorrection', rotations="true", recompute="0"))
    #else:
    #    t.append(ET.Element('UncoupledConstraintCorrection'))

    addConstraints(o, t)

    t.extend(collisionModelParts(o))

    #t.append(ET.Element('RequiredPlugin', name='SurfLabSplineSurface'));
    #t.append(ET.Element('BiCubicSplineSurface'));

    return t


def exportVolumetric(o, opt):
    name = fixName(o.name)
    t = ET.Element("Node", name = name)
    t.set('author-parent' , 'SolverNode')
    t.set('author-order', 1)

    topotetra = name + '-topology'
    c = exportTetrahedralTopology(o, opt, topotetra)
    t.append(c)

    mo = createMechanicalObject(o)

    mo.set('position','@'+topotetra+'.position')
    t.append(mo)
    t.append(ET.Element('TetrahedronSetTopologyModifier', removeIsolated = "false"))
    t.append(ET.Element('TetrahedronSetTopologyAlgorithms', template = 'Vec3d'))
    t.append(ET.Element('TetrahedronSetGeometryAlgorithms', template = 'Vec3d'))

    # set massDensity later
    t.append(ET.Element("DiagonalMass"))

    # set youngModulus and poissonRatio later, and method=large
    tetrahedralCorotationalFEMForceField = ET.Element('TetrahedralCorotationalFEMForceField')
    addElasticityParameters(o,tetrahedralCorotationalFEMForceField)
    t.append(tetrahedralCorotationalFEMForceField)

    if o.get('precomputeConstraints') == True:
        t.append(ET.Element('PrecomputedConstraintCorrection', rotations="true", recompute="0"))
    else:
        t.append(ET.Element('UncoupledConstraintCorrection'))


    addConstraints(o, t)

    if o.get('carvable'):
        n = ET.Element('Node', name="triangle-surface")
        n.append(ET.Element("TriangleSetTopologyContainer",name="topotri"))
        n.append(ET.Element("TriangleSetTopologyModifier",))
        n.append(ET.Element("TriangleSetTopologyAlgorithms", template="Vec3d" ))
        n.append(ET.Element("TriangleSetGeometryAlgorithms", template="Vec3d"))

        n.append(ET.Element('Tetra2TriangleTopologicalMapping', input="@../"+topotetra, output="@topotri", flipNormals='1'))

        ogl = ET.Element("OglModel", name="Visual");
        addMaterial(o, ogl);
        n.append(ogl)
        n.append(ET.Element("IdentityMapping",input="@../MO",output="@Visual"))
        n.extend(collisionModelParts(o))
        t.append(n)

    else:
        n = ET.Element('Node', name="Collision")
        n.append(exportTopology(o,opt))
        moc = createMechanicalObject(o)
        moc.set('name', 'MOC')
        n.append(moc)
        n.extend(collisionModelParts(o))
        n.append(ET.Element("BarycentricMapping",object1="../MO",object2="MOC"))
        t.append(n)

        v = ET.Element('Node', name="Visual")
        v.append(exportVisual(o, opt, name = name + "-visual"))
        v.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",object1="../MO",object2=name + "-visual"))
        t.append(v)


    return t

def exportHexVolumetric(o, opt):
    name = fixName(o.name)
    t = ET.Element("Node", name = name)

    topotetra = name + '-topology'
    c = exportHexahedralTopology(o, opt, topotetra)
    t.append(c)

    mo = createMechanicalObject(o)

    mo.set('position','@'+topotetra+'.position')
    t.append(mo)
    t.append(ET.Element('HexahedronSetTopologyModifier', removeIsolated = "false"))
    t.append(ET.Element('HexahedronSetTopologyAlgorithms', template = 'Vec3d'))
    t.append(ET.Element('HexahedronSetGeometryAlgorithms', template = 'Vec3d'))

    # set massDensity later
    t.append(ET.Element("DiagonalMass"))

    # set youngModulus and poissonRatio later, and method=large
    h = ET.Element("HexahedronFEMForceField",template="Vec3d", method="large")
    addElasticityParameters(o,h)
    #h.set("rayleighStiffness", (o.get('rayleighStiffness')))
    t.append(h)
    # nn.append(ET.Element("IdentityMapping", object1='MO', object2='ctOglModel'))

    if o.get('precomputeConstraints') == True:
        t.append(ET.Element('PrecomputedConstraintCorrection', rotations="true", recompute="0"))
    else:
        t.append(ET.Element('UncoupledConstraintCorrection'))

    addConstraints(o, t)

    collisionGroup = int(o.get('collisionGroup', 1));

    if o.get('carvable'):
      nn = ET.Element('Node', name="quad-surface")
      nn.append(ET.Element("QuadSetTopologyContainer", name="quadSurf"))
      nn.append(ET.Element("QuadSetGeometryAlgorithms", template="Vec3d"))
      nn.append(ET.Element("QuadSetTopologyModifier"))
      nn.append(ET.Element("QuadSetTopologyAlgorithms", template="Vec3d"))
      nn.append(ET.Element("Hexa2QuadTopologicalMapping", input='@../' + topotetra, output="@quadSurf"))
      ogl = ET.Element("OglModel", name= name + '-visual');
      nn.append(ogl)
      addMaterial(o, ogl);
      nn.append(ET.Element('IdentityMapping', input="@../MO", output="@" + name + '-visual'))

      nnn = ET.Element('Node', name="triangle-surface")
      nnn.append(ET.Element('TriangleSetTopologyContainer',name='triSurf'))
      nnn.append(ET.Element('TriangleSetTopologyModifier'))
      nnn.append(ET.Element('TriangleSetTopologyAlgorithms', template="Vec3d"))
      nnn.append(ET.Element('TriangleSetGeometryAlgorithms', template="Vec3d"))
      # nnn.append(ET.Element('MechanicalObject', template="Vec3d", name="ctTriMO"))
      nnn.append(ET.Element('Quad2TriangleTopologicalMapping', input = "@../quadSurf", output = "@triSurf"))
      nnn.append(ET.Element('LineModel', bothSide="0", contactFriction="0", contactStiffness="500", group=collisionGroup, moving="1", selfCollision="0", simulated="1"))
      nnn.append(ET.Element('PointModel', bothSide="0", contactFriction="0", contactStiffness="500", group=collisionGroup, moving="1", selfCollision="0", simulated="1"))
      nnn.append(ET.Element('TriangleModel', bothSide="0", contactFriction="0", contactStiffness="500", group=collisionGroup, moving="1", selfCollision="0", simulated="1", tags="SuturingSurface")) 


      nn.append(nnn)
      t.append(nn)
    else:
        n = ET.Element('Node', name="Collision")
        n.append(exportTopology(o,opt))
        moc = createMechanicalObject(o)
        moc.set('name', 'MOC')
        n.append(moc)
        n.extend(collisionModelParts(o))
        n.append(ET.Element("BarycentricMapping",object1="../MO",object2="MOC"))
        t.append(n)

        v = ET.Element('Node', name="Visual")
        v.append(exportVisual(o, opt, name = name + "-visual"))
        v.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",object1="../MO",object2=name + "-visual"))
        t.append(v)

    return t

def cwisemul(a, b):
  return Vector([ a.x * b.x, a.y * b.y, a.z * b.z ])

def addConstraints(o, t):
    for q in o.children:
      if not q.hide_render:
        n = fixName(q.name)
        if q.name.startswith('BoxConstraint'):
            tl = q.matrix_world * Vector(q.bound_box[0])
            br = q.matrix_world * Vector(q.bound_box[6])
            b = array('d')
            b.extend(tl)
            b.extend(br)
            t.append(ET.Element("BoxROI",name=n,box=b))
            t.append(ET.Element("FixedConstraint", indices="@%s.indices" % n))
        elif q.name.startswith('SphereConstraint'):
            t.append(ET.Element("SphereROI",name=n,centers=(q.matrix_world.translation),radii=(max(cwisemul(q.parent.scale, q.scale)))))
            t.append(ET.Element("FixedConstraint", indices="@%s.indices" % n))

def collisionModelParts(o, obstacle = False, group = None, bothSide = 0):
    if o.get('suture', False):
      sutureTag = 'HapticSurface'
    else:
      sutureTag = ''
    if obstacle:
        M = "0"
    else:
        M = "1"

    sc = o.get('selfCollision',0)
    if group == None: group = o.get('collisionGroup','1')
    return [
        ET.Element("PointModel",selfCollision=sc, contactFriction = (o.get('contactFriction', 0)), contactStiffness = (o.get('contactStiffness', 500)), group=group, moving = M, simulated = M, bothSide= bothSide ),
        ET.Element("LineModel",selfCollision=sc, contactFriction = (o.get('contactFriction', 0)), contactStiffness = (o.get('contactStiffness', 500)), group=group, moving = M, simulated = M, bothSide = bothSide ),
        ET.Element("TriangleModel",selfCollision=sc, contactFriction = (o.get('contactFriction', 0)), contactStiffness = (o.get('contactStiffness', 500)), group=group, moving = M, simulated = M, tags = sutureTag)
    ]

def exportSoftBody(o, opt):
    name=fixName(o.name)
    t = ET.Element("Node",name = name)
    t.set('author-parent' , 'SolverNode')
    t.set('author-order', 1)

    t.append(createMechanicalObject(o))
    t.append(ET.Element("UniformMass",template="Vec3d", mass=(o.get('mass') or 1)))
    v = ET.Element("Node",name="Visual")
    og = exportVisual(o, opt,name = name + '-visual', with_transform = False)
    og.set('template', 'ExtVec3f')
    v.append(og)
    v.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",input="@../MO",output='@' + name + "-visual"))
    t.append(v)

    # set n later
    sparseGridTopology = ET.Element("SparseGridTopology",position="@Visual/Visual.position",quads="@Visual/Visual.quads",triangles="@Visual/Visual.triangles",n="10 10 10")
    sparseGridTopology.set("n",[o.get('resX'),o.get('resY'),o.get('resZ')] )
    t.append(sparseGridTopology)

   # set young modulus later
    #t.append(ET.Element("HexahedronFEMForceField",template="Vec3d",youngModulus=(o.get('youngModulus')),poissonRatio=(o.get('poissonRatio'))))
    h = ET.Element("HexahedronFEMForceField",template="Vec3d", method="large")
    addElasticityParameters(o,h)
    h.set("rayleighStiffness", (o.get('rayleighStiffness')))
    t.append(h)

    t.append(ET.fromstring('<UncoupledConstraintCorrection />'))
    addConstraints(o, t)

    c = ET.Element("Node",name="Collision")
    c.append(exportTopology(o,opt))
    moc = createMechanicalObject(o)
    moc.set('name', 'MOC')
    c.append(moc)
    c.extend(collisionModelParts(o))
    c.append(ET.Element("BarycentricMapping",input="@../",output="@./"))
    t.append(c)
    return t


def exportInstrument(o, opt):
    n = fixName(o.name)
    t = ET.Element("Node", name = n, tags='instrument')
    t.set('author-parent' , 'Haptic-Instrument')
    t.set('author-order', 1)

    for i in o.children:
        if i.get('annotated_type') == 'INSTRUMENTTIP':
            toolFunction = o.get('function', 'grasp')
            if toolFunction == 'clamp':
                child1 = ET.Element("Node", name= fixName(i.name) + "__UpperJaw")
                if i.type == 'MESH':
                    child1.append(exportTopology(i, opt))
                mo1 = createMechanicalObject(i)
                mo1.set('name', 'CM1')
                mo1.set('position','1 -1 0')
                child1.append(mo1)
                pm1 = ET.Element("TPointModel", name = 'toolTip1',
                                 template="Vec3d",
                                 contactStiffness="0.01", bothSide="true", proximity = o.get('proximity', 0.02),
                                 group= o.get('collisionGroup')
                                 )
                child1.append(pm1)
                child1.append(ET.Element("RigidMapping", input="@../../instrumentState",output="@CM1",index= 0))
                t.append(child1)
                
                child2 = ET.Element("Node", name= fixName(i.name) + "__LowerJaw")
                if i.type == 'MESH':
                    child2.append(exportTopology(i, opt))
                mo2 = createMechanicalObject(i)
                mo2.set('name', 'CM2')
                mo2.set('position','-1 -1 0')
                child2.append(mo2)
                pm2 = ET.Element("TPointModel", name = 'toolTip2',
                                 template="Vec3d",
                                 contactStiffness="0.01", bothSide="true", proximity = o.get('proximity', 0.02),
                                 group= o.get('collisionGroup')
                                 )
                child2.append(pm)
                child2.append(ET.Element("RigidMapping", input="@../../instrumentState",output="@CM2",index= 0))
                t.append(child2)

                t.append(ET.Element("HapticManager", toolModel = '' , omniDriver = '@../../RigidLayer/driver', graspStiffness = "1e3", attachStiffness="1e12", grasp_force_scale = "-1e-3",
                                    upperJaw = '@'+fixName(i.name) + "__UpperJaw/toolTip1", lowerJaw = '@'+fixName(i.name) + "__LowerJaw/toolTip2", clampScale = "0.1 1 0.1" ))
            else:
                child = ET.Element("Node", name= fixName(i.name) + "__CM")
                if i.type == 'MESH':
                    child.append(exportTopology(i, opt))
                mo = createMechanicalObject(i)
                mo.set('name', 'CM');
                child.append(mo)
                pm = ET.Element("TPointModel", name = 'toolTip',
                                 template="Vec3d",
                                 contactStiffness="0.01", bothSide="true", proximity = o.get('proximity', 0.02),
                                 group= o.get('collisionGroup')
                                 )
                if toolFunction == 'carve':
                  pm.set('tags', 'CarvingTool')
                elif toolFunction == 'suture':
                  pm.set('tags', 'SuturingTool')
                else:
                  pm.set('tags', 'GraspingTool')

                child.append(pm)
                child.append(ET.Element("RigidMapping", input="@../../instrumentState",output="@CM",index= 0))
                child.append(ET.Element("HapticManager", toolModel = '@toolTip' , omniDriver = '@../../../RigidLayer/driver', graspStiffness = "1e3", attachStiffness="1e12", grasp_force_scale = "-1e-3", duration = "50"))
                t.append(child)

            break

    #Children start here
    #index is a custom property of a child object if index is missing, then set index=1
    for i in o.children:
      if i.type == 'MESH':
        idx = i.get('index', 3)
        name = fixName(i.name)
        child =  ET.Element("Node", name = fixName(i.name))
        child.append(exportVisual(i, opt, name = name + '-visual', with_transform = True))
        child.append(ET.Element("RigidMapping", input="@../../instrumentState", output="@"+name+"-visual", index= idx))
        t.append(child)

    return t

def exportCM(o,opt):
    """
    This function generates a XML hierarchy for a simple obstacle
    collision model.
    """
    t = ET.Element("Node",name= fixName(o.name))
    t.set('author-parent', 'SolverNode')
    t.set('author-order', 1)
    momain = createMechanicalObject(o)
    t.append(momain)
    for i in o.children:
        if not i.hide_render:
            annotated_type = i.get('annotated_type')
            if annotated_type == 'COLLISIONMODEL':
                c = ET.Element("Node",name = i.name)
                c.append(exportTopology(i,opt))
                c.append(createMechanicalObject(i))
                c.extend(collisionModelParts(o))
                #c.append(ET.Element("BarycentricMapping",input="@../",output="@./"))
                t.append(c)
            elif annotated_type == 'SPARSEGRID':
                # set n later
                s = ET.Element("SparseGridTopology",name = i.name)
                generateTopology(i,s, opt)
                # set young modulus later
                t.append(geometryNode(s))

                h = ET.Element("HexahedronFEMForceField",template="Vec3d")
                addElasticityParameters(i,h)
                t.append(h)
            else:
                v = ET.Element("Node",name = i.name)
                og = exportVisual(i, opt,name = fixName(i.name)+ '-visual', with_transform = False)
                og.set('template', 'ExtVec3f')
                v.append(og)
                #v.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",input="@../MO",output='@' + fixName(i.name) + "-visual"))
                t.append(v)
    return t

def exportCloth(o, opt):
    name=fixName(o.name)
    t = ET.Element("Node",name=name)
    t.set('author-parent', 'SolverNode')
    t.set('author-order', 1)

    t.append(exportTopologyContainer(o,opt))

    t.append(ET.Element("TriangleSetTopologyModifier", removeIsolated = "false"))
    t.append(ET.Element("TriangleSetTopologyAlgorithms", template="Vec3d" ))
    t.append(ET.Element("TriangleSetGeometryAlgorithms", template="Vec3d"))

    momain = createMechanicalObject(o)
    t.append(momain)

    t.append(ET.Element("DiagonalMass", template="Vec3d", massDensity="0.15"))

    tfff=ET.Element("TriangularFEMForceField", template="Vec3d",  method="large" )
    addElasticityParameters(o,tfff)
    tfff.set("damping", (o.get('stretchDamping')))
    t.append(tfff)

    triangularBendingSprings = ET.Element("TriangularBendingSprings", template="Vec3d")
    triangularBendingSprings.set("stiffness", (o.get('bendingStiffness')))
    triangularBendingSprings.set("damping", (o.get('bendingDamping')))
    t.append(triangularBendingSprings)
    addConstraints(o,t)
    t.extend(collisionModelParts(o))

    t.append(ET.Element('UncoupledConstraintCorrection'))

    ogl = ET.Element("OglModel", name= name + '-visual');
    addMaterial(o, ogl);
    t.append(ogl)

    t.append(ET.Element("IdentityMapping",template="Vec3d,ExtVec3f",input="@MO",output='@' + name + "-visual"))
    return t

def pointInsideSphere(v,s):
    center = s.location
    radius = max(s.scale)
    distance = (v - center).length
    if (distance < radius):
        return True
    else:
        return False

def verticesInsideSphere(o, m, s):
    vindex = []
    for v in m.vertices:
        if pointInsideSphere((o.matrix_world*v.co), s):
            vindex.append(v.index)
    #print(vindex)
    return vindex

def matchVertices(o1, o2, s, opt):
    amf = s.get('alwaysMatchFor')

    o = [o1, o2]
    m = [o1.to_mesh(opt.scene, True, 'PREVIEW'), o2.to_mesh(opt.scene, True, 'PREVIEW')]
    v = [verticesInsideSphere(o1, m[0], s), verticesInsideSphere(o2, m[1], s)]

    sph = s.copy()
    while (amf and len(v[amf-1]) == 0):
        sph.scale = sph.scale*2.0
        v[amf-1] = verticesInsideSphere(o[amf-1], m[amf-1], sph)
    bpy.data.objects.remove(sph)

    v3 = []
    for i in v[0]:
        mindist = 1E+38
        minindex = -1
        for j in v[1]:
            #print(i)
            #print(j)
            dist = (o1.matrix_world*m[0].vertices[i].co - o2.matrix_world*m[1].vertices[j].co).length
            if dist < mindist :
                minindex = j
                mindist = dist
        if minindex != -1:
            v3.append((i,minindex, mindist))
    return v3

def exportAttachConstraint(o, o1, o2, opt):
    stiffness = o.get('stiffness', 500)
    springs = [
        vector_to_string([i, j, stiffness, .1, d]) for (i,j,d) in matchVertices(o1,o2,o, opt)
        ]
    ff = ET.Element("StiffSpringForceField", object1='@' + fixName(o1.name), object2='@' + fixName(o2.name),
                    spring = ' '.join(springs))
                        
    ff.set('author-parent', 'SolverNode')
    ff.set('author-order', 100)
    

    return ff

import bmesh

def triangulatedBMesh(o, opt):
    """
    Return a triangulated mesh of o in the BMesh object format
    """
    m = o.to_mesh(opt.scene, True, 'PREVIEW')
    bm = bmesh.new()
    bm.from_mesh(m)
    r = bmesh.ops.triangulate(bm, faces = bm.faces)
    return bm, r['faces']

def exportTopologyContainer(o,opt):

    bm, triangles = triangulatedBMesh(o, opt)
    position = array('d')
    for v in bm.verts:
      position.extend([v.co[0],v.co[1],v.co[2]])
    edges = [ ([ v.index for v in e.verts ]) for e in bm.edges ]
    triangles = [ ([ v.index for v in f.verts ]) for f in triangles ]
    bm.free()

    t = ET.Element("TriangleSetTopologyContainer")

    t.set("position", (position))
    t.set("edges", (edges))
    t.set("triangles", (triangles))
    return geometryNode(opt, t)

def generateTopology(o, t, opt):

    bm, triangles = triangulatedBMesh(o, opt)
    position = array('d')
    for v in bm.verts:
      position.extend([v.co[0],v.co[1],v.co[2]])
    triangles = [ [ v.index for v in f.verts ] for f in triangles ]
    bm.free()

    t.set("position", (position))
    t.set("triangles", (triangles))

    return t

def addElasticityParameters(o, t):
    if o.get('youngModulus') != None :
        t.set("youngModulus", (o.get('youngModulus')))
    if o.get('poissonRatio') != None :
        t.set("poissonRatio", (o.get('poissonRatio')))
    if o.get('rayleighStiffness') != None:
        t.set("rayleighStiffness", (o.get('rayleighStiffness')))
    return t


def exportObstacle(o, opt):
    name=fixName(o.name)
    t = ET.Element("Node",name=name)
    t.set('author-parent', 'root')
    t.set('author-order', 1)
    t.append(exportVisual(o, opt, name = name+'-visual', with_transform = True))
    if True or len(o.data.vertices) < 200:
        t.append(exportTopology(o,opt))
        t.append(createMechanicalObject(o))
        t.extend(collisionModelParts(o,obstacle = True))
    else:
        t.append(ET.Element("SparseGridTopology",position="@Visual.position",quads="@Visual.quads",triangles="@Visual.triangles",n="10 10 10"))
        t.append(createMechanicalObject(o))
        t.append(ET.Element('TSphereModel'))
    t.append(ET.fromstring('<UncoupledConstraintCorrection />'))
    return t

def exportRigid(o, opt):
    name=fixName(o.name)
    t = ET.Element("Node",name=name)
    t.set('author-parent', 'SolverNode')
    t.set('author-order', 1)
    
    t.append(exportVisual(o, opt, name = name + '-visual', with_transform = False))
    t.append(exportTopology(o,opt))

    mo = createMechanicalObject(o)
    mo.set('template','Rigid')
    t.append(mo)
    t.append(ET.Element("RigidMapping",template='Rigid,ExtVec3f',input="@MO",output='@' + name + "-visual"))
    t.extend(collisionModelParts(o,obstacle = False))
    return t

def exportTopology(o,opt):
    t = ET.Element("MeshTopology",name=fixName(o.name) + '-topology')
    generateTopology(o,t,opt)
    return geometryNode(opt, t)

def fixName(name):
    return name.replace(".","_")

def addMaterial(o, t):
    m = o.data
    if len(m.materials) >= 1 :
        mat = m.materials[0]

        d = vector_to_string(mat.diffuse_color*mat.diffuse_intensity)
        a = vector_to_string(mat.diffuse_color*mat.ambient)
        s = vector_to_string(mat.specular_color*mat.specular_intensity)
        e = vector_to_string(mat.diffuse_color*mat.emit)
        tr = mat.alpha
        ss = mat.specular_hardness
        text = "Default Diffuse 1 %s %s Ambient 1 %s 1 Specular 1 %s 1 Emissive 1 %s 1 Shininess 1 %d " % (d,tr,a,s,e,ss)


        t.set("material", text)

        if len(mat.texture_slots) >= 1 and mat.texture_slots[0] != None :
            tex = mat.texture_slots[0].texture
            if tex.type == 'IMAGE' :
                t.set("texturename", bpy.path.abspath(tex.image.filepath))
                t.set("material","")
    if o.get('3dtexture','') != '':
        t.set("texturename", o.get('3dtexture'))
        t.set("genTex3d", '1')

def exportVisual(o, opt, name = None,with_transform = True):

    m = o.to_mesh(opt.scene, True, 'RENDER')
    t = ET.Element("OglModel",name=name or fixName(o.name))

    if with_transform :
        t.set("translation", (o.location))
        t.set("rotation", (rotation_to_XYZ_euler(o)))
        t.set("scale3d", (o.scale))

    position = np.empty([len(m.vertices),3],dtype=float)
    for i,v in enumerate(m.vertices):
        position[i] = v.co

    t.set("position", position)
    normal   = np.empty([len(m.vertices),3],dtype=float)
    for i,v in enumerate(m.vertices):
        normal[i] = v.normal

    t.set("normal", normal)

    triangles = [ (f.vertices) for f in m.polygons if len(f.vertices) == 3 ]
    quads     = [ (f.vertices) for f in m.polygons if len(f.vertices) == 4 ]
    t.set("triangles", (triangles))
    t.set("quads", (quads))

    if len(m.uv_layers) >= 1 :
        uvl = m.uv_layers[0].data
        ## allocate a mapping between vertex indices and loop indices
        mapping = array('I',[ 0 for i in range(0,len(m.vertices)) ])
        for l in m.loops: mapping[l.vertex_index] = l.index
        texcoords = [ (uvl[mapping[i]].uv) for i in range(0,len(m.vertices))]
        t.set("texcoords", (texcoords))

    addMaterial(o, t);
    return geometryNode(opt, t)

def exportCurveTopology(o, opt):
    t = ET.Element("MeshTopology",name=fixName(o.name) + '-topology')
    m = o.to_mesh(opt.scene, True, 'PREVIEW')

    position = array('d')
    for v in m.vertices:
      position.extend([v.co[0],v.co[1],v.co[2]])
    edges = array('I')
    for e in m.edges:
      edges.extend(e.vertices)

    t.set("position", position)
    t.set("edges", edges)

    return geometryNode(opt, t)

def exportThickCurve(o, opt):

    thickness = o.get('thickness', 0.3)
    t = ET.Element("Node", name = fixName(o.name))
    t.set('author-parent', 'root')
    t.set('author-order', 1)
    
    t.append(exportCurveTopology(o, opt))
    t.append(createMechanicalObject(o))
    t.append(ET.Element("Line", proximity = thickness, moving="0", simulated="0"))
    t.append(ET.Element("Point", proximity = thickness, moving="0", simulated="0"))
    return t;

def has_modifier(o,name_of_modifier):
    for i in o.modifiers:
        if i.type == name_of_modifier:
            return True
    return False


def exportObject(opt, o):
    t = None
    if not o.hide_render and o.parent == None:
        annotated_type = o.get('annotated_type')
        name = fixName(o.name)
        if o.type == 'MESH' or o.type == 'SURFACE' or o.type == 'CURVE':
            if has_modifier(o,'SOFT_BODY') or annotated_type == 'SOFT_BODY':
                t = exportSoftBody(o, opt)
            elif has_modifier(o,'COLLISION') or annotated_type == 'COLLISION':
                t = exportObstacle(o, opt)
            elif has_modifier(o,'CLOTH') or annotated_type == 'CLOTH':
                t = exportCloth(o, opt)
            elif o.rigid_body != None and o.rigid_body.enabled or annotated_type == 'RIGID':
                t = exportRigid(o, opt)
            elif annotated_type == 'CONNECTIVETISSUE':
                t = exportConnectiveTissue(o, opt)
            # elif annotated_type == 'VOLUMETRIC':
                # t = exportVolumetric(o, opt)
            elif annotated_type == 'VOLUMETRIC' and o.type == 'MESH' and hasattr(o.data,'tetrahedra') and len(o.data.tetrahedra) > 0:
                t = exportVolumetric(o, opt)
            elif annotated_type == 'VOLUMETRIC' and o.type == 'MESH' and hasattr(o.data,'hexahedra') and len(o.data.hexahedra) > 0:
                print("here ------------------------------------------")
                name = fixName(o.name)
                t = ET.Element("Node", name = name)
                t = exportHexVolumetric(o, opt)
            elif annotated_type == 'THICKSHELL':
                if o.get('degree',1) == 3:
                    t= export3BThickQuadShell(o, opt)
                else:
                    t = exportThickQuadShell(o, opt)
            elif annotated_type == 'THICKCURVE':
                t = exportThickCurve(o, opt)
            elif annotated_type == None or annotated_type == 'VISUAL':
                t = exportVisual(o, opt)

        elif o.type == "LAMP":
            if o.data.type == 'SPOT':
                t = ET.Element("SpotLight", name=fixName(o.name))
                o.rotation_mode = "QUATERNION"
                t.set("position", (o.location))
                t.set("color", (o.data.color))
                direction = o.rotation_quaternion * Vector((0,0,-1))
                t.set("direction",(direction))
            elif o.data.type == 'POINT':
                t = ET.Element("PositionalLight", name=fixName(o.name))
                t.set("position", (o.location))
                t.set("color", (o.data.color))
        elif o.type == "EMPTY":
            if has_modifier(o,'CM') or annotated_type == 'CM':
                t = exportCM(o,opt)
    return t


def exportConstraints(opt, o):
    result = []
    if not o.hide_render and o.parent == None:
        annotated_type = o.get('annotated_type')
        o_list = []
        if  has_modifier(o,'ATTACHCONSTRAINT') or annotated_type == 'ATTACHCONSTRAINT':
            o_list.append(o)
        elif  has_modifier(o,'ATTACHCONSTRAINTGROUP') or annotated_type == 'ATTACHCONSTRAINTGROUP':
            o_list = o.children

        for o in o_list:
            if (isinstance(o.get('object1'),str) and isinstance(o.get('object2'),str)):
                o1 = bpy.data.objects[o.get('object1')]
                o2 = bpy.data.objects[o.get('object2')]
                result.append(exportAttachConstraint(o, o1, o2, opt))

    return result

def exportConnectiveTissue(o, opt):
    scene = opt.scene

    if o.type == 'MESH' and hasattr(o.data,'tetrahedra') and len(o.data.tetrahedra) > 0:
      t = exportVolumetric(o, opt)
    elif o.type == 'MESH' and hasattr(o.data,'hexahedra') and len(o.data.hexahedra) > 0:
      t = exportHexVolumetric(o, opt)
    else:
      raise ExportException("While processing %s: Tetrahedral or Hexahedral mesh expected!" % o.name)
    
    oMesh = o.to_mesh(opt.scene, True, 'PREVIEW')
    
    oTop = scene.objects[o.get('topObject')]
    oBot = scene.objects[o.get('botObject')]
    hasTop = True
    if oTop == oBot:
      hasTop = False
    
    if hasTop:
      topVertices = o.get('topVertices')
      otopMesh = oTop.to_mesh(opt.scene, True, 'PREVIEW')
      ntop = len(otopMesh.vertices)

    
    
    
    botVertices = o.get('botVertices')       
    obotMesh = oBot.to_mesh(opt.scene, True, 'PREVIEW')    
     
    
    bpy.ops.object.select_all(action='DESELECT')
    o.select = True
    if hasTop:
      oTop.select = True 
    oBot.select = True
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

    maxDim = 1e+9 # todo: maxDim = max dimension of the whole scene
    stiffness = o.get('attach_stiffness', 500)
    
    if hasTop:
      map2top = []
      for i, v in enumerate(topVertices):
          oVertex = oMesh.vertices[v].co
          footprint = oTop.closest_point_on_mesh(oVertex,maxDim)
          if footprint[2]==-1:
              print('Error: _init_.py: corresponding vertex not found')
              return             
          face = otopMesh.polygons[footprint[2]].vertices 
          # in *face* find the closest vertex to *oVertex*
          smallestDistance = maxDim   
          optimalVert = -1        
          for j,vj in enumerate(face):        
              distance_vj = (otopMesh.vertices[vj].co-oVertex).length 
              # map2top.append((v,vj,distance_vj))    
              if distance_vj < smallestDistance:            
                  optimalVert = vj 
                  smallestDistance = distance_vj              
          if optimalVert > 0:
              map2top.append((v,optimalVert,smallestDistance))            
      springsTop = [
          vector_to_string([i, j, stiffness, .1, d]) for (i,j,d) in map2top
          ]
      ffTop = ET.Element("StiffSpringForceField", object1='@' + fixName(o.name), object2='@' + fixName(oTop.name), 
                      spring = ' '.join(springsTop))
      t.append(ffTop)

    map2bot = []
    for i, v in enumerate(botVertices):
        oVertex = oMesh.vertices[v].co
        footprint = oBot.closest_point_on_mesh(oVertex,maxDim)
        if footprint[2]==-1:
            print('Error: _init_.py: corresponding vertex not found')
            return
        face = obotMesh.polygons[footprint[2]].vertices
        smallestDistance = maxDim
        optimalVert = -1
        for j,vj in enumerate(face):
            distance_vj = (obotMesh.vertices[vj].co-oVertex).length
            # map2bot.append((v,vj,distance_vj))
            if distance_vj < smallestDistance:
                optimalVert = vj
                smallestDistance = distance_vj
        if optimalVert > 0:
            map2bot.append((v,optimalVert,smallestDistance))

    springsBot = [
        vector_to_string([i, j, stiffness, .1, d]) for (i,j,d) in map2bot
        ]
    ffBot = ET.Element("StiffSpringForceField", object1='@' + fixName(o.name), object2='@' + fixName(oBot.name),
                    spring = ' '.join(springsBot))
    t.append(ffBot)
    t.set('author-order', 50)
    return t

def exportHaptic(l, scene, opt):
    hapticExists = False
    nodes = []
    instruments = []

    # Stuff at the root that are needed for a haptic scene
    nodes.append(ET.Element("RequiredPlugin", pluginName="Sensable"))
    nodes.append(ET.Element("RequiredPlugin", pluginName="SofaSuturing"))
    nodes.append(ET.Element("RequiredPlugin", pluginName="SaLua"))
    nodes.append(ET.Element("LuaController", source = "changeInstrumentController.lua", listening=1))

    # Prepare the instruments, they are included in each haptic
    for o in l:
        if not o.hide_render and o.get("annotated_type") == 'INSTRUMENT':
            t = exportInstrument(o, opt)
            if opt.separate:
              t = exportSeparateFile(opt, t, o.name)
            instruments.append(t)

    for o in l:
        if not o.hide_render and o.get("annotated_type") == 'HAPTIC':
            n = fixName(o.name)
            t = ET.Element("Node", name = n, tags='haptic')
            omniTag = n + "__omni"

            ## Omni driver wrapper
            rl = ET.Element("Node", name="RigidLayer")
            
            rl.append(ET.Element("NewOmniDriver",
                                 name = 'driver',
                                 deviceName = (o.get('deviceName',o.name)),
                                 tags= omniTag, scale = (o.get("scale", 300)),
                                 permanent="true", listening="true", alignOmniWithCamera="true",
                                 forceScale = (o.get("forceScale", 0.01))));
            rl.append(ET.Element("MechanicalObject", name="ToolRealPosition", tags=omniTag, template="Rigid", position="0 0 0 0 0 0 1",free_position="0 0 0 0 0 0 1"))
            nt = ET.Element("Node",name = "Tool");
            nt.append(ET.Element("MechanicalObject", template="Rigid", name="RealPosition"))
            nt.append(ET.Element("SubsetMapping", indices="0"));
            rl.append(nt);
            t.append(rl)

            # State of the tool
            isn = ET.Element("Node",name = "Instrument__"+n);
            isn.append(ET.Element("EulerImplicitSolver", rayleighMass="0.1", rayleighStiffness="0.1"))
            isn.append(ET.Element("CGLinearSolver",iterations="100", tolerance="1.0e-20", threshold="1.0e-20"))
            isn.append(ET.Element("MechanicalObject", name = "instrumentState", template="Rigid3d", position="0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1", free_position="0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1" ))
            isn.append(ET.Element("UniformMass", template = "Rigid3d", name="mass", totalmass="0.1"))
            isn.append(ET.Element("LCPForceFeedback", activate=(o.get('forceFeedback',"true")), tags=omniTag, forceCoef="1.0"))
            isn.extend(instruments)
            isn.append(ET.Element("RestShapeSpringsForceField", template="Rigid",stiffness="10000000",angularStiffness="2000000", external_rest_shape="../RigidLayer/ToolRealPosition", points = "0"))
            isn.append(ET.Element("UncoupledConstraintCorrection"))
            t.append(isn)

            hapticExists = True
            if opt.separate:
              t = exportSeparateFile(opt, t, o.name)
            nodes.append(t)

    if hapticExists:
        return nodes
    else:
        return []

def exportScene(opt):
    scene = opt.scene
    selection = opt.selection_only
    separate = opt.separate
    dir = opt.directory

    root= ET.Element("Node")
    root.set("name", "root")
    if scene.use_gravity :
        root.set("gravity",(scene.gravity))
    else:
        root.set("gravity","0 0 0")

    if scene.get('displayFlags') != None and scene.get('displayFlags') != "" :
        root.append(ET.Element("VisualStyle",displayFlags=scene['displayFlags']))
    if scene.get('includes') != None :
        for i in scene['includes'].split(';') :
            if i.strip() != "":
                root.append(ET.Element("include", href=i))

    #lcp = ET.Element("LCPConstraintSolver", tolerance="1e-6", maxIt = "1000", mu = scene.get('mu', '1e-6'))
    lcp = ET.Element("GenericConstraintSolver", tolerance="1e-3", maxIterations = "1000")
    root.append(lcp)

    root.append(ET.Element('FreeMotionAnimationLoop'))

    root.append(ET.Element("CollisionPipeline", depth="6"))
    root.append(ET.Element("BruteForceDetection"))

    mpi = ET.Element("LocalMinDistance", angleCone = "0.0")
    if scene.get('alarmDistance'):
        mpi.set("alarmDistance",(scene.get('alarmDistance')))
    if scene.get('contactDistance'):
        mpi.set("contactDistance", (scene.get('contactDistance')))
    root.append(mpi)

    root.append(ET.Element("CollisionGroup"))

    #root.append(ET.Element("DefaultContactManager"))
    root.append(ET.fromstring('<CollisionResponse name="Response" response="FrictionContact"/>'))

    # TODO: put all the objects that need a solver e.g. soft bodies, volumetric and attach constraints
    #  into a separate node call it "solverNode"
    # and keep the obstacles in the root.
    # and delete all indiviual solvers in separate objects.
    solverNode = ET.Element("Node", name="SolverNode")
    addSolvers(solverNode)

    root.append(ET.Element("LightManager"))
    root.append(ET.Element("OglSceneFrame"))
    if (selection == True):
        l = list(bpy.context.selected_objects)
    else:
        l = list(scene.objects)
    l.reverse()

    root.extend(exportHaptic(l, scene, opt))


    for o in l:
        t = exportObject(opt, o)
        name = fixName(o.name)
        annotated_type = o.get('annotated_type')
        if (t != None):
            if separate:
              t = exportSeparateFile(opt, t, name)
            if(has_modifier(o,'COLLISION') or o.get("annotated_type") == 'COLLISION'):
                root.append(t)
            else:
                solverNode.append(t)

    for o in l:
        t_list = exportConstraints(opt, o)
        name = fixName(o.name)
        for t in t_list:
            if separate:
              t = exportSeparateFile(opt, t, name)
            solverNode.append(t)

    root.append(solverNode)
    return root

class ExportOptions:
    pass

def writeNodesToFile(root, filepath, opt):
    if opt.file_format == '.salua':
        writeElementTreeToLua(root, filepath)
    else:
        stringify_etree(root)
        ET.ElementTree(root).write(filepath)

# ExportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


FILEFORMATS = [ ('.salua', 'SaLua', 'Lua based scene file'), ('.scn', 'XML', 'XML scene file') ]

class ExportToSofa(Operator, ExportHelper):
    """Export to SOFA scene"""
    bl_idname = "export.tosofa"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Export To SOFA"

    # ExportHelper mixin class uses this
    filename_ext = EnumProperty(
      name ='File format', description='File format of the SOFA scene file to be created',
       items = FILEFORMATS, default='.scn' )

    filter_glob = StringProperty(
            default='*.scn;*.salua',
            options={'HIDDEN'},
            )

    use_selection = BoolProperty(
            name="Selection Only",
            description="Export Selected Objects Only",
            default=False,
            )

    export_separate = BoolProperty(
            name="Isolate objects into separate files",
            description="Export Objects into Separate Files and Include all in one *.scn File",
            default=False,
            )

    isolate_geometry = BoolProperty(
            name="Isolate geometry into separate files",
            description="Put each geometry components of objects into separate files",
            default=False,
            )

    @classmethod
    def poll(cls, context):
        return context.scene is not None

    def execute(self, context):
        self.report({'INFO'}, "Exporting to %s" % self.filepath)
        try:
            opt = ExportOptions()
            opt.isolate_geometry = self.isolate_geometry
            opt.scene = context.scene
            opt.separate = self.export_separate
            opt.selection_only = self.use_selection
            opt.directory = os.path.dirname(self.filepath)
            opt.file_format = self.filename_ext
            root = exportScene(opt)
            writeNodesToFile(root, self.filepath, opt)

            return {'FINISHED'}
        except ExportException as et:
            self.report({'ERROR'}, "Export failed: %s" % et.message)
            return { 'CANCELLED' }
