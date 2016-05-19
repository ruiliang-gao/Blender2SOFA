import bpy
import os
import xml.etree.ElementTree as ET
from mathutils import Vector, Euler, Quaternion
from math import degrees, sqrt
from array import array
from io import StringIO
from .lua_format import *
import numpy as np
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator
import math
import bmesh

FILEFORMATS = [ ('.salua', 'SaLua', 'Lua based scene file'), ('.scn', 'XML', 'XML scene file') ]

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
        elif isinstance(o, bool):
            node.set(a, str(int(o)))
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

def rotation_to_quaternion(o):
    if o.rotation_mode == 'QUATERNION':
        q = o.rotation_quaternion
    else:
        q = Euler(o.rotation_euler, o.rotation_mode).to_quaternion()
    return Vector([q[1],q[2],q[3],q[0]])

def createMechanicalObject(o):
    t = ET.Element("MechanicalObject",template="Vec3d",name="MO")
    t.set("translation", o.location)
    t.set("rotation", rotation_to_XYZ_euler(o))
    t.set("scale3d", o.scale)
    return t

def addSolvers(t):
    t.append(ET.Element("EulerImplicitSolver", rayleighMass="0.0", rayleighStiffness="0.0"))
    t.append(ET.Element("CGLinearSolver",iterations="50", tolerance="1.0e-10", threshold="1.0e-6"))

def exportTetrahedralTopology(o, opt, name):
    if o.type == 'MESH' and hasattr(o.data,'tetrahedra') and len(o.data.tetrahedra) > 0:
      m = o.data
    else:
      raise ExportException("While processing %s: Tetrahedral mesh expected!" % o.name)

    points =  np.empty((len(m.vertices),3))
    for i, v in enumerate(m.vertices):
        points[i] = v.co

    tetrahedra = np.empty([len(m.tetrahedra), 4],dtype=int)
    for i, f in enumerate(m.tetrahedra):
        tetrahedra[i] = f.vertices

    c =  ET.Element('TetrahedronSetTopologyContainer', name= name, createTriangleArray='true', points = points, tetrahedra = tetrahedra)
    return geometryNode(opt,c)

def exportHexahedralTopology(o, opt, name):
    if o.type == 'MESH' and hasattr(o.data,'hexahedra') and len(o.data.hexahedra) > 0:
      m = o.data
    else:
      raise ExportException("While processing %s: Hexahedral mesh expected!" % o.name)

    points =  np.empty((len(m.vertices),3))
    for i, v in enumerate(m.vertices):
        points[i] = v.co

    hexahedra = np.empty([len(m.hexahedra), 8],dtype=int)
    for i, f in enumerate(m.hexahedra):
        hexahedra[i] = f.vertices

    c =  ET.Element('HexahedronSetTopologyContainer', name= name, points = points, hexahedra = hexahedra) # createTriangleArray='1'
    return geometryNode(opt,c)

def exportThickShellTopologies(o, opt, name):
    m = o.to_mesh(opt.scene, True, 'PREVIEW')
    thickness = o.thickness
    layerCount    = o.layerCount
    assert(layerCount >= 1)
    V = len(m.vertices)
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
      shell[1][i*2  ] = [f.vertices[0],f.vertices[1],f.vertices[2]]
      shell[1][i*2+1] = [f.vertices[0],f.vertices[2],f.vertices[3]]
      shell[0][i*2  ] = [f.vertices[0],f.vertices[2],f.vertices[1]]
      shell[0][i*2+1] = [f.vertices[0],f.vertices[3],f.vertices[2]]

    oshell = ET.Element('MeshTopology', name = name + "-outer", triangles = shell[1], points = points[V*layerCount:V*(layerCount+1), ...])
    ishell = ET.Element('MeshTopology', name = name + "-inner", triangles = shell[0], points = points[0:V, ...])
    c =  ET.Element('HexahedronSetTopologyContainer', name= name, points = points, hexahedra = hexahedra)
    return geometryNode(opt,c), geometryNode(opt, oshell), geometryNode(opt, ishell)

def addConstraintCorrection(o, t):
    if o.precomputeConstraints:
        t.append(ET.Element('PrecomputedConstraintCorrection', rotations="true", recompute="false"))
    else:
        t.append(ET.Element('UncoupledConstraintCorrection'))

def exportThickCurveTopology(o, opt, name):
    m = o.to_mesh(opt.scene, True, 'PREVIEW')

    points =  np.empty([len(m.vertices),3], dtype=float)
    for i, v in enumerate(m.vertices):
        points[i] = v.co

    H = int(len(m.vertices)/4-1)
    hexahedra = np.empty([H, 8], dtype=int)
    for i in range(H):
      for j in range(8):
        hexahedra[i][j] = i*4 + j

    return geometryNode(opt, ET.Element('HexahedronSetTopologyContainer',name = name, points = points, hexahedra = hexahedra))

def exportThickCurve(o, opt):
    name = fixName(o.name)
    t = ET.Element('Node', name = name)
    t.set('author-parent', 'SolverNode')
    t.set('author-order', 1)

    topo = name + '-topology'
    t.append(exportThickCurveTopology(o, opt, topo))

    t.append(createMechanicalObject(o))
    t.append(ET.Element('HexahedronSetTopologyModifier', removeIsolated = 'false'))
    t.append(ET.Element('HexahedronSetTopologyAlgorithms'))
    t.append(ET.Element('HexahedronSetGeometryAlgorithms'))

    # set massDensity later
    t.append(ET.Element("DiagonalMass"))

    h = ET.Element("HexahedronFEMForceField",template="Vec3d", method="large")
    addElasticityParameters(o,h)
    t.append(h)
    addConstraintCorrection(o, t)
    addConstraints(o, t)


    if o.carvable:
      qs = ET.Element('Node', name="quad-surface")
      qs.append(ET.Element("QuadSetTopologyContainer", name=name + "-quadSurf"))
      qs.append(ET.Element("QuadSetGeometryAlgorithms", template="Vec3d"))
      qs.append(ET.Element("QuadSetTopologyModifier"))
      qs.append(ET.Element("QuadSetTopologyAlgorithms", template="Vec3d"))
      qs.append(ET.Element("Hexa2QuadTopologicalMapping", input='@../' + topo, output="@" + name + "-quadSurf"))
      ogl = ET.Element("OglModel", name= name + '-visual');
      qs.append(ogl)
      addMaterial(o, ogl);
      qs.append(ET.Element('IdentityMapping', input="@../MO", output="@" + name + '-visual'))

      ts = ET.Element('Node', name="triangle-surface")
      ts.append(ET.Element('TriangleSetTopologyContainer',name=name + '-triSurf'))
      ts.append(ET.Element('TriangleSetTopologyModifier'))
      ts.append(ET.Element('TriangleSetTopologyAlgorithms', template="Vec3d"))
      ts.append(ET.Element('TriangleSetGeometryAlgorithms', template="Vec3d"))
      ts.append(ET.Element('Quad2TriangleTopologicalMapping', input = "@../" + name + "-quadSurf", output = "@" + name + "-triSurf"))
      ts.extend(collisionModelParts(o))

      qs.append(ts)
      t.append(qs)
    else:
        n = ET.Element('Node', name="Collision")
        n.append(exportTriangularTopology(o,opt))
        moc = createMechanicalObject(o)
        moc.set('name', 'MOC')
        n.append(moc)
        n.extend(collisionModelParts(o))
        n.append(ET.Element("BarycentricMapping",input="@../MO",output="@MOC"))
        t.append(n)

        v = ET.Element('Node', name="Visual")
        v.append(exportVisual(o, opt, name = name + "-visual"))
        v.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",input="@../MO",output= '@' + name + "-visual"))
        t.append(v)

    return t

def exportThickQuadShell(o, opt):
    name = fixName(o.name)
    t = ET.Element("Node", name = name)
    t.set('author-parent' , 'SolverNode')
    t.set('author-order', 1)

    topo = name + '-hexahedral-topology'
    c, oshell, ishell = exportThickShellTopologies(o, opt, topo)
    t.append(c)

    t.append(createMechanicalObject(o))

    t.append(ET.Element('HexahedronSetTopologyModifier'))
    t.append(ET.Element('HexahedronSetTopologyAlgorithms'))
    t.append(ET.Element('HexahedronSetGeometryAlgorithms'))

    # TODO: set massDensity later
    t.append(ET.Element("DiagonalMass"))
    h = ET.Element("HexahedronFEMForceField", method="large")
    addElasticityParameters(o,h)
    t.append(h)
    addConstraintCorrection(o, t)
    addConstraints(o, t)


    for i, tp in enumerate([ oshell, ishell ]):
      n = ET.Element('Node')
      if i == 0:
        n.set('name', 'CollisionOuter')
      else:
        n.set('name', 'CollisionInner')
      n.append(tp)
      n.append(ET.Element('TriangleSetTopologyContainer', src = "@" + tp.get('name')))
      n.append(ET.Element('EdgeSetTopologyModifier'))
      moc = createMechanicalObject(o)
      moc.set('name', 'MOC')
      n.append(moc)
      n.extend(collisionModelParts(o, group = o.collisionGroup + i, bothSide = 0))
      n.append(ET.Element("BarycentricMapping",input="@../MO",output="@MOC"))
      t.append(n)

    v = ET.Element('Node', name="Visual")
    v.append(ET.Element("QuadSetTopologyContainer", name= name + "-quadSurf"))
    v.append(ET.Element("QuadSetGeometryAlgorithms", template="Vec3d"))
    v.append(ET.Element("QuadSetTopologyModifier"))
    v.append(ET.Element("QuadSetTopologyAlgorithms", template="Vec3d"))
    v.append(ET.Element("Hexa2QuadTopologicalMapping", input='@../' + topo, output="@" + name + "-quadSurf"))
    smoothSurface = False
    if smoothSurface:
        v.append(ET.Element('RequiredPlugin', name='SurfLabSplineSurface'));
        b3 = ET.Element('BiCubicSplineSurface');
        addMaterialToBicubic(o, b3);
        v.append(b3);
    else:
        v.append(exportVisual(o, opt, name = name + "-visual"))
        v.append(ET.Element("BarycentricMapping",template="Vec3d,ExtVec3f",input="@../MO",output='@' + name + "-visual"))
    t.append(v)


    return t




def exportVolumetric(o, opt):
    name = fixName(o.name)
    t = ET.Element("Node", name = name)
    t.set('author-parent' , 'SolverNode')
    t.set('author-order', 1)

    topotetra = name + '-topology'
    t.append(exportTetrahedralTopology(o, opt, topotetra))
    t.append(createMechanicalObject(o))
    t.append(ET.Element('TetrahedronSetTopologyModifier', removeIsolated = "false"))
    t.append(ET.Element('TetrahedronSetTopologyAlgorithms', template = 'Vec3d'))
    t.append(ET.Element('TetrahedronSetGeometryAlgorithms', template = 'Vec3d'))

    # set massDensity later
    t.append(ET.Element("DiagonalMass"))
    f = ET.Element('TetrahedralCorotationalFEMForceField')
    addElasticityParameters(o,f)
    t.append(f)
    addConstraintCorrection(o, t)
    addConstraints(o, t)

    if o.carvable:
        n = ET.Element('Node', name="triangle-surface")
        n.append(ET.Element("TriangleSetTopologyContainer",name="topotri"))
        n.append(ET.Element("TriangleSetTopologyModifier",))
        n.append(ET.Element("TriangleSetTopologyAlgorithms", template="Vec3d" ))
        n.append(ET.Element("TriangleSetGeometryAlgorithms", template="Vec3d"))

        n.append(ET.Element('Tetra2TriangleTopologicalMapping', input="@../"+topotetra, output="@topotri", flipNormals='true'))

        ogl = ET.Element("OglModel", name="Visual");
        addMaterial(o, ogl);
        n.append(ogl)
        n.append(ET.Element("IdentityMapping",input="@../MO",output="@Visual"))
        n.extend(collisionModelParts(o))
        t.append(n)

    else:
        n = ET.Element('Node', name="Collision")
        n.append(exportTriangularTopology(o,opt))
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

    addConnectionsToTissue(t, o, opt)
    return t

def exportHexVolumetric(o, opt):
    name = fixName(o.name)
    t = ET.Element("Node", name = name)

    topotetra = name + '-topology'
    t.append(exportHexahedralTopology(o, opt, topotetra))

    t.append(createMechanicalObject(o))
    t.append(ET.Element('HexahedronSetTopologyModifier', removeIsolated = "false"))
    t.append(ET.Element('HexahedronSetTopologyAlgorithms', template = 'Vec3d'))
    t.append(ET.Element('HexahedronSetGeometryAlgorithms', template = 'Vec3d'))

    # set massDensity later
    t.append(ET.Element("DiagonalMass"))
    h = ET.Element("HexahedronFEMForceField",method="large")
    addElasticityParameters(o,h)
    t.append(h)
    addConstraintCorrection(o, t)
    addConstraints(o, t)

    if o.carvable:
      qs = ET.Element('Node', name="quad-surface")
      qs.append(ET.Element("QuadSetTopologyContainer", name=name + "-quadSurf"))
      qs.append(ET.Element("QuadSetGeometryAlgorithms", template="Vec3d"))
      qs.append(ET.Element("QuadSetTopologyModifier"))
      qs.append(ET.Element("QuadSetTopologyAlgorithms", template="Vec3d"))
      qs.append(ET.Element("Hexa2QuadTopologicalMapping", input='@../' + topotetra, output="@" + name + "-quadSurf"))
      ogl = ET.Element("OglModel", name= name + '-visual');
      qs.append(ogl)
      addMaterial(o, ogl);
      qs.append(ET.Element('IdentityMapping', input="@../MO", output="@" + name + '-visual'))

      ts = ET.Element('Node', name="triangle-surface")
      ts.append(ET.Element('TriangleSetTopologyContainer',name=name + '-triSurf'))
      ts.append(ET.Element('TriangleSetTopologyModifier'))
      ts.append(ET.Element('TriangleSetTopologyAlgorithms', template="Vec3d"))
      ts.append(ET.Element('TriangleSetGeometryAlgorithms', template="Vec3d"))
      ts.append(ET.Element('Quad2TriangleTopologicalMapping', input = "@../" + name + "-quadSurf", output = "@" + name + "-triSurf"))
      ts.extend(collisionModelParts(o))

      qs.append(ts)
      t.append(qs)
    else:
        n = ET.Element('Node', name="Collision")
        n.append(exportTriangularTopology(o,opt))
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
        if q.name.startswith('BoxConstraint') or q.template == 'BOXCONSTRAINT':
            tl = q.matrix_world * Vector(q.bound_box[0])
            br = q.matrix_world * Vector(q.bound_box[6])
            b = array('d')
            b.extend(tl)
            b.extend(br)
            t.append(ET.Element("BoxROI",name=n,box=b))
            t.append(ET.Element("FixedConstraint", indices="@%s.indices" % n))
        elif q.name.startswith('SphereConstraint') or q.template == 'SPHERECONSTRAINT':
            t.append(ET.Element("SphereROI",name=n,centers=(q.matrix_world.translation),radii=(max(cwisemul(q.parent.scale, q.scale)))))
            t.append(ET.Element("FixedConstraint", indices="@%s.indices" % n))

def collisionModelParts(o, obstacle = False, group = None, bothSide = 0):
    if o.suture:
      sutureTag = 'HapticSurface'
    else:
      sutureTag = ''
    M = not obstacle
    sc = o.selfCollision
    if group == None:  group = o.collisionGroup
    return [
        ET.Element("PointModel",selfCollision=sc, contactFriction = o.contactFriction, contactStiffness = o.contactStiffness, group=group, moving = M, simulated = M, bothSide= bothSide ),
        ET.Element("LineModel",selfCollision=sc, contactFriction = o.contactFriction, contactStiffness = o.contactStiffness, group=group, moving = M, simulated = M, bothSide= bothSide ),
        ET.Element("TriangleModel", tags = sutureTag,selfCollision=sc, contactFriction = o.contactFriction, contactStiffness = o.contactStiffness, group=group, moving = M, simulated = M, bothSide= bothSide )
    ]


def exportInstrument(o, opt):
    n = fixName(o.name)
    t = ET.Element("Node", name = n, tags='instrument')
    t.set('author-parent' , 'Haptic-Instrument')
    t.set('author-order', 1)

    # Collision parts of the instrument, the instrument tips
    # an instrument usually has one tip, but in case of clamp object it can have two tips
    tip_names = []
    for i in o.children:
        if i.template == 'INSTRUMENTTIP':
            n = fixName(i.name)
            child = ET.Element("Node", name= n)
            tip_names.append(n)
            if i.type == 'MESH':
                child.append(exportTriangularTopology(i, opt))
            mo = createMechanicalObject(i)
            mo.set('name', 'CM');
            child.append(mo)
            pm = ET.Element("TPointModel", name = 'toolTip',
                             contactStiffness="0.01", bothSide="0", proximity = i.proximity,
                             group= o.collisionGroup
                             )
            if o.toolFunction == 'CARVE':
              pm.set('tags', 'CarvingTool')
            elif o.toolFunction == 'SUTURE':
              pm.set('tags', 'SuturingTool')
            else:
              pm.set('tags', 'GraspingTool')

            child.append(pm)
            child.append(ET.Element("RigidMapping", input="@../../instrumentState",output="@CM",index= 0))
            t.append(child)

    hm = ET.Element("HapticManager", omniDriver = '@../../RigidLayer/driver',
        graspStiffness = "1e3", attachStiffness="1e12", grasp_force_scale = "-1e-3", duration = "50")

    if len(tip_names) == 1:
        hm.set('toolModel', '@'+ tip_names[0] + '/toolTip')
    elif len(tip_names) == 2:
        hm.set('upperJaw', '@'+ tip_names[0] + '/toolTip')
        hm.set('lowerJaw', '@'+ tip_names[1] + '/toolTip')
        hm.set('clampScale', '1 0.1 0.1')

    t.append(hm)

    # Visual parts of the instrument
    for i in o.children:
      if i.type == 'MESH':
        INSTRUMENT_PART_MAP = { 'LEFTJAW': 1, 'RIGHTJAW': 2, 'FIXED': 3 }
        idx = INSTRUMENT_PART_MAP[i.instrumentPart]
        name = fixName(i.name)
        child =  ET.Element("Node", name = fixName(i.name))
        child.append(exportVisual(i, opt, name = name + '-visual', with_transform = True))
        child.append(ET.Element("RigidMapping", input="@../../instrumentState", output="@"+name+"-visual", index= idx))
        t.append(child)

    return t


def exportCloth(o, opt):
    name=fixName(o.name)
    t = ET.Element("Node",name=name)
    t.set('author-parent', 'SolverNode')
    t.set('author-order', 1)

    # Toploogy
    t.append(exportTriangularTopologyContainer(o,opt))
    t.append(ET.Element("TriangleSetTopologyModifier", removeIsolated = "false"))
    t.append(ET.Element("TriangleSetTopologyAlgorithms"))
    t.append(ET.Element("TriangleSetGeometryAlgorithms"))

    # Mechanical Object
    t.append(createMechanicalObject(o))
    t.append(ET.Element("DiagonalMass"))

    # Force fields
    tfff=ET.Element("TriangularFEMForceField", method="large" )
    addElasticityParameters(o,tfff)
    t.append(tfff)
    t.append(ET.Element("TriangularBendingSprings",
        stiffness= o.bendingStiffness, damping = o.damping))

    # Collision and Constraints
    addConstraints(o,t)
    addConstraintCorrection(o, t)
    t.extend(collisionModelParts(o))

    # Visual
    ogl = ET.Element("OglModel", name= name + '-visual');
    addMaterial(o, ogl);
    t.append(ogl)
    t.append(ET.Element("IdentityMapping",template="Vec3d,ExtVec3f",input="@MO",output='@' + name + "-visual"))
    return t

def pointInsideSphere(v,s,f):
    center = s.location
    radius = max(s.scale)
    distance = (v - center).length
    if (distance < radius * f):
        return True
    else:
        return False

def verticesInsideSphere(o, m, s, factor = 1):
    vindex = []
    for v in m.vertices:
        if pointInsideSphere(o.matrix_world*v.co, s, factor):
            vindex.append(v.index)
    return vindex

def exportAttachConstraint(o, opt):
    amf1 = o.alwaysMatchForObject1
    amf2 = o.alwaysMatchForObject2
    stiffness = o.attachStiffness
    o1, o2 = opt.scene.objects[o.object1], opt.scene.objects[o.object2]
    m1, m2 = o1.to_mesh(opt.scene, True, 'PREVIEW'), o2.to_mesh(opt.scene, True, 'PREVIEW')
    v1, v2 = verticesInsideSphere(o1, m1, o), verticesInsideSphere(o2, m2, o)

    # Find points inside sphere by enlarging the sphere gradually
    f = 1
    while amf1 and len(v1) == 0 and f < 1000:
        f *= 1.404
        v1 = verticesInsideSphere(o1, m1, o, f)

    f = 1
    while amf2 and len(v2) == 0 and f < 1000:
        f *= 1.404
        v2 = verticesInsideSphere(o2, m2, o, f)


    # Find the matching vertex pairs, the tuples are:
    #    (index_from_first_object, index_from_second_object, distance_between_two_points)
    vertexPairs = []
    for i in v1:
        mindist = 1E+38
        minindex = -1
        for j in v2:
            dist = (o1.matrix_world*m1.vertices[i].co - o2.matrix_world*m2.vertices[j].co).length
            if dist < mindist :
                minindex = j
                mindist = dist
        if minindex != -1:
            vertexPairs.append((i,minindex, mindist))

    # Create the springs between the vertex pairs
    springs = [ vector_to_string([i, j, stiffness, .1, d]) for (i,j,d) in vertexPairs ]

    ff = ET.Element("StiffSpringForceField", object1='@' + fixName(o1.name), object2='@' + fixName(o2.name), spring = ' '.join(springs))
    ff.set('author-parent', 'SolverNode')
    ff.set('author-order', 100)
    return ff

def exportTriangularTopologyContainer(o,opt):
    t = ET.Element("TriangleSetTopologyContainer")
    addTriangularTopology(o, t, opt)
    return geometryNode(opt, t)

def addTriangularTopology(o, t, opt):
    # First triangulate the mesh
    m = o.to_mesh(opt.scene, True, 'PREVIEW')
    bm = bmesh.new()
    bm.from_mesh(m)
    r = bmesh.ops.triangulate(bm, faces = bm.faces)
    triangles = r['faces']

    # Then create position, edge and tri arrays
    position = np.empty([len(bm.verts), 3],dtype=float)
    for i, v in enumerate(bm.verts):
        position[i] = v.co
    edges = np.empty([len(bm.edges), 2],dtype=int)
    for i, e in enumerate(bm.edges):
        edges[i] = [ v.index for v in e.verts ]
    tri = np.empty([len(triangles), 3],dtype=int)
    for i, f in enumerate(triangles):
        tri[i] = [ v.index for v in f.verts ]
    bm.free()

    t.set("position", position)
    t.set("edges", edges)
    t.set("triangles", tri)
    return t

def addElasticityParameters(o, t):
    t.set("youngModulus", o.youngModulus)
    t.set("poissonRatio", o.poissonRatio)
    t.set("rayleighStiffness", o.rayleighStiffness)
    t.set("damping", o.damping)
    return t


def exportObstacle(o, opt):
    name=fixName(o.name)
    t = ET.Element("Node",name=name)
    t.set('author-parent', 'root')
    t.set('author-order', 1)
    t.append(exportVisual(o, opt, name = name+'-visual', with_transform = True))
    t.append(exportTriangularTopologyContainer(o,opt))
    t.append(createMechanicalObject(o))
    t.extend(collisionModelParts(o,obstacle = True))
    t.append(ET.Element('UncoupledConstraintCorrection'))
    return t

# TODO: debug this and make it work
def exportRigid(o, opt):
    name=fixName(o.name)
    t = ET.Element("Node",name=name)
    t.set('author-parent', 'SolverNode')
    t.set('author-order', 1)

    t.append(exportVisual(o, opt, name = name + '-visual', with_transform = False))
    t.append(exportTriangularTopology(o,opt))

    mo = createMechanicalObject(o)

    mo.set('template','Rigid')
    t.append(mo)
    t.append(ET.Element("RigidMapping",template='Rigid,ExtVec3f',input="@MO",output='@' + name + "-visual"))
    t.extend(collisionModelParts(o,obstacle = False))
    t.append(ET.Element('UncoupledConstraintCorrection'))
    return t

def exportTriangularTopology(o,opt):
    t = ET.Element("MeshTopology",name=fixName(o.name) + '-topology')
    addTriangularTopology(o,t,opt)
    return geometryNode(opt, t)

def fixName(name):
    return name.replace(".","_")

def addMaterialToBicubic(o, t):
    m = o.data
    if len(m.materials) >= 1 :
        mat = m.materials[0]

        a = mat.alpha # alpha should go at the end of each color
        t.set('diffuseColor', mat.diffuse_color*mat.diffuse_intensity)
        t.set('ambientIntensity', mat.ambient)
        t.set('specularColor', mat.specular_color*mat.specular_intensity)
        t.set('shininess', mat.specular_hardness)
        #t.set('emit', mat.diffuse_color*mat.emit)

        #if len(mat.texture_slots) >= 1 and mat.texture_slots[0] != None :
        #    tex = mat.texture_slots[0].texture
        #    if tex.type == 'IMAGE' :
        #        t.set("texturename", bpy.path.abspath(tex.image.filepath))


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
    if o.texture3d != '':
        t.set("texturename", o.texture3d)
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

    triangles = [ f.vertices for f in m.polygons if len(f.vertices) == 3 ]
    quads     = [ f.vertices for f in m.polygons if len(f.vertices) == 4 ]
    t.set("triangles", triangles)
    t.set("quads", quads)

    if len(m.uv_layers) >= 1 :
        uvl = m.uv_layers[0].data
        ## allocate a mapping between vertex indices and loop indices
        mapping = array('I',[ 0 for i in range(0,len(m.vertices)) ])
        for l in m.loops: mapping[l.vertex_index] = l.index
        texcoords = [ (uvl[mapping[i]].uv) for i in range(0,len(m.vertices))]
        t.set("texcoords", texcoords)

    addMaterial(o, t);
    return geometryNode(opt, t)


def exportObject(opt, o):
    t = None
    if not o.hide_render:
        annotated_type = o.template
        name = fixName(o.name)
        if o.type == 'MESH' or o.type == 'SURFACE' or o.type == 'CURVE':
            if annotated_type == 'COLLISION':
                t = exportObstacle(o, opt)
            elif annotated_type == 'CLOTH':
                t = exportCloth(o, opt)
            elif annotated_type == 'VOLUMETRIC':
                if o.type == 'MESH' and hasattr(o.data,'tetrahedra') and len(o.data.tetrahedra) > 0:
                    t = exportVolumetric(o, opt)
                elif o.type == 'MESH' and hasattr(o.data,'hexahedra') and len(o.data.hexahedra) > 0:
                    t = exportHexVolumetric(o, opt)
                else:
                    raise ExportException("Volumetric mesh expected: '%s'" % o.name)
            elif annotated_type == 'THICKSHELL':
                t = exportThickQuadShell(o, opt)
            elif annotated_type == 'THICKCURVE':
                t = exportThickCurve(o, opt)
            elif annotated_type == 'VISUAL':
                t = exportVisual(o, opt)
            elif annotated_type == 'RIGID':
                t = exportRigid(o, opt)
        elif o.type == 'LAMP':
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
    return t


def addConnectionsBetween(t, o, q, opt):
    t.append(ET.Element("RequiredPlugin", name = "SurfLabConnectingTissue"))
    t.append(ET.Element("ConnectingTissue", object1='@' + fixName(o.name), object2='@' + fixName(q.name),useConstraint="false", threshold=o.attachThreshold))

def addConnectionsToTissue(t, o, opt):
    if o.object1 in opt.scene.objects:
        addConnectionsBetween(t, o, opt.scene.objects[o.object1], opt)
    if o.object2 in opt.scene.objects:
        addConnectionsBetween(t, o, opt.scene.objects[o.object2], opt)


def exportHaptic(l, opt):
    scene = opt.scene
    hapticDevices = opt.pref.hapticDevices
    # If there are no haptic devices, then haptic is not enabled
    if len(hapticDevices) == 0:
        return []

    nodes = []
    instruments = []

    # Stuff at the root that are needed for a haptic scene
    nodes.append(ET.Element("RequiredPlugin", pluginName="Sensable"))
    nodes.append(ET.Element("RequiredPlugin", pluginName="SofaSuturing"))
    nodes.append(ET.Element("RequiredPlugin", pluginName="SaLua"))
    nodes.append(ET.Element("LuaController", source = "changeInstrumentController.lua", listening=1))

    # Prepare the instruments, they are included in each haptic
    for o in l:
        if not o.hide_render and o.template == 'INSTRUMENT':
            instruments.append(objectNode(opt, exportInstrument(o, opt)))

    if scene.hapticWorkspaceBox in scene.objects:
        b = scene.objects[scene.hapticWorkspaceBox]
        positionBase = b.location
        orientationBase = rotation_to_quaternion(b)
        scaleBase = pow(b.scale[0] * b.scale[1] * b.scale[2], 1./3)
    else:
        positionBase = [0, 0, 0]
        orientationBase = [0, 0, 0, 1]
        scaleBase = 1


    for hp in hapticDevices:
        n = hp.deviceName
        t = ET.Element("Node", name = hp.deviceName, tags='haptic')
        omniTag = n + "__omni"

        ## Omni driver wrapper
        rl = ET.Element("Node", name="RigidLayer")
        rl.append(ET.Element("NewOmniDriver",
                             name = 'driver',
                             deviceName = hp.deviceName,
                             tags= omniTag, scale = hp.scale * scaleBase , positionBase = positionBase, orientationBase = orientationBase, desirePosition = positionBase,
                             permanent="true", listening="true", alignOmniWithCamera="false",
                             forceScale = hp.forceScale));
        rl.append(ET.Element("MechanicalObject", name="ToolRealPosition", tags=omniTag, template="Rigid", position="0 0 0 0 0 0 1",free_position="0 0 0 0 0 0 1"))
        nt = ET.Element("Node",name = "Tool");
        nt.append(ET.Element("MechanicalObject", template="Rigid", name="RealPosition"))
        nt.append(ET.Element("SubsetMapping", indices="0"));
        rl.append(nt);
        t.append(rl)

        # State of the tool
        isn = ET.Element("Node",name = "Instruments_of_"+n);
        isn.append(ET.Element("EulerImplicitSolver", rayleighMass="0.0", rayleighStiffness="0.0"))
        isn.append(ET.Element("CGLinearSolver",iterations="100", tolerance="1.0e-20", threshold="1.0e-20"))
        isn.append(ET.Element("MechanicalObject", name = "instrumentState", template="Rigid3d", position="0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1", free_position="0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1" ))
        isn.append(ET.Element("UniformMass", template = "Rigid3d", name="mass", totalmass="0.1"))
        isn.append(ET.Element("LCPForceFeedback", activate=hp.forceFeedback, tags=omniTag, forceCoef="1.0"))
        isn.extend(instruments)
        isn.append(ET.Element("RestShapeSpringsForceField", template="Rigid",stiffness="1e12",angularStiffness="1e12", external_rest_shape="../RigidLayer/ToolRealPosition", points = "0"))
        isn.append(ET.Element("UncoupledConstraintCorrection"))
        t.append(isn)

        nodes.append(objectNode(opt, t))

    return nodes

def objectNode(opt, t):
    if t != None and opt.separate:
        return exportSeparateFile(opt, t, t.get('name'))
    else:
        return t


def fovOfCamera(c):
    """Calculate field of view in degrees from focal length of a lens,
    assuming the standard film size"""
    correction = 717 / 1024.0
    return 2 * math.atan(c.sensor_width / (2 * c.lens)) * 180 / math.pi * correction


def exportCamera(o, opt):
    fov = fovOfCamera(o.data)
    position=o.location
    orientation=rotation_to_quaternion(o)
    lookAt = o.matrix_world * Vector((0,0,-1))
    return ET.Element("InteractiveCamera", position=position, orientation=orientation, fieldOfView=fov, distance=1)

def exportScene(opt):
    scene = opt.scene
    selection = opt.selection_only
    separate = opt.separate
    dir = opt.directory

    root= ET.Element("Node")
    root.set("name", "root")
    if scene.use_gravity :
        root.set("gravity",scene.gravity)
    else:
        root.set("gravity",[0,0,0])
    root.set("dt",0.01)


    if scene.camera is not None:
        root.append(exportCamera(scene.camera, opt))


    #lcp = ET.Element("LCPConstraintSolver", tolerance="1e-6", maxIt = "1000", mu = scene.mu, '1e-6'))
    lcp = ET.Element("GenericConstraintSolver", tolerance="1e-6", maxIterations = "1000")
    root.append(lcp)

    root.append(ET.Element('FreeMotionAnimationLoop'))
    root.append(ET.Element("CollisionPipeline", depth="6"))
    root.append(ET.Element("BruteForceDetection"))
    root.append(ET.Element("LocalMinDistance", angleCone = "0.0", alarmDistance=scene.alarmDistance,contactDistance=scene.contactDistance))
    root.append(ET.Element("CollisionGroup"))
    root.append(ET.Element('CollisionResponse', response="FrictionContact"))

    solverNode = ET.Element("Node", name="SolverNode")
    addSolvers(solverNode)


    root.append(ET.Element("LightManager"))
    if scene.showXYZFrame:
      root.append(ET.Element("OglSceneFrame"))

    if selection:
        l = list(bpy.context.selected_objects)
    else:
        l = list(scene.objects)
    l.reverse()

    root.extend(exportHaptic(l, opt))

    for o in l:
        t = objectNode(opt, exportObject(opt, o))
        if t != None:
            if o.template == 'COLLISION':
                root.append(t)
            else:
                solverNode.append(t)

    for o in l:
        if not o.hide_render and (o.object1 != '' or o.object2 != ''):
            addConnectionsToTissue(solverNode, o, opt)
        if not o.hide_render and o.template == 'ATTACHCONSTRAINT':
            solverNode.append( objectNode(opt, exportAttachConstraint(o, opt)) )
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


class ExportToSofa(Operator, ExportHelper):
    """Export to SOFA scene"""
    bl_idname = "export.tosofa"
    bl_label = "Export To SOFA"

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
            opt.pref = context.user_preferences.addons[__package__].preferences
            root = exportScene(opt)
            writeNodesToFile(root, self.filepath, opt)

            return {'FINISHED'}
        except ExportException as et:
            self.report({'ERROR'}, "Export failed: %s" % et.message)
            return { 'CANCELLED' }
