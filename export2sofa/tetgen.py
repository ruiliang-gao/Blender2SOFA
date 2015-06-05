from ctypes import *
import numpy as np
import numpy.ctypeslib as npc 


p_int = POINTER(c_int)
c_real = c_double
p_real = POINTER(c_real)
c_void = None

class Polygon(Structure):
    _fields_ = [
            ('vertexlist', POINTER(c_int)),
            ('numberofvertices', c_int)
            ]

    def get_vertices(self, v):
        if self.vertexlist:
            return npc.as_array(self.vertexlist,shape=self.numberofvertices)
        else:
            return np.empty(0)

    def set_vertices(self, v):
        """
        Set the vertices using an iterable of integer type or
        an array of integers. The array will not be copied
        """
        self.numberofvertices = len(v)
        if isinstance(v, np.ndarray):
            self.vertexlist = npc.as_ctypes(v);
        else:
            self.vertexlist = (c_int * len(v))(*v)

    vertices = property(get_vertices, set_vertices)

    def __init__(self, vertices):
        self.vertices = vertices

class Facet(Structure):
    _fields_ = [
            ('polygonlist', POINTER(Polygon)),
            ('numberofpolygons', c_int),
            ('holelist', p_real),
            ('numberofholes', c_int)
            ]

    def get_polygons(self):
        ArrayType = (Polygon * self.numberofpolygons)
        return ArrayType.from_address(addressof(self.polygonlist))

    def set_polygons(self, polygons):
        self.numberofpolygons = len(polygons)
        self.polygonlist = (Polygon * len(polygons))(*polygons)

    polygons = property(get_polygons, set_polygons)

    def get_holes(self):
        ArrayType = (c_int * self.numberofholes)
        return ArrayType.from_address(addressof(self.holelist))

    def set_holes(self, holes):
        self.numberofholes = len(holes)
        self.holelist = (c_int * len(holes))(*holes);

    def __init__(self, polygons, holes = None):
        self.polygons = polygons
        if holes != None:
            self.holes = holes


class VoroEdge(Structure):
    _fields_ = [
            ('v1', c_int),('v2', c_int),
            ('vnormal', c_real * 3)
            ]

class VoroFacet(Structure):
    _fields_ = [
            ('c1', c_int),('c2', c_int),
            ('elist', p_int)
            ]

class PointParam(Structure):
    _fields_ = [
            ('uv', c_real * 2),
            ('tag', c_int),
            ('type', c_int)
            ]


class TetGenIO(Structure):

    GetVertexParamOnEdge = CFUNCTYPE(c_real, c_int, c_int)
    GetSteinerOnEdge = CFUNCTYPE(c_void, c_int, c_real, p_real)
    GetVertexParamOnFace = CFUNCTYPE(c_void, c_void_p, c_int, c_int, p_real)
    GetEdgeSteinerParamOnFace = CFUNCTYPE(c_void, c_void_p, c_int, c_real, c_int, p_real)
    GetSteinerOnFace = CFUNCTYPE(c_void, c_void_p, c_int, p_real, p_real)

    TetSizeFunc = CFUNCTYPE(c_bool, p_real, p_real, p_real, p_real, p_real, c_real)

    def __init__(self):
        self.firstnumber = 0
        self.mesh_dim = 3;
        self.useindex = 0;
        self.numberofpoints = 0
        self.numberofpointattributes = 0
        self.numberofpointmtrs = 0
        
        self.numberoftetrahedra = 0
        self.numberofcorners = 0
        self.numberoftetrahedronattributes = 0

        self.numberoffacets = 0
        self.numberofholes = 0

        self.numberofregions = 0
        self.numberoffacetconstraints = 0
        self.numberofsegmentconstraints = 0

        self.numberoftrifaces = 0
        self.numberofedges = 0
        self.numberofvpoints = 0
        self.numberofvedges = 0
        self.numberofvfacets = 0
        self.numberofvcells = 0

        self.goemhandle = None
        self.getvertexparamonedge = self.GetVertexParamOnEdge()
        self.getsteineronedge = self.GetSteinerOnEdge()
        self.getvertexparamonface = self.GetVertexParamOnFace()
        self.getedgesteinerparamonface = self.GetEdgeSteinerParamOnFace()
        self.getsteineronface = self.GetSteinerOnFace()
        self.tetunsitable = self.TetSizeFunc()

    _fields_ = [
            ('firstnumber', c_int),
            ('mesh_dim', c_int),
            ('useindex', c_int),

            ('pointlist', p_real),
            ('pointattributelist', p_real),
            ('pointmtrlist', p_real),
            ('pointmarkerlist', p_int),
            ('pointparamlist',POINTER(PointParam)),
            ('numberofpoints',c_int),
            ('numberofpointattributes',c_int),
            ('numberofpointmtrs',c_int),
            
            ('tetrahedronlist', p_int),
            ('tetrahedronattributelist', p_real),
            ('tetrahedronvolumelist', p_real),
            ('neighborlist', p_int),
            ('numberoftetrahedra', c_int),
            ('numberofcorners', c_int),
            ('numberoftetrahedronattributes',c_int),

            ('facetlist', POINTER(Facet)),
            ('facetmarkerlist', p_int),
            ('numberoffacets', c_int),

            ('holelist', p_real),
            ('numberofholes', c_int),

            ('regionlist', p_real),
            ('numberofregions', c_int),

            ('facetconstraintlist', p_real),
            ('numberoffacetconstraints',c_int),

            ('segmentconstraintlist', p_real),
            ('numberofsegmentconstraints', c_int),

            ('trifacelist', p_int),
            ('trifacemarkerlist', p_int),
            ('o2facelist', p_int),
            ('adjtetlist', p_int),
            ('numberoftrifaces', c_int),

            ('edgelist', p_int),
            ('edgemarkerlist', p_int),
            ('o2edgelist', p_int),
            ('edgeadjtetlist', p_int),
            ('numberofedges', c_int),
            
            ('vpointlist', p_real),
            ('vedgelist', POINTER(VoroEdge)),
            ('vfacetlist', POINTER(VoroFacet)),
            ('vcellist', POINTER(p_int)),
            ('numberofvpoints', c_int),
            ('numberofvedges', c_int),
            ('numberofvfacets', c_int),
            ('numberofvcells', c_int),

            ('geomhandle', c_void_p),
            ('getvertexparamonedge', GetVertexParamOnEdge),
            ('getsteineronedge', GetSteinerOnEdge),
            ('getvertexparamonface', GetVertexParamOnFace),
            ('getedgesteinerparamonface', GetEdgeSteinerParamOnFace),
            ('getsteineronface', GetSteinerOnFace),
            ('tetunsuitable', TetSizeFunc),


            ]
    
    def get_points(self):
        if self.pointlist:
            return npc.as_array(self.pointlist,shape=(self.numberofpoints,self.mesh_dim))
        else:
            return np.empty((0,self.mesh_dim))

    def set_points(self,points):
        self.numberofpoints = int(points.size / self.mesh_dim)
        if isinstance(points, np.ndarray):
            self.pointlist = npc.as_ctypes(points.reshape(points.size))
        else:
            self.pointlist = (c_real * len(points))(*points)
    points = property(get_points, set_points)

    def get_facets(self):
        ArrayType = (Facet * self.numberoffacets)
        return ArrayType.from_address(addressof(self.facetlist))

    def set_facets(self,facets):
        self.numberoffacets = len(facets)
        self.facetlist = (Facet * len(facets))(*facets)

    facets = property(get_facets, set_facets)

    def get_trifaces(self):
        if self.trifacelist:
            return npc.as_array(self.trifacelist,shape=(self.numberoftrifaces,3))
        else:
            return np.empty((0,3))

    trifaces = property(get_trifaces)
        

    def get_tetrahedra(self):
        if self.tetrahedronlist:
            return npc.as_array(self.tetrahedronlist,shape=(self.numberoftetrahedra,self.numberofcorners))
        else:
            return np.empty((0,self.numberofcorners))

    def set_tetrahedra(self, th):
        self.numberoftetrahedra = len(th) / self.numberofcorners
        if isinstance(th, np.ndarray):
            self.tetrahedronlist = npc.as_ctypes(th)
        else:
            self.tetrahedronlist = (c_int * len(th))(*th)

    tetrahedra = property(get_tetrahedra, set_tetrahedra)

    def get_facetmarkers(self):
        if self.facetmarkrlist :
            return npc.as_array(self.facetmarkrlist,shape=self.numberoffacets)
        else:
            return np.empty(0)

    def set_facetmarkers(self,fm):
        assert(len(fm) == self.numberoffacets)
        if isinstance(fm, np.ndarray):
            self.facetmarkrlist = npc.as_ctypes(fm)
        else:
            self.facetmarkrlist = (c_int * self.numberoffacets)(*fm)
    facetmarkers = property(get_facetmarkers, set_facetmarkers)




from os import path
from sys import platform
if platform == 'win32':
    library_file = 'tetgen.dll'
elif platform == 'unix':
    library_file = 'libtetgen.so'
elif platform == 'darwin':
    library_file = 'libtetgen.dylib'
else:
    raise RuntimeError("Platform not supported");

PATH = path.dirname(__file__);
libtetgen = cdll.LoadLibrary(path.join(PATH, library_file))

_tetrahedralize = libtetgen.tetrahedralize
_tetrahedralize.restype = c_int
_tetrahedralize.argtypes = [ c_char_p, POINTER(TetGenIO), POINTER(TetGenIO), POINTER(TetGenIO), POINTER(TetGenIO), POINTER(c_char_p) ]

class TetGenError(RuntimeError):
    def __init__(self, code, message):
        RuntimeError.__init__(self, "TetGen Error", code, message)
        self.errorCode = code
        self.errorMessage = message

def tetrahedralize(switches, input):
    output = TetGenIO()
    errorMessage = c_char_p()
    errorCode = _tetrahedralize(switches.encode('ascii'), byref(input), byref(output), None, None, byref(errorMessage))
    if errorCode == 0:
        return output
    else:
        raise TetGenError(errorCode, (errorMessage.value).decode('ascii'))


