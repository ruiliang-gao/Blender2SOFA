from ctypes import *

p_int = POINTER(c_int)
c_real = c_double
p_real = POINTER(c_real)
c_void = None

class Polygon(Structure):
    _fields_ = [
            ('vertexlist', POINTER(c_int)),
            ('numberofvertices', c_int)
            ]

class Facet(Structure):
    _fields_ = [
            ('polygonlist', POINTER(Polygon)),
            ('numberofpolygons', c_int),
            ('holelist', p_real),
            ('numberofholes', c_int)
            ]

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



from os import path
from sys import platform
if platform == 'win32':
    library_file = 'tetgen.dll'
elif platform == 'unix':
    library_file = 'libtetgen.so'
else:
    raise RuntimeError("Platform not supported");

PATH = path.dirname(__file__);
libtetgen = cdll.LoadLibrary(path.join(PATH, library_file))

tetrahedralize = libtetgen.tetrahedralize
tetrahedralize.argtypes = [ c_char_p, POINTER(TetGenIO), POINTER(TetGenIO), POINTER(TetGenIO), POINTER(TetGenIO) ]

