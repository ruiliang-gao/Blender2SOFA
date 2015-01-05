from ctypes import *

p_int = POINTER(c_int)
c_real = c_double
p_real = POINTER(c_real)

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
    GetEdgeSteinerParamOnFace(c_void, c_void_p, c_int, c_real, c_int, p_real)
    GetSteinerOnFace = CFUNCTYPE(c_void, c_void_p, c_int, p_real, p_real)

    TetSizeFunc = CFUNCTYPE(c_bool, p_real, p_real, p_real, p_real, p_real, c_real)

    _fields_ = [
            ('firstnumber', c_int),
            ('mesh_dim', c_int),
            ('useindex', c_int),

            ('pointlist', p_real),
            ('pointattributelist', p_real),
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
            ('numberoftetrahedronattributes',c_int)

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
