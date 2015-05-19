from .cgaltetrahedralize import *
import numpy as N

def convert(m):
    position =  N.empty((len(m.vertices),3),order='C')
    for i, v in enumerate(m.vertices):
        position[i][0] = v.co[0]
        position[i][1] = v.co[1]
        position[i][2] = v.co[2]
    triangles = N.empty((len(m.polygons),3),dtype=int,order='C')
    for i, f in enumerate(m.polygons):
        triangles[i][0] = f.vertices[0]
        triangles[i][1] = f.vertices[1]
        triangles[i][2] = f.vertices[2]

    inmesh = TriangleMesh()
    inmesh.vertices = position
    inmesh.triangles = triangles

    outmesh = TetrahedralMesh()
    tetrahedralize(pointer(inmesh),pointer(outmesh))
    return ( outmesh.vertices, outmesh.tetrahedra )
