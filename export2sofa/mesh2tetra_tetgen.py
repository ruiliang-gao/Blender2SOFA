from .tetgen import *
import numpy as N

def convert(m):
    a = TetGenIO()
    a.firstnumber = 0
    a.mesh_dim = 3
    position =  N.empty((len(m.vertices),3))
    for i, v in enumerate(m.vertices):
        position[i][0] = v.co[0]
        position[i][1] = v.co[1]
        position[i][2] = v.co[2]
    a.points = position
    a.facets = [ Facet([ Polygon(f.vertices) ]) for f in m.polygons ]
    
    b = TetGenIO()
    # Tetrahedralize the PLC. Switches are chosen to read a PLC (p),
    #   do quality mesh generation (q) with a specified quality bound
    #   (1.414), and apply a maximum volume constraint (a0.1).
    tetrahedralize(b"p", pointer(a), pointer(b), None, None)
    return ( b.points, b.tetrahedra )
