from tetgen import *

a = TetGenIO()

a.firstnumber = 1
a.mesh_dim = 3
a.numberofpoints = 8
a.pointlist = (c_real * 24)(
	0, 0, 0,
	2, 0, 0,
	2, 2, 0,
	0, 2, 0,
	0, 0, 12,
	2, 0, 12,
	2, 2, 12,
	0, 2, 12,
	)


# VertexLists
vertexlists = [
	[1,2,3,4],
	[5,6,7,8],
	[1,5,6,2],
	[2,6,7,3],
	[3,7,8,4],
	[4,8,5,1]
   ]

facets = []
for i in range(0,6):
    f = Facet()
    f.numberofholes = 0
    f.holelist = None
    f.numberofpolygons = 1
    p = Polygon()
    v = vertexlists[i];
    p.numberofvertices = len(v)
    p.vertexlist = (c_int * len(v))(*v)
    f.polygonlist = pointer(p)
    facets.append(f)


a.numberoffacets = len(facets);
a.facetlist = (Facet * a.numberoffacets)(*facets)
a.facetmarkrlist = (c_int * a.numberoffacets)(-1,-2,0,0,0,0)

b = TetGenIO();
tetrahedralize("pq1.414a0.1", pointer(a), pointer(b), None, None)


