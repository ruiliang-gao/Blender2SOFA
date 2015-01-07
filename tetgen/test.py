from tetgen import *

a = TetGenIO()

a.firstnumber = 1
a.mesh_dim = 3
a.points = np.array([0, 0, 0,
	2, 0, 0,
	2, 2, 0,
	0, 2, 0,
	0, 0, 12,
	2, 0, 12,
	2, 2, 12,
	0, 2, 12], dtype = np.float)
	


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
    v = vertexlists[i];
    p = Polygon(v)
    f = Facet([ p ])
    print(f.polygons)
    facets.append(f)


a.facets = facets
a.facetmarkers = np.array([-1,-2,0,0,0,0],dtype = np.int)

b = TetGenIO();
tetrahedralize("pq1.414a0.1", pointer(a), pointer(b), None, None)

tt = b.tetrahedra
print(b.numberoftetrahedra)
print(b.numberofcorners)
for i in range(0,b.numberoftetrahedra):
    print tt[i],
    print "\n"

tt = b.points
print(b.numberoftetrahedra)
print(b.numberofcorners)
for i in range(0,b.numberofpoints):
    print tt[i],
    print "\n"
