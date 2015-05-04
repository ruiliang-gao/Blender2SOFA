#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>

#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/refine_mesh_3.h>

#include <CGAL/Polyhedron_incremental_builder_3.h>

#include "CGALTetrahedralize.h"

// Domain
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::Polyhedral_mesh_domain_3<Polyhedron, K> Mesh_domain;

typedef CGAL::Mesh_triangulation_3<Mesh_domain>::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3;

// Criteria
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;

// To avoid verbose function and named parameters call
using namespace CGAL::parameters;


typedef Polyhedron::HalfedgeDS             HalfedgeDS;

// A modifier creating a triangle with the incremental builder.
template <class HDS>
class BuildTriangleMesh : public CGAL::Modifier_base<HDS> {
    typedef typename HDS::Vertex   Vertex;
    typedef typename Vertex::Point Point;
    struct TriangleMesh* _mesh;
public:
    BuildTriangleMesh(struct TriangleMesh* mesh): _mesh(mesh) {}
    void operator()(HDS& hds) {
        // Postcondition: hds is a valid polyhedral surface.
        CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
        B.begin_surface(_mesh->vertexCount, _mesh->triangleCount, _mesh->triangleCount*3/2);
        
        /* Copy all the vertices from the input mesh into the half-edge DS */
        for (int i = 0; i < _mesh->vertexCount; i++)
            B.add_vertex(Point(_mesh->vertex[i][0], _mesh->vertex[i][1], _mesh->vertex[i][2]));

        /* Copy all the triangle facets from input mesh into the half-edge DS */
        for (int i = 0; i < _mesh->triangleCount; i++)
            B.add_facet(_mesh->triangle[i], _mesh->triangle[i] + 3);

        B.end_surface();
    }
};



struct ExtractIndex : public boost::static_visitor<int> {
    int operator()(int i)const
    {
        return i;
    }
    int operator()(std::pair<int, int> p)const
    {
        return p.second;
    }
};

int tetrahedralize(struct TriangleMesh* inmesh, struct TetrahedralMesh* outmesh /* parameters */)
{
    /* Convert inmesh to a CGAL polyhedral mesh */
    Polyhedron P;
    BuildTriangleMesh<HalfedgeDS> meshBuilder(inmesh);
    P.delegate(meshBuilder);
    CGAL_assertion(P.is_pure_triangle() && P.is_closed() && !P.is_empty());

    /* Tetrahedralize the polyhedral mesh */
    // Create domain
    Mesh_domain domain(P);
    // Mesh criteria (no cell_size set)
    // Mesh criteria
    Mesh_criteria criteria(facet_angle = 25, facet_size = 0.15, facet_distance = 0.008,
        cell_radius_edge_ratio = 3);

    // Mesh generation
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, no_perturb(), no_exude());

    /* Convert CGAL polyhedral mesh into a tetrahedral outmesh*/
    auto t = c3t3.triangulation();
    std::map<C3t3::Vertex_handle, int> V;

    outmesh->tetrahedronCount = 0;
    outmesh->triangleCount = 0;
    outmesh->vertexCount = (int) t.number_of_vertices();
    outmesh->vertex = new double[outmesh->vertexCount][3];
    int j = 0;
    for (auto i = t.finite_vertices_begin(); i != t.finite_vertices_end(); i++, j++)
    {
        V[i] = j;
        for (int k = 0; k < 3; k++) outmesh->vertex[j][k] = i->point()[k];
    }


    outmesh->tetrahedronCount = (int)c3t3.number_of_cells_in_complex();
    outmesh->tetrahedron = new int[outmesh->tetrahedronCount][4];
    j = 0;
    for (auto i = c3t3.cells_in_complex_begin(); i != c3t3.cells_in_complex_end(); i++, j++)
        for (int k = 0; k < 4; k++) 
            outmesh->tetrahedron[j][k] = V[i->vertex(k)];


    outmesh->triangleCount = (int) c3t3.number_of_facets_in_complex();
    outmesh->triangle = new int[c3t3.number_of_facets_in_complex()][3];
    j = 0;
    for (auto i = c3t3.facets_in_complex_begin(); i != c3t3.facets_in_complex_end(); i++, j++)
        for (int k = 0, l = 0; k < 4; k++)
            if (k != i->second)
                outmesh->triangle[j][l++] = V[ i->first->vertex(k) ];

    return 0;
}