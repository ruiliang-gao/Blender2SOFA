#pragma once

struct TriangleMesh {
    double(*vertex)[3]; int vertexCount;
    int(*triangle)[3]; int triangleCount;
};

struct TetrahedralMesh {
    double(*vertex)[3]; int vertexCount;
    int(*triangle)[3]; int triangleCount;
    int(*tetrahedron)[4]; int tetrahedronCount;
};

struct TetrahedralizeParameters {
    double cell_size;
    double facet_angle;
    double facet_size;
    double facet_distance;
    double cell_radius_edge_ratio;
};

#ifdef _MSC_VER
#define DLLAPI __declspec(dllexport)
#endif


#ifdef __cplusplus
extern "C" {
#endif

    DLLAPI int tetrahedralize(struct TriangleMesh* inmesh, struct TetrahedralMesh* outmesh, struct TetrahedralizeParameters* parameters);

#ifdef __cplusplus
}
#endif