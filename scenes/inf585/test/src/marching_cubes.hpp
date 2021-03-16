#pragma once
#include "vcl/vcl.hpp"
#include <vector>
#include <algorithm>

// comparateur de pair<int,int>
class int_pair_comparator
{
public:
    bool operator()(const std::pair<int,int>& a,const std::pair<int,int>& b) const
    {
        std::pair<int,int> ca = a;
        std::pair<int,int> cb = b;
        if (a.first > a.second)
        {
            ca.first = a.second;
            ca.second = a.first;
        }
        if (b.first > b.second)
        {
            cb.first = b.second;
            cb.second = b.first;
        }

        if( (ca.first<cb.first) || (ca.first==cb.first && ca.second<cb.second))
            return true;
        return false;
    }
private:
};


/** \brief helper to apply marching cube on a volume data */
class marching_cubes
{

public:

    // ************************************************ //
    // ************************************************ //
    // SLICE
    // ************************************************ //
    // ************************************************ //

    std::vector<vcl::vec3> vert_positions;
    std::vector<int> number_of_triangles_per_vertex;
    std::vector<std::pair<int,int> > edges;

    /** \brief apply marching cube on data */
    vcl::mesh marching_cube(const vcl::grid_3D<float>& values, const int grid_size, const double& isovalue=0.0);

private:

    /** \brief add polygons corresponding to the current cell in the mesh */
    void create_polygon(const std::array<double,8>& cell_value, const int& type_of_cube, const std::array<int,3>& x, vcl::mesh* mesh, std::map<std::pair<int,int>, unsigned int, int_pair_comparator>* edge_to_vertex_index, const vcl::grid_3D<float>& potential, const int grid_size);

    /** /brief helper indexation for the marching cube */
    static int EdgeTable[256];
    /** /brief helper indexation for the marching cube */
    static int TriangulationTable[256][16];
};
