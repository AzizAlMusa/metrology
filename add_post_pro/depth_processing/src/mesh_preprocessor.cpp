#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <array>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/IO/STL_reader.h>


// #include <CGAL/IO/Geomview_stream.h>
// #include <CGAL/IO/Polyhedron_geomview_ostream.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <CGAL/Polygon_mesh_processing/remesh.h>


#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/distance.h>  // for Poisson disk sampling
#include <CGAL/point_generators_3.h>

#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>



typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

typedef K::FT FT;
typedef CGAL::Polyhedron_3<K> Polyhedron;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mesh_preprocessor");
    ros::NodeHandle nh;

    std::string package_path = ros::package::getPath("depth_processing");
    std::string input_filename = package_path + "/saved_models/cube.stl";

    // Create a surface mesh
    Surface_mesh mesh;

    // Open and read the STL file
    std::ifstream input_file(input_filename);
    if (!input_file)
    {
        ROS_ERROR("Error: Unable to open STL file %s", input_filename.c_str());
        return 1;
    }

    // Declare vectors to store points and facets
    std::vector<std::array<double, 3>> points;
    std::vector<std::array<int, 3>> facets;

    // Populate the vectors by reading the STL file
    if (!CGAL::read_STL(input_file, points, facets))
    {
        ROS_ERROR("Error reading STL file %s", input_filename.c_str());
        return 1;
    }

    // Populate the surface mesh using the extracted points and facets
    std::vector<Surface_mesh::Vertex_index> vertices;

    for (const auto& point : points)
    {
        // Add each point as a vertex to the mesh and store the vertex index
        Surface_mesh::Vertex_index v = mesh.add_vertex(Point_3(point[0], point[1], point[2]));
        vertices.push_back(v);
    }

    // Add facets (triangles) to the mesh using vertex indices
    for (const auto& facet : facets)
    {
        // Add each facet as a triangle to the mesh
        mesh.add_face(vertices[facet[0]], vertices[facet[1]], vertices[facet[2]]);
    }

    
    /////////// Isotropic Remeshing  /////////////////////////////
    // Define remeshing parameters
    double initial_edge_length = 0.0001;  // Initial edge length
    double target_vertex_count = 50000; // Desired vertex count
    double current_vertex_count = mesh.number_of_vertices();

    // Continue remeshing until the desired vertex count is achieved
    // while (current_vertex_count < target_vertex_count)
    // {
    // Perform isotropic remeshing to refine the mesh
    CGAL::Polygon_mesh_processing::isotropic_remeshing(
        faces(mesh), initial_edge_length, mesh
    );
    

        // Update the current vertex count
    current_vertex_count = mesh.number_of_vertices();
    ROS_INFO("Number of Vertices After Remeshing: %d", mesh.number_of_vertices());
    initial_edge_length *= 0.9;
    // }


    // //////// Poisson Disk Sampling /////////////////////
    // double sampling_radius = 0.0001;  // You may need to adjust this
    // std::vector<Point_3> sampled_points;
    // CGAL::Random_points_on_triangle_mesh_3<Surface_mesh, CGAL::Creator_uniform_3<double, Point_3>> g(mesh);
    // CGAL::copy_n_unique(g, target_vertex_count, std::back_inserter(sampled_points));
    
    // // Advancing front surface reconstruction
    // Surface_mesh reconstructed_mesh;
    // CGAL::advancing_front_surface_reconstruction(sampled_points.begin(),
    //                                              sampled_points.end(),
    //                                              reconstructed_mesh);

    // // Replace your mesh with the reconstructed one
    // mesh = reconstructed_mesh;


    // Convert surface mesh to polyhedron for use in geomview 
    Polyhedron P;
    CGAL::copy_face_graph(mesh, P);


    // Log the number of vertices after remeshing
    ROS_INFO("Number of Vertices After Remeshing: %d", mesh.number_of_vertices());


    // Save the the remeshed surface mesh as an STL in ASCII format
    std::string output_filename = package_path + "/saved_models/remeshed_cube_cgal.stl";
    std::ofstream output_file(output_filename);
    if (!output_file)
    {
        ROS_ERROR("Error: Unable to open output STL file %s", output_filename.c_str());
        return 1;
    }

    // Write the ASCII STL header
    output_file << "solid remeshed_cube" << std::endl;

    // Iterate over faces and write them to the STL file
    for (Surface_mesh::Face_index face : faces(mesh))
    {
        Surface_mesh::Halfedge_index he = halfedge(face, mesh);
        output_file << "  facet normal 0 0 0" << std::endl;
        output_file << "    outer loop" << std::endl;
        for (int i = 0; i < 3; ++i)
        {
            Surface_mesh::Vertex_index vertex = target(he, mesh);
            Point_3 point = mesh.point(vertex);
            output_file << "      vertex " << point.x() << " " << point.y() << " " << point.z() << std::endl;
            he = next(he, mesh);
        }
        output_file << "    endloop" << std::endl;
        output_file << "  endfacet" << std::endl;
    }

    // Write the ASCII STL footer
    output_file << "endsolid remeshed_cube" << std::endl;

    output_file.close();

    ROS_INFO("Saved remeshed model to %s", output_filename.c_str());


    return 0;
}

