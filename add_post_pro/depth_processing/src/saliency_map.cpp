#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h> // Include for vtk_lib_io.h
#include <pcl/surface/vtk_smoothing/vtk_utils.h> // Include for vtk_smoothing/vtk_utils.h
#include <pcl/filters/random_sample.h>
#include <vtkSmartPointer.h>
#include <vtkSTLReader.h>
#include <vtkPolyData.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <pcl/filters/uniform_sampling.h>

#include <vtkPolyDataPointSampler.h>
#include <vtkCellLocator.h>
#include <vtkTriangle.h>
#include <vtkMassProperties.h>
#include <vtkCellArray.h>

class SaliencyMapGenerator {
public:
    SaliencyMapGenerator(const std::string& stl_file_path, int num_points_to_sample)
        : stl_file_path_(stl_file_path), num_points_to_sample_(num_points_to_sample) {}

    void GenerateSaliencyMap() {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = LoadAndSampleSTLModel();
        if (!cloud) {
            std::cerr << "Failed to load and sample the STL model." << std::endl;
            return;
        }
        // DownsamplePointCloud(cloud);
        CalculateSaliencyScores(cloud);

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Saliency Visualization"));
        viewer->addPointCloud(cloud, "Saliency Cloud");
        viewer->spin();
    }

private:
    std::string stl_file_path_;
    int num_points_to_sample_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr LoadAndSampleSTLModel() {
        // Initialize VTK STL reader
        vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();

        // Make sure the file path is correct
        if (stl_file_path_.empty()) {
            return nullptr;
        }

        reader->SetFileName(stl_file_path_.c_str());
        reader->Update();

        vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();

        // Sample points from the polydata using the new function
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = sampleModel(polydata, num_points_to_sample_);


        return cloud;

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampleModel(const vtkSmartPointer<vtkPolyData>& polydata, int numPointsToSample)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // Build a cell locator for the polydata
        vtkSmartPointer<vtkCellLocator> cellLocator = vtkSmartPointer<vtkCellLocator>::New();
        cellLocator->SetDataSet(polydata);
        cellLocator->BuildLocator();

        // Number of points to sample
        double totalArea = 0.0;
        std::vector<double> triangleAreas;
        vtkCellArray* cells = polydata->GetPolys();

        // Iterate through the triangles in the cell array
        cells->InitTraversal();
        vtkIdType npts;
        vtkIdType* pts;
        
        while (cells->GetNextCell(npts, pts))
        {
            if (npts == 3)
            {
                double p0[3], p1[3], p2[3];
                polydata->GetPoint(pts[0], p0);
                polydata->GetPoint(pts[1], p1);
                polydata->GetPoint(pts[2], p2);

                double v1[3], v2[3], cross[3];
                for (int i = 0; i < 3; i++)
                {
                    v1[i] = p1[i] - p0[i];
                    v2[i] = p2[i] - p0[i];
                }
                
                cross[0] = v1[1] * v2[2] - v1[2] * v2[1];
                cross[1] = v1[2] * v2[0] - v1[0] * v2[2];
                cross[2] = v1[0] * v2[1] - v1[1] * v2[0];

                double triangleArea = 0.5 * vtkMath::Norm(cross);
                totalArea += triangleArea;
                triangleAreas.push_back(triangleArea);
            }
        }

        int numTriangles = polydata->GetNumberOfCells();

        for (vtkIdType cellId = 0; cellId < polydata->GetNumberOfCells(); cellId++) {
            double cellArea = polydata->GetCell(cellId)->GetLength2();
            int pointsInThisCell = static_cast<int>(numPointsToSample * (triangleAreas[cellId] / totalArea));

            for (int j = 0; j < pointsInThisCell; j++) {
                double pcoords[3];
                double weights[3];
                int subId;
                double point[3];

                double r1 = static_cast<double>(rand()) / RAND_MAX;
                double r2 = static_cast<double>(rand()) / RAND_MAX * (1.0 - r1);
                double r3 = 1.0 - r1 - r2;

                pcoords[0] = r1;
                pcoords[1] = r2;
                pcoords[2] = r3;

                polydata->GetCell(cellId)->EvaluateLocation(subId, pcoords, point, weights);

                pcl::PointXYZRGB pclPoint;
                pclPoint.x = point[0];
                pclPoint.y = point[1];
                pclPoint.z = point[2];
                pclPoint.r = 255;
                pclPoint.g = 255;
                pclPoint.b = 255;

                cloud->points.push_back(pclPoint);
            }
        }

        return cloud;
    }

 
   
    float CalculateNormalVariance(size_t index, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree, float search_radius) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        pcl::Normal queryNormal = normals->points[index];

        if (tree->radiusSearch(index, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            float sum = 0;
            float sum_squared = 0;
            for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                pcl::Normal n = normals->points[pointIdxRadiusSearch[i]];
                float dot = n.normal_x * queryNormal.normal_x + n.normal_y * queryNormal.normal_y + n.normal_z * queryNormal.normal_z;
                sum += dot;
                sum_squared += dot * dot;
            }
            float mean = sum / pointIdxRadiusSearch.size();
            float variance = (sum_squared / pointIdxRadiusSearch.size()) - (mean * mean);
            return variance;
        }
        return 0;
    }

    void PrintPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int numPointsToPrint) {
        for (int i = 0; i < numPointsToPrint && i < cloud->points.size(); ++i) {
            pcl::PointXYZRGB& point = cloud->points[i];
            std::cout << "Point " << i << ": X=" << point.x << ", Y=" << point.y << ", Z=" << point.z << std::endl;
        }
    }

    void PrintNormals(pcl::PointCloud<pcl::Normal>::Ptr normals, int numNormalsToPrint) {
        for (int i = 0; i < numNormalsToPrint && i < normals->points.size(); ++i) {
            pcl::Normal& normal = normals->points[i];
            std::cout << "Normal " << i << ": Nx=" << normal.normal_x << ", Ny=" << normal.normal_y << ", Nz=" << normal.normal_z << std::endl;
        }
    }
    void CalculateSaliencyScores(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        std::vector<float> saliencyScores;  // Vector to store saliency scores
        float maxSaliency = 0.0f;  // Variable to store the maximum saliency value
        // Step 1: Compute Normals
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(0.03);  // 3cm, adjust as needed
        ne.compute(*normals);

        // Step 2: Calculate Variance of Normals and Update Saliency Scores
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            pcl::PointXYZRGB& point = cloud->points[i];
            float variance = CalculateNormalVariance(i, normals, tree, 0.03);  // Using 3cm as the search radius
            float saliency = variance;  // In this example, we directly use variance as saliency, adjust as needed
              // Store the variance as a saliency score
            saliencyScores.push_back(variance);
            // Step 3: Assign Color based on Saliency

        }
        // Find the maximum saliency score
        if (!saliencyScores.empty()) {
            maxSaliency = *std::max_element(saliencyScores.begin(), saliencyScores.end());
        }
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            pcl::PointXYZRGB& point = cloud->points[i];
   
            // Step 3: Assign Color based on Saliency
            AssignColorToPointCloudPoint(point, saliencyScores[i], maxSaliency);
        }
    }
    void AssignColorToPointCloudPoint(pcl::PointXYZRGB& point, float saliency, float maxSaliency) {
        float colorFactor = saliency / maxSaliency;
         // Apply a logarithmic transformation to the colorFactor
         float logColorFactor = std::log(saliency + 1) / std::log(maxSaliency + 1);

        uint8_t r = static_cast<uint8_t>(colorFactor * 255.0);
        uint8_t g = 0;
        uint8_t b = static_cast<uint8_t>((1.0 - colorFactor) * 255.0);
        uint32_t rgb = static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b);
        point.rgb = *reinterpret_cast<float*>(&rgb);
    }
};




int main(int argc, char** argv) {
    ros::init(argc, argv, "saliency_map_generator_node");
    ros::NodeHandle nh;

    std::string package_path = ros::package::getPath("depth_processing");
    std::string stl_file_path = package_path + "/saved_models/bell_scaled.stl";
    int num_points_to_sample = 50000;

    SaliencyMapGenerator saliency_generator(stl_file_path, num_points_to_sample);
    saliency_generator.GenerateSaliencyMap();

    return 0;
}