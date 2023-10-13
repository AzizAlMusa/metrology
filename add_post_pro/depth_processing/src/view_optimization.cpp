#include <ros/ros.h>
#include <ViewPlanning/view_plan.h>
#include <chrono>
#include <random>
#include <cmath>
#include <iostream>
#include <fstream>


// Define the basicViewPlanning method
void basicViewPlanning(const std::string& file_path) {
    // Create an instance of the ViewPlanning class
    ViewPlanning vp;

    std::chrono::high_resolution_clock::time_point start_time, end_time;

    // Load the STL model
    vp.loadModel(file_path);
    vp.RandomizeTriangleColors();

    // Create three viewpoints with random positions and orientations
    vp.createViewpoint(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 1.0, 45.0);
    vp.createViewpoint(0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.1, 1.0, 45.0);
    // vp.createViewpoint(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.1, 1.0, 45.0);
    // vp.createViewpoint(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.1, 1.0, 45.0);
    // vp.createViewpoint(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.1, 1.0, 45.0);
    // vp.createViewpoint(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.1, 1.0, 45.0);
    // vp.createViewpoint(0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.1, 1.0, 45.0);
    // vp.createViewpoint(-0.5, -0.5, -0.5, 0.0, 0.0, 0.0, 0.1, 1.0, 45.0);

    auto viewpoints = vp.get_viewpoints();
    std::vector<std::vector<int>> visibility_mat;
    vtkSmartPointer<vtkPoints> aggregatedWorldPoints = vtkSmartPointer<vtkPoints>::New();
    int index = 0;


    
    double totalElapsedTime = 0.0;
    
    // Loop through each viewpoint
    for (auto& viewpoint : viewpoints) {
        std::cout << "Iteration: " << index << std::endl;
       
        // Show frustum of camera
        // vp.addCameraFrustumToRenderer(viewpoint);
       
        vtkSmartPointer<vtkFloatArray> depthData = vp.capture_depth_data(viewpoint);

       

       
        // Convert depth data to 3D world points and compute visibility
        vtkSmartPointer<vtkPoints> worldPoints = vp.convertDepthTo3D(depthData, viewpoint);
        
           // Record the start time
        start_time = std::chrono::high_resolution_clock::now();
        // Calculate depth data
        std::vector<int> visibleTriangles = vp.compute_visibility(viewpoint, worldPoints, index);
           //  // Record the end time
        end_time = std::chrono::high_resolution_clock::now();
        // // Calculate and print the elapsed time
        std::chrono::duration<double> elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
        std::cout << "Elapsed time: " << elapsed_time.count() << " seconds" << std::endl;

        ++index;

      
        // vp.addPointCloudToRenderer(worldPoints);
        totalElapsedTime += elapsed_time.count();
        vp.showViewpointSolution(viewpoint, 0.1);
    }

    double averageTime = totalElapsedTime / viewpoints.size();
    std::cout << "Average time: " << averageTime << " seconds" << std::endl;
        




    // Compute the visibility matrix
    visibility_mat = vp.get_visibility_matrix();

    // Color visible triangles in the model
    // vp.RandomizeTriangleColors();
    vp.ColorVisibleTriangles(visibility_mat);
    // vp.ColorOverlap();


    float score = vp.computeCoverageScore();
    std::cout << score << std::endl;
    vp.print_visibility_matrix();
    vp.visualizeScene();
}

// Define the greedySearch method
void greedySearch(const std::string& file_path, int maxIterations, float initialThreshold, int maxConsecutiveIterations, float minRadius, float maxRadius) {
    
    // Create an instance of the ViewPlanning class
    ViewPlanning vp;

    std::chrono::high_resolution_clock::time_point start_time, end_time;

    // Load the STL model
    vp.loadModel(file_path);

    // Initialize variables for view planning
    float threshold = initialThreshold;
    int consecutiveIterations = 0;
    float epsilon = 0.0001;

    // Define random number generator for generating viewpoint positions sampled from a sphere surface
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> radiusDist(minRadius, maxRadius);
    std::uniform_real_distribution<float> thetaDist(0.0f, 2.0f * M_PI);
    std::uniform_real_distribution<float> phiDist(-M_PI / 2.0f, M_PI / 2.0f);

    int loop_index = 0;
    int viewpoint_index = 0;
    float previousScore = 0.0;
    bool notComplete = false;

    std::vector<double> allCoverageScores, importantCoverageScores;

    while (!notComplete) {
        std::cout << "Starting at iteration: " << loop_index << std::endl;
        if (loop_index >= maxIterations) {
            break;
        }

        // Generate random viewpoint within specified radius range
        float radius = radiusDist(gen);
        float theta = thetaDist(gen);
        float phi = phiDist(gen);

        // Convert spherical coordinates to Cartesian coordinates
        float x = radius * cos(phi) * cos(theta);
        float y = radius * cos(phi) * sin(theta);
        float z = radius * sin(phi);

        // Create the viewpoint
        vp.createViewpoint(x, y, z, 0.0, 0.0, 0.0, 0.1, 1.0, 45.0);
        auto viewpoint = vp.getLastViewpoint();

        // Capture depth data
        vtkSmartPointer<vtkFloatArray> depthData = vp.capture_depth_data(viewpoint);

        // Convert depth data to 3D world points and compute visibility
        vtkSmartPointer<vtkPoints> worldPoints = vp.convertDepthTo3D(depthData, viewpoint);
        vp.compute_visibility(viewpoint, worldPoints, viewpoint_index);

        // Compute the visibility score
        float currentScore = vp.computeCoverageScore();
        allCoverageScores.push_back(currentScore);
        importantCoverageScores.push_back(previousScore);
        std::cout << "Current Score: " << currentScore << std::endl;

        // Check if the score improved by the threshold
        if (((currentScore - previousScore) >= threshold) || (currentScore >= 1.0 - epsilon)) {
            // Show viewpoint solution
            vp.showViewpointSolution(viewpoint, 0.1);
            std::cout << "Found viewpoint" << std::endl;
            previousScore = currentScore;
            notComplete = currentScore >= (1.0 - epsilon);
            consecutiveIterations = 0;
            threshold = initialThreshold;
        } else {
       
            consecutiveIterations++;
            if (consecutiveIterations >= maxConsecutiveIterations) {
                // Reduce the threshold by half
                threshold /= 2.0;
                consecutiveIterations = 0;
            }
            // Remove the last added viewpoint as it did not improve coverage
            if (loop_index != 0) {
                --viewpoint_index;
                vp.removeLastViewpoint();
            }
        }

        ++loop_index;
        ++viewpoint_index;
    }
    importantCoverageScores.push_back(previousScore);

    // Save coverage scores to CSV
    std::ofstream csvFile("coverage_scores.csv");
    if (csvFile.is_open()) {
        csvFile << "Index,Data\n";
        for (int i = 0; i < allCoverageScores.size(); ++i) {
            csvFile << i << "," << allCoverageScores[i] << "\n";
        }
        csvFile.close();
        std::cout << "CSV file saved successfully." << std::endl;
    } else {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // Save important coverage scores to CSV
    std::ofstream csvFile2("important_scores.csv");
    if (csvFile2.is_open()) {
        csvFile2 << "Index,Data\n";
        for (int i = 0; i < importantCoverageScores.size(); ++i) {
            csvFile2 << i << "," << importantCoverageScores[i] << "\n";
        }
        csvFile2.close();
        std::cout << "CSV file saved successfully." << std::endl;
    } else {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // Final coverage score
    float finalScore = vp.computeCoverageScore();
    std::cout << "Final Coverage Score: " << finalScore << std::endl;
    std::cout << "Total viewpoints: " << vp.get_visibility_matrix().size() << std::endl;
    vp.ColorOverlap();
    vp.visualizeScene();
}

int main(int argc, char** argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "view_optimization_node");
    ros::NodeHandle nh;
    
    // STL file path
    std::string package_path = ros::package::getPath("depth_processing");
    std::string stl_file_path = package_path + "/saved_models/car_door_scaled.stl";

    // greedy search paramaters
    int maxIterations = 500;
    float initialThreshold = 0.4;
    int maxConsecutiveIterations = 4;
    float minRadius = 0.75f;
    float maxRadius = 1.0f;

    greedySearch(stl_file_path, maxIterations, initialThreshold, maxConsecutiveIterations, minRadius, maxRadius);
    // basicViewPlanning(stl_file_path);

    return 0;


}
