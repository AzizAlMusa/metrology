#include <ros/ros.h>
#include <ros/package.h>
#include <random>
#include <cstdlib> // For std::rand() and std::srand()
#include <ctime>   // For std::time()
#include <unordered_set>
#include <iomanip> // Include the <iomanip> library for formatting

// These are VTK (Visualization Toolkit) headers for 3D visualization
#include <vtkCamera.h>
#include <vtkSmartPointer.h>
#include <vtkSTLReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkLine.h>

#include <vtkPlanes.h>
#include <vtkFrustumSource.h>
#include <vtkOutlineFilter.h>

#include <vtkProperty.h>
#include <vtkImageCast.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>

// These are headers related to point cloud processing
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// More VTK headers for visualization
#include <vtkImageMapper.h>
#include <vtkImageActor.h>

#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPropPicker.h>

#include <vtkLineSource.h>
#include <vtkUnsignedCharArray.h>
#include <vtkCellData.h>
#include <vtkTriangle.h>
#include <vtkLookupTable.h>
#include <vtkPolyDataNormals.h>
#include <vtkGlyph3D.h>
#include <vtkArrowSource.h>
#include <vtkDoubleArray.h>
#include <vtkKdTreePointLocator.h>


#include <ViewPlanning/view_plan.h>



    // Constructor
    ViewPlanning::ViewPlanning() {
        // Initialization
        
        // Initialize VTK objects
        stlReader = vtkSmartPointer<vtkSTLReader>::New();
        mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        actor = vtkSmartPointer<vtkActor>::New();
        renderer = vtkSmartPointer<vtkRenderer>::New();
        renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

        // Set renderer background color to black
        renderer->SetBackground(0.0, 0.0, 0.0);

        // Create a camera for the current viewpoint
        current_viewpoint = vtkSmartPointer<vtkCamera>::New();
        current_viewpoint->SetPosition(2, 2, 2);
        current_viewpoint->SetFocalPoint(0, 0, 0);

        // Set default width and height for rendering
        width = 1280;
        height = 720;

        // Seed the random number generator with the current time
        std::srand(static_cast<unsigned int>(std::time(nullptr)));

        totalModelArea = 0;

        // Create a new renderer and render window for off-screen rendering
        // offscreenMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        // offscreenActor = vtkSmartPointer<vtkActor>::New();
        // offscreenRenderer = vtkSmartPointer<vtkRenderer>::New();
        // offscreenWindow = vtkSmartPointer<vtkRenderWindow>::New();
        // offscreenWindow->SetOffScreenRendering(1);
        // offscreenWindow->AddRenderer(offscreenRenderer);


    }


       void  ViewPlanning::loadModel(const std::string& stlFilePath) {
        // Load the STL file
        stlReader->SetFileName(stlFilePath.c_str());

        // Explicitly update the reader to load the file
        stlReader->Update();

        // Get the output polydata
        polydata = stlReader->GetOutput();

        // Compute Normals
        vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
        normalGenerator->SetInputData(polydata);
        normalGenerator->ComputeCellNormalsOn();
        normalGenerator->Update();
        polydata = normalGenerator->GetOutput();

        // Compute centroids and create new vtkPolyData for them
        vtkSmartPointer<vtkPoints> centroids = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkFloatArray> centroidNormals = vtkSmartPointer<vtkFloatArray>::New();
        centroidNormals->SetNumberOfComponents(3);

        // Calculate centroids and centroid normals
        vtkFloatArray* cellNormals = vtkFloatArray::SafeDownCast(polydata->GetCellData()->GetNormals());

        for (vtkIdType i = 0; i < polydata->GetNumberOfCells(); ++i) {
            vtkCell* cell = polydata->GetCell(i);

            double p0[3], p1[3], p2[3];
            cell->GetPoints()->GetPoint(0, p0);
            cell->GetPoints()->GetPoint(1, p1);
            cell->GetPoints()->GetPoint(2, p2);

            double centroid[3] = {(p0[0] + p1[0] + p2[0]) / 3,
                                (p0[1] + p1[1] + p2[1]) / 3,
                                (p0[2] + p1[2] + p2[2]) / 3};

            centroids->InsertNextPoint(centroid);

            double normal[3];
            cellNormals->GetTuple(i, normal);
            centroidNormals->InsertNextTuple(normal);
        }

        // Create a new vtkPolyData for centroids and associate normals with points
        centroidPolyData = vtkSmartPointer<vtkPolyData>::New();
        centroidPolyData->SetPoints(centroids);
        centroidPolyData->GetPointData()->SetNormals(centroidNormals);
         // Extract points from the pre-calculated centroidPolyData
        vtkSmartPointer<vtkPoints> centroidPoints = centroidPolyData->GetPoints();


        // Connect the original polydata to the mapper and actor
        mapper->SetInputData(polydata);
        actor->SetMapper(mapper);

        this->computeTriangleAreas();
        // // Do the same for the offscreen space
        // vtkSmartPointer<vtkPolyData> offscreenPolydata = vtkSmartPointer<vtkPolyData>::New();
        // offscreenPolydata->DeepCopy(polydata);
        // offscreenMapper->SetInputData(offscreenPolydata);
        // offscreenActor->SetMapper(offscreenMapper);

        

    }

    void ViewPlanning::computeTriangleAreas(){
        // Access the cells in the polydata
        vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

        // Create an array to store the cell areas
        vtkSmartPointer<vtkDoubleArray> cellAreas = vtkSmartPointer<vtkDoubleArray>::New();
        cellAreas->SetName("Cell_Areas");
        cellAreas->SetNumberOfComponents(1);

        // Iterate through the cells
        cells->InitTraversal();
        vtkIdType npts;
        vtkIdType* pts;
        double p0[3], p1[3], p2[3]; // Points for the triangle vertices

        int index = 0;
        while (cells->GetNextCell(npts, pts)) {
            // Assuming the cell is a triangle, get its vertices
            polydata->GetPoint(pts[0], p0);
            polydata->GetPoint(pts[1], p1);
            polydata->GetPoint(pts[2], p2);

            // Calculate the area of the triangle formed by these vertices
            double area = 0.5 * vtkTriangle::TriangleArea(p0, p1, p2);

            // Add the area to the cellAreas array
            cellAreas->InsertNextValue(area);

            totalModelArea += area;
        }

        // Add the cellAreas array to the polydata's cell data
        polydata->GetCellData()->AddArray(cellAreas);
    }

    void  ViewPlanning::RandomizeTriangleColors() {

        // Create a vtkUnsignedCharArray to store the colors
        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3); // Three components per color for RGB
        colors->SetName("TriangleColors");

        // Generate a random RGB color for each triangle (cell)
        for(vtkIdType i = 0; i < this->polydata->GetNumberOfCells(); i++) {
            unsigned char color[3];
            color[0] = std::rand() % 256; // Random value for R (0 to 255)
            color[1] = std::rand() % 256; // Random value for G (0 to 255)
            color[2] = std::rand() % 256; // Random value for B (0 to 255)

            colors->InsertNextTupleValue(color);
        }

        // Add the color data to the PolyData's cell data
        this->polydata->GetCellData()->SetScalars(colors);

        // Update the mapper to reflect the new colors
        this->mapper->SetInputData(this->polydata);
        this->mapper->Update();
    }


    void  ViewPlanning::createCamera(double posX, double posY, double posZ,
                  double focalX, double focalY, double focalZ) {

        // Create a new vtkCamera
        vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
        
        // Set the position of the camera
        camera->SetPosition(posX, posY, posZ);
        
        // Set the focal point of the camera
        camera->SetFocalPoint(focalX, focalY, focalZ);
        
        // Set the created camera as the active camera for the renderer
        renderer->SetActiveCamera(camera);
    }


    void  ViewPlanning::createViewpoint(double posX, double posY, double posZ,
                        double focalX, double focalY, double focalZ,
                        double near, double far, double angle, bool showFrame) {
        // Create a new vtkCamera
        vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();

        // Set camera properties
        camera->SetPosition(posX, posY, posZ);
        camera->SetFocalPoint(focalX, focalY, focalZ);
        camera->SetClippingRange(near, far);
        camera->SetViewAngle(angle);

        // Calculate the projection direction and normalize it
        double projection_direction[3];
        camera->GetDirectionOfProjection(projection_direction);
        vtkMath::Normalize(projection_direction);

        // Choose an initial Up vector
        double up[3] = {0, 0, 1};

        // Check for parallelism and adjust the Up vector if necessary
        double dotProduct = vtkMath::Dot(projection_direction, up);
        if (std::abs(dotProduct) > 0.999) {  // Nearly parallel
            up[0] = 0; up[1] = 1; up[2] = 0;  // Choose a different Up vector
        }

        // Calculate the Right vector using cross product and normalize it
        double right[3];
        vtkMath::Cross(up, projection_direction, right);
        vtkMath::Normalize(right);

        // Recalculate the Up vector to make sure it's orthogonal to Forward and Right and normalize it
        vtkMath::Cross(projection_direction, right, up);
        vtkMath::Normalize(up);
        
        if (showFrame){
            
            float scale = 0.1;
            // Create line actors to visualize the vectors
            vtkSmartPointer<vtkLineSource> projectionLine = vtkSmartPointer<vtkLineSource>::New();
            projectionLine->SetPoint1(posX, posY, posZ);
            projectionLine->SetPoint2(posX + projection_direction[0] * scale, posY + projection_direction[1] * scale, posZ + projection_direction[2] * scale);

            vtkSmartPointer<vtkLineSource> rightLine = vtkSmartPointer<vtkLineSource>::New();
            rightLine->SetPoint1(posX, posY, posZ);
            rightLine->SetPoint2(posX + right[0] * scale, posY + right[1] * scale, posZ + right[2] * scale);

            vtkSmartPointer<vtkLineSource> upLine = vtkSmartPointer<vtkLineSource>::New();
            upLine->SetPoint1(posX, posY, posZ);
            upLine->SetPoint2(posX + up[0] * scale, posY + up[1] * scale, posZ + up[2] * scale);

            // Create mappers and actors for the lines
            vtkSmartPointer<vtkPolyDataMapper> projectionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            projectionMapper->SetInputConnection(projectionLine->GetOutputPort());

            vtkSmartPointer<vtkActor> projectionActor = vtkSmartPointer<vtkActor>::New();
            projectionActor->SetMapper(projectionMapper);
            projectionActor->GetProperty()->SetColor(0.0, 0.0, 1.0); // Blue for projection_direction

            vtkSmartPointer<vtkPolyDataMapper> rightMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            rightMapper->SetInputConnection(rightLine->GetOutputPort());

            vtkSmartPointer<vtkActor> rightActor = vtkSmartPointer<vtkActor>::New();
            rightActor->SetMapper(rightMapper);
            rightActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // Red for right

            vtkSmartPointer<vtkPolyDataMapper> upMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            upMapper->SetInputConnection(upLine->GetOutputPort());

            vtkSmartPointer<vtkActor> upActor = vtkSmartPointer<vtkActor>::New();
            upActor->SetMapper(upMapper);
            upActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // Green for up

            // Adding the line actors to the renderer
            renderer->AddActor(projectionActor);
            renderer->AddActor(rightActor);
            renderer->AddActor(upActor);
                    
        }
        // Set the Up vector for the camera
        camera->SetViewUp(up);

        // Add the camera to the vector of viewpoints
        viewpoints.push_back(camera);
    }


    vtkSmartPointer<vtkCamera>  ViewPlanning::get_current_viewpoint() {
        // Getter method for the current viewpoint camera
        return current_viewpoint;
    }

    vtkSmartPointer<vtkCamera> ViewPlanning::getLastViewpoint(){

        return viewpoints.back();
    }

    std::vector<vtkSmartPointer<vtkCamera>>  ViewPlanning::get_viewpoints() {
        // Getter method for the vector of viewpoints
        return viewpoints;
    }

    void ViewPlanning::removeLastViewpoint(){
        // Check if the vector is not empty before removing the last element
        if (!viewpoints.empty()) {
            viewpoints.pop_back();
        } else {
            std::cout << "Vector is empty. Nothing to remove." << std::endl;
        }

        // Check if the 2D vector is not empty before removing the last row
        if (!visibilityMatrix.empty()) {
            visibilityMatrix.pop_back();
        } else {
            std::cout << "2D vector is empty. Nothing to remove." << std::endl;
        }
    }

    void ViewPlanning::removeViewPoint(int index) {
        // Check if the index is within bounds
        if (index < viewpoints.size()) {
            // Use the erase() method to remove the element at the specified index
            viewpoints.erase(viewpoints.begin() + index);
        }

        // Check if the index is within bounds
        if (index < visibilityMatrix.size()) {
            // Use the erase() method to remove the row at the specified index
            visibilityMatrix.erase(visibilityMatrix.begin() + index);
        }
    }

    std::vector<std::vector<int>>   ViewPlanning::get_visibility_matrix() {
        // Getter method for the visibility matrix
        return visibilityMatrix;
    }

    std::vector<int>  ViewPlanning::get_visibility_sum() {
        // Getter method for the sum of each column in the visibility matrix

        // Check if visibilityMatrix is empty or uninitialized
        if (visibilityMatrix.empty() || visibilityMatrix[0].empty()) {
            // Check the number of cells in polydata
            int numCells = polydata ? polydata->GetNumberOfCells() : 0;
            
            // Return a vector of zeros with the size of the number of cells in polydata
            return std::vector<int>(numCells, 0);
        }

        // Initialize a vector to hold the sum of each column, set initially to zeros
        std::vector<int> columnSums(visibilityMatrix[0].size(), 0);

        // Loop through each row
        for (const auto& row : visibilityMatrix) {
            // Loop through each column in the row
            for (size_t j = 0; j < row.size(); ++j) {
                // Add the value to the sum for that column
                columnSums[j] += row[j];
            }
        }

        return columnSums;
    }


    bool ViewPlanning::completeCoverage() {

        // Get the column sums
        std::vector<int> columnSums = get_visibility_sum();
    
        // Check if all column sums are greater than 0
        for (int sum : columnSums) {
            if (sum <= 0) {
                return false;  // At least one sum is not greater than 0
            }
        }

        return true;  // All column sums are greater than 0
    }


    float ViewPlanning::computeCoverageScore() {
     
        // Get the sum of each column from visibility_mat
        std::vector<int> columnSums = get_visibility_sum();
   
        // Create a vtkDoubleArray to store the visible area percentages
        vtkSmartPointer<vtkDoubleArray> visibleAreaPercentages = vtkSmartPointer<vtkDoubleArray>::New();
        visibleAreaPercentages->SetName("Visible_Area_Percentage");
        visibleAreaPercentages->SetNumberOfComponents(1);
        
        double totalVisibleArea = 0.0;
        // Loop through each triangle (cell) in the PolyData
        for (vtkIdType i = 0; i < this->polydata->GetNumberOfCells(); i++) {
                // Check the sum for the current triangle (column)
            if (columnSums[i] > 0) {
                   
                 double cellArea = this->polydata->GetCellData()->GetArray("Cell_Areas")->GetTuple1(i);
                 totalVisibleArea += cellArea;
            }
            
        }   
  
        float visibilityPercentage = totalVisibleArea / totalModelArea;
           
        return visibilityPercentage;
    }

    void  ViewPlanning::captureImage(const std::string& filename, vtkSmartPointer<vtkCamera> camera) {

        // Store the original active camera
        vtkSmartPointer<vtkCamera> originalCamera = renderer->GetActiveCamera();

        // Set the provided camera as active for capturing
        renderer->SetActiveCamera(camera);
        renderer->Render(); // Re-render to apply the camera view

        // Capture image
        vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
            vtkSmartPointer<vtkWindowToImageFilter>::New();
        windowToImageFilter->SetInput(renderWindow);
        windowToImageFilter->Update();

        vtkSmartPointer<vtkPNGWriter> writer =
            vtkSmartPointer<vtkPNGWriter>::New();
        writer->SetFileName(filename.c_str());
        writer->SetInputConnection(windowToImageFilter->GetOutputPort());
        writer->Write();

        // Revert to the original active camera
        renderer->SetActiveCamera(originalCamera);
    }

    void  ViewPlanning::captureImageOffscreen(const std::string& filename, vtkSmartPointer<vtkCamera> camera) {

        // Create a new renderer and render window for off-screen rendering
        vtkSmartPointer<vtkRenderer> offscreenRenderer = vtkSmartPointer<vtkRenderer>::New();
        vtkSmartPointer<vtkRenderWindow> offscreenWindow = vtkSmartPointer<vtkRenderWindow>::New();
        
        // Add the existing actor to the off-screen renderer
        offscreenRenderer->AddActor(actor);
   
        // Add the off-screen renderer to the off-screen window
        offscreenWindow->AddRenderer(offscreenRenderer);
        
        // Enable off-screen rendering
        offscreenWindow->SetOffScreenRendering(1);
        
        // Apply the provided camera settings to the off-screen renderer
        offscreenRenderer->SetActiveCamera(camera);

        // Set the size of the off-screen renderer to match the aspect ratio
        offscreenWindow->SetSize(width, height);
        
        // Render the scene off-screen
        offscreenWindow->Render();
        
        // Capture the image
        vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
            vtkSmartPointer<vtkWindowToImageFilter>::New();
        windowToImageFilter->SetInput(offscreenWindow);
        windowToImageFilter->Update();
        
        vtkSmartPointer<vtkPNGWriter> writer =
            vtkSmartPointer<vtkPNGWriter>::New();
        writer->SetFileName(filename.c_str());
        writer->SetInputConnection(windowToImageFilter->GetOutputPort());
        writer->Write();
    }


    vtkSmartPointer<vtkFloatArray>  ViewPlanning::capture_depth_data(vtkSmartPointer<vtkCamera> camera) {

        // Create a new renderer and render window for off-screen rendering
        vtkSmartPointer<vtkRenderer> offscreenRenderer = vtkSmartPointer<vtkRenderer>::New();
        vtkSmartPointer<vtkRenderWindow> offscreenWindow = vtkSmartPointer<vtkRenderWindow>::New();
  
        // Enable off-screen rendering
        offscreenWindow->SetOffScreenRendering(1);
        offscreenWindow->AddRenderer(offscreenRenderer);
        
        // Add the existing actor to the off-screen renderer
        offscreenRenderer->AddActor(actor);

        // Apply the provided camera settings to the off-screen renderer
        offscreenRenderer->SetActiveCamera(camera);

        // Set the size of the off-screen renderer to match the aspect ratio
        offscreenWindow->SetSize(width, height);

        // Render the scene off-screen
        offscreenWindow->Render();
        
        double cameraPosition[3];
        camera->GetPosition(cameraPosition);

        // Get the size of the render window
        int* size = offscreenWindow->GetSize();
        int width = size[0];
        int height = size[1];
        
        // Debug print the near and far clipping planes
        double nearClip = camera->GetClippingRange()[0];
        double farClip = camera->GetClippingRange()[1];

        // Create a float array to store depth data
        vtkSmartPointer<vtkFloatArray> depthData = vtkSmartPointer<vtkFloatArray>::New();
        depthData->SetNumberOfComponents(1);
        depthData->SetNumberOfTuples(width * height);
    
        // Read the depth buffer
        offscreenWindow->GetZbufferData(0, 0, width - 1, height - 1, depthData->GetPointer(0));


        // // Create a vtkFloatArray to store depth data
        // vtkSmartPointer<vtkFloatArray> z_values = vtkSmartPointer<vtkFloatArray>::New();
        // z_values->SetNumberOfComponents(1);
        // z_values->SetNumberOfTuples(width * height);

        // // Loop through the depth data values
        // for (int y = 0; y < height; ++y) {
        //     for (int x = 0; x < width; ++x) {

        //         // Calculate the index into the depthData array
        //         int index = y * width + x;

        //         // Access the depth value at the current pixel
        //         float depthValue = depthData->GetValue(index);
                
        //         // Perspective formula to convert from z buffers to real world z depth
        //         // Reference: https://discourse.vtk.org/t/absolute-values-for-depth-image/326 
        //         // Currently not using this
        //         float z_value = -2 * nearClip * farClip / (((depthValue - 0.5) * 2.0 * (farClip - nearClip)) - nearClip - farClip);
        //         z_values->SetTuple1(index, z_value);
        //     }   
        // }

     




        return depthData;
    }

   void  ViewPlanning::save_depth_image(vtkSmartPointer<vtkFloatArray> depthData, const std::string& filename) {

        // Create an image data object to store the normalized depth data
        vtkSmartPointer<vtkImageData> depthImage = vtkSmartPointer<vtkImageData>::New();
        depthImage->SetDimensions(width, height, 1);
        depthImage->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
        
        // Find min and max depth values for normalization
        float minDepth = std::numeric_limits<float>::max();
        float maxDepth = std::numeric_limits<float>::min();
        for (int i = 0; i < width * height; ++i) {
            float depthValue = depthData->GetValue(i);
            minDepth = std::min(minDepth, depthValue);
            maxDepth = std::max(maxDepth, depthValue);
        }

        // Copy and normalize depth data to the image data object
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float depthValue = depthData->GetValue(y * width + x); // flipping such that farther means darker
                
                // Normalize depth value to 0-255 range for image
                unsigned char normalizedDepth = 255 - static_cast<unsigned char>(255 * (depthValue - minDepth) / (maxDepth - minDepth));
                depthImage->SetScalarComponentFromFloat(x, y, 0, 0, normalizedDepth);
            }
        }
        
        // Write the depth image to a PNG file
        vtkSmartPointer<vtkPNGWriter> pngWriter = vtkSmartPointer<vtkPNGWriter>::New();
        pngWriter->SetFileName(filename.c_str());
        pngWriter->SetInputData(depthImage);
        pngWriter->Write();
    }


    vtkSmartPointer<vtkPoints> ViewPlanning::convertDepthTo3D(vtkSmartPointer<vtkFloatArray> depthValues, vtkSmartPointer<vtkCamera> camera) {
        
        // Create a vtkPoints object to store the 3D world coordinates
        vtkSmartPointer<vtkPoints> worldCoordinates = vtkSmartPointer<vtkPoints>::New();

        // Get camera parameters
        double nearClip = camera->GetClippingRange()[0];
        double farClip = camera->GetClippingRange()[1];
        double cameraPosition[3];
        camera->GetPosition(cameraPosition);

        // Compute the aspect ratio (assuming this does not change during the loop)
        double aspect = static_cast<double>(this->width) / static_cast<double>(this->height);

        // Precompute the inverse matrices
        vtkMatrix4x4* inverseProjectionMatrix = vtkMatrix4x4::New();
        vtkMatrix4x4::Invert(camera->GetProjectionTransformMatrix(aspect, 0, 1), inverseProjectionMatrix);

        vtkMatrix4x4* inverseModelViewMatrix = vtkMatrix4x4::New();
        vtkMatrix4x4::Invert(camera->GetModelViewTransformMatrix(), inverseModelViewMatrix);

        // Loop through the depth values
        for (int y = 0; y < this->height; ++y) {
            for (int x = 0; x < this->width; ++x) {

                // Calculate the index into the depthValues array
                int index = y * this->width + x;

                // Access the depth value at the current pixel
                float depthValue = depthValues->GetValue(index);

                // Cull unreflected depths
                if (depthValue == 1.0) {
                    continue;
                }

                // Convert pixel coordinates to NDC
                double NDC_X = (2.0 * x / this->width - 1.0);
                double NDC_Y = (2.0 * y / this->height - 1.0);
                double NDC_Z = depthValue;  // Assuming the depth is already in NDC

                // Create 4D homogeneous coordinates
                double homoCoordinates[4] = {NDC_X, NDC_Y, NDC_Z, 1.0};

                // Convert NDC to view coordinates
                inverseProjectionMatrix->MultiplyPoint(homoCoordinates, homoCoordinates);

                // Convert to Cartesian coordinates in view space
                for (int i = 0; i < 4; ++i) {
                    homoCoordinates[i] /= homoCoordinates[3];
                }

                // Convert view coordinates to world coordinates
                inverseModelViewMatrix->MultiplyPoint(homoCoordinates, homoCoordinates);

                // Insert point into worldCoordinates
                worldCoordinates->InsertNextPoint(homoCoordinates[0], homoCoordinates[1], homoCoordinates[2]);
            }
        }

        // Clean up
        inverseProjectionMatrix->Delete();
        inverseModelViewMatrix->Delete();

        return worldCoordinates;
    }



    pcl::PointCloud<pcl::PointXYZ>::Ptr  ViewPlanning::convertToPointCloud(vtkSmartPointer<vtkPoints> worldPoints) {

        // Create a new PCL PointCloud object
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Reserve space for the points in the point cloud
        cloud->points.resize(worldPoints->GetNumberOfPoints());

        // Iterate through the VTK points and convert to PCL points
        for (vtkIdType i = 0; i < worldPoints->GetNumberOfPoints(); ++i) {
            double* point = worldPoints->GetPoint(i);
            
            // Create a PCL PointXYZ point and populate its coordinates
            pcl::PointXYZ pclPoint;
            pclPoint.x = static_cast<float>(point[0]);
            pclPoint.y = static_cast<float>(point[1]);
            pclPoint.z = static_cast<float>(point[2]);
            
            // Assign the PCL point to the corresponding index in the PointCloud
            cloud->points[i] = pclPoint;
        }

        return cloud;
    }



    vtkSmartPointer<vtkPolyData>  ViewPlanning::convertPointCloudToVtk(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

        // Create VTK points and polydata objects
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

        // Iterate through the PCL point cloud and convert each point to VTK
        for (auto& point : cloud->points) {
            // Insert the PCL point coordinates as a VTK point
            points->InsertNextPoint(point.x, point.y, point.z);
        }

        // Set the VTK points in the polydata
        polydata->SetPoints(points);

        return polydata;
    }



    void  ViewPlanning::addPointCloudToRenderer(vtkSmartPointer<vtkPoints> worldPoints) {

        // Create a VTK polydata object to hold the point cloud
        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(worldPoints);

        // Create a VTK vertex glyph filter to render points as squares
        vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vertexGlyphFilter->SetInputData(polyData);

        // Create a mapper and actor to display the points
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(vertexGlyphFilter->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        
        // Adjust the point size and color as needed
        actor->GetProperty()->SetPointSize(2.0); // Set point size to 2.0 (adjust as needed)
        actor->GetProperty()->SetColor(0.0, 0.0, 1.0); // Set the color to blue (R=0, G=0, B=1)

        // Add the actor to the renderer
        renderer->AddActor(actor);
    }


    
    void  ViewPlanning::RemoveBackFacingTriangles(vtkSmartPointer<vtkCamera> viewpoint, std::vector<int>& visibleTriangles, vtkFloatArray* cellNormals) {

        // Create a vector to store indices of back-facing triangles
        std::vector<int> backFacingTriangles;

        // Loop through the indices of visible triangles
        for (int index : visibleTriangles) {
            double normal[3];
            cellNormals->GetTuple(index, normal);

            // Get the cell (triangle) and its first point for illustration
            vtkCell* cell = polydata->GetCell(index);
            double point[3];
            cell->GetPoints()->GetPoint(0, point);

            // Calculate the view vector from the viewpoint to the point
            double viewVector[3];
            for (int j = 0; j < 3; ++j) {
                viewVector[j] = viewpoint->GetPosition()[j] - point[j];
            }

            // Calculate the dot product between the normal and view vector
            double dotProduct = normal[0] * viewVector[0] + normal[1] * viewVector[1] + normal[2] * viewVector[2];

            // If the dot product is less than or equal to 0, the triangle is back-facing
            if (dotProduct <= 0) {
                backFacingTriangles.push_back(index);
            }
        }

        // Remove back-facing triangles from the list of visible triangles
        std::vector<int> newVisibleTriangles;
        std::unordered_set<int> backFacingSet(backFacingTriangles.begin(), backFacingTriangles.end());

        for (int index : visibleTriangles) {
            if (backFacingSet.find(index) == backFacingSet.end()) {
                newVisibleTriangles.push_back(index);
            }
        }

        // Update the list of visible triangles with non-back-facing ones
        visibleTriangles = newVisibleTriangles;
    }



    void  ViewPlanning::RemoveOutsideFrustumTriangles(vtkSmartPointer<vtkCamera> viewpoint, std::vector<int>& visibleTriangles) {

        // Calculate the frustum planes of the viewpoint
        double planes[24];
        double aspect = static_cast<double>(width) / static_cast<double>(height);
        viewpoint->GetFrustumPlanes(aspect, planes);

        // Iterate through the list of visible triangles
        auto it = visibleTriangles.begin();
        while (it != visibleTriangles.end()) {
            int index = *it;

            // Get the cell (triangle) from the polydata
            vtkCell* cell = polydata->GetCell(index);
            bool isOutside = false;

            // Check against each of the 6 frustum planes
            for (int i = 0; i < 6; ++i) { // There are 6 planes in the frustum
                double* plane = &planes[i * 4]; // Each plane has 4 coefficients
                isOutside = true; // Assume outside until proven inside for each plane

                // Check each vertex of the triangle against the current frustum plane
                for (int j = 0; j < 3; ++j) { // There are 3 vertices in a triangle
                    double point[3];
                    cell->GetPoints()->GetPoint(j, point);

                    // Calculate the signed distance from the point to the plane
                    double distance = plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3];
                    
                    // If the distance is non-negative, the point is inside the plane
                    if (distance >= 0) {
                        isOutside = false; // One point was inside this plane, so not fully outside
                        break;
                    }
                }

                // If the triangle is fully outside one plane, exit the loop
                if (isOutside) break;
            }

            // If the triangle is outside the frustum, remove it from the list of visible triangles
            if (isOutside) {
                it = visibleTriangles.erase(it); // Remove if outside the frustum
            } else {
                ++it;
            }
        }
    }

    // void  ViewPlanning::computeTrianglesWithPointCloud(vtkSmartPointer<vtkPoints> worldPoints, std::vector<int>& visibleTriangles) {

    //     // // Extract points from the pre-calculated centroidPolyData
    //     vtkSmartPointer<vtkPoints> centroidPoints = centroidPolyData->GetPoints();

    //     // Initialize a K-d tree and set its input points as centroids
    //     vtkSmartPointer<vtkKdTreePointLocator> kdTree = vtkSmartPointer<vtkKdTreePointLocator>::New();
    //     kdTree->SetDataSet(centroidPolyData);
    //     kdTree->BuildLocator();

    //     // List to keep track of visible centroid indices
    //     std::vector<int> visibleCentroidIndices;

    //     // Search for each worldPoint in the K-d tree
    //     for (vtkIdType i = 0; i < worldPoints->GetNumberOfPoints(); ++i) {
    //         double testPoint[3];
    //         worldPoints->GetPoint(i, testPoint);

    //         // Find the index of the nearest centroid point
    //         vtkIdType id = kdTree->FindClosestPoint(testPoint);

    //         // Get the coordinates of the nearest centroid point
    //         double closestPoint[3];
    //         centroidPoints->GetPoint(id, closestPoint);

    //         // Calculate the distance between the worldPoint and the closest centroid
    //         double distance = sqrt(vtkMath::Distance2BetweenPoints(testPoint, closestPoint));

    //         // Check if the distance is within the specified threshold (e.g., 0.01 units)
    //         if (distance <= 1e-02) {
    //             // Add the index of the visible centroid to the list
    //             visibleCentroidIndices.push_back(id);
    //         }
    //     }

    //     // Final filtering step: Remove any triangles that don't have visible centroids
    //     std::vector<int> finalVisibleTriangles;
    //     for (int triangleIndex : visibleTriangles) {
    //         if (std::find(visibleCentroidIndices.begin(), visibleCentroidIndices.end(), triangleIndex) != visibleCentroidIndices.end()) {
    //             finalVisibleTriangles.push_back(triangleIndex);
    //         }
    //     }
        
    //     // Update the list of visible triangles
    //     visibleTriangles = finalVisibleTriangles;
    // }

    void  ViewPlanning::computeTrianglesWithPointCloud(vtkSmartPointer<vtkPoints> worldPoints, std::vector<int>& visibleTriangles) {

        // // Extract points from the pre-calculated centroidPolyData
        vtkSmartPointer<vtkPoints> centroidPoints = centroidPolyData->GetPoints();

        // Initialize a K-d tree and set its input points as centroids
        vtkSmartPointer<vtkKdTreePointLocator> kdTree = vtkSmartPointer<vtkKdTreePointLocator>::New();
        kdTree->SetDataSet(centroidPolyData);
        kdTree->BuildLocator();

        std::unordered_set<int> visibleCentroidIndices;  // Use a set to prevent duplicates

        // Search for each worldPoint in the K-d tree
        for (vtkIdType i = 0; i < worldPoints->GetNumberOfPoints(); ++i) {
        double testPoint[3];
        worldPoints->GetPoint(i, testPoint);

        // Initialize an empty list of IDs that are within the radius
        vtkSmartPointer<vtkIdList> result = vtkSmartPointer<vtkIdList>::New();

        // Perform radius search
        double radius = 1e-02;  // Define your radius here
        kdTree->FindPointsWithinRadius(radius, testPoint, result);

        for (vtkIdType j = 0; j < result->GetNumberOfIds(); ++j) {
            vtkIdType id = result->GetId(j);
            visibleCentroidIndices.insert(id);  // Inserts only unique values
        }
        }

        // Final filtering step: Remove any triangles that don't have visible centroids
        std::vector<int> finalVisibleTriangles;
        for (int triangleIndex : visibleTriangles) {
            if (visibleCentroidIndices.find(triangleIndex) != visibleCentroidIndices.end()) {
                finalVisibleTriangles.push_back(triangleIndex);
            }
        }

        // Update the list of visible triangles
        visibleTriangles = finalVisibleTriangles;

    }



    std::vector<int>  ViewPlanning::compute_visibility(vtkSmartPointer<vtkCamera> viewpoint, vtkSmartPointer<vtkPoints> worldPoints, int viewpoint_index) {

        // Initialize a list of visible triangles
        std::vector<int> visibleTriangles(polydata->GetNumberOfCells());
       
        // Insert and initialize a row for the current viewpoint in the visibility matrix
        visibilityMatrix.push_back(std::vector<int>(polydata->GetNumberOfCells(), 0));

        // Initialize visibleTriangles to contain all triangle indices
        std::iota(visibleTriangles.begin(), visibleTriangles.end(), 0);

        // Get the normals of the faces
        vtkFloatArray* cellNormals = vtkFloatArray::SafeDownCast(polydata->GetCellData()->GetNormals());

        // Step 1: Remove back-facing triangles
        RemoveBackFacingTriangles(viewpoint, visibleTriangles, cellNormals);

        // Step 2: Frustum culling
        RemoveOutsideFrustumTriangles(viewpoint, visibleTriangles);

        // Step 3: Efficient search for nearest triangle
        computeTrianglesWithPointCloud(worldPoints, visibleTriangles);
        
    
        // Step 4: Update visibility matrix
        for (int i = 0; i < polydata->GetNumberOfCells(); ++i) {
              
            if (std::find(visibleTriangles.begin(), visibleTriangles.end(), i) != visibleTriangles.end()) {
                // This triangle is visible from the current viewpoint
                
                visibilityMatrix[viewpoint_index][i] = 1;
            } else {
                // This triangle is not visible from the current viewpoint
       
                visibilityMatrix[viewpoint_index][i] = 0;
            }
        }
        
        // Return the list of visible triangles
        return visibleTriangles;
    }



    void  ViewPlanning::ColorVisibleTriangles(const std::vector<std::vector<int>> visibility_mat) {

        // Get the sum of each column from visibility_mat
        std::vector<int> columnSums = get_visibility_sum();

        // Create a vtkUnsignedCharArray to store the colors
        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);  // Three components per color for RGB
        colors->SetName("TriangleColors");

        // Loop through each triangle (cell) in the PolyData
        for(vtkIdType i = 0; i < this->polydata->GetNumberOfCells(); i++) {
            unsigned char color[3];

            // Check the sum for the current triangle (column)
            if (columnSums[i] > 0) {
                // Color the visible triangles green
                color[0] = 0;
                color[1] = 255;
                color[2] = 0;
            } else {
                // Color the non-visible triangles red
                color[0] = 255;
                color[1] = 0;
                color[2] = 0;
            }

            // Insert the color for the current triangle
            colors->InsertNextTupleValue(color);
        }

        // Add the color data to the PolyData
        this->polydata->GetCellData()->SetScalars(colors);

        // Update the mapper to reflect the new colors
        this->mapper->SetInputData(this->polydata);
        this->mapper->Update();
    }

    void ViewPlanning::ColorOverlap(const std::vector<std::vector<int>> visibility_mat){
        // Get the sum of each column from visibility_mat
        std::vector<int> columnSums = get_visibility_sum();

        int maxColumnSum = 5;
        // // Define the color range (from darkest blue to lightest blue)
        // unsigned char minColor[3] = {0, 0, 255};  // Darkest blue
        // unsigned char maxColor[3] = {173, 216, 230};  // Lightest blue
      
        // Define the color range (from darkest green to lightest green)
        unsigned char minColor[3] = {0, 128, 0};   // Darkest green
        unsigned char maxColor[3] = {144, 238, 144};  // Lightest green

        // Create a vtkUnsignedCharArray to store the colors
        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);  // Three components per color for RGB
        colors->SetName("TriangleColors");

        // Loop through each triangle (cell) in the PolyData
        for(vtkIdType i = 0; i < this->polydata->GetNumberOfCells(); i++) {
            unsigned char color[3];

            // Calculate the shade of blue based on columnSums[i]
            double fraction = static_cast<double>(columnSums[i]) / static_cast<double>(maxColumnSum);

            // Interpolate the color between minColor and maxColor
            for (int j = 0; j < 3; j++) {
                color[j] = static_cast<unsigned char>(minColor[j] + fraction * (maxColor[j] - minColor[j]));
            }

            // Insert the color for the current triangle
            colors->InsertNextTupleValue(color);
        }

        // Add the color data to the PolyData
        this->polydata->GetCellData()->SetScalars(colors);

        // Update the mapper to reflect the new colors
        this->mapper->SetInputData(this->polydata);
        this->mapper->Update();
        }

    void  ViewPlanning::visualizeScene() {
      
        // Add the actor (the 3D model) to the renderer
        renderer->AddActor(actor);

        // Add the renderer to the render window
        renderWindow->AddRenderer(renderer);

        // Set the render window size
        renderWindow->SetSize(1280, 720);

        // Create a render window interactor
        vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
        renderWindowInteractor->SetRenderWindow(renderWindow);

        // Create the axes actor for the world coordinate frame
        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

        // Create an orientation marker widget
        vtkSmartPointer<vtkOrientationMarkerWidget> marker =
            vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        marker->SetOrientationMarker(axes);
        marker->SetInteractor(renderWindowInteractor);
        marker->SetEnabled(1);
        marker->InteractiveOn();

        // Set the interaction style to trackball camera
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
            vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        renderWindowInteractor->SetInteractorStyle(style);

        // Start the render window interactor to interact with the scene
        renderWindowInteractor->Start();
    }


    void  ViewPlanning::addCameraFrustumToRenderer(vtkSmartPointer<vtkCamera> camera) {

        // Calculate the frustum planes for the given camera
        double planes[24];
        double aspect = static_cast<double>(width) / static_cast<double>(height);
        camera->GetFrustumPlanes(aspect, planes);

        // Create a vtkPlanes object to represent the frustum planes
        vtkSmartPointer<vtkPlanes> frustumPlanes = vtkSmartPointer<vtkPlanes>::New();
        frustumPlanes->SetFrustumPlanes(planes);

        // Create a vtkFrustumSource to generate the frustum geometry
        vtkSmartPointer<vtkFrustumSource> frustumSource = vtkSmartPointer<vtkFrustumSource>::New();
        frustumSource->ShowLinesOn();
        frustumSource->SetPlanes(frustumPlanes);
        frustumSource->Update();

        // Create a vtkPolyData object to hold the frustum geometry
        vtkSmartPointer<vtkPolyData> frustum = vtkSmartPointer<vtkPolyData>::New();
        frustum->ShallowCopy(frustumSource->GetOutput());

        // Create a vtkPolyDataMapper to map the frustum geometry to visual representation
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(frustum);

        // Create a vtkActor to display the frustum outline
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetRepresentationToWireframe(); // Display as wireframe
        actor->GetProperty()->LightingOff(); // Disable lighting for this actor

        // Add the frustum outline actor to the renderer
        renderer->AddActor(actor);
 
    }

    void ViewPlanning::showViewpointSolution(vtkSmartPointer<vtkCamera> camera, double scale) {

       // Calculate the frustum planes for the given camera
        double planes[24];
        double aspect = static_cast<double>(width) / static_cast<double>(height);
        camera->GetFrustumPlanes(aspect, planes);

        // Create a vtkPlanes object to represent the frustum planes
        vtkSmartPointer<vtkPlanes> frustumPlanes = vtkSmartPointer<vtkPlanes>::New();
        frustumPlanes->SetFrustumPlanes(planes);

        // Create a vtkFrustumSource to generate the frustum geometry
        vtkSmartPointer<vtkFrustumSource> frustumSource = vtkSmartPointer<vtkFrustumSource>::New();
        frustumSource->ShowLinesOff(); // Turn off lines
        frustumSource->SetPlanes(frustumPlanes);
        frustumSource->Update();

        // Create a vtkPolyData object to hold the frustum geometry
        vtkSmartPointer<vtkPolyData> frustum = vtkSmartPointer<vtkPolyData>::New();
        frustum->ShallowCopy(frustumSource->GetOutput());

        // Create a transform filter for applying the transformations
        vtkSmartPointer<vtkTransformFilter> transformFilter = vtkSmartPointer<vtkTransformFilter>::New();
        transformFilter->SetInputData(frustum);

        // Get the camera's position
        double cameraPosition[3];
        camera->GetPosition(cameraPosition);

        // Create a vtkTransform object for transformations
        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        
        // Translate frustum to origin
        transform->Translate(cameraPosition[0], cameraPosition[1], cameraPosition[2]);

        // Perform scaling at origin
        transform->Scale(scale, scale, scale);

        // Translate back to the original camera position
        transform->Translate(-cameraPosition[0], -cameraPosition[1], -cameraPosition[2]);

        // Apply the transformations
        transformFilter->SetTransform(transform);
        transformFilter->Update();

        // Create a vtkPolyDataMapper to map the frustum geometry to visual representation
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(transformFilter->GetOutputPort());

        // Create a vtkActor to display the scaled frustum
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetRepresentationToSurface(); // Display as surface
        actor->GetProperty()->LightingOn(); // Enable lighting for this actor

        // Add the scaled frustum actor to the renderer
        renderer->AddActor(actor);
    }




// private:

//     vtkSmartPointer<vtkSTLReader> stlReader;
//     vtkSmartPointer<vtkPolyDataMapper> mapper;
//     vtkSmartPointer<vtkActor> actor;
//     vtkSmartPointer<vtkRenderer> renderer;
//     vtkSmartPointer<vtkRenderWindow> renderWindow;

//     vtkSmartPointer<vtkCamera> current_viewpoint;
//     std::vector<vtkSmartPointer<vtkCamera>> viewpoints;

//     int width, height;

//     vtkPolyData* polydata;
//     vtkSmartPointer<vtkPoints> centroids;
//     vtkSmartPointer<vtkFloatArray> centroidNormals;
//     vtkSmartPointer<vtkPolyData> centroidPolyData;

//     std::vector<std::vector<int>> visibilityMatrix;



// int main(int argc, char **argv) {

//     ros::init(argc, argv, "view_planning_node");

//     // Initialize the ViewPlanning class
//     ViewPlanning vp;

//     // Load the STL model. Replace this with your own model path.
//     std::string package_path = ros::package::getPath("depth_processing");
//     std::string stl_file_path = package_path + "/saved_models/cube_3072.stl";
//     vp.loadModel(stl_file_path);
//     vp.RandomizeTriangleColors();

//     // Create three viewpoints with random positions and orientations
//     vp.createViewpoint(0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.1, 0.75, 45.0);
//     vp.createViewpoint(0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.1, 0.75, 45.0);
//     vp.createViewpoint(0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.1, 0.75, 45.0);

//     auto viewpoints = vp.get_viewpoints();
//     std::vector<std::vector<int>> visibility_mat;
//     vtkSmartPointer<vtkPoints> aggregatedWorldPoints = vtkSmartPointer<vtkPoints>::New();
//     int index = 0;

//     // Loop through each viewpoint
//     for (auto& viewpoint : viewpoints) {
//         // Show frustum of camera
//         vp.addCameraFrustumToRenderer(viewpoint);

//         // Capture an RGB image
//         std::string image_output1 = package_path + "/images/viewpoint" + std::to_string(index) + ".png";
//         vp.captureImageOffscreen(image_output1, viewpoint);

//         // Calculate depth data
//         vtkSmartPointer<vtkFloatArray> depthData = vp.capture_depth_data(viewpoint);

//         // Save depth data as an image
//         std::string filename = package_path + "/images/depth_image" + std::to_string(index) + ".png";
//         vp.save_depth_image(depthData, filename);

//         // Convert depth data to 3D world points and compute visibility
//         vtkSmartPointer<vtkPoints> worldPoints = vp.convertDepthTo3D(depthData, viewpoint);
//         std::vector<int> visibleTriangles = vp.compute_visibility(viewpoint, worldPoints, index);

//         // Append points to the aggregatedWorldPoints
//         for (vtkIdType i = 0; i < worldPoints->GetNumberOfPoints(); ++i) {
//             double point[3];
//             worldPoints->GetPoint(i, point);
//             aggregatedWorldPoints->InsertNextPoint(point);
//         }

//         ++index;
//     }

//     // Compute the visibility matrix
//     visibility_mat = vp.get_visibility_matrix();

//     // Color visible triangles in the model
//     vp.ColorVisibleTriangles(visibility_mat);

//     // Uncomment this line if you want to add the point cloud to the renderer
//     // vp.addPointCloudToRenderer(aggregatedWorldPoints);

//     // Loop through the visibility matrix and print it with proper formatting
//     for (const std::vector<int>& row : visibility_mat) {
//         for (int value : row) {
//             std::cout << std::left << std::setw(5) << value;
//         }
//         std::cout << "\n"; // Use "\n" for a newline
//     }

//     // Visualize the scene
//     vp.visualizeScene();

//     return 0;
// }

