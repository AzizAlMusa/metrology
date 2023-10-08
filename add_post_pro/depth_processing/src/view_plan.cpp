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
#include <vtkLookupTable.h>
#include <vtkPolyDataNormals.h>
#include <vtkGlyph3D.h>
#include <vtkArrowSource.h>
#include <vtkDoubleArray.h>
#include <vtkKdTreePointLocator.h>


class ViewPlanning {
public:
    // Constructor
    ViewPlanning() {
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
    }


       void loadModel(const std::string& stlFilePath) {

        // Load the STL file
        stlReader->SetFileName(stlFilePath.c_str());

        // Explicitly update the reader to load the file
        stlReader->Update();

        // Get the output
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

        // PLOT NORMALS AND CALCULATE CENTROIDS AND CENTROID NORMALS 
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

        // // Create arrow for glyphs
        // vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();

        // // Create glyphs at centroids
        // vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
        // glyph3D->SetSourceConnection(arrowSource->GetOutputPort());
        // glyph3D->SetInputData(centroidPolyData);
        // glyph3D->SetVectorModeToUseNormal();

        // // Create a glyph actor and add it to the renderer
        // vtkSmartPointer<vtkActor> glyphActor = vtkSmartPointer<vtkActor>::New();
        // vtkSmartPointer<vtkPolyDataMapper> glyphMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        // glyphMapper->SetInputConnection(glyph3D->GetOutputPort());
        // glyphActor->SetMapper(glyphMapper);

        // renderer->AddActor(glyphActor);

        // Connect the original polydata to the mapper and actor as before
        mapper->SetInputData(polydata);
        actor->SetMapper(mapper);
    }


    void RandomizeTriangleColors() {
        // Create a vtkUnsignedCharArray to store the colors
        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3); // Three components per color for RGB
        colors->SetName("TriangleColors");

        // Generate a random RGB color for each triangle (cell)
        for(vtkIdType i = 0; i < this->polydata->GetNumberOfCells(); i++) {
            unsigned char color[3];
            color[0] = std::rand() % 256; // Random value for R
            color[1] = std::rand() % 256; // Random value for G
            color[2] = std::rand() % 256; // Random value for B

            colors->InsertNextTupleValue(color);
        }

        // Add the color data to the PolyData
        this->polydata->GetCellData()->SetScalars(colors);

        // Update the mapper
        this->mapper->SetInputData(this->polydata);
        this->mapper->Update();
    }


    void createCamera(double posX, double posY, double posZ,
                      double focalX, double focalY, double focalZ) {
        vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
        camera->SetPosition(posX, posY, posZ);
        camera->SetFocalPoint(focalX, focalY, focalZ);
        renderer->SetActiveCamera(camera);
    }

    void createViewpoint(double posX, double posY, double posZ, 
                         double focalX, double focalY, double focalZ,
                         double near, double far, double angle) {


        vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
        
        // Set position, focal point, etc.
        camera->SetPosition(posX, posY, posZ);
        camera->SetFocalPoint(focalX, focalY, focalZ);
        camera->SetClippingRange(near, far);
        camera->SetViewAngle(angle);
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

        // Calculate the Right vector using cross product
        double right[3];
        vtkMath::Cross(up, projection_direction, right);
        
        // Normalize the Right vector
        vtkMath::Normalize(right);

        // Recalculate the Up vector to make sure it's orthogonal to Forward and Right
        vtkMath::Cross(projection_direction, right, up);
        
        // Normalize the Up vector
        vtkMath::Normalize(up);


         // Create line actors to visualize the vectors
        vtkSmartPointer<vtkLineSource> projectionLine = vtkSmartPointer<vtkLineSource>::New();
        projectionLine->SetPoint1(posX, posY, posZ);
        projectionLine->SetPoint2(posX + projection_direction[0], posY + projection_direction[1], posZ + projection_direction[2]);

        vtkSmartPointer<vtkLineSource> rightLine = vtkSmartPointer<vtkLineSource>::New();
        rightLine->SetPoint1(posX, posY, posZ);
        rightLine->SetPoint2(posX + right[0], posY + right[1], posZ + right[2]);

        vtkSmartPointer<vtkLineSource> upLine = vtkSmartPointer<vtkLineSource>::New();
        upLine->SetPoint1(posX, posY, posZ);
        upLine->SetPoint2(posX + up[0], posY + up[1], posZ + up[2]);

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

        camera->SetViewUp(up);

        // Adding to vector 
        viewpoints.push_back(camera);
    
    }

    vtkSmartPointer<vtkCamera> get_current_viewpoint() {
        return current_viewpoint;
    }

    std::vector<vtkSmartPointer<vtkCamera>> get_viewpoints() {
        return viewpoints;
    }

    std::vector<std::vector<int>>  get_visibility_matrix() {
        return visibilityMatrix;
    }

    std::vector<int> get_visibility_sum() {
        
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


    void captureImage(const std::string& filename, vtkSmartPointer<vtkCamera> camera) {
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

    void captureImageOffscreen(const std::string& filename, vtkSmartPointer<vtkCamera> camera) {
        // Create a new renderer and render window for off-screen rendering
        vtkSmartPointer<vtkRenderer> offscreenRenderer = vtkSmartPointer<vtkRenderer>::New();
        vtkSmartPointer<vtkRenderWindow> offscreenWindow = vtkSmartPointer<vtkRenderWindow>::New();
        
        offscreenRenderer->AddActor(actor);  // Use the existing actor
        offscreenWindow->AddRenderer(offscreenRenderer);
        offscreenWindow->SetOffScreenRendering(1);  // Enable off-screen rendering
        
        // Apply the provided camera settings to the off-screen renderer
        offscreenRenderer->SetActiveCamera(camera);


        // Set the size of the offscreen renderer to match the aspect ratio
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

    vtkSmartPointer<vtkFloatArray> capture_depth_data(vtkSmartPointer<vtkCamera> camera) {

        // Create a new renderer and render window for off-screen rendering
        vtkSmartPointer<vtkRenderer> offscreenRenderer = vtkSmartPointer<vtkRenderer>::New();
        vtkSmartPointer<vtkRenderWindow> offscreenWindow = vtkSmartPointer<vtkRenderWindow>::New();
        
        offscreenRenderer->AddActor(actor);  // Use the existing actor
        offscreenWindow->AddRenderer(offscreenRenderer);
        offscreenWindow->SetOffScreenRendering(1);  // Enable off-screen rendering
        
        // Apply the provided camera settings to the off-screen renderer
        offscreenRenderer->SetActiveCamera(camera);

        // Set the size of the offscreen renderer to match the aspect ratio
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


        // Create a vtkFloatArray to store depth data
        vtkSmartPointer<vtkFloatArray> z_values = vtkSmartPointer<vtkFloatArray>::New();
        z_values->SetNumberOfComponents(1);
        z_values->SetNumberOfTuples(width * height);


        // Loop through the depth data values
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {

                // Calculate the index into the depthData array
                int index = y * width + x;

                // Access the depth value at the current pixel
                float depthValue = depthData->GetValue(index);
                
                // Perspective formula to convert from z buffers to real world z depth: https://discourse.vtk.org/t/absolute-values-for-depth-image/326 
                float z_value =   -2 * nearClip * farClip / (((depthValue - 0.5) * 2.0 * (farClip - nearClip)) - nearClip - farClip);
                z_values->SetTuple1(index, z_value);
        

               
            }   
        }

        projection_matrix = camera->GetProjectionTransformMatrix(offscreenRenderer);

        return depthData;
    }

    void save_depth_image(vtkSmartPointer<vtkFloatArray> depthData, const std::string& filename) {

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

      
            // Copy and normalize depth data to image data object
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    float depthValue =  depthData->GetValue(y * width + x); // flipping such that farther means darker
                
                    unsigned char normalizedDepth = 255 - static_cast<unsigned char>(255 * (depthValue - minDepth) / (maxDepth - minDepth));
                    depthImage->SetScalarComponentFromFloat(x, y, 0, 0, normalizedDepth);
                }
            }
            
            vtkSmartPointer<vtkPNGWriter> pngWriter = vtkSmartPointer<vtkPNGWriter>::New();
            pngWriter->SetFileName(filename.c_str());
            pngWriter->SetInputData(depthImage);
            pngWriter->Write();
        }

        void printMatrix(vtkMatrix4x4* matrix) {
        std::cout << "Matrix contents:" << std::endl;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                std::cout << matrix->GetElement(i, j) << " ";
            }
            std::cout << std::endl;
        }
    }

    vtkSmartPointer<vtkPoints> convertDepthTo3D(vtkSmartPointer<vtkFloatArray> depthValues, vtkSmartPointer<vtkCamera> camera) {

        // Create a vtkPoints object to store the 3D world coordinates
        vtkSmartPointer<vtkPoints> worldCoordinates = vtkSmartPointer<vtkPoints>::New();

        // Get camera parameters
        double nearClip = camera->GetClippingRange()[0];
        double farClip = camera->GetClippingRange()[1];
        double cameraPosition[3];
        camera->GetPosition(cameraPosition);

        
        
        // Compute the aspect ratio
        double aspect = static_cast<double>(this->width) / static_cast<double>(this->height);

   
        // Loop through the depth values
        for (int y = 0; y < this->height; ++y) {
            for (int x = 0; x < this->width; ++x) {

                // Calculate the index into the depthValues array
                int index = y * this->width + x;

                // Access the depth value at the current pixel
                float depthValue = depthValues->GetValue(index);

                // cull unreflected depths
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
                vtkMatrix4x4* inverseProjectionMatrix = vtkMatrix4x4::New();
                vtkMatrix4x4::Invert(camera->GetProjectionTransformMatrix(aspect, 0, 1), inverseProjectionMatrix);
                inverseProjectionMatrix->MultiplyPoint(homoCoordinates, homoCoordinates);

                // Convert to Cartesian coordinates in view space
                for (int i = 0; i < 4; ++i) {
                    homoCoordinates[i] /= homoCoordinates[3];
                }

                // Convert view coordinates to world coordinates
                vtkMatrix4x4* inverseModelViewMatrix = vtkMatrix4x4::New();
                vtkMatrix4x4::Invert(camera->GetModelViewTransformMatrix(), inverseModelViewMatrix);
                inverseModelViewMatrix->MultiplyPoint(homoCoordinates, homoCoordinates);

                // Insert point into worldCoordinates
                worldCoordinates->InsertNextPoint(homoCoordinates[0], homoCoordinates[1], homoCoordinates[2]);
            }
        }

    return worldCoordinates;
}


    pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointCloud(vtkSmartPointer<vtkPoints> worldPoints) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Reserve space for the points in the point cloud
    cloud->points.resize(worldPoints->GetNumberOfPoints());

    // Iterate through the VTK points and convert to PCL points
    for (vtkIdType i = 0; i < worldPoints->GetNumberOfPoints(); ++i) {
        double* point = worldPoints->GetPoint(i);
        pcl::PointXYZ pclPoint;
        pclPoint.x = static_cast<float>(point[0]);
        pclPoint.y = static_cast<float>(point[1]);
        pclPoint.z = static_cast<float>(point[2]);
        cloud->points[i] = pclPoint;
    }

    return cloud;
}

    pcl::PointCloud<pcl::PointXYZ>::Ptr depthDataToPointcloud(vtkSmartPointer<vtkFloatArray> depthData,  vtkSmartPointer<vtkCamera> camera) {
        // Create a new point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Set the cloud size
        cloud->width = width;
        cloud->height = height;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        // Get camera parameters from vtkCamera
        double focalPoint[3];
        camera->GetFocalPoint(focalPoint);
        double position[3];
        camera->GetPosition(position);

        // Calculate focal lengths and optical center
        double fx = focalPoint[0] - position[0];
        double fy = focalPoint[1] - position[1];
        double cx = width / 2.0;
        double cy = height / 2.0;

        // Convert depth data to point cloud
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float depthValue = depthData->GetValue(y * width + x);
                
                // Skip invalid depth values
                if (depthValue <= 0) {
                    continue;
                }
                
                // Calculate the 3D coordinates of the point
                pcl::PointXYZ& point = cloud->points[y * width + x];
                point.z = depthValue;
                point.x = (x - cx) * depthValue / fx;
                point.y = (y - cy) * depthValue / fy;
            }
        }
        
        return cloud;
    }

    vtkSmartPointer<vtkPolyData> convertPointCloudToVtk(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

        for (auto& point : cloud->points) {
            points->InsertNextPoint(point.x, point.y, point.z);
            
        }

        polydata->SetPoints(points);
        return polydata;
    }


    void addPointCloudToRenderer(vtkSmartPointer<vtkPoints> worldPoints) {
        // Create a vtkPolyData object to hold the point cloud
        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(worldPoints);

        // Create a vtkVertexGlyphFilter to render points as squares
        vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vertexGlyphFilter->SetInputData(polyData);

        // Create a mapper and actor to display the points
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(vertexGlyphFilter->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetPointSize(2.0); // Adjust the point size as needed
        actor->GetProperty()->SetColor(0.0, 0.0, 1.0); // Set the color to blue (R=0, G=0, B=1)

        // Add the actor to the renderer
        renderer->AddActor(actor);
    }

    
    void RemoveBackFacingTriangles(vtkSmartPointer<vtkCamera> viewpoint, std::vector<int>& visibleTriangles, vtkFloatArray* cellNormals) {
            
        std::vector<int> backFacingTriangles;

        for (int index : visibleTriangles) {
            double normal[3];
            cellNormals->GetTuple(index, normal);

            vtkCell* cell = polydata->GetCell(index);
            double point[3];
            cell->GetPoints()->GetPoint(0, point);  // Taking the first point of the triangle for illustration

            double viewVector[3];
            for (int j = 0; j < 3; ++j) {
                viewVector[j] = viewpoint->GetPosition()[j] - point[j];
            }

            double dotProduct = normal[0] * viewVector[0] + normal[1] * viewVector[1] + normal[2] * viewVector[2];
            
            if (dotProduct <= 0) {
                backFacingTriangles.push_back(index);
            }
        }

        // Remove back-facing triangles from visibleTriangles
        std::vector<int> newVisibleTriangles;
        std::unordered_set<int> backFacingSet(backFacingTriangles.begin(), backFacingTriangles.end());
        
        for (int index : visibleTriangles) {
            if (backFacingSet.find(index) == backFacingSet.end()) {
                newVisibleTriangles.push_back(index);
            }
        }

        // Update visibleTriangles
        visibleTriangles = newVisibleTriangles;
    }


    void RemoveOutsideFrustumTriangles(vtkSmartPointer<vtkCamera> viewpoint, std::vector<int>& visibleTriangles) {
        
        double planes[24];
        double aspect = static_cast<double>(width) / static_cast<double>(height);
        viewpoint->GetFrustumPlanes(aspect, planes);

        auto it = visibleTriangles.begin();
        while (it != visibleTriangles.end()) {
            int index = *it;
            
            vtkCell* cell = polydata->GetCell(index);
            bool isOutside = false;

            for (int i = 0; i < 6; ++i) { // 6 planes in the frustum
                double* plane = &planes[i * 4]; // Each plane has 4 coefficients
                isOutside = true; // Assume outside until proven inside for each plane
                
                for (int j = 0; j < 3; ++j) { // 3 vertices in a triangle
                    double point[3];
                    cell->GetPoints()->GetPoint(j, point);
                    double distance = plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3];
                    if (distance >= 0) {
                        isOutside = false; // One point was inside this plane, so not fully outside
                        break;
                    }
                }

                if (isOutside) break; // Fully outside one plane, so exit loop
            }

            if (isOutside) {
                it = visibleTriangles.erase(it); // Remove if outside frustum
            } else {
                ++it;
            }
        }
    }

    void computeTrianglesWithPointCloud(vtkSmartPointer<vtkPoints> worldPoints, std::vector<int>& visibleTriangles){
         // Extract points from the pre-calculated centroidPolyData
        vtkSmartPointer<vtkPoints> centroidPoints = centroidPolyData->GetPoints();
        // Check if the points object is valid

        // Initialize a K-d tree and set its input points as centroids
        vtkSmartPointer<vtkKdTreePointLocator> kdTree = vtkSmartPointer<vtkKdTreePointLocator>::New();
        kdTree->SetDataSet(centroidPolyData);
        kdTree->BuildLocator();

        // List to keep track of visible centroid indices
        std::vector<int> visibleCentroidIndices;

        // Search for each worldPoint in the K-d tree
        for (vtkIdType i = 0; i < worldPoints->GetNumberOfPoints(); ++i) {
            double testPoint[3];
            worldPoints->GetPoint(i, testPoint);

            // Find the index of the nearest centroid point
            vtkIdType id = kdTree->FindClosestPoint(testPoint);

            // Get the coordinates of the nearest centroid point
            double closestPoint[3];
            centroidPoints->GetPoint(id, closestPoint);

            // Calculate the distance between the worldPoint and the closest centroid
            double distance = sqrt(vtkMath::Distance2BetweenPoints(testPoint, closestPoint));

            // Check if the distance is within the specified threshold
            if (distance <= 1e-02) {
                // Add the index of the visible centroid to the list
                visibleCentroidIndices.push_back(id);
            }
        }

        // Final filtering step: Remove any triangles that don't have visible centroids
        std::vector<int> finalVisibleTriangles;
        for (int triangleIndex : visibleTriangles) {
            if (std::find(visibleCentroidIndices.begin(), visibleCentroidIndices.end(), triangleIndex) != visibleCentroidIndices.end()) {
                finalVisibleTriangles.push_back(triangleIndex);
            }
        }
        visibleTriangles = finalVisibleTriangles;
    }


    std::vector<int> compute_visibility(vtkSmartPointer<vtkCamera> viewpoint, vtkSmartPointer<vtkPoints> worldPoints, int viewpoint_index) {
        
        // list of visible triangles
        std::vector<int> visibleTriangles(polydata->GetNumberOfCells());

        // Insert and initialize row for the current viewpoint in visibility matrix
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
 

        // Code for compiling list based on previous steps
   

        return visibleTriangles;
    }


    void ColorVisibleTriangles(const std::vector<std::vector<int>> visibility_mat) {
        // Get the sum of each column from visibility_mat
        std::vector<int> columnSums = get_visibility_sum();

        // Create a vtkUnsignedCharArray to store the colors
        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);  // Three components per color for RGB
        colors->SetName("TriangleColors");

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

            colors->InsertNextTupleValue(color);
        }

        // Add the color data to the PolyData
        this->polydata->GetCellData()->SetScalars(colors);

        // Update the mapper
        this->mapper->SetInputData(this->polydata);
        this->mapper->Update();
    }


    void visualizeScene() {
        // Visualize the scene
        renderer->AddActor(actor);
        renderWindow->AddRenderer(renderer);

        // Set the render window size
        renderWindow->SetSize(1280, 720);

        vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
        renderWindowInteractor->SetRenderWindow(renderWindow);


         // Create the axes actor for the world coordinate frame
        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
        
        // Create orientation marker
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

        renderWindowInteractor->Start();
    }

    void addCameraFrustumToRenderer(vtkSmartPointer<vtkCamera> camera) {
        double planes[24];
        
        double aspect = static_cast<double>(width) / static_cast<double>(height);
        camera->GetFrustumPlanes(aspect, planes);

        vtkSmartPointer<vtkPlanes> frustumPlanes = vtkSmartPointer<vtkPlanes>::New();
        frustumPlanes->SetFrustumPlanes(planes);

        vtkSmartPointer<vtkFrustumSource> frustumSource = vtkSmartPointer<vtkFrustumSource>::New();
        frustumSource->ShowLinesOn();
        frustumSource->SetPlanes(frustumPlanes);
        
        frustumSource->Update();

        vtkSmartPointer<vtkPolyData> frustum = vtkSmartPointer<vtkPolyData>::New();
        frustum->ShallowCopy(frustumSource->GetOutput());

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(frustum);

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetRepresentationToWireframe();
        // Disable lighting for this actor
        actor->GetProperty()->LightingOff();
        // Add the frustum outline to the renderer
        renderer->AddActor(actor);
    }

private:

    vtkSmartPointer<vtkSTLReader> stlReader;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;

    vtkSmartPointer<vtkCamera> current_viewpoint;
    std::vector<vtkSmartPointer<vtkCamera>> viewpoints;

    int width, height;

    vtkPolyData* polydata;
    vtkSmartPointer<vtkPoints> centroids;
    vtkSmartPointer<vtkFloatArray> centroidNormals;
    vtkSmartPointer<vtkPolyData> centroidPolyData;

    std::vector<std::vector<int>> visibilityMatrix;

    // FOR TESTING
    vtkSmartPointer<vtkMatrix4x4> projection_matrix;

};

int main(int argc, char **argv) {


    ros::init(argc, argv, "view_planning_node");

    // Initialize the ViewPlanning class
    ViewPlanning vp;

    // Load the STL model. Replace this with your own model path.
    std::string package_path = ros::package::getPath("depth_processing");
    std::string stl_file_path = package_path + "/saved_models/cube_3072.stl";
    vp.loadModel(stl_file_path);
    vp.RandomizeTriangleColors();

    /// Create the three viewpoints
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1.0);
    double random1 = dis(gen) - 0.5;
    double random2 = dis(gen) - 0.5;
    double random3 = dis(gen) - 0.5;

    vp.createViewpoint(0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.1, 0.75, 45.0);
    vp.createViewpoint(0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.1, 0.75, 45.0);
    vp.createViewpoint(0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.1, 0.75, 45.0);

    auto viewpoints = vp.get_viewpoints();
    std::vector<std::vector<int>> visibility_mat;
    vtkSmartPointer<vtkPoints> aggregatedWorldPoints = vtkSmartPointer<vtkPoints>::New();
    // Create a PNG writer and save the image
    int index = 0;
    for(auto& viewpoint : viewpoints){

        // Show frustum of camera
        vp.addCameraFrustumToRenderer(viewpoint);

        // Take an RGB image
        std::string image_output1 = package_path + "/images/viewpoint" + std::to_string(index) + ".png";
        vp.captureImageOffscreen(image_output1, viewpoint);

        // Calculate depth data
        vtkSmartPointer<vtkFloatArray> depthData = vp.capture_depth_data(viewpoint);
  

        // Save depth data as image
        std::string filename = package_path + "/images/depth_image" + std::to_string(index) + ".png";
        vp.save_depth_image(depthData, filename);

        vtkSmartPointer<vtkPoints> worldPoints = vp.convertDepthTo3D(depthData, viewpoint);
        std::vector<int> visibleTriangles = vp.compute_visibility(viewpoint, worldPoints, index);

       
       


        // Append points to the aggregatedWorldPoints
        for(vtkIdType i = 0; i < worldPoints->GetNumberOfPoints(); ++i) {
            double point[3];
            worldPoints->GetPoint(i, point);
            aggregatedWorldPoints->InsertNextPoint(point);
        }

       ++index;
    }

    visibility_mat = vp.get_visibility_matrix();

    vp.ColorVisibleTriangles(visibility_mat);

    // vp.addPointCloudToRenderer(aggregatedWorldPoints);
       
    // Loop through the matrix and print it with proper formatting
    for (const std::vector<int>& row : visibility_mat) {
        for (int value : row) {
            std::cout << std::left << std::setw(5) << value ;
        }
        std::cout << "\n"; // Use "\n" for a newline
    }
    
    
    vp.visualizeScene();

    return 0;

}
