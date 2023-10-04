#include <ros/ros.h>
#include <ros/package.h>

#include <vtkCamera.h> // Include for vtkCamera
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



class ViewPlanning {
public:
    ViewPlanning() {
        // Initialization
        stlReader = vtkSmartPointer<vtkSTLReader>::New();
        mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        actor = vtkSmartPointer<vtkActor>::New();
        renderer = vtkSmartPointer<vtkRenderer>::New();
        renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

        renderer->SetBackground(0.0, 0.0, 0.0);
        current_viewpoint = vtkSmartPointer<vtkCamera>::New();
        current_viewpoint->SetPosition(2, 2, 2);
        current_viewpoint->SetFocalPoint(0, 0, 0);
    }

    void loadModel(const std::string& stlFilePath) {
        // Load the STL file
        stlReader->SetFileName(stlFilePath.c_str());
        mapper->SetInputConnection(stlReader->GetOutputPort());
        actor->SetMapper(mapper);
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
                         double near, double far, double angle,
                         int width, int height) {
        vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
        
        // Set position, focal point, etc.
        camera->SetPosition(posX, posY, posZ);
        camera->SetFocalPoint(focalX, focalY, focalZ);
        camera->SetClippingRange(near, far);
        camera->SetViewAngle(angle);
        
        // Adding to vector
        viewpoints.push_back(camera);
        
        // If you also want to set aspect ratio, you can do that based on the width and height
        double aspectRatio = static_cast<double>(width) / static_cast<double>(height);
        camera->SetViewAngle(2.0 * atan(tan(angle * 3.14159 / 360.0) * aspectRatio) * 180.0 / 3.14159);
    }

    vtkSmartPointer<vtkCamera> get_current_viewpoint() {
        return current_viewpoint;
    }

    std::vector<vtkSmartPointer<vtkCamera>> get_viewpoints() {
        return viewpoints;
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

        // Compute aspect ratio from the camera's view angle
        double angle = camera->GetViewAngle();
        double aspectRatio = tan(angle * 3.14159 / 360.0);

        // Set the size of the offscreen renderer to match the aspect ratio
        int baseWidth = 1280; // Choose a base height that suits your needs
        int baseHeight = static_cast<int>(baseWidth * aspectRatio);
        offscreenWindow->SetSize(baseWidth, baseHeight);
        
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
        
        // Compute aspect ratio from the camera's view angle
        double angle = camera->GetViewAngle();
        double aspectRatio = tan(angle * 3.14159 / 360.0);

        // Set the size of the offscreen renderer to match the aspect ratio
        int baseWidth = 1280; // Choose a base height that suits your needs
        int baseHeight = static_cast<int>(baseWidth * aspectRatio);
        offscreenWindow->SetSize(baseWidth, baseHeight);

        // Render the scene off-screen
        offscreenWindow->Render();
        
        // Get the size of the render window
        int* size = offscreenWindow->GetSize();
        int width = size[0];
        int height = size[1];
        
        // Create a float array to store depth data
        vtkSmartPointer<vtkFloatArray> depthData = vtkSmartPointer<vtkFloatArray>::New();
        depthData->SetNumberOfComponents(1);
        depthData->SetNumberOfTuples(width * height);
        
        // Read the depth buffer
        offscreenWindow->GetZbufferData(0, 0, width - 1, height - 1, depthData->GetPointer(0));

        return depthData;
        
    }

    void save_depth_image(vtkSmartPointer<vtkFloatArray> depthData, int width, int height, const std::string& filename) {

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

    // vtkSmartPointer<vtkPolyData> depthDataToPointcloud(vtkSmartPointer<vtkFloatArray> depthData, int width, int height, vtkSmartPointer<vtkCamera> camera) {

    //     // Create an image data object to store the depth data
    //     vtkSmartPointer<vtkImageData> depthImage = vtkSmartPointer<vtkImageData>::New();
    //     depthImage->SetDimensions(width, height, 1);
    //     depthImage->AllocateScalars(VTK_FLOAT, 1);

    //     // Copy depth data to image data object
    //     for (int y = 0; y < height; ++y) {
    //         for (int x = 0; x < width; ++x) {
    //             float depthValue = depthData->GetValue(y * width + x);
    //             depthImage->SetScalarComponentFromFloat(x, y, 0, 0, depthValue);
    //         }
    //     }

    //     // Create vtkDepthImageToPointCloud object
    //     vtkSmartPointer<vtkDepthImageToPointCloud> depthToPointCloud = vtkSmartPointer<vtkDepthImageToPointCloud>::New();
    //     depthToPointCloud->SetInputData(depthImage);
    //     depthToPointCloud->SetCamera(camera);
    //     depthToPointCloud->Update();

    //     // Get the point cloud as vtkPolyData
    //     vtkSmartPointer<vtkPolyData> pointCloud = depthToPointCloud->GetOutput();

    //     return pointCloud;
    // }


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

        renderWindowInteractor->Start();
    }

    void addCameraFrustumToRenderer(vtkSmartPointer<vtkCamera> camera) {
        double planes[24];
        camera->GetFrustumPlanes(1.0, planes);

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
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "view_planning_node");

    // Initialize the ViewPlanning class
    ViewPlanning vp;

    // Load the STL model. Replace this with your own model path.
    std::string package_path = ros::package::getPath("depth_processing");
    std::string stl_file_path = package_path + "/saved_models/cube.stl";
    vp.loadModel(stl_file_path);

    // Create a camera
    // vp.createCamera(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
    // Visualize the scene
       /// Create the three viewpoints
    vp.createViewpoint(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.1, 3.0, 45.0, 1280, 720);
    // vp.createViewpoint(0.0, 2.0, 2.0, 0.0, 0.0, 0.0, 0.1, 3.0, 45.0, 1280, 720);
    // vp.createViewpoint(-1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.1, 3.0, 45.0, 1280, 720);

    auto viewpoints = vp.get_viewpoints();

    // Create a PNG writer and save the image
    int index = 0;
    for(auto& viewpoint : viewpoints){

        // Show frustum of camera
        vp.addCameraFrustumToRenderer(viewpoint);

        // Calculate depth data
        vtkSmartPointer<vtkFloatArray> depthData = vp.capture_depth_data(viewpoint);

        // Save depth data as image
        std::string filename = package_path + "/images/depth_image" + std::to_string(index) + ".png";
        vp.save_depth_image(depthData, 1280, 720, filename);


        // vtkSmartPointer<vtkPolyData> pointCloud = depthDataToPointcloud(depthData, 1280, 720, viewpoint)
        ++index;
    }

    
    vp.visualizeScene();

    // Capture an image
     // Load the STL model. Replace this with your own model path.
    std::string image_output = package_path + "/images/screenshot.png";

    // Capture images from the viewpoints
    std::string image_output1 = package_path + "/images/viewpoint1.png";
    std::string image_output2 = package_path + "/images/viewpoint2.png";
    std::string image_output3 = package_path + "/images/viewpoint3.png";

    
    vp.captureImageOffscreen(image_output1, vp.get_viewpoints()[0]);
    vp.captureImageOffscreen(image_output2, vp.get_viewpoints()[1]);
    vp.captureImageOffscreen(image_output3, vp.get_viewpoints()[2]);


    return 0;
}
