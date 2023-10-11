#ifndef VIEW_PLAN_H
#define VIEW_PLAN_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <random>
#include <cstdlib>
#include <ctime>
#include <unordered_set>
#include <iomanip>

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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
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
#include <vtkTransformFilter.h>
#include <vtkTransform.h>

class ViewPlanning {
public:
    // Constructor
    ViewPlanning();

    void loadModel(const std::string& stlFilePath);
    void computeTriangleAreas();
    void RandomizeTriangleColors();
    void createCamera(double posX, double posY, double posZ,
                  double focalX, double focalY, double focalZ);
    void createViewpoint(double posX, double posY, double posZ,
                        double focalX, double focalY, double focalZ,
                        double near, double far, double angle, bool showFrame = false);
    vtkSmartPointer<vtkCamera> get_current_viewpoint();
    vtkSmartPointer<vtkCamera> getLastViewpoint();
    std::vector<vtkSmartPointer<vtkCamera>> get_viewpoints();
    void removeLastViewpoint();
    void removeViewPoint(int index);
    std::vector<std::vector<int>> get_visibility_matrix();
    std::vector<int> get_visibility_sum();
    bool completeCoverage();
    float computeCoverageScore();
    void captureImage(const std::string& filename, vtkSmartPointer<vtkCamera> camera);
    void captureImageOffscreen(const std::string& filename, vtkSmartPointer<vtkCamera> camera);
    vtkSmartPointer<vtkFloatArray> capture_depth_data(vtkSmartPointer<vtkCamera> camera);
    void save_depth_image(vtkSmartPointer<vtkFloatArray> depthData, const std::string& filename);
    vtkSmartPointer<vtkPoints> convertDepthTo3D(vtkSmartPointer<vtkFloatArray> depthValues, vtkSmartPointer<vtkCamera> camera);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointCloud(vtkSmartPointer<vtkPoints> worldPoints);
    vtkSmartPointer<vtkPolyData> convertPointCloudToVtk(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void addPointCloudToRenderer(vtkSmartPointer<vtkPoints> worldPoints);
    void RemoveBackFacingTriangles(vtkSmartPointer<vtkCamera> viewpoint, std::vector<int>& visibleTriangles, vtkFloatArray* cellNormals);
    void RemoveOutsideFrustumTriangles(vtkSmartPointer<vtkCamera> viewpoint, std::vector<int>& visibleTriangles);
    void computeTrianglesWithPointCloud(vtkSmartPointer<vtkPoints> worldPoints, std::vector<int>& visibleTriangles);
    std::vector<int> compute_visibility(vtkSmartPointer<vtkCamera> viewpoint, vtkSmartPointer<vtkPoints> worldPoints, int viewpoint_index);
    void ColorVisibleTriangles(const std::vector<std::vector<int>> visibility_mat);
    void ColorOverlap(const std::vector<std::vector<int>> visibility_mat);
    void visualizeScene();
    void addCameraFrustumToRenderer(vtkSmartPointer<vtkCamera> camera);
    void showViewpointSolution(vtkSmartPointer<vtkCamera> camera, double scale);

private:
    vtkSmartPointer<vtkSTLReader> stlReader;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkCamera> current_viewpoint;
    std::vector<vtkSmartPointer<vtkCamera>> viewpoints;

    // vtkSmartPointer<vtkPolyDataMapper> offscreenMapper;
    // vtkSmartPointer<vtkActor> offscreenActor;
    // vtkSmartPointer<vtkRenderer> offscreenRenderer;
    // vtkSmartPointer<vtkRenderWindow> offscreenWindow;

    int width, height;
    vtkPolyData* polydata;
    vtkSmartPointer<vtkPoints> centroids;
    vtkSmartPointer<vtkFloatArray> centroidNormals;
    vtkSmartPointer<vtkPolyData> centroidPolyData;
    std::vector<std::vector<int>> visibilityMatrix;
    float totalModelArea;
};
#endif // VIEW_PLAN_H