// Assume that the LIDAR publishes data as a PointCloud2 (we can modify the driver do the conversion)
// Assume that the LIDAR considers the rover to be at location (0,0,0) in the point cloud
// Assume that the point cloud data will always fit into the voxel grid
// Assume that the point cloud uses the following coordinate system: positive x is to the right, positive y is straight ahead, positive z is upwards
// Assume that the angle given from LIDAR is in radians
// Assume angle=0 corresponds to the Lidar pointing straight ahead; negative angle means turned left, positive means turned right

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

using namespace std;


// location of horizontal LIDAR (in voxel cells)
double hLidarX = 50;
double hLidarY = 50;
double hLidarZ = 0;

// size of a occupancy grid cell (in whatever units are used by the PointCloud2)
double mapResolution;

// size of the occupancy grid
unsigned int gSizeX = 100;
unsigned int gSizeY = 100;

const int CAM = 1 << 6; // 2^0
const int LIDAR = 1 << 7; // 2^1

double sickRange;

struct conversionParams
{
    // Assume cellSize is in metres / cell
    // Assume cells are square
    // Assume xOffset and yOffset are in cells
    //    (this seems to be more convenient, but it's easy enough to change)
    double xOffset, yOffset, cellSize;
    int mapHeight, mapWidth;

    conversionParams(double xO, double yO, double cS, int mH, int mW)
        : xOffset(xO), yOffset(yO), cellSize(cS),
          mapHeight(mH), mapWidth(mW)
    {  }
};

typedef geometry_msgs::Point32 Pfloat;

void lidarFillOccupancyGrid(nav_msgs::OccupancyGrid *grid, sensor_msgs::PointCloud *pc);
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void publishMap(ros::Publisher mapPublisher);
nav_msgs::OccupancyGrid * InitializeOccupancyGrid();
void camCallbackLines(const geometry_msgs::Polygon::ConstPtr& msg);
void camFillOccupancyGrid(nav_msgs::OccupancyGrid *map, Pfloat start, Pfloat end, const conversionParams& params);
bool clipToWindow(Pfloat& start, Pfloat& end, int width, int height);
void spoofLidar(ros::Publisher lidarPublisher);
    
nav_msgs::OccupancyGrid *grid;

// take each point in the pc and mark the correspondng occupancy grid cell as occupied  
void lidarFillOccupancyGrid(nav_msgs::OccupancyGrid *grid, sensor_msgs::PointCloud *pc) {
    for (size_t i = 0; i < pc->points.size(); i++) {
        int x = (unsigned int)(pc->points[i].x / mapResolution + (gSizeX / 2));
        int y = (unsigned int)(pc->points[i].y / mapResolution + (gSizeY / 2));
        if (x < gSizeX && x >= 0 && y < gSizeY && y >= 0)
        {
            grid->data[y * gSizeX + x] |= LIDAR;
        }
    }
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    cout << "GOT TEH LIDAR DATA" << endl;
    
    // clear the LIDAR data from the map
    int clearLidarMask = (~LIDAR);
    for (unsigned int i = 0; i < gSizeX; i++) {
        for (unsigned int j = 0; j < gSizeY; j++) {
            grid->data[j * gSizeX + i] &= clearLidarMask;
        }
    }
    
    // Project to a point cloud
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud cloud;
    projector.projectLaser(*msg, cloud);

    // update the grid with the new data from msg
    lidarFillOccupancyGrid(grid, &cloud);
}

void camCallbackLines(const geometry_msgs::Polygon::ConstPtr& msg) {
    cout << "GOT TEH CAM DATA" << endl;
    
    // clear the CAM data from the map
    int clearCamMask = (~CAM);
    for (unsigned int i = 0; i < gSizeX; i++) {
        for (unsigned int j = 0; j < gSizeY; j++) {
            grid->data[j * gSizeX + i] &= clearCamMask;
        }
    }

    // extract the list of points in the message
    const std::vector<geometry_msgs::Point32>& points = msg->points;

    // we expect each pair of points to represent a line segment
    // If there an odd number of points, there's something wrong, and ignore the message
    if (points.size() % 2 != 0) {
        std::cout << "Oops, ignoring incoming message because the number of points isn't even!" << std::endl;
        return;
    }

    int x = gSizeX / 2;
    int y = gSizeY / 2;
    conversionParams p(x, y, 1.0, gSizeY, gSizeX);
    
    // Based on the line segments we are given, actually fill the cells in the occupancy grid which correspond to obstacles
    for (int i = 0; i < points.size() / 2; i++) {
        camFillOccupancyGrid(grid, points[2 * i], points[2 * i + 1], p); 
    }
}

// Consider the line segment defined by start -> end
// Find all cells in the map which this segment touches, and set those cells to the value fillVal
// Note that this function expects that the given start and end points are expressed as distances,
// (i.e. if start = (5, 6) that means that the point is 5m right and 6m up of the origin)
// You need to pass in some conversion params to tell this function how to convert these distances
// to actual cell locations in the map
// PS- actually, the scale conversion already happened, but the offset didn't. So cellSize should be 1.0.
void camFillOccupancyGrid(nav_msgs::OccupancyGrid *map, Pfloat start, Pfloat end, const conversionParams& params) {
    start.x = (start.x / params.cellSize) + params.xOffset;
    end.x = (end.x / params.cellSize) + params.xOffset;
    start.y = (start.y / params.cellSize) + params.yOffset;
    end.y = (end.y / params.cellSize) + params.yOffset;

    int xStart = (int)start.x;
    int xEnd = (int)end.x;
    int yStart = (int)start.y;
    int yEnd = (int)end.y;

    bool success = clipToWindow(start, end, map->info.width, map->info.height);

    // Oops, the line was completely outside the map! There's nothing we can do, so just return
    if (!success)
    {
        std::cout << "Warning: you specified a line which was competely outside of the window." << std::endl;
        std::cout << "Skipping that line..." << std::endl;
        return;
    }

    // Special case when xStart == xEnd
    if (xStart == xEnd)
    {
        // Swap start and end so that yStart < yEnd
        if (yEnd < yStart)
        {
            int temp = yStart;
            yStart = yEnd;
            yEnd = temp;
        }

        // Just iterate over all y values touched by the line
        for (int y = yStart; y <= yEnd; y++)
        {
            map->data[y * gSizeX + xStart] &= CAM;
        }

        return;
    }

    // Okay, general case here

    // Swap start and end so that xStart < xEnd
    if (xEnd < xStart)
    {
        int temp = xStart;
        xStart = xEnd;
        xEnd = temp;

        temp = yStart;
        yStart = yEnd;
        yEnd = temp;
    
        Pfloat tempP = start;
        start = end;
        end = tempP;
    }
    
    double slope = (end.y - start.y) / (end.x - start.x);

    // For each x column that this line touches, we want to figure out which y cells in that column
    // are hit by the line
    for (int x = xStart; x <= xEnd; x++)
    {
        int xNext = x + 1;
        double xDelta = x - start.x;
        double xNextDelta = xNext - start.x;
    
        // Calculate the range of affected y cells for this x column
        int yStart = (int)((x == xStart) ? start.y : start.y + (xDelta*slope));
        int yEnd = (int)((x == xEnd) ? end.y : start.y + (xNextDelta*slope));
    
        // Flip start and end so that yStart < yEnd
        if (yEnd < yStart)
        {
            int temp = yStart;
            yStart = yEnd;
            yEnd = temp;
        }
        
        // Fill all of the affect y cells
        for (int y = yStart; y <= yEnd; y++)
        {
            map->data[y * gSizeX + x] &= CAM;
        }
    }       
}

// Check if the line defined by start -> end fits inside the width described by the given
// width and height. If not, then attempt to clip the line segment to show only the portion
// that fits inside the window.
// Returns true if the line fits in the window, or could be clipped to fit in the window.
// Returns false if the entire line lies outside the window.
// Assumes that the points start and end have already been converted from distances (metres)
// to cell numbers. So if start = (5, 6) that means the cell map[5][6], and *not* the points
// 5m right and 6m up of the origin.
bool clipToWindow(Pfloat& start, Pfloat& end, int width, int height)
{
    const double epsilon = 0.0001;

    // Clip x < 0
    if (start.x < 0 && end.x < 0)
    {
        return false;
    }
    else if (start.x < 0)
    {
        double slope = (end.y - start.y) / (end.x - start.x);
        double clipAmount = 0 - start.x;
        start.x = 0;
        start.y += clipAmount * slope;
    }
    else if (end.x < 0)
    {
        double slope = (start.y - end.y) / (start.x - end.x);
        double clipAmount = 0 - end.x;
        end.x = 0;
        end.y += clipAmount * slope;
    }

    // Clip y < 0
    if (start.y < 0 && end.y < 0)
    {
        return false;
    }
    else if (start.y < 0)
    {
        double slope = (end.x - start.x) / (end.y - start.y);
        double clipAmount = 0 - start.y;
        start.y = 0;
        start.x += clipAmount * slope;
    }
    else if (end.y < 0)
    {
        double slope = (start.x - end.x) / (start.y - end.y);
        double clipAmount = 0 - end.y;
        end.y = 0;
        end.x += clipAmount * slope;
    }

    // Clip x >= width
    // Note: since we're trying to clip so that x < width, we use a small value
    // epsilon, and clip so that x = width - epsilon
    if (start.x > (width - epsilon) && end.x > (width - epsilon))
    {
        return false;
    }
    else if (start.x > (width - epsilon))
    {
        double slope = (end.y - start.y) / (start.x - end.x);
        double clipAmount = start.x - (width - epsilon);
        start.x = width - epsilon;
        start.y += clipAmount * slope;
    }
    else if (end.x > (width - epsilon))
    {
        double slope = (start.y - end.y) / (end.x - start.x);
        double clipAmount = end.x - (width - epsilon);
        end.x = width - epsilon;
        end.y += clipAmount * slope;
    }

    // Clip y >= height
    // Note: since we're trying to clip so that y < width, we use a small value
    // epsilon, and clip so that y = height - epsilon
    if (start.y > (height - epsilon) && end.y > (height - epsilon))
    {
        return false;
    }
    else if (start.y > (height - epsilon))
    {
        double slope = (end.x - start.x) / (start.y - end.y);
        double clipAmount = start.y - (height - epsilon);
        start.y = height - epsilon;
        start.x += clipAmount * slope;
    }
    else if (end.y > (height - epsilon))
    {
        double slope = (start.x - end.x) / (end.y - start.y);
        double clipAmount = end.y - (height - epsilon);
        end.y = height - epsilon;
        end.x += clipAmount * slope;
    }

    return true;
}



// FOR TESTING PURPOSES
const double LIDAR_ANGLE_MIN = -3.14159;
const double LIDAR_ANGLE_MAX = 3.14159;
const double LIDAR_ANGLE_INCREMENT = 0.00872664619237;
const double LIDAR_TIME_INCREMENT = 3.6968576751e-05;
const double LIDAR_SCAN_TIME = 0.019999999553;
const double LIDAR_RANGE_MIN = 0.00999999977648;
const double LIDAR_RANGE_MAX = 20.0;
void spoofLidar(ros::Publisher lidarPublisher) {
    sensor_msgs::LaserScan scanMsg;

    //scanMsg.header.frame_id = "/laser";
    scanMsg.header.stamp = ros::Time::now();
    
    scanMsg.angle_min = LIDAR_ANGLE_MIN;
    scanMsg.angle_max = LIDAR_ANGLE_MAX;
    scanMsg.angle_increment = LIDAR_ANGLE_INCREMENT;
    
    scanMsg.time_increment = LIDAR_TIME_INCREMENT;
    
    scanMsg.scan_time = LIDAR_SCAN_TIME;

    scanMsg.range_min = LIDAR_RANGE_MIN;
    scanMsg.range_max = LIDAR_RANGE_MAX;

    const double ds = 5;
    for (double lidarAngle = LIDAR_ANGLE_MIN; lidarAngle < LIDAR_ANGLE_MAX; lidarAngle += LIDAR_ANGLE_INCREMENT) {
        scanMsg.ranges.push_back(ds);        
    }

    scanMsg.header.frame_id = "/actual-position";
    lidarPublisher.publish(scanMsg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "localmapserver");
    ros::NodeHandle n;
    
    ros::Publisher mapPublisher = n.advertise <nav_msgs::OccupancyGrid> ("art/map/local", 1000);

    ros::Subscriber hLidarSubscriber = n.subscribe <sensor_msgs::LaserScan> ("scan", 1, lidarCallback);

    // Subscribe to cam data
    // We only need the most recent one, right? So queue size of 1 is okay?
    // Switch which of these is commented out to change the method of passing
    // cam data to this module
    ros::Subscriber camSub = n.subscribe("cam_data", 1, camCallbackLines);

    // Load parameters
    n.param<double>("/art/setup/map_resolution", mapResolution, 0.05);
    n.param<double>("/art/setup/sensors/sick_max_range", sickRange, 20);

    // Setup map constants
    gSizeX = (1 / mapResolution) * sickRange;
    gSizeY = (1 / mapResolution) * sickRange;
    
    // Place the horizontal lidar in the middle of the map
    hLidarX = gSizeX / 2;
    hLidarY = gSizeY / 2;
    hLidarZ = 0;    

    grid = InitializeOccupancyGrid();

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        publishMap(mapPublisher);
        
        ros::spinOnce();

        loop_rate.sleep();
    }
}

nav_msgs::OccupancyGrid* InitializeOccupancyGrid() {
    nav_msgs::OccupancyGrid *map = new nav_msgs::OccupancyGrid();
        
    map->info.resolution = mapResolution;
    map->info.width = gSizeX;
    map->info.height = gSizeY;
    map->info.origin.position.x = -mapResolution * (gSizeX / 2);
    map->info.origin.position.y = -mapResolution * (gSizeY / 2);
    map->data.resize(map->info.width * map->info.height);
    map->header.frame_id = "/actual-position";
    
    return map;
}

void publishMap(ros::Publisher mapPublisher) {
    grid->header.stamp = ros::Time::now();
    //grid->header.frame_id = "/map";

    mapPublisher.publish(*grid);
}
