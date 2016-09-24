#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

// NOTE: I tried to sort of organize declarations by type here (i.e. class declarations
// are in a different section from function prototypes or whatever, but because of
// various dependencies of things and because of the fact that I put class method
// definitions here instead of in the .cpp file, basically I had to fudge things a
// little in some cases. All that to say, if you have trouble adding a new declaration
// in the "right" place, don't feel bad about just forgetting the organization and
// putting it wherever the damn compiler wants you to put it :)

// HEADERS
#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <visualization_msgs/Marker.h>

using namespace std;

// MACROS

#define at(m,x,y) ((m).data[(x) + (y)*(m).info.width])

// STRUCTS
    
// structure to store gps info
// latitude is in radians north of the equator (negative for south)
// longitude is in radians east of the prime meridian (negative for west)
struct GpsInfo
{
    double latitude;
    double longitude;

    GpsInfo(double lat, double lon)
        : latitude(lat), longitude(lon)
    { }
};

// structure to store gps info in a way that matches up almost exactly with
// the way that we get it raw from the GPS
// We will convert this to GpsInfo internally because it's easier to work with
struct RawGpsInfo
{
    int latDegrees;
    double latArcMinutes;
    int lonDegrees;
    double lonArcMinutes;
    bool north;
    bool east;
};

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

struct Vec
{
    double x, y;
    Vec(double xVal, double yVal) : x(xVal), y(yVal) { }
    Vec operator-(const Vec& rhs) const { return Vec(x - rhs.x, y - rhs.y); }
    Vec operator+(const Vec& rhs) const { return Vec(x + rhs.x, y + rhs.y); }
    Vec operator*(double scalar) const { return Vec(x * scalar, y * scalar); }
    double dot(const Vec& rhs) const { return x*rhs.x + y*rhs.y; }
    double scalarProjectOnto(const Vec& rhs) const { return (this->dot(rhs) / rhs.dot(rhs)); }
    Vec projectOnto(const Vec& rhs) const { return rhs * (this->dot(rhs) / rhs.dot(rhs)); }
    double magnitude() const { return sqrt(x*x + y*y); }
};

// EXCEPTIONS

// This is an exception that gets thrown when you try to access a cell in the
// map that doesn't exist.
class OutOfBoundsException : public exception
{
    virtual const char* what() const throw()
    {
        return "Oops, you tried to access a Map grid cell with invalid indices!";
    }
};

// This is an exception that gets thrown when you try to read a file that
// doesn't exist (or that you don't have access to, or whatever)
class cantReadFileException : public exception
{
    virtual const char* what() const throw()
    {
        return "Oops, can't open the file you specified!";
    }
};

// CONSTANTS

const double PI = 3.14159265359;
const int occupancyGridOccupied = 100; // A constant that represents that an occupancy grid cell has an obstacle in it
const OutOfBoundsException outOfBounds;
const cantReadFileException cantReadFile;
const int maxInt = numeric_limits<int>::max();
const double infinity = numeric_limits<double>::infinity();
const GpsInfo globalGpsGoalLocation(-10.0e-6, 10.0e-6); // Our gps goal location. For now it's preprogrammed
//const GpsInfo globalGpsGoalLocation(0.0, 10.0e-6); // Our gps goal location. For now it's preprogrammed
const bool useCostFunction = true;
const int obstacleExpansionDistance = 2; // how many cells to expand obstacles
const bool useBlinders = true;
const double blindersAngleD = -30;
const int blindersDisplacement = 5;
const double reachedGoalDistance = 0.35; // in metres

// TYPEDEFS

typedef geometry_msgs::Point32 Pfloat;

// CLASSES

// A point (x, y) of type T
template <class T>
class P
{
public:
    P()
    {
        x = 0;
        y = 0;
    }
    P(T xInput, T yInput)
    {
        x = xInput;     
        y = yInput;
    }

    bool operator==(const P& other) const
    {
        return other.x == x && other.y == y;
    }

    bool operator!=(const P& other) const
    {
        return !((*this) == other);
    }
                
    T x, y;
};

// A 2D Map of things of type T
template <class T>
class Map2D
{
public:
    // When you build a map, you must specify the x and y size.
    // Maps cannot be resized after being created.
    Map2D(int x, int y)
    {
        xSize = x;
        ySize = y;
        data = vector<T>(xSize * ySize);
    }

    // Optionally, specify a default value to initialize everything in the grid
    Map2D(int x, int y, T val)
    {
        xSize = x;
        ySize = y;
        data = vector<T>(xSize * ySize, val);
    }
                
    // Get the value at the given location
    T get(int x, int y)
    {
        if (x < xSize && x >= 0 && y < ySize && y >= 0)
        {
            return data[(xSize * y) + x];
        }
        else
        {
            throw outOfBounds;
        }
    }
                
    // Set the value at the given location
    void set(int x, int y, T val)
    {
        if (x < xSize && x >= 0 && y < ySize && y >= 0)
        {
            data[(xSize * y) + x] = val;
        }
        else
        {
            throw outOfBounds;
        }
    }
                
    // How big is the map in the x direction?
    int getXSize()
    {
        return xSize;
    }
                
    // How big is the map in the y direction?
    int getYSize()
    {
        return ySize;
    }
                
private:
    vector<T> data;
    int xSize;
    int ySize;
};

// Abstract base class for goals that A* can plan towards. Implementations are below.
// Note, there's currently only one implementation, PointGoal (which is when the goal is a single
// point). But there were others before, and there may be others in the future.
class Goal
{
public:
	// If this goal is inside of an obstacle, try to move it so that it's outside
    virtual void moveOutsideOfObstacles(const P<int>& currentPosition, const nav_msgs::OccupancyGrid& map) = 0;

    // Check if this goal is outside of the bounds of the given map
    virtual bool isOutOfBounds(const nav_msgs::OccupancyGrid& map) const = 0;

    // Check if the goal is inside of an obstacle
    virtual bool isInsideObstacles(const nav_msgs::OccupancyGrid& map) const = 0;

    // Get a string representing what type of goal this is (e.g. PointGoal, LineGoal)
    virtual string getGoalType() const = 0;

    // Get the heuristic distance from the given point to this goal. Used in A*.
    virtual double getHeuristicDistanceFrom(const P<int>& start) const = 0;

    // Check if the given point p is inside this goal
    virtual bool contains(const P<int>& p) const = 0;

    // Allocate and return a copy of this goal. The caller now owns this copy and is
    // responsible for deleting it when they are done.
    virtual Goal* clone() const = 0;

    // Destructor. Does nothing.
    virtual ~Goal();
};

class PointGoal : public Goal
{
public:
    PointGoal(int xVal, int yVal) : x(xVal), y(yVal) { }
    
    virtual void moveOutsideOfObstacles(const P<int>& currentPosition, const nav_msgs::OccupancyGrid& g);
    virtual bool isInsideObstacles(const nav_msgs::OccupancyGrid& map) const;
    virtual bool isOutOfBounds(const nav_msgs::OccupancyGrid& map) const;
    virtual string getGoalType() const;
    virtual double getHeuristicDistanceFrom(const P<int>& start) const;
    virtual bool contains(const P<int>& p) const;
    virtual PointGoal* clone() const;
    virtual ~PointGoal();

private:
    int x;
    int y;
};

// TYPEDEFS THAT DEPEND ON CLASS DEFINITIONS

typedef P<int> Point;
typedef Map2D<bool> Map;
typedef pair<double, Point> scoredPoint; // Used for A*. A point and an associated score.
typedef vector<Point> Path;

// Used for A*. We keep a priority queue of points to visit.
// Lower score => higher priority.
// Thus lower priority => higher score.
// We provide the following class as a 'less than' sorting operator
// for the queue to keep the class in order.
class PriorityLessThan
{
public:
    bool operator() (const scoredPoint& a, const scoredPoint& b)
    {
        return a.first > b.first;
    }
};

typedef priority_queue<scoredPoint, vector<scoredPoint>, PriorityLessThan> scoredQueue; // A* priority queue type

// FUNCTION PROTOTYPES

double degreesToRadians(int degrees, double arcMinutes, bool positive);
GpsInfo convertGpsInfoToRadians(const RawGpsInfo& info);
void gpsDelta(const GpsInfo& a, const GpsInfo& b, double& metres, double& bearing);
double getDistance(Point& a, Point& b);
Map2D<int> getDistanceMap(const nav_msgs::OccupancyGrid& obstacles, int limit = 30);
void expandObstacles(nav_msgs::OccupancyGrid& obstacles, int amount);
Map2D<double> getCostMap(Map2D<int> distMap);
geometry_msgs::Point32 convertPoint(const Point& p);
geometry_msgs::PointStamped createPointMessage(const Point& p);
nav_msgs::Path createPathMessage(const Path& path,
				 const nav_msgs::OccupancyGrid::ConstPtr& referenceMap = 
				 nav_msgs::OccupancyGrid::ConstPtr());
Path reconstructPath(Map2D<Point>& cameFrom, Point start, Point end);
Path doAStar(Map2D<double>& costMap, const nav_msgs::OccupancyGrid& obstacleMap, Point start, const Goal& end); // Important that Goal is a reference so that it acts polymorphically!
Path findPath(nav_msgs::OccupancyGrid& g, Point currentPosition, const Goal& goal); // Important that Goal is a reference so that it acts polymorphically!
void boundsCheck(int val1, int val2, int size);
Point getGoalPoint(Point& origin, conversionParams& params, const GpsInfo& robotLocation, const GpsInfo& goalLocation, double bearing);
Point toMap(const geometry_msgs::Point32 p, const conversionParams& params);
bool clipToWindow(Pfloat& start, Pfloat& end, int width, int height);
void fillOccupancyGrid(nav_msgs::OccupancyGrid& map, Pfloat start, Pfloat end, int fillVal, const conversionParams& params);
void addObstaclesAndPathToMap(const char* inputFile, const char* outputFile, const nav_msgs::OccupancyGrid& m, const vector<geometry_msgs::Point32>& psFloat, Path& path, const conversionParams& params);
void getSizeOfBitmap(const char* filename, int& width, int& height);
void initializeOccupancyGrid(nav_msgs::OccupancyGrid& grid, int width, int height, int value);
int test_convert_main(int argc, char** argv);

// ROS SUBSCRIBER CALLBACKS

void gpsCallback(const geometry_msgs::Point32::ConstPtr& gpsLatLon);
void imuCallback(const std_msgs::Float64::ConstPtr& msg);
void camCallbackOccupancyGrid(const nav_msgs::OccupancyGrid::Ptr& m);
void camCallbackLines(const geometry_msgs::Polygon::ConstPtr& msg);

// TEMPLATED FUNCTION DEFINITIONS

// The only purpose of this function is to be a method to easily create test paths in
// the source code.
// Say you want a path from (0, 0) to (30, 30) to (50, 50).
// You can write that as {{0,0}, {30,30}, {50,50}}
// And then pass that into this function to make it into Polygon message (which is what
// we're using as a method to pass paths between ROS nodes)
template <size_t N>
geometry_msgs::Polygon makePolygon( const double (&data)[N][2] )
{
    geometry_msgs::Polygon poly;
    vector<geometry_msgs::Point32>& v = poly.points;

    for (size_t i = 0; i < N; i++)
    {
        geometry_msgs::Point32 p;
        p.x = data[i][0];
        p.y = data[i][1];
        v.push_back(p);
    }
    
    return poly;
}

// GLOBALS

// These represent our current gps location and bearing
// These are updated by the gps and imu callbacks
// Bearing is measure in radians east of due north
GpsInfo globalGpsLocation(0.0, 0.0);
double globalBearing = degreesToRadians(70, 0.0, true); // imu yaw (radians)
geometry_msgs::Point globalPositionSLAM; // Used to represent where we currently are according to SLAM (since the last time we reset)
geometry_msgs::Point globalGoalPositionSLAM; // Used to represent where we currently are according to SLAM (since the last time we reset)
double globalBearingSLAM = 0; // Used to represent current yaw (radians) according to slam (relative to our orientation the last time we reset)
ros::Publisher pathPublisher; // Used to publish the paths that we find
ros::Publisher rvizPathPublisher; // A scaled path which will render better in rviz
ros::Publisher mapPublisher; // Used to publish the occupancy grid map that we build to represent our surroundings
ros::Publisher currentPointPublisher;
ros::Publisher goalPointPublisher;
ros::Publisher linesPublisher; // Used to publish the bird's-eye lines that we get from vision
ros::Publisher slamCommandsPublisher; // Used to communicate instructions (e.g. reset) to SLAM


#endif // PATH_PLANNER_H
