// TODO:
// Dynamically resize the path planning grid to fit the goal location?

#include <vector>
#include <limits>
#include <queue>
#include <algorithm>
#include <math.h>
#include "path_planner.h"

using namespace std;

// Convert degrees (in the format that we get raw from the GPS) into radians
double degreesToRadians(int degrees, double arcMinutes, bool positive)
{
    double totalDegrees = degrees + (arcMinutes/60.0);
    if (!positive)
    {
        totalDegrees *= -1;
    }

    return totalDegrees / 180.0 * PI;
}

// Convert from raw gps structure to the useful internal representation (radians)
GpsInfo convertGpsInfoToRadians(const RawGpsInfo& info)
{
    double latitude = degreesToRadians(info.latDegrees, info.latArcMinutes, info.north);
    double longitude = degreesToRadians(info.lonDegrees, info.lonArcMinutes, info.east);
    return GpsInfo(latitude, longitude);
}

// Returns the distance (metres) and bearing (radians) to get *from* a *to* b
// Bearing of 0 is north, positive means turn due east, negative means turn due west
// Uses: http://www.movable-type.co.uk/scripts/latlong.html
// Uses the equirectangular approximation, since we're talking about extremely small distances
void gpsDelta(const GpsInfo& a, const GpsInfo& b, double& metres, double& bearing)
{
    // There might be some wrap-around issues here, but, like, we're not going to be anywhere
    // near Greenwich, so we should be fine :)
    // PS these values are in radians, and are measured relative to the equator for latitude and
	// relative to the prime meridian for longitude. +ve means due north and due east.
    double latDelta = b.latitude - a.latitude;
    double lonDelta = b.longitude - a.longitude;
    double averageLat = (a.latitude + b.latitude) / 2;

    const double radiusOfEarth = 6371000; // metres

    double x = latDelta; // This is actually up-down, but in the context of our north-is-zero coordinate system, it makes most sense to call this the x axis
    double y = lonDelta * cos(averageLat);

    metres = sqrt(x*x + y*y) * radiusOfEarth;

    bearing = atan2(y, x);
}

// Expand the obstacles in the given grid by the given 'amount'
void expandObstacles(nav_msgs::OccupancyGrid& obstacles, int amount)
{
    cout << "expanding amount is: " << amount << endl;
    // Get the size of the grid we are working with.
    int xSize = obstacles.info.width;
    int ySize = obstacles.info.height;

    // We're essentially going to do a breadth first search (BFS) starting from all of
    // the obstacles, and continuing for 'amount' steps. All of the cells that we find
    // are within 'amount' cells of the obstacles, and so should also be made into
    // obstacles.
    vector<Point> thisLevel;
    vector<Point> nextLevel;

    // First find all of the obstacles
    for (int x = 0; x < xSize; x++)
    {
        for (int y = 0; y < ySize; y++)
        {
            if (at(obstacles, x, y) != 0)
            {
                thisLevel.push_back(Point(x, y));
            }
        }
    }

    // Next do 'amount' steps of BFS
    for (int dist = 0; dist < amount; dist++)
    {
        // Iterate over all of the points that we found in the previous iteration (thisLevel)
        vector<Point>::iterator it = thisLevel.begin();
        vector<Point>::iterator end = thisLevel.end();
        for (; it != end; ++it)
        {
            Point p = *it;

            // Now iterate over this point's neighbours
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    // Don't consider this point a neighbour of itself :)
                    if (i == 0 && j == 0) continue;

                    Point n(p.x + i, p.y + j);

                    // Ignore points that are outside the map
                    if (n.x < 0 || n.y < 0 || n.x >= xSize || n.y >= ySize) continue;

                    // This neighbour is within 'amount' cells of an obstacles. If not already
                    // an obstacles, we will make it an obstacle.
                    if (at(obstacles, n.x, n.y) == 0)
                    {
                        at(obstacles, n.x, n.y) = occupancyGridOccupied;

                        // Add this to the set of points to explore in the next iteration of BFS
                        nextLevel.push_back(n);
                    }
                }
            }
        }

        // Prepare for the next iteration
        thisLevel = nextLevel;
        nextLevel.clear();
    }
}


// Calculate the distance between each cell in the grid and the nearest obstacle
// Note that this function considers "one cell" in any direction (including diagonals)
// the same distance. Which isn't exactly true, but it's Good Enough.
Map2D<int> getDistanceMap(const nav_msgs::OccupancyGrid& obstacles, int limit)
{
    int xSize = obstacles.info.width;
    int ySize = obstacles.info.height;

    // This is the map which will contain the distances.
    Map2D<int> distMap(xSize, ySize, maxInt);

    // Do a breadth first search (BFS) around all of the obstacles.
    vector<Point> thisLevel;
    vector<Point> nextLevel;

    for (int x = 0; x < xSize; x++)
    {
        for (int y = 0; y < ySize; y++)
        {
            // Initialize the BFS with all of the obstacles
            if (at(obstacles, x, y) != 0)
            {
                thisLevel.push_back(Point(x, y));
            }
        }
    }

    // Only calculate distances up to the limit distance. If a cell is greater than 'limit'
    // away from the nearest obstacle, then we just say it's "far enough", and we don't care.
    for (int dist = 0; dist <= limit; dist++)
    {
        // Iterate over everything in this level
        vector<Point>::iterator it = thisLevel.begin();
        vector<Point>::iterator end = thisLevel.end();
        for (; it != end; ++it)
        {
            Point p = *it;

            // Ignore points that are outside the map
            if (p.x < 0 || p.y < 0 || p.x >= xSize || p.y >= ySize)
            {
                continue;
            }

            // Update the current point's distance if applicable
            if (dist < distMap.get(p.x, p.y))
            {
                distMap.set(p.x, p.y, dist);
                
                // Now iterate over this point's neighbours
                for (int i = -1; i <= 1; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        // Don't consider this point a neighbour of itself :)
                        if (i == 0 && j == 0) continue;

                        // Add neighbour to the set of points to explore next iteration of BFS
                        nextLevel.push_back(Point(p.x + i, p.y + j));
                    }
                }
            }
        }

        // Prepare for next iteration
        thisLevel = nextLevel;
        nextLevel.clear();
    }

    return distMap;
}

// Generate a map containing the cost of entering each cell in the map
// This cost can be based on distance to nearest obstacle
Map2D<double> getCostMap(Map2D<int> distMap)
{
    int xSize = distMap.getXSize();
    int ySize = distMap.getYSize();

    Map2D<double> costMap(xSize, ySize, 0);

    // Just iterate over the distance map and calculate the corresponding cost for each cell
    for (int x = 0; x < xSize; x++)
    {
        for (int y = 0; y < ySize; y++)
        {
            int dist = distMap.get(x, y);
            double cost = 0;

            if (useCostFunction)
            {
            	// Inverse square cost function
                // if (dist != 0 && dist != maxInt)
                // {
                //
                //     cost = 300.0 / (dist*dist);
                // }

            	// Simple fixed cost for getting within a fixed distance of obstacles
                if (dist <= 4)
                {
                    cost = 10.0;
                }
            }
            
            costMap.set(x, y, cost);
        }
    }

    return costMap;
}

geometry_msgs::PointStamped createPointMessage(const Point& p)
{
    geometry_msgs::PointStamped msg;
    msg.header.frame_id = "/map";
    msg.point.x = p.x;
    msg.point.y = p.y;

    return msg;
}

// Convert a Path (which is just a simple vector of points) into a ROS
// nav_msgs::Path message which can then be published
nav_msgs::Path createPathMessage(const Path& path, const nav_msgs::OccupancyGrid::ConstPtr& referenceMap)
{
    nav_msgs::Path msg;
    msg.header.frame_id = (referenceMap ? referenceMap->header.frame_id : "/map");
    double scale = (referenceMap ? referenceMap->info.resolution : 1);
    double xOffset = (referenceMap ? referenceMap->info.origin.position.x : 0);
    double yOffset = (referenceMap ? referenceMap->info.origin.position.y : 0);
    
    for (Path::const_iterator it = path.begin(); it != path.end(); it++)
    {
        const Point& p = *it;
        
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = (referenceMap ? referenceMap->header.frame_id : "/map");
        poseStamped.pose.position.x = p.x * scale + xOffset;
        poseStamped.pose.position.y = p.y * scale + yOffset;

        msg.poses.push_back(poseStamped);
    }

    return msg;
}

// At the end of A*, we need to reconstruct the path by working backwards from the end point
Path reconstructPath(Map2D<Point>& cameFrom, Point start, Point end)
{
    Path path;
    
    // Start at the end point, work backwards
    Point current = end;
    path.push_back(current);

    // Iterate until we get to the start point
    while (current != start)
    {
        // We are currently in the 'current' grid cell. How did the robot get here?
        // Let's check the cameFrom map to see!
        current = cameFrom.get(current.x, current.y);
        path.push_back(current);
    }

    // Flip the path since we reconstructed it in reverse
    reverse(path.begin(), path.end());

    return path;
}

Path smoothPath(const Path& in)
{
	int lookahead = 20;
	Path out;

	// Bail early if we don't have enough points to do this properly!
	if (in.size() <= lookahead) return in;

	double x = in[0].x;
	double y = in[0].y;
	int goalX = in.back().x;
	int goalY = in.back().y;
	int i = lookahead;
	while ((x-goalX)*(x-goalX) + (y-goalY)*(y-goalY) > 1)
	{
		double deltaX = in[i].x - x;
		double deltaY = in[i].y - y;

		double dist = sqrt(deltaX*deltaX + deltaY*deltaY);
		double normalX = deltaX / dist;
		double normalY = deltaY / dist;

		double minStepSize = 0.2;
		double maxStepSize = sqrt(2);
		double stepSize = dist - (lookahead * 0.8);
		stepSize = max(stepSize, minStepSize);
		stepSize = min(stepSize, maxStepSize);

		x += normalX * stepSize;
		y += normalY * stepSize;

		out.push_back(Point(x, y));
		if (i < in.size() - 1) i++;
	}

	cout << "Input path size: " << in.size() << endl;
	cout << "Output path size: " << out.size() << endl;

	return out;
}

void initializeOccupancyGrid(nav_msgs::OccupancyGrid& grid, int width, int height, int value)
{
    grid.header.frame_id = "/map";
    grid.info.width = width;
    grid.info.height = height;
    grid.info.resolution = 1;
    grid.data.resize(grid.info.width * grid.info.height);

    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            at(grid, i, j) = value;
        }
    }
}


// The A* implementation
Path doAStar(Map2D<double>& costMap, const nav_msgs::OccupancyGrid& obstacleMap, Point start, const Goal& goal)
{
    int xSize = costMap.getXSize();
    int ySize = costMap.getYSize();

    // Initialize some data structures we'll use to keep track of stuff
    Map2D<bool> visited(xSize, ySize, false);
    Map2D<double> cumulativeScore(xSize, ySize, infinity);
    Map2D<double> estimatedTotalScore(xSize, ySize, infinity);
    Map2D<Point> cameFrom(xSize, ySize);
    scoredQueue pq;

    // Add the first point (the starting point)
    cumulativeScore.set(start.x, start.y, 0);
    double estimatedScore = goal.getHeuristicDistanceFrom(start);
    estimatedTotalScore.set(start.x, start.y, estimatedScore);
    pq.push(scoredPoint(estimatedScore, start));

    // Iterate until the priority queue is empty (or until we find a path and break out early)
    while (!pq.empty())
    {
        // Pop the point with the lowest score off the queue
        scoredPoint sp = pq.top();
        pq.pop();
        Point p = sp.second;

        // Check if we've visited this node before, and if so skip it
        if (visited.get(p.x, p.y))
        {
            continue;
        }

        // Okay, we haven't seen this point before. Let's visit it now.
        visited.set(p.x, p.y, true);

        // Check if we're at the goal -- if so, reconstruct the path and return it
        if (goal.contains(p))
        {
            return reconstructPath(cameFrom, start, p);
        }

        // Look up the cost to get to this point so far
        double cScore = cumulativeScore.get(p.x, p.y);

        // Alright, find this point's neighbours
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                Point neighbour(p.x + i, p.y + j);

                // Don't consider the current point a neighbour of itself, obviously
                if (neighbour == p)
                {
                    continue;
                }

                // Don't consider anything that's off the map
                if (neighbour.x < 0 || neighbour.y < 0 || neighbour.x >= xSize || neighbour.y >= ySize)
                {
                    continue;
                }

                // Don't consider anything that's an obstacle
                if (at(obstacleMap, neighbour.x, neighbour.y) != 0)
                {
                    continue;
                }

                // Don't consider anything that's already been visited
                if (visited.get(neighbour.x, neighbour.y))
                {
                    continue;
                }

                // Calculate this neighbour's tentative new score if we were to travel to it from the current point
                bool isOnDiagonal = (abs(i) == 1 && abs(j) == 1);
                double distanceCost = (isOnDiagonal ? sqrt(2) : 1.0);
                double obstacleCost = costMap.get(neighbour.x, neighbour.y);
                double tentativeCScore = cScore + distanceCost + obstacleCost;

                // If this new score is better than anything we've seen for this neighbour,
                // then update the neighbour
                if (tentativeCScore < cumulativeScore.get(neighbour.x, neighbour.y))
                {
                    cumulativeScore.set(neighbour.x, neighbour.y, tentativeCScore);
                    double estimatedScore = tentativeCScore + goal.getHeuristicDistanceFrom(neighbour);
                    estimatedTotalScore.set(neighbour.x, neighbour.y, estimatedScore);
                    cameFrom.set(neighbour.x, neighbour.y, p);
                    pq.push(scoredPoint(estimatedScore, neighbour));
                }
            }
        }
    }
         
    // Huh? We couldn't find a way to get to the goal! Just return an empty path.
    return Path();
}

// Add fake obstacles to the map so that the rover doesn't think it can drive through its blind spots
// The blinders will look like this:
// ***********A******C
// ******************
// *****************
// ****************
// ***************
// **************
// *************
// ***********B    R -->> (rover facing this direction)
// *************
// **************
// ***************
// ****************
// *****************
// ******************
// *******************
// Where R is the rover, * are the blinder obstacles, and A, B, C are also blinder obstacles
// that are just labeled so that we can refer to them.
// The blinders are parameterized by two numbers:
//    -displacement, which is the distance from B to R
//    -angleD, which is the angle ABC (in degrees).
void addBlinders(nav_msgs::OccupancyGrid& g, Point currentPosition, double angleD, int displacement)
{
	// Figure out where the blinders start (i.e. coordinates of point B)
    int startX = currentPosition.x - displacement;
    int startY = currentPosition.y;
    
    // Convert angleD to radians.
    double angleR = angleD * PI / 180;

    // Find the "slope" of the blinders ... this is the distance AC divided by the distance AB
    double slope = tan(angleR);

    // Iterate over every row in the grid
    for (int y = 0; y < g.info.height; y++)
    {
    	// In this row, we're going to add blinder obstacles from the leftmost column (column # 0)
    	// up until some end point (column # endX). Let's calculate what endX should be
        int deltaY = abs(y - startY);
        double deltaX = deltaY * slope;
        int endX = startX + deltaX;

        // Make sure endX is in the grid (i.e. in the range [0, g.info.width])
        if (endX < 0) endX = 0;
        if (endX > g.info.width) endX = g.info.width;
        
        // Actually add the obstacles
        for (int x = 0; x < endX; x++)
        {
            at(g, x, y) = occupancyGridOccupied;
        }
    }
}

vector<Point> getJaggedPoints(const Path& path) {
	vector<Point> jagged;
	int lookahead = 5;
	for (int i = lookahead; i < path.size() - lookahead; i++) {
		Point behind = path[i - lookahead];
		Point ahead = path[i + lookahead];
		Point current = path[i];

		Point cb(behind.x - current.x, behind.y - current.y);
		Point ca(ahead.x - current.x, ahead.y - current.y);

		// dot product
		double dotProduct = cb.x * ca.x + cb.y * ca.y;
		double cosAngle = dotProduct / sqrt(cb.x*cb.x + cb.y*cb.y) / sqrt(ca.x*ca.x + ca.y*ca.y);
		double angle = acos(cosAngle) * 180 / PI;

		double okay = 180 - 45;

		if (angle < okay) {
			jagged.push_back(current);
			cout << "Jagged at: " << current.x << ", " << current.y << endl;
		}
	}



	return jagged;
}

Path removeJaggedPoints(
		const Path& path,
		Map2D<double>& costMap,
		nav_msgs::OccupancyGrid& g,
		Point currentPosition,
		const Goal& adjustedGoal) {
	cout << "Computing jagged points" << endl;
	vector<Point> jagged = getJaggedPoints(path);
	cout << "Adding little obstacles" << endl;
	for (int i = 0; i < jagged.size(); i++)
	{
		for (int dx = -5; dx <= 5; dx++) {
			for (int dy = -5; dy <= 5; dy++) {
				double distance = sqrt(dx*dx + dy*dy);
				if (distance <= 5) {
					cout << "Trying to set coordinate " << jagged[i].x + dx << ", " << jagged[i].y + dy << endl;
					at(g, jagged[i].x + dx, jagged[i].y + dy) = 100;
				}
			}
		}
	}
	cout << "Running AStar again..." << endl;
	Path betterPath = doAStar(costMap, g, currentPosition, adjustedGoal);
	cout << "Done!" << endl;
	return betterPath;
}

// Calculate the distance map and then the cost map
// Then do A*
Path findPath(nav_msgs::OccupancyGrid& g, Point currentPosition, const Goal& goal)
{
    if (goal.isOutOfBounds(g))
    {
        cout << "Oops, goal (type " << goal.getGoalType() << ") was outside the occupancy grid! Forget it ..." << endl;
        Path emptyPath;
        return emptyPath;
    }
    // Make all of the obstacles a little bit bigger than they
    // actually are so we don't go near them.
    cout << "Expanding obstacles ..." << endl;
    expandObstacles(g, obstacleExpansionDistance);
    // The distance map associates each cell in the grid with a
    // number telling us how close the nearest obstacle is.
    cout << "Computing distance map .." << endl;
    Map2D<int> distanceMap = getDistanceMap(g);
    // The cost map tells us how much it "costs" to drive through
    // each cell of the grid. A* will find the lowest cost path.
    cout << "Computing cost map ..." << endl;
    Map2D<double> costMap = getCostMap(distanceMap);

    if (useBlinders)
    {
    	// The rover has blind spots. This will add fake obstacles into those blind spots
    	// so that the rover doesn't think it can drive through the blind spots.
        // NOTE: blinders must be added *after* doing obstacle expansion and cost map,
        // because we don't want to expand the blinders or associate a cost with being
        // near them. We just don't want the rover to think it can get around any obstacle
        // by driving through its blind spots :)
        cout << "Adding blinders ..." << endl;
        addBlinders(g, currentPosition, blindersAngleD, blindersDisplacement);
    }

    Path path;
    
    // If the current goal point is inside an obstacle, we'll never be able to get to it!
    // Jiggle it around until it's outside
    Goal* adjustedGoal = goal.clone();
    if (adjustedGoal->isInsideObstacles(g))
    {
        cout << "Trying to remove goal from obstacles ..." << endl;
        adjustedGoal->moveOutsideOfObstacles(currentPosition, g);
    }

    // Check if we goal is actually outside obstacle now
    if (adjustedGoal->isInsideObstacles(g))
    {
        // Oh noes! Still inside an obstacle!
        cout << "Couldn't remove goal from obstacles! Bailing!" << endl;
    }
    else
    {
    	// Okay, this is the meat. Actually run the A* algorithm
        cout << "Running A* ..." << endl;
        path = doAStar(costMap, g, currentPosition, *adjustedGoal);

        // Now let's try to smooth the path in one of two ways.
        // Uncomment one of these two lines.
	// Update: currently disabled both types of smoothing
        // path = smoothPath(path);
        //path = removeJaggedPoints(path, costMap, g, currentPosition, *adjustedGoal);
    }
    
    delete adjustedGoal;
    return path;
}
    
// For a robot located in cell 'origin' of a grid, with grid parameters 'params',
// where robot is located at gps location robotLocation, and goal is goalLocation, and
// positive y direction (up) on the grid is assumed to be in the direction the robot is
// facing, and assuming that this direction is given by 'bearing' in radians east of north,
// then this function returns the grid coordinates of the goal point.
// Assume that when we say the robot is at the point origin, we mean that it is centred in
// this cell of the grid. And we want to find the cell in which the goal point lies.
Point getGoalPoint(Point& origin, conversionParams& params, const GpsInfo& robotLocation, const GpsInfo& goalLocation, double bearing)
{
    // Find the distance and bearing from the current gps location to the goal point
    double gpsDeltaDistance, gpsDeltaBearing;
    gpsDelta(robotLocation, goalLocation, gpsDeltaDistance, gpsDeltaBearing);

    // Take into account the fact that the robot is not necessarily facing north
    gpsDeltaBearing -= bearing;

    // Calculate how far up and to the right of the robot's current location the goal lies
    double metresUp = cos(gpsDeltaBearing) * gpsDeltaDistance;
    double metresRight = sin(gpsDeltaBearing) * gpsDeltaDistance;

    int cellsUp = (int)(round(metresUp / params.cellSize));
    int cellsRight = (int)(round(metresRight / params.cellSize));

    return Point(origin.x + cellsRight, origin.y + cellsUp);
}

// Use the given conversion parameters to go from distances (metres) stored in p
// to an actual cell index in the grid
Point toMap(const geometry_msgs::Point32 p, const conversionParams& params)
{
    int x = (int)((p.x / params.cellSize) + params.xOffset);
    int y = (int)((p.y / params.cellSize) + params.yOffset);

    return Point(x, y);
}    

// http://stackoverflow.com/questions/9296059/read-pixel-value-in-bmp-file
// Draws the path that your function calculated onto the input image. Essentially a way
// to visualize the path. You must specify a blank input image to read, the output image,
// and the path. This function will essentially copy the input image onto the output
// image, but will superimpose the path on top in black.
// Why do you need to specify a blank input image??
// Because I couldn't be bothered to figure out the file format of bitmaps beyond what I needed
// to do this, so I don't actually know how to write a complete proper bitmap file header :)
// So I need a template file to copy it from
void addObstaclesAndPathToMap(const char* inputFile, const char* outputFile, const nav_msgs::OccupancyGrid& m, const vector<geometry_msgs::Point32>& psFloat, Path& path, const conversionParams& params)
{
    vector<Point> ps;

    // First, just iterate over all of the input points specified  (which should be in distance format) and
    // convert them into actual grid cell format
    for (int i = 0; i < psFloat.size(); i++)
    {
        ps.push_back(toMap(psFloat[i], params));
    }

    // Open files
    FILE* in = fopen(inputFile, "rb");
    if (!in)
    {
        throw cantReadFile;
    }
    FILE* out = fopen(outputFile, "wb");
    if (!out)
    {
        throw cantReadFile;
    }

    // Read the header
    unsigned char header[54];
    fread(header, sizeof(unsigned char), 54, in);
    
    // Extract width & height
    int width = *((int*)(header + 18));
    int height = *((int*)(header + 22));

    // Width and height of template file should match width and height of the map we're given
    if (width != m.info.width || height != m.info.height)
    {
        return;
    }

    // 3 bytes per pixel. Each row aligned to 4-byte boundary, so do this weird bit-shifting trick
    // to round up to the nearest multiple of 4.
    int bytesPerRow = (width*3 + 3) & (~3);

    // Now let's create a temporary map, and use it to store which cells are in the path
    Map pathCells(width, height);
    for (int i = 0; i < path.size(); i++)
    {
        Point p = path[i];        
        if (p.x >= pathCells.getXSize() || p.y >= pathCells.getYSize())
        {
            cout << "Hmmm ... your path seems to be too big to fit onto this bitmap!" << endl;
        }
        //cout << "Path point: (" << (int)p.x << ", " << (int)p.y << ")" << endl;
        pathCells.set(p.x, p.y, true);
    }

    // Copy the header verbatim into the output file
    fwrite(header, sizeof(unsigned char), 54, out);

    // Now iterate over the input image and copy it into the output image along with the path
    for (int y = 0; y < height; y++)
    {
        // Read a row from the input image
        unsigned char row[bytesPerRow];
        fread(row, sizeof(unsigned char), bytesPerRow, in);

        // Iterate over each pixel in the row
        for (int x = 0; x < width; x++)
        {
            if (std::find(ps.begin(), ps.end(), Point(x, y)) != ps.end())
            {
                // If this pixel is a one of the points, set it to red
                row[x*3] = 0; // blue
                row[x*3 + 1] = 0; // green
                row[x*3 + 2] = 255; // red
            }
            else if (pathCells.get(x, y))
            {
                // If this pixel is in the path, set it to blue
                row[x*3] = 255; // blue
                row[x*3 + 1] = 0; // green
                row[x*3 + 2] = 0; // red
            }
            else if (at(m, x, y) != 0)
            {
                // If this pixel is an obstacle, set it black
                row[x*3] = 0; // blue
                row[x*3 + 1] = 0; // green
                row[x*3 + 2] = 0; // red
            }
        }

        // Spit the row back out to the output image
        fwrite(row, sizeof(unsigned char), bytesPerRow, out);
    }

    fclose(in);
    fclose(out);
}

// Return by reference the width and height of the bitmap specified by filename
// Must be a 24-bit bitmap file format
void getSizeOfBitmap(const char* filename, int& width, int& height)
{
    FILE* f = fopen(filename, "rb");
    if (!f)
    {
        throw cantReadFile;
    }

    // Read the header
    unsigned char header[54];
    fread(header, sizeof(unsigned char), 54, f);
    
    // Extract width & height
    width = *((int*)(header + 18));
    height = *((int*)(header + 22));
}

// Callback for when we receive data from the GPS
// Assumes that the message is sent as a point, and the latitude
// is stored in radians in the x coordinate of the point, and the
// longitude is stored in radians in the y coordinate of the point.
// Yes, major hack, I know.
void gpsCallback(const geometry_msgs::Point32::ConstPtr& gpsLatLon)
{
    // Just set the global "current gps location"
    globalGpsLocation.latitude = gpsLatLon->x;
    globalGpsLocation.longitude = gpsLatLon->y;
}

// Callback for when we receive data from the IMU
void imuCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // Just set the global "current imu bearing". This is a number (in radians) which measures
	// how far east we are pointing compared to due north. So 0 means we are pointing north.
	// +ve pi/2 means we are pointing east. -ve pi/2 means west. Etc.
    globalBearing = msg->data;
    ROS_INFO("NOW AT BEARING: %f", globalBearing);
}

// Callback for when we receive processed data about obstacles from the 
// camera.
void camCallbackOccupancyGrid(const nav_msgs::OccupancyGrid::Ptr& m)
{
    int width = m->info.width;
    int height = m->info.height;

    // The occupancy grid data is in cells. We want to be able to convert between # of cells
    // and a physical length in metres. Construct conversion params which tells us how to
    // convert between metres and cells, and also useful information like how many cells are
    // in the grid and where the "origin" on the grid should be. We take the origin to be the
    // point where the robot is located. Since this obstacle map is always constructed relative
    // to the robot, the robot will always be in the centre of the map
    double x = m->info.width / 2 + 0.5; // x value of the "origin", where the robot is
    double y = m->info.height / 2 + 0.5; // y value of the "origin", where the robot is
    Point currentCell((int)x, (int)y);
    conversionParams mapServerParams(
    		x, // x value of the "origin" (in cells)
    		y, // y value of the "origin" (in cells)
    		1.0, // number of metres per cell (this should probably be smaller!)
    		height, // height of the grid (in cells)
    		width // width of the grid (in cells)
    );

    // Calculate where the goal point is on the grid
    // This is the GPS way of getting the goal point. Only basic unit testing, no integration tests.
    // Essentially we feed in information about the robot's currently position, and where we would
    // like to end up, and this function will tell us what cell in the map we should set as our
    // goal point.
    Point goalCell = getGoalPoint(
    		currentCell, // where (in cells) is the robot currently located?
    		mapServerParams, // how to convert between cells and metres?
    		globalGpsLocation, // where (in gps latitude and longitude) is the robot located?
    		globalGpsGoalLocation, // where (in gps latitude and longitude) are we trying to get to?
    		globalBearing // what direction is the robot facing (in radians east of due north)?
    );

    // Do A* n' stuff
    Path path = findPath(*m, currentCell, PointGoal(goalCell.x, goalCell.y));

    // Publish the resulting path we found
    pathPublisher.publish(createPathMessage(path));

    // These next publishing are just for the purposes of visualization, so we can more easily
    // see what the algorithm is doing

    // Publish a scaled version of the path that will render better
    rvizPathPublisher.publish(createPathMessage(path, m));

    // Publish start & end points
    currentPointPublisher.publish(createPointMessage(currentCell));
    goalPointPublisher.publish(createPointMessage(goalCell));

    // The findPath call may have "expanded" the obstacles in the map to give itself some buffer room.
    // Let's republish the map so that we can see what changed.
    cout << "Publishing expanded obstacles" << endl;

    // Stupid test
    for (Path::const_iterator it = path.begin(); it != path.end(); ++it) {
      at(*m, it->x, it->y) = 50;
    }

    mapPublisher.publish(*m);
}

// Goal methods

Goal::~Goal()
{
	// Nothing to do!
}

// PointGoal methods

bool PointGoal::isInsideObstacles(const nav_msgs::OccupancyGrid& map) const
{
    return (at(map, x, y) != 0);
}

bool PointGoal::isOutOfBounds(const nav_msgs::OccupancyGrid& map) const
{
    return (x >= map.info.width ||
            y >= map.info.height ||
            x < 0 ||
            y < 0);
}

void PointGoal::moveOutsideOfObstacles(const Point& currentPosition, const nav_msgs::OccupancyGrid& g)
{
    double deltaX = x - currentPosition.x;
    double deltaY = y - currentPosition.y;
    double distance = sqrt(deltaX*deltaX + deltaY*deltaY);

    for (double d = distance; d > 0; d -= 1.0)
    {
        double potentialX = currentPosition.x + deltaX * (d / distance);
        double potentialY = currentPosition.y + deltaY * (d / distance);

        if (at(g, potentialX, potentialY) == 0)
        {
            // Update myself and GTFO!
            x = potentialX;
            y = potentialY;
            return;
        }
    }

    // Ah! couldn't do anything! Goal will remain unchanged
}

string PointGoal::getGoalType() const
{
    return "Point";
}

double PointGoal::getHeuristicDistanceFrom(const P<int>& start) const
{
    // Return euclidean distance between start and this goal
    double dx = x - start.x;
    double dy = y - start.y;
    return sqrt(dx*dx + dy*dy);
}

bool PointGoal::contains(const P<int>& p) const
{
    return p.x == x && p.y == y;
}

PointGoal* PointGoal::clone() const
{
    return new PointGoal(*this);
}

PointGoal::~PointGoal()
{
	// Nothing to do!
}

// http://stackoverflow.com/questions/9296059/read-pixel-value-in-bmp-file
// Call this function and pass in the filename of a bitmap you want to read.
// Will return a map of obstacles that is read from the bitmap.
// You must also pass in two points. This function will set them to whatever
// the current location and goal point are in the bitmap.
nav_msgs::OccupancyGrid readMapFromBitmap(const char* filename, Point& currentLocation, Point& goal)
// Obstacles should be red (RGB 255, 0, 0)
// Current location should be blue (RGB 0, 0, 255)
// Goal should be green (RGB 0, 255, 0)
{
    FILE* f = fopen(filename, "rb");
    if (!f)
    {
        throw cantReadFile;
    }

    // Read the header
    unsigned char header[54];
    fread(header, sizeof(unsigned char), 54, f);
    
    // Extract width & height
    int width = *((int*)(header + 18));
    int height = *((int*)(header + 22));

    cout << "Reading a bitmap that is " << width << " by " << height << endl;

    // Creat the map
    nav_msgs::OccupancyGrid map;
    initializeOccupancyGrid(map, width, height, 0);

    // 3 bytes per pixel. Each row aligned to 4-byte boundary, so do this weird bit-shifting trick
    // to round up to the nearest multiple of 4.
    int bytesPerRow = (width*3 + 3) & (~3);

    for (int y = 0; y < height; y++)
    {
        // Read in a row at a time
        unsigned char row[bytesPerRow];
        fread(row, sizeof(unsigned char), bytesPerRow, f);

        // Iterate over each pixel in the row
        for (int x = 0; x < width; x++)
        {
            // Extract R,G,B values from the pixel
            unsigned char blue = row[x*3];
            unsigned char green = row[x*3 + 1];
            unsigned char red = row[x*3 + 2];

            // Check if the pixel is an obstacle
            if (red == 255 && green == 0 && blue == 0)
            {
                at(map, x, y) = occupancyGridOccupied;
            }
            // Check if the pixel is the current location
            else if (blue == 255 && red == 0 && green == 0)
            {
                currentLocation.x = x;
                currentLocation.y = y;
            }
            // Check if the pixel is the goal
            else if (green == 255 && red == 0 && blue == 0)
            {
                goal.x = x;
                goal.y = y;
            }
        }
    }

    fclose(f);

    return map;

}

// Draws the path that your function calculated onto the input image. Essentially a way
// to visualize the path. You must specify the input image to read, the output image,
// and the path. This function will essentially copy the input image onto the output
// image, but will superimpose the path on top in black.
void addPathToMap(const char* inputFile, const char* outputFile, Path& path, Goal& goal)
{
    // Open files
    FILE* in = fopen(inputFile, "rb");
    if (!in)
    {
        throw cantReadFile;
    }
    FILE* out = fopen(outputFile, "wb");
    if (!out)
    {
        throw cantReadFile;
    }

    // Read the header
    unsigned char header[54];
    fread(header, sizeof(unsigned char), 54, in);
    
    // Extract width & height
    int width = *((int*)(header + 18));
    int height = *((int*)(header + 22));

    // 3 bytes per pixel. Each row aligned to 4-byte boundary, so do this weird bit-shifting trick
    // to round up to the nearest multiple of 4.
    int bytesPerRow = (width*3 + 3) & (~3);

    cout << "Adding a path to a bitmap that is " << width << " by " << height << endl;

    // Now let's create a temporary map, and use it to store which cells are in the path
    Map2D<bool> pathCells(width, height);
    for (int i = 0; i < path.size(); i++)
    {
        Point p = path[i];        
        if (p.x >= pathCells.getXSize() || p.y >= pathCells.getYSize())
        {
            cout << "Hmmm ... your path seems to be too big to fit onto this bitmap!" << endl;
        }
        pathCells.set(p.x, p.y, true);
    }

    // Copy the header verbatim into the output file
    fwrite(header, sizeof(unsigned char), 54, out);

    // Now iterate over the input image and copy it into the output image along with the path
    for (int y = 0; y < height; y++)
    {
        // Read a row from the input image
        unsigned char row[bytesPerRow];
        fread(row, sizeof(unsigned char), bytesPerRow, in);

        // Iterate over each pixel in the row
        for (int x = 0; x < width; x++)
        {
            if (pathCells.get(x, y))
            {
                // If this pixel is in the path, set it to black
                row[x*3] = 0; // blue
                row[x*3 + 1] = 0; // green
                row[x*3 + 2] = 0; // red
            }
            // else if the pixel is white (background), let's recolour it!
            else if (row[x*3] == 255 && row[x*3 + 1] == 255 && row[x*3 + 2] == 255)
            {
                P<int> p(x, y);
                if (goal.contains(p))
                {
                    // If this pixel is in the goal, set it to purple
                    row[x*3] = 255; // blue
                    row[x*3 + 1] = 0; // green
                    row[x*3 + 2] = 255; // red
                }
                else
                {
                    // Else color this pixel some shade of cyan depending on how far away from goal
                    row[x*3] = 255; // blue
                    row[x*3 + 1] = min(255, (int)(goal.getHeuristicDistanceFrom(p) * 2)); // green
                    row[x*3 + 2] = 0; // red
                }
            }
        }

        // Spit the row back out to the output image
        fwrite(row, sizeof(unsigned char), bytesPerRow, out);
    }

    fclose(in);
    fclose(out);
}

int test_path_planner_main(int argc, char** argv)
{
    if (argc != 3)
    {
        cout << "Oops, not right number args" << endl;
        exit(-1);
    }
    
    const char* input = argv[1];
    const char* output = argv[2];

    Point currentLocation;
    Point goalPoint;
    nav_msgs::OccupancyGrid map = readMapFromBitmap(input, currentLocation, goalPoint);

    Goal* goal = new PointGoal(goalPoint.x, goalPoint.y);
    
    Path path = findPath(map, currentLocation, *goal);

    addPathToMap(input, output, path, *goal);

    delete goal;

    return 0;
}

// Replace main with this function to interactively test the GPS conversion.
int test_convert_main(int argc, char** argv)
{
    while (!cin.eof())
    {
        int a, b, c;
        string d;

        cout << "Start: ";
        cin >> a >> b >> c >> d;
        cout << "Saw: " << a << ", " << b << ", " << c << ", " << d << endl;

        RawGpsInfo rawStart;
        rawStart.latDegrees = a;
        rawStart.latArcMinutes = b + (c / 60.0);
        rawStart.north = (d == "N");

        cin >> a >> b >> c >> d;
        cout << "Saw: " << a << ", " << b << ", " << c << ", " << d << endl;

        rawStart.lonDegrees = a;
        rawStart.lonArcMinutes = b + (c / 60.0);
        rawStart.east = (d == "E");

        cout << "End: ";
        cin >> a >> b >> c >> d;
        cout << "Saw: " << a << ", " << b << ", " << c << ", " << d << endl;

        RawGpsInfo rawEnd;
        rawEnd.latDegrees = a;
        rawEnd.latArcMinutes = b + (c / 60.0);
        rawEnd.north = (d == "N");

        cin >> a >> b >> c >> d;
        cout << "Saw: " << a  << ", " << b << ", " << c << ", " << d << endl;

        rawEnd.lonDegrees = a;
        rawEnd.lonArcMinutes = b + (c / 60.0);
        rawEnd.east = (d == "E");

        double metres, bearing;
        GpsInfo start = convertGpsInfoToRadians(rawStart);
        GpsInfo end = convertGpsInfoToRadians(rawEnd);
        gpsDelta(start, end, metres, bearing);

        cout << "Distance (m): " << metres << endl;
        cout << "Bearing: " << (int)(180 * bearing / PI) << " deg " << ((180 * bearing / PI) - (int)(180 * bearing / PI)) * 60 << " arcmin" << endl;
    }

    return 0;                
}

int ros_main(int argc, char** argv)
{
    // Setup this ros node
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;

    // Subscribe to cam data
    ros::Subscriber camSub = n.subscribe("art/map/local", 1, camCallbackOccupancyGrid);

    //Subscribe to GPS data
    // We only need the most recent one, right? So queue size of 1 is okay?
    ros::Subscriber gpsSub = n.subscribe("gps_data", 1, gpsCallback);

    //Subscribe to IMU data
    // We only need the most recent one, right? So queue size of 1 is okay?
    ros::Subscriber imuSub = n.subscribe("imu_data", 1, imuCallback);

    // Initialize publisher to publish the path that we find
    pathPublisher = n.advertise<nav_msgs::Path>("path", 1000);
    rvizPathPublisher = n.advertise<nav_msgs::Path>("rviz_path", 1000);

    // The rest of the publishers are to output data that's helpful for visualization,
    // and is not required for the algorithm to run.

    // Initialize publisher to publish the map that we build
    mapPublisher = n.advertise<nav_msgs::OccupancyGrid>("art/map/localexpanded", 1000);

    // Initialize publishers to publish goal and current points
    currentPointPublisher = n.advertise<geometry_msgs::PointStamped>("currentPoint", 1000);
    goalPointPublisher = n.advertise<geometry_msgs::PointStamped>("goalPoint", 1000);

    // Initialize publihser to publish visuzlizations of line data from vision
    linesPublisher = n.advertise<visualization_msgs::Marker>("visionLines", 1000);

    // Tell ROS to process any incoming messages (like the path we just published!)
    ros::spin();

    return 0;
}


int main(int argc, char** argv)
{
    ros_main(argc, argv);
    //test_path_planner_main(argc, argv);
    //test_convert_main(argc, argv);
}
