#include "Clusters.hpp"

using namespace cv;
using namespace std;


// Treat the point u as a vector, and get its magnitude
double mag(const Point2f& u)
{
    return sqrt(u.x*u.x + u.y*u.y);
}

// Treat u and v as vectors, and get the projection of u onto v
Point2f project(const Point2f& u, const Point2f& v)
{
    return (u.dot(v) / v.dot(v)) * v;
}

// Treat u and v as vectors
// The output will be the size of u project v
// But it will be negative if u project v points in the opposite direction from v
// I don't know if this corresponds to any "real" concept, but it happened
// to be useful for me when trying to determine if two lines are close enough
// to be grouped into the same cluster
double projectDist(const Point2f& u, Point2f& v)
{
    return (u.dot(v) / v.dot(v)) * mag(v);
}

// a is the candidate
// b is the template to match against
// matching is not commutative! It probably should be, but it's not. Deal.
// This function expects to be given the two endpoints of each line. That's why it expects
// four points.
bool linesMatch(const Point2f& a1, const Point2f& a2, const Point2f& b1, const Point2f& b2)
{
    // Some "magic constants"
    const double gapTolerance = 50;
    const double perpTolerance = 20;
    const double angleTolerance = 20;

    // Gap tolerance is about this scenario:
    // -----------     --------------
    //     line    gap!    line
    // If the gap is less than the tolerance, we can merge the two lines

    // perpTolerance (perp for perpendicular distance) is about this scenario:
    // -------------------- line
    //          
    //   perpendicular gap!
    //          
    // -------------------- line
    // If the perpendicular distance is less than the tolerance, we can merge the two lines

    // angleTolerance is about when the two lines are not exactly parallel
    // If the angle between them is less than the tolerance, we can merge the two lines
    // (No, I'm not going to try to draw lines at an angle to each other with ASCII art)
    // (Who do you think I am?)

    Point2f a = a2 - a1; // Vector pointing along line a
    Point2f b = b2 - b1; // Vector pointing along line b
    
    // Get angle between the two lines
    double cosAngleBetween = a.dot(b) / (mag(a) * mag(b));
    double angleDiff = acos(cosAngleBetween) * 180.0 / CV_PI; // in degrees
    if (angleDiff > 90.0)
    {
        // There are two supplementary angle diffs (i.e. they add up to 180 degrees) 
        // We want the smaller one.
        // So if we get the bigger one, subtract from 180 to find the other.
        angleDiff = 180 - angleDiff;
    }

    // Get the perpendicular distance between one endpoint of a and the line b
    Point2f b1a1 = a1 - b1;
    double a1PerpDist = mag(b1a1 - project(b1a1, b));

    // Get the perpendicular distance between the other endpoint of a and the line b
    Point2f b1a2 = a2 - b1;
    double a2PerpDist = mag(b1a2 - project(b1a2, b));

    // Okay, we're now going to calculate "gap". Two steps:

    // Project one endpoint of a onto the line b, and see how far off from the actual line b you are.
    //
    //                                                    * A1
    //                                                    |
    //                                                    | (project A1 onto line b)
    //                                                    |
    //                                                    |
    //    line b _____________________________            *
    //                                        ^^^^^^^^^^^^^
    //                                        we care about
    //                                        this distance
    double a1ProjDist = projectDist(b1a1, b);
    double a1ParallelDist = (a1ProjDist > 0) ? std::max(0.0, a1ProjDist - mag(b)) : -a1ProjDist;

    // Project the other endpoint of a onto the line b, and see how far off from the actual line b you are.
    // Same diagram as above.
    double a2ProjDist = projectDist(b1a2, b);
    double a2ParallelDist = (a2ProjDist > 0) ? std::max(0.0, a2ProjDist - mag(b)) : -a2ProjDist;

    // Define the "gap" between the two lines to be the minimum of these two calculated distances
    double gap = min(a1ParallelDist, a2ParallelDist);

    // Check if all of the differences we measured are within acceptable tolerances
    // If so, yay! The two lines match!
    return (angleDiff < angleTolerance) && (a1PerpDist < perpTolerance) && (a2PerpDist < perpTolerance) && (gap < gapTolerance);
}


// A cluster is "better" than another if it represents a "longer" line.
// Can experiment with different metrics as well.
bool clusterGreaterThan(Cluster* a, Cluster* b)
{
    return a->getLength() > b->getLength();
}

// Add the given line to this cluster
void Cluster::add(const Vec4i& line)
{
    // Increment line count
    lineCount++;

    // Increment total distance of all lines in cluster
    Point2f start(line[0], line[1]);
    Point2f end(line[2], line[3]);
    totalDist += mag(end - start);

    // Recompute the largest pair-wise distance between any two points
    // in this cluster. This is overall an O(n^2) operation (bad), which
    // we just kind of divide up over all of the ::add calls to this
    // cluster. Whatevs.
    // Iterate over all existing lines.
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i v = lines[i];
        Point2f vStart(v[0], v[1]);
        Point2f vEnd(v[2], v[3]);

        // There are four new distances to consider (from each of the endpoints of the new
        // line to each of the endpoints of this exisiting line)
        length = std::max(length, mag(start - vStart));
        length = std::max(length, mag(end - vStart));
        length = std::max(length, mag(start - vEnd));
        length = std::max(length, mag(end - vEnd));
    }

    // Add the line to the set in the cluster
    lines.push_back(line);
}


// Ranking clusters.

// Do greedy clustering
// Input is an array of the lines to cluster
// Outputs two things:
//   1. An array of the clusters it finds.
//   2. The clusterAssignments array.
//      If clusterAssignments[i] = j, that means that lines[i] belongs to clusters[j]
void greedyClustering(const vector<Vec4i>& lines, vector<int>& clusterAssignments, vector<Cluster*>& clusters)
{
    // This is O(n^2) (or worse, depending on how exactly you count the cluster-merging stuff),
    // which is not great, but number of lines is pretty small, so oh well?
    // Plus I'm pretty sure we could easily get this down to O(n*number_clusters),
    // (which is pretty much O(n) since there are few clusters) if instead of comparing
    // a line to all other lines in a candidate cluster, we just compare it to
    // some sort of cumulative average. And then as long as we can update that 
    // cumulative average in O(1), we are done.
    // And for cluster merging, there is always disjoint-forest if that becomes an issue

    // We need to assign each line to a cluster, so we need the clusterAssignments array to
    // be as big as the lines array
    clusterAssignments.resize(lines.size());

    int clusterCount = 0;
 
    // Iterate over all lines          
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i a=lines[i];

        // Extract the two endpoints of this line
        Point2f a1(a[0], a[1]);
        Point2f a2(a[2], a[3]);

        // Iterate over all lines which we've already assigned to a cluster, and see
        // which ones this new line matches
        vector<int> matchedClusters;
        for (size_t j = 0; j < i; j++)
        {
            Vec4i b=lines[j];

            // Extract the two endpoints of the already-assigned line
            Point2f b1(b[0], b[1]);
            Point2f b2(b[2], b[3]);

            // Check if the candidate line matches the already-assigned line
            // PS "match" here means the two lines are close enough to be considered
            // part of the same cluster
            if (linesMatch(a1, a2, b1, b2))
            {
                matchedClusters.push_back(clusterAssignments[j]);
            }
        }

        if (matchedClusters.empty())
        {
            // If the line didn't match any existing clusters, it's the first line
            // in a new cluster
            clusterAssignments[i] = clusterCount++;
        }
        else
        {
            // If the line did match some clusters, then we want to merge together all
            // of the clusters that it matched and then add the line to that merged cluster

            // Aribitrarily pick one of the matched clusters. We'll merge all the
            // others into this one.
            int cluster = matchedClusters[0];

            // Iterate over the already-assigned lines and do the cluster merging
            for (size_t j = 0; j < i; j++)
            {
                // If the given line belonged to any of the matched clusters, then assign
                // it to the merged cluster
                if (find(matchedClusters.begin(), matchedClusters.end(), clusterAssignments[j]) != matchedClusters.end())
                {
                    clusterAssignments[j] = cluster;
                }
            }

            // Add the new line to the merged cluster as well
            clusterAssignments[i] = cluster;
        }
    }

    // Okay, now we've assigned each line to a cluster. But we haven't actually *built* the clusters
    // and calculated their stats. So let's do that.

    // The cluster ids are currently sparse. I.e. there are some ids that no lines are assigned to,
    // because we initially created a cluster with that id, but then it got merged into another
    // cluster. We're going to compactify the ids so they are not sparse. So let's make a map from
    // spare id -> dense id. We'll initialize everything to -1 to indicate "no assignment yet"
    vector<int> sparseToDenseMapping(clusterCount, -1);

    // Iterate over the lines, and add them to the right cluster
    for (size_t i = 0; i < lines.size(); i++)
    {
        // Grab a line (and the cluster it is assigned to)
        Vec4i line = lines[i];
        int sparseIndex = clusterAssignments[i];
        int denseIndex = sparseToDenseMapping[sparseIndex];
        if (denseIndex == -1)
        {
            // This is a cluster we haven't see in this loop yet!
            // Let's create space for it in the clusters array,
            // and also assign it a sparse->dense mapping
            denseIndex = clusters.size();
            sparseToDenseMapping[sparseIndex] = denseIndex;
            clusters.push_back(new Cluster());
        }

        // Replace the sparse index with the dense index in the clusterAssignments
        clusterAssignments[i] = denseIndex;
        
        // Add the line to its cluster
        clusters[denseIndex]->add(line);
    }
}


