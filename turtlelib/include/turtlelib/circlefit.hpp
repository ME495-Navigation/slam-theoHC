#ifndef TURTLELIB_CIRCLEFIT_HPP_INCLUDE_GUARD
#define TURTLELIB_CIRCLEFIT_HPP_INCLUDE_GUARD

#include <turtlelib/geometry2d.hpp>
#include <vector>

namespace turtlelib{

    struct Circle{
        Point2D center;
        double radius;
    };

    /// @brief Cluster LIDAR points into groups if adjacent points are more than a distance apart, returning a vector of the clusters
    /// @param distances Underlying lidar distances, ordered by increasing angle.
    /// @param increment Angular increment between lidar points.
    /// @param threshold Distance threshold for clustering points. If the distance between two adjacent points is greater than this threshold, they will be placed in different clusters.
    /// @param min_group_size The minimum number of points required for a cluster to be returned.
    /// @return A vector of clusters, where each cluster is a vector of Point2D representing the positions of the points in that cluster.
    std::vector<std::vector<Point2D>> clusterPoints(const std::vector<double> distances, const double increment, const double threshold, const size_t min_group_size=3);

    bool isClusterConvex(const std::vector<Point2D> & cluster);

    /// @brief Fit a cluster of points to a circle
    /// @param points The points to fit
    /// @return Center and radius of the circle
    Circle fitCircle(const std::vector<Point2D> & points);
    
}

#endif