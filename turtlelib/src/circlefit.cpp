#include "turtlelib/circlefit.hpp"
#include "circlefit.hpp"

namespace turtlelib{
    std::vector<std::vector<Point2D>> clusterPoints(const std::vector<double> distances, const double increment, const double threshold, const size_t min_group_size){
        std::vector<Point2D> points;
        for(size_t i = 0; i < distances.size(); i++) {
            double angle = i * increment;
            double x = distances.at(i) * std::cos(angle);
            double y = distances.at(i) * std::sin(angle);
            points.push_back({x, y});
        }
        
        std::vector<std::vector<Point2D>> clusters;
        std::vector<Point2D> current_cluster;
        std::vector<int> cluster_first_indices;
        current_cluster.push_back(points.at(0));
        cluster_first_indices.push_back(0);
        for(size_t i = 1; i < points.size(); i++) {
            double dist = turtlelib::magnitude(points.at(i) - points.at(i-1));
            if(dist > threshold) {
                if(current_cluster.size() >= min_group_size) {
                    if(isClusterConvex(current_cluster)) {
                        clusters.push_back(current_cluster);
                    }
                }if(current_cluster.size() < min_group_size && !cluster_first_indices.empty()){
                    cluster_first_indices.pop_back();
                }
                current_cluster.clear();
                current_cluster.push_back(points.at(i));
                cluster_first_indices.push_back(i);
            }else{
                current_cluster.push_back(points.at(i));
            }
        }

        double firstLastDist = distance(points.front(), points.back());
        if(firstLastDist < threshold){
            if(cluster_first_indices.front() == 0){ //If the first cluster starts at index 0, just append to the end of it
                clusters.front().insert(clusters.front().begin(), current_cluster.begin(), current_cluster.end());
            }else{
                current_cluster.push_back(points.front()); // Otherwise, the last cluster wraps around
                if(distance(points.at(0), points.at(1)) < threshold){ // We also need to check the second point, because if there's a cluster (last, first, second), we need to catch it.
                    current_cluster.push_back(points.at(1));
                }
            }
        }

        if(current_cluster.size() >= min_group_size) {
            if(isClusterConvex(current_cluster)) {
                clusters.push_back(current_cluster);
            }
        }

        return clusters;
    }

    bool isClusterConvex(const std::vector<Point2D> & cluster){
        if(cluster.size() < 3) {
            return false;
        }

        double avgClusterDistance = 0;
        for(size_t i = 1; i < cluster.size() - 1; i++) {
            avgClusterDistance += distance(cluster.at(i));
        }
        avgClusterDistance /= (cluster.size() - 2);

        return avgClusterDistance < distance(cluster.front()) && avgClusterDistance < distance(cluster.back());
    }
}
