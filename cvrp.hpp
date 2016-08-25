#ifndef CVRP_HPP
#define CVRP_HPP

#include "tsp.hpp"

#include <vector>
#include <list>

namespace CVRP {

ListRouteInfoSet getRoutes(const PointSet &stations, const VehicleSet &vehicles,
                           const DemandMatrixColumn &demand,
                           const DistanceMatrix &distance);
};

#endif // CVRP_HPP
