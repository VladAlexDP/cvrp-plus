#ifndef TSP_HPP
#define TSP_HPP

#include "types.hpp"

#include <tuple>

namespace TSP {

std::pair<Route, Distance_t> getRoute(const DistanceMatrix &distances,
                                      const DemandMatrixColumn &demand,
                                      const Vehicle &vehicle);

Cost_t routeCost(Route &route, const DistanceMatrix &distances,
                 const DemandMatrixColumn &demand, const Vehicle &vehicle);

};

#endif // TSP_HPP
