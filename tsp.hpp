#ifndef TSP_HPP
#define TSP_HPP

#include "types.hpp"

#include <tuple>

namespace TSP {

std::pair<Route, Distance_t> getRoute(const DistanceMatrix &distances,
                                      const DemandMatrixColumn &demand,
                                      const Vehicle &vehicle);

Cost_t routeCost(Route &route, const DistanceMatrix &distances,
                 const DemandMatrixColumn &demand, const Vehicle &vehicle,
                 Load_t startLoad);

Cost_t routeCost(ListRoute &route, const DistanceMatrix &distances,
                 const DemandMatrixColumn &demand, const Vehicle &vehicle,
                 Load_t startLoad);

Cost_t getCost(Distance_t distance, Load_t load, const Vehicle &vehicle);

Cost_t getInferiority(Distance_t distance, Demand_t demand, Load_t load,
                      const Vehicle &vehicle);
};

#endif // TSP_HPP
