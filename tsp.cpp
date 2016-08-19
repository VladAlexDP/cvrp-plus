#include "tsp.hpp"

#include <vector>
#include <memory>
#include <algorithm>
#include <iterator>
#include <set>
#include <limits>

Cost_t getCost(Distance_t distance, Load_t load, const Vehicle &vehicle) {
  return (vehicle.tare + load) * distance;
}

Cost_t getInferiority(Distance_t distance, Demand_t demand, Load_t load,
                      const Vehicle &vehicle) {
  return getCost(distance, load, vehicle) /
         (vehicle.costOfWeightDistance * demand);
}

Station_t bestNeighbour(const DistanceMatrix &distances,
                        const DemandMatrixColumn &demand, Load_t load,
                        const Vehicle &vehicle, const Station_t &currentStation,
                        const StationSet &visitedStations) {
  Cost_t min_cost = std::numeric_limits<Cost_t>::max();
  Station_t best_station = distances.size();
  for (Station_t station = 0; station < distances.size(); ++station) {
    if (station != currentStation && demand[station] > 0 &&
        visitedStations.find(station) == visitedStations.end()) {
      Cost_t cost = getInferiority(distances[currentStation][station],
                                   demand[station], load, vehicle);
      if (cost < min_cost) {
        min_cost = cost;
        best_station = station;
      }
    }
  }
  return best_station;
}

std::pair<StationSet, Demand_t>
stationsInDemand(const DemandMatrixColumn &demand) {
  StationSet result;
  Demand_t total_demand = 0;
  for (Station_t station = 0; station < demand.size(); ++station) {
    if (demand[station] > 0) {
      result.insert(station);
      total_demand += demand[station];
    }
  }
  return std::make_pair(result, total_demand);
}

Route bestNeighbourRoute(const DistanceMatrix &distances,
                         const DemandMatrixColumn &demand,
                         const Vehicle &vehicle) {
  const Station_t STARTING_STATION = 0;

  StationSet stations_in_demand;
  Demand_t total_demand;
  std::tie(stations_in_demand, total_demand) = stationsInDemand(demand);

  Route result;
  result.reserve(stations_in_demand.size() + 2);
  result.push_back(STARTING_STATION);

  StationSet visited_stations;
  visited_stations.insert(STARTING_STATION);

  Station_t current_station = STARTING_STATION;
  Load_t current_load = total_demand;
  while (result.size() != stations_in_demand.size() + 1) {

    current_station = bestNeighbour(distances, demand, current_load, vehicle,
                                    current_station, visited_stations);

    if (current_station != distances.size()) {
      visited_stations.insert(current_station);
      result.push_back(current_station);
    }
  }
  result.push_back(STARTING_STATION);
  return result;
}

Cost_t TSP::routeCost(Route &route, const DistanceMatrix &distances,
                      const DemandMatrixColumn &demand,
                      const Vehicle &vehicle) {
  Cost_t total_cost = 0;
  Load_t current_load = stationsInDemand(demand).second;
  route[0].distance_at = 0;
  route[0].load_at = current_load;
  route[0].total_cost_at = 0;

  for (Station_t station = 0; station < route.size() - 1; ++station) {
    Station_t from = route[station], to = route[station + 1];
    total_cost += getCost(distances[from][to], current_load, vehicle);
    current_load -= demand[to];
    route[station + 1].distance_at =
        route[station].distance_at + distances[from][to];
    route[station + 1].load_at = current_load;
    route[station + 1].total_cost_at = total_cost;
  }
  return total_cost;
}

Route twoOptSwap(const Route &route, std::size_t from, std::size_t to) {
  Route new_route;
  new_route.reserve(route.size());

  for (std::size_t i = 0; i < from; ++i)
    new_route.push_back(route[i]);

  for (std::size_t i = to; i >= from; --i)
    new_route.push_back(route[i]);

  for (std::size_t i = to + 1; i < route.size(); ++i)
    new_route.push_back(route[i]);

  return new_route;
}

std::pair<Route, Cost_t> twoOpt(const Route &route,
                                const DistanceMatrix &distances,
                                const DemandMatrixColumn &demand,
                                const Vehicle &vehicle) {
  bool improvement_made;
  Route best_route = route;
  Cost_t best_cost;
  std::size_t nodes_to_swap = route.size() - 2;
  do {
    improvement_made = false;
    best_cost = TSP::routeCost(best_route, distances, demand, vehicle);
    for (Station_t i = 1; i < nodes_to_swap - 1; ++i) {
      for (Station_t k = i + 1; k < nodes_to_swap; ++k) {
        Route new_route = twoOptSwap(best_route, i, k);
        Cost_t new_cost = TSP::routeCost(new_route, distances, demand, vehicle);
        if (new_cost < best_cost) {
          best_route = std::move(new_route);
          best_cost = new_cost;
          improvement_made = true;
        }
      }
    }
  } while (improvement_made);
  return {best_route, best_cost};
}

std::pair<Route, Cost_t> TSP::getRoute(const DistanceMatrix &distances,
                                       const DemandMatrixColumn &demand,
                                       const Vehicle &vehicle) {
  Route best_route = bestNeighbourRoute(distances, demand, vehicle);
  return twoOpt(best_route, distances, demand, vehicle);
}
