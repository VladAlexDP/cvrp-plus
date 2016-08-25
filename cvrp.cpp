#include "cvrp.hpp"

#include <algorithm>
#include <random>
#include <ctime>

#include <iostream>

Cost_t calcCoustraint(Demand_t total_demand, const Vehicle &vehicle) {
  return std::max(0.0f, total_demand - vehicle.capacity);
}

Cost_t calcFitness(Cost_t total_cost, Demand_t total_demand,
                   const Vehicle &vehicle) {
  static const float a = 10.0f;
  return total_cost + a * calcCoustraint(total_demand, vehicle);
}

Demand_t calcDemand(const DemandMatrixColumn &matrixColumn) {
  Demand_t result = 0;
  for (Demand_t d : matrixColumn) {
    result += d;
  }
  return result;
}
DemandMatrix randomDemandMatrix(const DemandMatrixColumn &demand,
                                std::size_t vehicle_count) {
  DemandMatrix result(vehicle_count, DemandMatrixColumn(demand.size()));
  std::mt19937 rand_generator(std::time(0));
  std::size_t station = 0;
  for (Demand_t d : demand) {
    std::size_t random_value = rand_generator() % vehicle_count;
    result[random_value][station] = d;
    ++station;
  }
  return result;
}

DemandMatrix angleDemandMatrix(const PointSet &stations,
                               const DemandMatrixColumn &demand,
                               const VehicleSet &vehicles) {
  Demand_t total_demand = calcDemand(demand);
  Demand_t total_capacity = std::accumulate(
      vehicles.begin(), vehicles.end(), 0,
      [](const Demand_t sum, const Vehicle &v) { return sum + v.capacity; });

  PointSet stations_by_angle(stations);
  std::sort(stations_by_angle.begin(), stations_by_angle.end(),
            [](const Point &lhs, const Point &rhs) {
              return std::atan(lhs.y / lhs.x) < std::atan(rhs.y / rhs.x);
            });

  Demand_t k = std::min(total_capacity / total_demand, 1.0f);
  VehicleSet::const_iterator current_vehicle = vehicles.begin();
  std::size_t current_index = 0;
  Demand_t current_demand_sum = 0.0f;

  DemandMatrix result(vehicles.size(), DemandMatrixColumn(demand.size()));
  for (const Point &s : stations_by_angle) {
    if (current_demand_sum + k * demand[s.id] >= current_vehicle->capacity) {
      current_demand_sum = 0;
      if(current_index < vehicles.size() - 1) {
        ++current_index;
        ++current_vehicle;
      }
    }
    current_demand_sum += k * demand[s.id];
    result[current_index][s.id] = demand[s.id];
  }

  return result;
}

ListRoute::const_iterator bestStationsInRoute(const Station_t &station,
                                              const ListRoute &route,
                                              const DistanceMatrix &distances,
                                              const DemandMatrixColumn &demand,
                                              const Vehicle &vehicle) {
  auto last = route.end();
  --last;

  if (route.size() == 2)
    return last;

  ListRoute::const_iterator result;
  Cost_t min_cost = std::numeric_limits<Cost_t>::max();

  for (auto it = route.begin(); it != last; ++it) {
    Cost_t cost_first =
        TSP::getInferiority(distances[*it][station], demand[station],
                            it->load_before - demand[*it], vehicle);

    auto it_next = it;
    std::advance(it_next, 1);

    Cost_t cost_second =
        TSP::getInferiority(distances[station][*it_next], demand[*it_next],
                            it_next->load_before, vehicle);

    Cost_t cost_sum = cost_first + cost_second;
    if (cost_sum < min_cost) {
      min_cost = cost_sum;
      result = it_next;
    }
  }

  return result;
}

Cost_t refreshRouteCost(ListRoute::iterator begin, ListRoute::iterator end,
                        Load_t load_change, const DistanceMatrix &distances,
                        const Vehicle &vehicle) {
  Cost_t new_cost = 0;
  auto last = end;
  --last;
  for (auto it = begin; it != last; ++it) {
    auto it_next = it;
    ++it_next;

    it_next->add_cost_at = TSP::getCost(
        distances[*it][*it_next], it_next->load_before + load_change, vehicle);

    new_cost += it_next->add_cost_at;
    it_next->total_cost_at = new_cost;
  }
  return new_cost;
}

Cost_t calcAddRouteCost(ListRoute::iterator begin, ListRoute::iterator end,
                        const DistanceMatrix &distances,
                        const DemandMatrixColumn &demand,
                        const Vehicle &vehicle) {
  Cost_t total_cost = begin->total_cost_at;

  auto new_st_it = begin;
  ++new_st_it;
  Load_t load_before_new = begin->load_before - demand[*begin];

  new_st_it->add_cost_at =
      TSP::getCost(distances[*begin][*new_st_it], load_before_new, vehicle);

  total_cost += new_st_it->add_cost_at;
  new_st_it->total_cost_at = total_cost;

  Demand_t demand_at_new = demand[new_st_it->station_id];

  return total_cost +
         refreshRouteCost(new_st_it, end, -demand_at_new, distances, vehicle);
}

Cost_t calcDecRouteCost(ListRoute::iterator begin, ListRoute::iterator end,
                        Demand_t demand_at_removed,
                        const DistanceMatrix &distances,
                        const Vehicle &vehicle) {
  Cost_t total_cost = begin->total_cost_at;

  return total_cost +
         refreshRouteCost(begin, end, demand_at_removed, distances, vehicle);
}

Demand_t demandFromRoute(const ListRoute &route,
                         const DemandMatrixColumn &demand) {
  Demand_t result = 0.0f;
  for (const Station_t &s : route) {
    result += demand[s.station_id];
  }
  return result;
}

std::pair<ListRoute, ListRoute>
moveStationToRoute(const ListRoute::const_iterator station_it,
                   const ListRoute &from, const ListRoute &to,
                   const DistanceMatrix &distances,
                   const DemandMatrixColumn &demand, const Vehicle &vehicle) {

  long distance = std::distance(from.begin(), station_it);

  ListRoute new_first(from), new_second(to);

  ListRoute::iterator station_to_remove = new_first.begin();
  std::advance(station_to_remove, distance);
  new_first.erase(station_to_remove);

  ListRoute::const_iterator best_place =
      bestStationsInRoute(*station_it, new_second, distances, demand, vehicle);
  new_second.insert(best_place, *station_it);

  return {new_first, new_second};
}

std::tuple<ListRouteInfo, ListRouteInfo, Cost_t>
oneSwap(const ListRouteInfo &routeFirst, const ListRouteInfo &routeSecond,
        const DistanceMatrix &distances, const DemandMatrixColumn &demand,
        const Vehicle &vehicleFirst, const Vehicle &vehicleSecond) {

  ListRouteInfo best_first = routeFirst;
  ListRouteInfo best_second = routeSecond;
  best_first.fitness = calcFitness(
      routeFirst.cost, demandFromRoute(routeFirst.route, demand), vehicleFirst);
  best_second.fitness =
      calcFitness(routeSecond.cost, demandFromRoute(routeSecond.route, demand),
                  vehicleSecond);

  Cost_t best_fitness = best_first.fitness + best_second.fitness;

  auto begin = best_first.route.begin();
  ++begin;
  auto end = best_first.route.end();
  --end;

  for (auto it = begin; it != end;) {
    if (it->station_id == 0)
      break;

    ListRouteInfo new_first, new_second;
    std::tie(new_first.route, new_second.route) =
        moveStationToRoute(it, best_first.route, best_second.route, distances,
                           demand, vehicleSecond);

    new_first.cost =
        TSP::routeCost(new_first.route, distances, demand, vehicleFirst,
                       demandFromRoute(new_first.route, demand));

    new_second.cost =
        TSP::routeCost(new_second.route, distances, demand, vehicleSecond,
                       demandFromRoute(new_second.route, demand));

    new_first.fitness = calcFitness(
        new_first.cost, demandFromRoute(new_first.route, demand), vehicleFirst);
    new_second.fitness =
        calcFitness(new_second.cost, demandFromRoute(new_second.route, demand),
                    vehicleSecond);

    Cost_t new_fitness = new_first.fitness + new_second.fitness;

    if (new_fitness < best_fitness) {
      best_fitness = new_fitness;
      best_first = new_first;
      best_second = new_second;

      // We have to update iterators because route was changed
      long distance_new = std::distance(best_first.route.begin(), it);
      it = best_first.route.begin();
      std::advance(it, distance_new); // it points to station after removed

      end = best_first.route.end();
      --end; // we dont need to swap last one
    } else {
      ++it; // just move iterator to next station
    }
  }

  return {best_first, best_second, best_fitness};
}

std::tuple<ListRouteInfo, ListRouteInfo, Cost_t>
twoSwap(const ListRouteInfo &routeFirst, const ListRouteInfo &routeSecond,
        const DistanceMatrix &distances, const DemandMatrixColumn &demand,
        const Vehicle &vehicleFirst, const Vehicle &vehicleSecond) {

  ListRouteInfo best_first = routeFirst;
  ListRouteInfo best_second = routeSecond;
  best_first.fitness = calcFitness(
      routeFirst.cost, demandFromRoute(routeFirst.route, demand), vehicleFirst);
  best_second.fitness =
      calcFitness(routeSecond.cost, demandFromRoute(routeSecond.route, demand),
                  vehicleSecond);

  Cost_t best_fitness = best_first.fitness + best_second.fitness;

  auto begin_first = best_first.route.begin();
  ++begin_first;
  auto end_first = best_first.route.end();
  --end_first;
  auto begin_second = best_second.route.begin();
  ++begin_second;
  auto end_second = best_second.route.end();
  --end_second;

  for (auto it_first = begin_first; it_first != end_first; ++it_first) {
    for (auto it_second = begin_second; it_second != end_second;) {

      if (it_first->station_id == 0 || it_second->station_id == 0)
        break;

      auto it_first_next = it_first;
      ++it_first_next;
      auto it_second_next = it_second;
      ++it_second_next;
      Station_t next_station_first = *it_first_next,
                next_station_second = *it_second_next;

      ListRouteInfo new_first, new_second;
      std::tie(new_first.route, new_second.route) =
          moveStationToRoute(it_first, best_first.route, best_second.route,
                             distances, demand, vehicleSecond);
      std::tie(new_second.route, new_first.route) =
          moveStationToRoute(it_second, best_second.route, best_first.route,
                             distances, demand, vehicleFirst);

      new_first.cost =
          TSP::routeCost(new_first.route, distances, demand, vehicleFirst,
                         demandFromRoute(new_first.route, demand));

      new_second.cost =
          TSP::routeCost(new_second.route, distances, demand, vehicleSecond,
                         demandFromRoute(new_second.route, demand));

      new_first.fitness =
          calcFitness(new_first.cost, demandFromRoute(new_first.route, demand),
                      vehicleFirst);
      new_second.fitness =
          calcFitness(new_second.cost,
                      demandFromRoute(new_second.route, demand), vehicleSecond);

      Cost_t new_fitness = new_first.fitness + new_second.fitness;

      if (new_fitness < best_fitness) {
        best_fitness = new_fitness;
        best_first = new_first;
        best_second = new_second;

        // We have to update iterators because route was changed
        it_first = std::find(best_first.route.begin(), best_first.route.end(),
                             next_station_first);
        it_second = std::find(best_second.route.begin(),
                              best_second.route.end(), next_station_second);

        end_first = best_first.route.end();
        --end_first; // we dont need to swap last one
        end_second = best_second.route.end();
        --end_second;
      } else {
        ++it_second; // just move iterator to next station
      }
    }
  }

  return {best_first, best_second, best_fitness};
}

std::pair<ListRoute, Distance_t>
routeFromDemand(const DistanceMatrix &distances,
                const DemandMatrixColumn &demand, const Vehicle &vehicle) {
  std::pair<Route, Distance_t> route =
      TSP::getRoute(distances, demand, vehicle);
  return {ListRoute(route.first.begin(), route.first.end()), route.second};
}

ListRouteInfoSet CVRP::getRoutes(const PointSet &stations,
                                 const VehicleSet &vehicles,
                                 const DemandMatrixColumn &demand,
                                 const DistanceMatrix &distance) {
  DemandMatrix demandMatrix = angleDemandMatrix(stations, demand, vehicles);
//  DemandMatrix demandMatrix = randomDemandMatrix(demand, vehicles.size());
  std::vector<ListRouteInfo> routes;
  for (std::size_t veh = 0; veh < vehicles.size(); ++veh) {
    std::pair<ListRoute, Distance_t> r =
        routeFromDemand(distance, demandMatrix[veh], vehicles[veh]);
    routes.push_back(ListRouteInfo{r.first, veh, r.second, 0.0f});
  }

  for (std::size_t i = 0; i < routes.size(); ++i) {
    for (std::size_t j = i + 1; j < routes.size(); ++j) {
      auto resultA = oneSwap(routes[i], routes[j], distance, demand,
                             vehicles[i], vehicles[j]);

      auto resultB = oneSwap(routes[j], routes[i], distance, demand,
                             vehicles[j], vehicles[i]);

      if (std::get<2>(resultA) < std::get<2>(resultB)) {
        routes[i] = std::get<0>(resultA);
        routes[j] = std::get<1>(resultA);
      } else {
        routes[j] = std::get<0>(resultB);
        routes[i] = std::get<1>(resultB);
      }
    }
  }

  for (std::size_t i = 0; i < routes.size(); ++i) {
    for (std::size_t j = i + 1; j < routes.size(); ++j) {
      auto result = twoSwap(routes[i], routes[j], distance, demand, vehicles[i],
                            vehicles[j]);
      routes[i] = std::get<0>(result);
      routes[j] = std::get<1>(result);
    }
  }

  return routes;
}
