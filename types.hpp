#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <set>
#include <list>
#include <limits>

template <typename T> using MatrixColumn = std::vector<T>;
template <typename T> using Matrix = std::vector<MatrixColumn<T>>;

using Distance_t = float;
using Demand_t = float;
using DistanceMatrixColumn = std::vector<Distance_t>;
using DistanceMatrix = std::vector<DistanceMatrixColumn>;
using DemandMatrixColumn = std::vector<Demand_t>;
using DemandMatrix = std::vector<DemandMatrixColumn>;
using Cost_t = float;
using Load_t = float;

struct Vehicle {
  Load_t tare;
  float costOfWeightDistance;
  Load_t capacity;
};

struct Station_t {
  std::size_t station_id;
  Load_t load_before;
  Cost_t add_cost_at;
  Cost_t total_cost_at;

  Station_t(std::size_t station_id = std::numeric_limits<std::size_t>::max())
      : station_id(station_id) {}
  operator std::size_t() const { return station_id; }
  std::size_t operator++() { return ++station_id; }
  std::size_t operator++(int) { return station_id++; }
  bool operator!=(const Station_t &rhs) const {
    return station_id != rhs.station_id;
  }
  bool operator!=(const std::size_t &rhs) const { return station_id != rhs; }
};

using StationSet = std::set<Station_t>;
using Route = std::vector<Station_t>;
using ListRoute = std::list<Station_t>;

struct ListRouteInfo {
  ListRoute route;
  std::size_t demand_table_index;
  Cost_t cost;
  Cost_t fitness;
};

using RouteSet = std::vector<Route>;
using ListRouteSet = std::vector<ListRoute>;
using ListRouteInfoSet = std::vector<ListRouteInfo>;

using VehicleSet = std::vector<Vehicle>;

struct Point {
  int id;
  double x;
  double y;
};

using PointSet = std::vector<Point>;

#endif // TYPES_HPP
