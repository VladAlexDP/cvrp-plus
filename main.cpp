#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <chrono>
#include <regex>
#include <cmath>

#include "cvrp.hpp"

using namespace TSP;
using namespace CVRP;

template <typename Func, typename... Args>
double benchmark(std::size_t times, Func function, Args... args) {
  using namespace std::chrono;
  auto start = high_resolution_clock::now();
  for (std::size_t i = 0; i < times; ++i) {
    function(args...);
  }
  auto end = high_resolution_clock::now();
  return double(duration_cast<milliseconds>(end - start).count()) /
         double(times);
}

template <typename Matrix> Matrix matrixFromFile(const std::string &filePath) {
  std::fstream file(filePath);

  if (!file.is_open()) {
    std::cout << "Can't open file " << filePath << std::endl;
    return Matrix();
  }

  Matrix matrix;
  std::string buff;
  while (std::getline(file, buff)) {
    std::istringstream iss(buff);
    std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};
    typename Matrix::value_type column;
    column.reserve(tokens.size());
    std::transform(tokens.begin(), tokens.end(), std::back_inserter(column),
                   [](const std::string &str) { return std::stof(str); });
    matrix.push_back(column);
  }
  file.close();
  return matrix;
}

template <typename T> T transpose(T matrix) {
  T result(matrix[0].size(), typename T::value_type(matrix.size()));

  for (std::size_t i = 0; i < matrix.size(); ++i)
    for (std::size_t j = 0; j < matrix[i].size(); ++j)
      result[j][i] = matrix[i][j];

  return result;
}

template <typename R>
std::string routeToString(const R &route, const std::string &delimiter = " ") {
  std::string result;
  result += std::to_string(route.front());
  for (auto it = ++route.begin(); it != route.end(); ++it) {
    result += delimiter;
    result += std::to_string(*it);
  }
  return result;
}

bool readHeader(std::fstream &file, std::string &name, Demand_t &capacity) {
  //  std::regex name_regex("NAME : ([a-zA-Z0-9-]+)");
  std::regex name_regex(R"(^[\s]*NAME\s*:\s*(.+)\s*$)");
  std::regex capacity_regex(R"(^[\s]*CAPACITY\s*:\s*([0-9]+)\s*$)");

  std::string buff;

  bool header_read = false;
  while (!header_read && std::getline(file, buff)) {
    std::smatch smatch;
    if (std::regex_match(buff, smatch, name_regex)) {
      name = smatch[1];
    } else if (std::regex_match(buff, smatch, capacity_regex)) {
      capacity = std::atof(smatch[1].str().c_str());
    } else if (buff.find("NODE_COORD_SECTION") != std::string::npos) {
      header_read = true;
    }
  }
  return header_read;
}

bool readCoords(std::fstream &file, const std::string &filepath,
                PointSet &stations) {
  std::regex coord_regex(R"(^[\s]*([0-9]+)\s*([0-9]+)\s*([0-9]+)\s*$)");
  std::string buff;
  bool coords_read = false;
  while (!coords_read && std::getline(file, buff)) {
    std::smatch smatch;
    if (std::regex_match(buff, smatch, coord_regex)) {
      stations.push_back(Point{std::atoi(smatch[1].str().c_str()) - 1,
                               std::atof(smatch[2].str().c_str()),
                               std::atof(smatch[3].str().c_str())});
    } else if (buff.find("DEMAND_SECTION") != std::string::npos) {
      coords_read = true;
    } else {
      std::cout << "Invalid coords " << buff << " in " << filepath << std::endl;
      return false;
    }
  }
  return coords_read;
}

bool readDemand(std::fstream &file, const std::string &filepath,
                DemandMatrixColumn &demand) {
  std::regex demand_regex(R"(^[\s]*[0-9]+\s*([0-9]+)\s*$)");
  std::string buff;
  bool demand_read = false;
  while (!demand_read && std::getline(file, buff)) {
    std::smatch smatch;
    if (std::regex_match(buff, smatch, demand_regex)) {
      demand.push_back(std::atof(smatch[1].str().c_str()));
    } else if (buff.find("DEPOT_SECTION") != std::string::npos) {
      demand_read = true;
    } else {
      std::cout << "Invalid demand " << buff << " in " << filepath << std::endl;
      return false;
    }
  }
  return demand_read;
}

int numberOfVehiclesFromName(const std::string &name) {
  std::regex regex(R"(^\s*.+k([0-9]+)\s*$)");
  std::smatch smatch;
  if (std::regex_match(name, smatch, regex)) {
    return std::atoi(smatch[1].str().c_str());
  }
  return 0;
}

bool parseAugeratFile(const std::string &filepath, std::string &name,
                      VehicleSet &vehicles, PointSet &stations,
                      DemandMatrixColumn &demand) {
  std::fstream file(filepath);
  if (!file.is_open()) {
    std::cout << "Can`t open file " << filepath << std::endl;
    return false;
  }

  Demand_t capacity;
  bool headOk = readHeader(file, name, capacity);
  if (!headOk) {
    return false;
  }
  bool coordsOk = readCoords(file, filepath, stations);
  if (!coordsOk) {
    return false;
  }
  bool demandOk = readDemand(file, filepath, demand);
  if (!demandOk) {
    return false;
  }
  file.close();

  int vehicleCount = numberOfVehiclesFromName(name);
  if (vehicleCount == 0) {
    std::cout << "Vehilce count not found in name" << std::endl;
    return false;
  }

  vehicles = VehicleSet(vehicleCount, Vehicle{20.0f, .2f, capacity});

  return true;
}

Distance_t distance(const Point &a, const Point &b) {
  return std::sqrt(std::pow((a.x - b.x), 2) + std::pow((a.y - b.y), 2));
}

DistanceMatrix calcDistanceMatrix(const PointSet &stations) {
  DistanceMatrix distances(stations.size(),
                           std::vector<float>(stations.size()));
  for (std::size_t i = 0; i < stations.size(); ++i) {
    for (std::size_t j = 0; j < stations.size(); ++j) {
      float d = distance(stations[i], stations[j]);
      distances[i][j] = d;
    }
  }

  return distances;
}

int main(int argc, char *argv[]) {
  std::string name;
  VehicleSet vehicles;
  PointSet stations;
  DemandMatrixColumn demand;

  if (!parseAugeratFile(argv[1], name, vehicles, stations, demand)) {
    std::cout << "Aborting..." << std::endl;
    return 1;
  }

  DistanceMatrix distances = calcDistanceMatrix(stations);

  Cost_t sum_fitness = 0.0f;
  Cost_t sum_cost = 0.0f;
  ListRouteInfoSet routes =
      CVRP::getRoutes(stations, vehicles, demand, distances);
  for (const ListRouteInfo &routeInfo : routes) {
    sum_fitness += routeInfo.fitness;
    sum_cost += routeInfo.cost;
    std::cout << "Route: " << routeToString(routeInfo.route,
                                            " - ") /*<< std::endl
              << "\tCost: " << routeInfo.cost
              << "\tFitness: " << routeInfo.fitness*/ << std::endl;
  }

  std::cout << "General fitness: " << sum_fitness << std::endl;
  std::cout << "General cost: " << sum_cost << std::endl;
  std::cout << "It took " << benchmark(100, CVRP::getRoutes, stations, vehicles,
                                       demand, distances) << " milliseconds\n";
  return 0;
}
