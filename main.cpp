#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <chrono>

#include "tsp.hpp"

using namespace TSP;

template <typename Func, typename... Args>
double benchmark(std::size_t times, Func function, Args... args) {
  using namespace std::chrono;
  auto start = high_resolution_clock::now();
  for (std::size_t i = 0; i < times; ++i) {
    function(args...);
  }
  auto end = high_resolution_clock::now();
  return duration_cast<duration<double>>(end - start).count() / times;
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

std::string routeToString(const Route &route,
                          const std::string &delimiter = " ") {
  std::string result;
  result += std::to_string(route.front());
  for (auto it = ++route.begin(); it != route.end(); ++it) {
    result += delimiter;
    result += std::to_string(*it);
  }
  return result;
}

int main(int /* argc */, char * /*argv*/ []) {
  DistanceMatrix distances = matrixFromFile<DistanceMatrix>("tests/test2.dist");
  DemandMatrix demand = matrixFromFile<DemandMatrix>("tests/test2.dem");
  demand = transpose(demand);

  Vehicle vehicle{20.0f, 0.3f};
  Route best_route;
  Distance_t route_cost;
  std::tie(best_route, route_cost) = getRoute(distances, demand[0], vehicle);
  std::cout << "Route: " << routeToString(best_route, " - ") << std::endl;
  std::cout << "Total cost: " << std::to_string(route_cost) << std::endl;
  std::cout << "It took "
            << benchmark(10, getRoute, distances, demand[0], vehicle) *
                   std::chrono::milliseconds::period::num /
                   std::chrono::milliseconds::period::den << "seconds"
            << std::endl;
  return 0;
}
