// Research 1: Nav2 NavFn core (A* vs Dijkstra) on a raw global costmap captured from Nav2, without ROS messaging.
//
//   navfn_bench <costmap.bin> <nx> <ny> <resolution> <jobs.txt> <out.csv> [allow_unknown=1] [tolerance_m=0.5]
//   jobs.txt: one job per line "<id> <start_mx> <start_my> <goal_mx> <goal_my> <repeat>" (costmap cells)
//   out.csv : id,rep,order,algorithm,ok,time_ms,reached_cells,path_cells,path_length_m
//
// Same calls as nav2_navfn_planner::NavfnPlanner::makePlan (Humble):
//   setCostmap(char map, isROS=true, allow_unknown) -> setStart(goal), setGoal(start) (NavFn propagates from the
//   robot start) -> calcNavFnAstar() | calcNavFnDijkstra(true) -> goal (or the closest cell within the tolerance)
//   must have a finite potential -> calcPath(max(nx, ny) * 4) from it.
// time_ms covers all of that. reached_cells = cells that got a finite potential (NavFn's explored area).
// Build (container): g++ -O2 -std=c++17 -I/opt/ros/humble/include navfn_bench.cpp
//                    -L/opt/ros/humble/lib -lnav2_navfn_planner -Wl,-rpath,/opt/ros/humble/lib -o navfn_bench
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "nav2_navfn_planner/navfn.hpp"

using nav2_navfn_planner::NavFn;

struct Result
{
  bool ok;
  double ms;
  long reached;
  int path_cells;
  double length_m;
};

static Result run_once(NavFn & nav, const std::vector<unsigned char> & cmap, int nx, int ny, double res,
                       bool allow_unknown, double tolerance, int sx, int sy, int gx, int gy, bool astar)
{
  auto t0 = std::chrono::steady_clock::now();
  nav.setCostmap(cmap.data(), true, allow_unknown);
  int map_start[2] = {sx, sy};
  int map_goal[2] = {gx, gy};
  nav.setStart(map_goal);
  nav.setGoal(map_start);
  if (astar) {
    nav.calcNavFnAstar();
  } else {
    nav.calcNavFnDijkstra(true);
  }
  // goal, or the closest cell to it within the tolerance that has a finite potential (makePlan's tolerance search)
  int bx = -1, by = -1;
  double best = 1e30;
  int r = static_cast<int>(std::ceil(tolerance / res));
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      int x = gx + dx, y = gy + dy;
      if (x < 0 || y < 0 || x >= nx || y >= ny) {continue;}
      double d2 = (dx * dx + dy * dy) * res * res;
      if (d2 > tolerance * tolerance) {continue;}
      if (nav.potarr[y * nx + x] < POT_HIGH && d2 < best) {
        best = d2;
        bx = x;
        by = y;
      }
    }
  }
  int len = 0;
  if (bx >= 0) {
    int g[2] = {bx, by};
    nav.setStart(g);
    len = nav.calcPath(std::max(nx, ny) * 4);
  }
  auto t1 = std::chrono::steady_clock::now();
  Result out;
  out.ok = len > 0;
  out.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  out.reached = 0;
  for (int i = 0; i < nx * ny; ++i) {
    if (nav.potarr[i] < POT_HIGH) {++out.reached;}
  }
  out.path_cells = len;
  out.length_m = 0.0;
  if (len > 1) {
    float * px = nav.getPathX();
    float * py = nav.getPathY();
    for (int i = 1; i < len; ++i) {
      out.length_m += std::hypot(px[i] - px[i - 1], py[i] - py[i - 1]) * res;
    }
  }
  return out;
}

int main(int argc, char ** argv)
{
  if (argc < 7) {
    std::cerr << "usage: navfn_bench <costmap.bin> <nx> <ny> <resolution> <jobs.txt> <out.csv> [allow_unknown] [tol]\n";
    return 2;
  }
  int nx = std::atoi(argv[2]), ny = std::atoi(argv[3]);
  double res = std::atof(argv[4]);
  bool allow_unknown = argc > 7 ? std::atoi(argv[7]) != 0 : true;
  double tol = argc > 8 ? std::atof(argv[8]) : 0.5;
  std::vector<unsigned char> cmap(static_cast<size_t>(nx) * ny);
  std::ifstream f(argv[1], std::ios::binary);
  f.read(reinterpret_cast<char *>(cmap.data()), cmap.size());
  if (f.gcount() != static_cast<std::streamsize>(cmap.size())) {
    std::cerr << "costmap size mismatch\n";
    return 1;
  }
  NavFn nav(nx, ny);
  std::ifstream jobs(argv[5]);
  std::ofstream out(argv[6]);
  out << "id,rep,order,algorithm,ok,time_ms,reached_cells,path_cells,path_length_m\n";
  std::string line;
  while (std::getline(jobs, line)) {
    std::istringstream ss(line);
    std::string id;
    int sx, sy, gx, gy, reps;
    if (!(ss >> id >> sx >> sy >> gx >> gy >> reps)) {continue;}
    for (int w = 0; w < 2; ++w) {  // warm-up, not recorded
      run_once(nav, cmap, nx, ny, res, allow_unknown, tol, sx, sy, gx, gy, w == 0);
    }
    for (int rep = 0; rep < reps; ++rep) {
      for (int k = 0; k < 2; ++k) {
        bool astar = (rep % 2 == 0) ? (k == 0) : (k == 1);  // counterbalanced order
        Result r = run_once(nav, cmap, nx, ny, res, allow_unknown, tol, sx, sy, gx, gy, astar);
        out << id << ',' << rep << ',' << k << ',' << (astar ? "astar" : "dijkstra") << ',' << (r.ok ? 1 : 0) << ','
            << r.ms << ',' << r.reached << ',' << r.path_cells << ',' << r.length_m << '\n';
      }
    }
  }
  return 0;
}
