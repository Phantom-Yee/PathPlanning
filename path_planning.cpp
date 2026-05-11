#define _USE_MATH_DEFINES
#include "path_planning.h"

#define main pathplanning_dummy_main
#include "CCCP_deepseek_cpp_20260126.cpp"
#undef main

#include <fstream>
#include <sstream>
#include <string>

static bool loadLatLonPoints(const char* file, std::vector<Point>& pts) {
    std::ifstream in(file);
    if (!in.is_open()) return false;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        double x = 0.0, y = 0.0;
        char comma = 0;
        if (!(ss >> x)) continue;
        if (ss.peek() == ',' || ss.peek() == ' ') ss >> comma;
        if (!(ss >> y)) continue;
        pts.emplace_back(x, y);
    }
    return pts.size() >= 3;
}

static bool saveLatLonPoints(const char* file, const std::vector<Point>& pts) {
    std::ofstream out(file, std::ios::trunc);
    if (!out.is_open()) return false;
    for (const auto& p : pts) {
        out << p.x << ", " << p.y << "\n";
    }
    return true;
}

static Point latlonToXY(const Point& p, const Point& origin) {
    const double R = 6378137.0;
    double dLat = (p.y - origin.y) * M_PI / 180.0;
    double dLon = (p.x - origin.x) * M_PI / 180.0;
    double x = dLon * R * std::cos(origin.y * M_PI / 180.0);
    double y = dLat * R;
    return Point(x, y);
}

static Point xyToLatlon(const Point& p, const Point& origin) {
    const double R = 6378137.0;
    double dLat = p.y / R;
    double dLon = p.x / (R * std::cos(origin.y * M_PI / 180.0));
    double lat = origin.y + dLat * 180.0 / M_PI;
    double lon = origin.x + dLon * 180.0 / M_PI;
    return Point(lon, lat);
}

extern "C" PATHPLANNING_API int generate_path_from_file(const char* input_path,
                                                        const char* output_path,
                                                        double robot_width,
                                                        double optimize_dist,
                                                        double step_size)
{
    if (!input_path || !output_path) return -1;
    if (robot_width <= 0.0 || optimize_dist <= 0.0 || step_size <= 0.0) return -2;

    std::vector<Point> boundaryLatLon;
    if (!loadLatLonPoints(input_path, boundaryLatLon)) return -3;

    Point origin = boundaryLatLon[0];
    std::vector<Point> boundaryXY;
    boundaryXY.reserve(boundaryLatLon.size());
    for (const auto& p : boundaryLatLon) {
        boundaryXY.push_back(latlonToXY(p, origin));
    }

    std::vector<Point> path = generateSpiralCoveragePath(boundaryXY, robot_width);
    std::vector<Point> optimized = optimizePath(path, optimize_dist);
    std::vector<Point> finalPath = discretizePath(optimized, step_size);

    std::vector<Point> finalLatLon;
    finalLatLon.reserve(finalPath.size());
    for (const auto& p : finalPath) {
        finalLatLon.push_back(xyToLatlon(p, origin));
    }

    if (!saveLatLonPoints(output_path, finalLatLon)) return -4;
    return 0;
}
