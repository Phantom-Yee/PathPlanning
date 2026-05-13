#define _USE_MATH_DEFINES
#include "pathplanning.h"

#define main pathplanning_dummy_main
#include "CCCP_deepseek_cpp_20260126.cpp"
#undef main

#include <fstream>
#include <sstream>
#include <string>

static bool loadLatLonPoints(const char* file, std::vector<Point>& pts) {
    std::ifstream in(file);
    if (!in.is_open()) return false;

    auto replace_all = [](std::string& s, const std::string& from, const std::string& to) {
        if (from.empty()) return;
        size_t pos = 0;
        while ((pos = s.find(from, pos)) != std::string::npos) {
            s.replace(pos, from.length(), to);
            pos += to.length();
        }
    };

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;

        // 去掉注释
        if (line.find('#') != std::string::npos) line = line.substr(0, line.find('#'));
        if (line.find("//") != std::string::npos) line = line.substr(0, line.find("//"));

        // 把各种分隔符统一替换为空格
        replace_all(line, u8"，", " ");
        replace_all(line, ",", " ");
        replace_all(line, ";", " ");
        replace_all(line, "\t", " ");
        replace_all(line, "(", " ");
        replace_all(line, ")", " ");

        std::stringstream ss(line);
        double x = 0.0, y = 0.0;
        if (!(ss >> x)) continue;
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

//static Point xyToLatlon(const Point& p, const Point& origin) {
//    const double R = 6378137.0;
//    double dLat = p.y / R;
//    double dLon = p.x / (R * std::cos(origin.y * M_PI / 180.0));
//    double lat = origin.y + dLat * 180.0 / M_PI;
//    double lon = origin.x + dLon * 180.0 / M_PI;
//    return Point(lon, lat);
//}

extern "C" PATHPLANNING_API int generate_path_from_file(const char* input_path,
                                                        const char* output_path,
                                                        double robot_width,
                                                        double optimize_dist,
                                                        double step_size)
{
    std::ofstream log("C:\\Users\\Alex\\Desktop\\pathplanning_log.txt", std::ios::app);
    log << "==== generate_path_from_file ====" << std::endl;
    log << "input_path: " << (input_path ? input_path : "null") << std::endl;
    log << "output_path: " << (output_path ? output_path : "null") << std::endl;
    log << "robot_width=" << robot_width
        << " optimize_dist=" << optimize_dist
        << " step_size=" << step_size << std::endl;

    if (!input_path || !output_path) { log << "return -1" << std::endl; return -1; }
    if (robot_width <= 0.0 || optimize_dist <= 0.0 || step_size <= 0.0) { log << "return -2" << std::endl; return -2; }

    std::vector<Point> boundaryLatLon;
    if (!loadLatLonPoints(input_path, boundaryLatLon)) { log << "return -3" << std::endl; return -3; }
    log << "boundary points=" << boundaryLatLon.size() << std::endl;

    Point origin = boundaryLatLon[0];
    std::vector<Point> boundaryXY;
    boundaryXY.reserve(boundaryLatLon.size());
    for (const auto& p : boundaryLatLon) {
        boundaryXY.push_back(latlonToXY(p, origin));
    }

    std::vector<Point> path = generateSpiralCoveragePath(boundaryXY, robot_width);
    log << "path size=" << path.size() << std::endl;

    std::vector<Point> optimized = optimizePath(path, optimize_dist);
    log << "optimized size=" << optimized.size() << std::endl;

    std::vector<Point> finalPath = discretizePath(optimized, step_size);
    log << "final size=" << finalPath.size() << std::endl;

//    std::vector<Point> finalLatLon;
//    finalLatLon.reserve(finalPath.size());
//    for (const auto& p : finalPath) {
//        finalLatLon.push_back(xyToLatlon(p, origin));
//    }

    if (!saveLatLonPoints(output_path, finalPath)) { return -4; }
    log << "return 0" << std::endl;
    return 0;
}
