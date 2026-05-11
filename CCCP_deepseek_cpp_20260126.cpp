/*
1. 算法思路：考虑机器人作业宽度固定的特点，基于"类等高线"生成间距固定的平行轨迹，实现螺旋式全覆盖路径规划。
“类等高线”，即从边界开始，每条边上的点按照相同的增量向内移动，这样形成的等间距线。
但是，需要注意的是，对于凸多边形，如果每条边都向内移动相同的距离，那么新的顶点（原顶点对应的新顶点）并不是简单地将原顶点沿内角平分线移动。
实际上，正确的做法是：将每条边向内偏移（沿法线方向移动一个距离），然后计算相邻两条偏移后线段的交点，这些交点就是新多边形的顶点。
2. 轨迹线段离散化时将距离下一个点距离不足一个步长的点滤掉，避免很短的离散化距离造成跟随控制不便
3. 多重停止条件组合：
基本检查：顶点数、面积绝对值
面积比例检查：面积缩减比例
距离检查：到外层/当前层的距离,如果内层多边形顶点到外层多边形最邻近边的距离小于0.5倍机器人工作宽度也停止生成。
几何退化检查：边长太短
相对面积检查：相对于原始面积的比例
层间距检查：相邻层间距是否合理
*/
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <array>

struct Point {
    double x, y;
    
    Point(double x = 0, double y = 0) : x(x), y(y) {}
    
    Point operator+(const Point& other) const { return Point(x + other.x, y + other.y); }
    Point operator-(const Point& other) const { return Point(x - other.x, y - other.y); }
    Point operator*(double scalar) const { return Point(x * scalar, y * scalar); }
    Point operator/(double scalar) const { return Point(x / scalar, y / scalar); }
    
    double dot(const Point& other) const { return x * other.x + y * other.y; }
    double cross(const Point& other) const { return x * other.y - y * other.x; }
    
    double distance(const Point& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    double length() const { return std::sqrt(x * x + y * y); }
    Point normalized() const {
        double len = length();
        return len == 0 ? Point(0, 0) : Point(x / len, y / len);
    }
    
    bool approxEqual(const Point& other, double epsilon = 1e-6) const {
        return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
    }
};

// 计算多边形面积
double polygonArea(const std::vector<Point>& polygon) {
    int n = polygon.size();
    if (n < 3) return 0.0;
    
    double area = 0.0;
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        area += polygon[i].x * polygon[j].y - polygon[j].x * polygon[i].y;
    }
    return std::abs(area) / 2.0;
}

// 检查多边形是否为凸多边形
bool isConvexPolygon(const std::vector<Point>& polygon) {
    int n = polygon.size();
    if (n < 3) return false;
    
    int sign = 0;
    for (int i = 0; i < n; i++) {
        Point a = polygon[i];
        Point b = polygon[(i + 1) % n];
        Point c = polygon[(i + 2) % n];
        
        Point ab = b - a;
        Point bc = c - b;
        double cross = ab.cross(bc);
        
        if (std::abs(cross) > 1e-10) {
            int curSign = cross > 0 ? 1 : -1;
            if (sign == 0) {
                sign = curSign;
            } else if (sign != curSign) {
                return false;
            }
        }
    }
    return true;
}

// 计算多边形中心（质心）
Point polygonCentroid(const std::vector<Point>& polygon) {
    int n = polygon.size();
    if (n == 0) return Point(0, 0);
    if (n == 1) return polygon[0];
    if (n == 2) return (polygon[0] + polygon[1]) * 0.5;
    
    double cx = 0.0, cy = 0.0, area = 0.0;
    
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        double cross = polygon[i].x * polygon[j].y - polygon[j].x * polygon[i].y;
        area += cross;
        cx += (polygon[i].x + polygon[j].x) * cross;
        cy += (polygon[i].y + polygon[j].y) * cross;
    }
    
    area /= 2.0;
    if (area == 0.0) {
        Point sum(0, 0);
        for (const auto& p : polygon) sum = sum + p;
        return sum / n;
    }
    
    return Point(cx / (6.0 * area), cy / (6.0 * area));
}

// 计算两条直线的交点（不考虑线段范围）
Point lineIntersection(const Point& p1, const Point& p2,
                      const Point& q1, const Point& q2) {
    Point r = p2 - p1;
    Point s = q2 - q1;
    
    double crossRS = r.cross(s);
    if (std::abs(crossRS) < 1e-10) {
        // 平行线，返回第一个线段的终点
        return p2;
    }
    
    Point qp = q1 - p1;
    double t = qp.cross(s) / crossRS;
    
    return Point(p1.x + r.x * t, p1.y + r.y * t);
}

std::vector<Point> segmentPolygonIntersection(const Point& p1, const Point& p2,
                                             const std::vector<Point>& polygon);
std::vector<Point> offsetPolygonByEdge(const std::vector<Point>& polygon, double distance);
double pointToPolygonDistance(const Point& point, const std::vector<Point>& polygon);

void appendPointIfNeeded(std::vector<Point>& path, const Point& p) {
    if (path.empty() || !p.approxEqual(path.back())) {
        path.push_back(p);
    }
}

Point pointOnContourByPhase(const std::vector<Point>& contour, double phase) {
    int n = static_cast<int>(contour.size());
    if (n == 0) return Point();
    if (n == 1) return contour[0];

    double wrapped = phase - std::floor(phase);
    double scaled = wrapped * n;
    int i = static_cast<int>(std::floor(scaled)) % n;
    double t = scaled - i;

    const Point& a = contour[i];
    const Point& b = contour[(i + 1) % n];
    return a * (1.0 - t) + b * t;
}

std::vector<Point> minimumAreaBoundingQuad(const std::vector<Point>& polygon) {
    if (polygon.size() < 3) return std::vector<Point>();

    double bestArea = std::numeric_limits<double>::max();
    double bestMinU = 0.0, bestMaxU = 0.0, bestMinV = 0.0, bestMaxV = 0.0;
    Point bestU(1.0, 0.0), bestV(0.0, 1.0);

    int n = static_cast<int>(polygon.size());
    for (int i = 0; i < n; i++) {
        Point a = polygon[i];
        Point b = polygon[(i + 1) % n];
        Point edge = b - a;
        double edgeLen = edge.length();
        if (edgeLen < 1e-10) continue;

        Point u = edge / edgeLen;
        Point v(-u.y, u.x);

        double minU = std::numeric_limits<double>::max();
        double maxU = -std::numeric_limits<double>::max();
        double minV = std::numeric_limits<double>::max();
        double maxV = -std::numeric_limits<double>::max();

        for (const auto& p : polygon) {
            double pu = p.dot(u);
            double pv = p.dot(v);
            minU = std::min(minU, pu);
            maxU = std::max(maxU, pu);
            minV = std::min(minV, pv);
            maxV = std::max(maxV, pv);
        }

        double area = (maxU - minU) * (maxV - minV);
        if (area < bestArea) {
            bestArea = area;
            bestMinU = minU;
            bestMaxU = maxU;
            bestMinV = minV;
            bestMaxV = maxV;
            bestU = u;
            bestV = v;
        }
    }

    Point p1 = bestU * bestMinU + bestV * bestMinV;
    Point p2 = bestU * bestMaxU + bestV * bestMinV;
    Point p3 = bestU * bestMaxU + bestV * bestMaxV;
    Point p4 = bestU * bestMinU + bestV * bestMaxV;
    return {p1, p2, p3, p4};
}

std::vector<Point> orientedBoundingQuadByDirection(const std::vector<Point>& polygon,
                                                   const Point& directionHint) {
    if (polygon.size() < 3) return std::vector<Point>();

    Point u = directionHint.normalized();
    if (u.length() < 1e-10) {
        u = (polygon[1] - polygon[0]).normalized();
    }
    if (u.length() < 1e-10) {
        return minimumAreaBoundingQuad(polygon);
    }
    Point v(-u.y, u.x);

    double minU = std::numeric_limits<double>::max();
    double maxU = -std::numeric_limits<double>::max();
    double minV = std::numeric_limits<double>::max();
    double maxV = -std::numeric_limits<double>::max();

    for (const auto& p : polygon) {
        double pu = p.dot(u);
        double pv = p.dot(v);
        minU = std::min(minU, pu);
        maxU = std::max(maxU, pu);
        minV = std::min(minV, pv);
        maxV = std::max(maxV, pv);
    }

    Point p1 = u * minU + v * minV;
    Point p2 = u * maxU + v * minV;
    Point p3 = u * maxU + v * maxV;
    Point p4 = u * minU + v * maxV;
    return {p1, p2, p3, p4};
}

std::vector<Point> generateZigzagInQuad(const std::vector<Point>& quad, double w, const Point& entryPoint) {
    if (quad.size() != 4 || w <= 0.0) return std::vector<Point>();

    Point origin = quad[0];
    Point edgeU = quad[1] - quad[0];
    Point edgeV = quad[3] - quad[0];
    double lenU = edgeU.length();
    double lenV = edgeV.length();

    if (lenU < 1e-10 || lenV < 1e-10) return std::vector<Point>();

    Point u = edgeU / lenU;
    Point v = edgeV / lenV;

    // 缩小边界留白，减少裁剪后边缘空白区。
    double margin = std::min(0.2 * w, 0.1 * std::min(lenU, lenV));
    double uMin = margin;
    double uMax = lenU - margin;
    double vMin = margin;
    double vMax = lenV - margin;

    if (uMax <= uMin || vMax < vMin) {
        uMin = 0.0;
        uMax = lenU;
        vMin = 0.0;
        vMax = lenV;
    }

    double spanV = std::max(0.0, vMax - vMin);
    // 关键约束：车道间距固定为w，不再把spanV均分到各车道。
    int laneCount = std::max(1, static_cast<int>(std::floor(spanV / w + 1e-9)) + 1);
    double usedSpan = (laneCount > 1) ? (laneCount - 1) * w : 0.0;
    double sidePadding = std::max(0.0, 0.5 * (spanV - usedSpan));
    double laneStartV = (laneCount == 1) ? (0.5 * (vMin + vMax)) : (vMin + sidePadding);

    auto buildPattern = [&](bool reverseLanes, bool firstLaneToMaxU) {
        std::vector<Point> pattern;
        for (int i = 0; i < laneCount; i++) {
            int laneIndex = reverseLanes ? (laneCount - 1 - i) : i;
            double laneV = laneStartV + laneIndex * w;
            laneV = std::max(vMin, std::min(vMax, laneV));
            Point left = origin + u * uMin + v * laneV;
            Point right = origin + u * uMax + v * laneV;

            bool toMax = (i % 2 == 0) ? firstLaneToMaxU : !firstLaneToMaxU;
            if (toMax) {
                pattern.push_back(left);
                pattern.push_back(right);
            } else {
                pattern.push_back(right);
                pattern.push_back(left);
            }
        }
        return pattern;
    };

    std::array<std::vector<Point>, 4> candidates = {
        buildPattern(false, true),
        buildPattern(false, false),
        buildPattern(true, true),
        buildPattern(true, false)
    };

    double bestDist = std::numeric_limits<double>::max();
    int bestIdx = 0;
    for (int i = 0; i < 4; i++) {
        if (candidates[i].empty()) continue;
        double d = entryPoint.distance(candidates[i][0]);
        if (d < bestDist) {
            bestDist = d;
            bestIdx = i;
        }
    }

    return candidates[bestIdx];
}

void appendSmoothTransition(std::vector<Point>& path,
                            const Point& targetStart,
                            const Point& targetNext,
                            double w) {
    if (path.empty()) {
        path.push_back(targetStart);
        return;
    }

    Point p0 = path.back();
    Point pPrev = (path.size() >= 2) ? path[path.size() - 2] : p0;
    Point t0 = (p0 - pPrev).normalized();
    Point t1 = (targetNext - targetStart).normalized();

    if (t0.length() < 1e-10) {
        t0 = (targetStart - p0).normalized();
    }
    if (t1.length() < 1e-10) {
        t1 = (targetStart - p0).normalized();
    }

    double d = p0.distance(targetStart);
    if (d < 1e-6) {
        appendPointIfNeeded(path, targetStart);
        return;
    }

    double handle = std::min(0.45 * d, std::max(0.6 * w, 0.2 * d));
    Point c1 = p0 + t0 * handle;
    Point c2 = targetStart - t1 * handle;

    int samples = std::max(8, static_cast<int>(std::ceil(d / std::max(1.0, 0.5 * w))));
    for (int i = 1; i <= samples; i++) {
        double t = static_cast<double>(i) / static_cast<double>(samples);
        double omt = 1.0 - t;
        Point p = p0 * (omt * omt * omt)
                + c1 * (3.0 * omt * omt * t)
                + c2 * (3.0 * omt * t * t)
                + targetStart * (t * t * t);
        appendPointIfNeeded(path, p);
    }
}

bool pointInConvexPolygon(const Point& p, const std::vector<Point>& polygon, double eps = 1e-8) {
    int n = static_cast<int>(polygon.size());
    if (n < 3) return false;

    int sign = 0;
    for (int i = 0; i < n; i++) {
        const Point& a = polygon[i];
        const Point& b = polygon[(i + 1) % n];
        double cross = (b - a).cross(p - a);

        if (std::abs(cross) <= eps) continue;
        int curSign = (cross > 0.0) ? 1 : -1;
        if (sign == 0) {
            sign = curSign;
        } else if (sign != curSign) {
            return false;
        }
    }
    return true;
}

double segmentParameter(const Point& a, const Point& b, const Point& p) {
    Point ab = b - a;
    double denom = ab.dot(ab);
    if (denom < 1e-12) return 0.0;
    return (p - a).dot(ab) / denom;
}

std::vector<Point> clipSegmentWithConvexPolygon(const Point& a,
                                                const Point& b,
                                                const std::vector<Point>& polygon) {
    std::vector<Point> candidates;

    if (pointInConvexPolygon(a, polygon)) {
        appendPointIfNeeded(candidates, a);
    }
    if (pointInConvexPolygon(b, polygon)) {
        appendPointIfNeeded(candidates, b);
    }

    std::vector<Point> intersections = segmentPolygonIntersection(a, b, polygon);
    for (const auto& p : intersections) {
        appendPointIfNeeded(candidates, p);
    }

    if (candidates.size() < 2) {
        return std::vector<Point>();
    }

    std::sort(candidates.begin(), candidates.end(), [&](const Point& p1, const Point& p2) {
        return segmentParameter(a, b, p1) < segmentParameter(a, b, p2);
    });

    if (candidates.front().distance(candidates.back()) < 1e-8) {
        return std::vector<Point>();
    }

    return {candidates.front(), candidates.back()};
}

std::vector<Point> generateZigzagInsidePolygonByQuad(const std::vector<Point>& quad,
                                                     const std::vector<Point>& polygon,
                                                     double w,
                                                     const Point& entryPoint) {
    std::vector<Point> rawZigzag = generateZigzagInQuad(quad, w, entryPoint);
    if (rawZigzag.size() < 2 || polygon.size() < 3) {
        return std::vector<Point>();
    }

    std::vector<Point> clipped;
    for (size_t i = 0; i + 1 < rawZigzag.size(); i += 2) {
        const Point& a = rawZigzag[i];
        const Point& b = rawZigzag[i + 1];
        std::vector<Point> clippedSeg = clipSegmentWithConvexPolygon(a, b, polygon);
        if (clippedSeg.size() == 2) {
            appendPointIfNeeded(clipped, clippedSeg[0]);
            appendPointIfNeeded(clipped, clippedSeg[1]);
        }
    }

    return clipped;
}

Point estimateTailDirection(const std::vector<Point>& path) {
    if (path.size() < 2) return Point(1.0, 0.0);
    const Point& tail = path.back();
    for (int i = static_cast<int>(path.size()) - 2; i >= 0; i--) {
        Point d = tail - path[i];
        if (d.length() > 1e-6) return d.normalized();
    }
    return Point(1.0, 0.0);
}

double zigzagEntryScore(const Point& entryPoint,
                        const Point& tailDir,
                        const Point& start,
                        const Point& next,
                        double w) {
    double distCost = entryPoint.distance(start);
    Point t = tailDir.normalized();
    Point z = (next - start).normalized();
    if (t.length() < 1e-10 || z.length() < 1e-10) {
        return distCost;
    }

    double cosTheta = std::max(-1.0, std::min(1.0, t.dot(z)));
    double turnPenalty = (1.0 - cosTheta) * (1.2 * w);
    return distCost + turnPenalty;
}

Point closestPointOnSegment(const Point& p, const Point& a, const Point& b) {
    Point ab = b - a;
    double denom = ab.dot(ab);
    if (denom < 1e-12) return a;
    double t = (p - a).dot(ab) / denom;
    t = std::max(0.0, std::min(1.0, t));
    return a + ab * t;
}

std::vector<Point> generateBoundaryToNextCorner(const std::vector<Point>& boundary,
                                                const Point& startPoint,
                                                const Point& dirHint) {
    if (boundary.size() < 2) return std::vector<Point>();

    int n = static_cast<int>(boundary.size());
    int bestEdge = -1;
    double bestDist = std::numeric_limits<double>::max();
    Point startOnEdge;

    for (int i = 0; i < n; i++) {
        Point a = boundary[i];
        Point b = boundary[(i + 1) % n];
        Point c = closestPointOnSegment(startPoint, a, b);
        double d = c.distance(startPoint);
        if (d < bestDist) {
            bestDist = d;
            bestEdge = i;
            startOnEdge = c;
        }
    }

    if (bestEdge < 0) return std::vector<Point>();

    Point edgeA = boundary[bestEdge];
    Point edgeB = boundary[(bestEdge + 1) % n];
    Point edgeDir = (edgeB - edgeA).normalized();
    Point hint = dirHint.normalized();
    if (hint.length() < 1e-10) hint = edgeDir;

    bool forward = (edgeDir.dot(hint) >= 0.0);
    std::vector<Point> starter;
    starter.push_back(startOnEdge);
    Point nextCorner = forward ? boundary[(bestEdge + 1) % n] : boundary[bestEdge];
    appendPointIfNeeded(starter, nextCorner);

    return starter;
}

std::vector<Point> generateParallelLeadSegment(const std::vector<Point>& boundary,
                                               double w,
                                               const Point& entryPoint,
                                               const Point& tangent) {
    if (boundary.size() < 3 || w <= 0.0) return std::vector<Point>();

    // 与最后一圈等高轨迹保持间距w的内偏移轮廓。
    std::vector<Point> leadContour = offsetPolygonByEdge(boundary, w);
    if (leadContour.size() < 3 || polygonArea(leadContour) < 1e-6) {
        return std::vector<Point>();
    }

    Point t = tangent.normalized();
    if (t.length() < 1e-10) t = Point(1.0, 0.0);

    int n = static_cast<int>(leadContour.size());
    int bestIdx = -1;
    double bestScore = -std::numeric_limits<double>::max();
    for (int i = 0; i < n; i++) {
        Point a = leadContour[i];
        Point b = leadContour[(i + 1) % n];
        Point e = b - a;
        double len = e.length();
        if (len < 1e-6) continue;

        Point dir = e / len;
        double parallel = std::abs(dir.dot(t));
        double dist = entryPoint.distance(closestPointOnSegment(entryPoint, a, b));
        double score = parallel * 1000.0 - dist;
        if (score > bestScore) {
            bestScore = score;
            bestIdx = i;
        }
    }

    if (bestIdx < 0) return std::vector<Point>();

    Point a = leadContour[bestIdx];
    Point b = leadContour[(bestIdx + 1) % n];
    Point e = b - a;
    double len = e.length();
    if (len < 1e-6) return std::vector<Point>();

    Point dir = e / len;
    if (dir.dot(t) < 0.0) {
        std::swap(a, b);
        dir = dir * -1.0;
    }

    Point start = closestPointOnSegment(entryPoint, a, b);
    double remain = (b - start).length();
    double leadLen = std::min(remain, 1.5 * w);
    if (leadLen < 0.5 * w) return std::vector<Point>();
    Point end = start + dir * leadLen;

    return {start, end};
}

std::vector<Point> generateForcedParallelStarterInPolygon(const std::vector<Point>& polygon,
                                                          const Point& startPoint,
                                                          const Point& parallelDir,
                                                          double w) {
    if (polygon.size() < 3 || w <= 0.0) return std::vector<Point>();

    Point dir = parallelDir.normalized();
    if (dir.length() < 1e-10) return std::vector<Point>();

    double far = 1e4;
    Point p1 = startPoint - dir * far;
    Point p2 = startPoint + dir * far;
    std::vector<Point> clipped = clipSegmentWithConvexPolygon(p1, p2, polygon);
    if (clipped.size() != 2) return std::vector<Point>();

    double tA = (clipped[0] - startPoint).dot(dir);
    double tB = (clipped[1] - startPoint).dot(dir);
    double tForward = std::max(tA, tB);
    if (tForward < 0.3 * w) return std::vector<Point>();

    double len = std::min(w, tForward);
    Point end = startPoint + dir * len;
    return {startPoint, end};
}

// 将每条边向内平移固定距离（生成类等高线）
std::vector<Point> offsetPolygonByEdge(const std::vector<Point>& polygon, double distance) {
    int n = polygon.size();
    if (n < 3) return polygon;
    
    std::vector<Point> result;
    Point center = polygonCentroid(polygon);
    
    for (int i = 0; i < n; i++) {
        Point prev = polygon[(i - 1 + n) % n];
        Point curr = polygon[i];
        Point next = polygon[(i + 1) % n];
        
        // 计算两条边的方向向量
        Point dir1 = (curr - prev).normalized();
        Point dir2 = (next - curr).normalized();
        
        // 计算内法线（垂直于边，指向多边形内部）
        Point normal1(-dir1.y, dir1.x);
        Point normal2(-dir2.y, dir2.x);
        
        // 确保法向量指向内部
        Point toCenter = (center - curr).normalized();
        if (normal1.dot(toCenter) < 0) normal1 = normal1 * -1;
        if (normal2.dot(toCenter) < 0) normal2 = normal2 * -1;
        
        // 偏移两条边
        Point offsetLine1Start = prev + normal1 * distance;
        Point offsetLine1End = curr + normal1 * distance;
        Point offsetLine2Start = curr + normal2 * distance;
        Point offsetLine2End = next + normal2 * distance;
        
        // 计算两条偏移边的交点
        Point intersection = lineIntersection(offsetLine1Start, offsetLine1End,
                                             offsetLine2Start, offsetLine2End);
        
        result.push_back(intersection);
    }
    
    return result;
}



// 计算点到多边形最近边的距离
double pointToPolygonDistance(const Point& point, const std::vector<Point>& polygon) {
    int n = polygon.size();
    if (n < 2) return std::numeric_limits<double>::max();
    
    double minDistance = std::numeric_limits<double>::max();
    
    for (int i = 0; i < n; i++) {
        Point p1 = polygon[i];
        Point p2 = polygon[(i + 1) % n];
        
        // 计算点到线段的距离
        Point lineVec = p2 - p1;
        Point pointVec = point - p1;
        
        double lineLength = lineVec.length();
        if (lineLength < 1e-10) {
            // 线段长度接近0，直接计算点到点的距离
            double dist = point.distance(p1);
            minDistance = std::min(minDistance, dist);
            continue;
        }
        
        Point unitLineVec = lineVec / lineLength;
        
        // 计算投影长度
        double projection = pointVec.dot(unitLineVec);
        
        if (projection <= 0) {
            // 投影在线段起点之前，最近点是起点
            double dist = point.distance(p1);
            minDistance = std::min(minDistance, dist);
        } else if (projection >= lineLength) {
            // 投影在线段终点之后，最近点是终点
            double dist = point.distance(p2);
            minDistance = std::min(minDistance, dist);
        } else {
            // 投影在线段上，计算垂直距离
            Point projectionPoint = p1 + unitLineVec * projection;
            double dist = point.distance(projectionPoint);
            minDistance = std::min(minDistance, dist);
        }
    }
    
    return minDistance;
}



// 改进的生成平行轨迹函数
std::vector<std::vector<Point>> generateParallelContours(const std::vector<Point>& polygon,
                                                        double w,
                                                        std::vector<Point>* remainingPolygon = nullptr,
                                                        double remainingAreaRatio = 0.08,
                                                        int maxLayers = 1000,
                                                        bool* thresholdHit = nullptr) {
    std::vector<std::vector<Point>> contours;

    if (thresholdHit) {
        *thresholdHit = false;
    }
    
    if (!isConvexPolygon(polygon)) {
        std::cerr << "警告：多边形可能不是凸多边形，路径规划可能不准确！" << std::endl;
    }
    
    // 添加最外层（原始多边形）
    contours.push_back(polygon);
    
    // 生成内层等高线
    std::vector<Point> currentPolygon = polygon;
    int layer = 1;
    double initialArea = polygonArea(polygon);
    double thresholdArea = std::max(w * w, initialArea * remainingAreaRatio);
    bool thresholdReached = false;
    
    while (layer <= maxLayers) {
        std::vector<Point> nextPolygon = offsetPolygonByEdge(currentPolygon, w);
        
        // 检查是否应该停止
        
        // 1. 基本检查：多边形顶点数、面积
        if (nextPolygon.size() < 3) {
            std::cout << "停止条件 " << layer << ": 顶点数不足 (" << nextPolygon.size() << " < 3)" << std::endl;
            break;
        }
        
        double currentArea = polygonArea(currentPolygon);
        double nextArea = polygonArea(nextPolygon);

        if (nextArea <= thresholdArea) {
            std::cout << "阈值触发 " << layer << ": 剩余面积 " << nextArea
                      << " <= " << thresholdArea << "，停止继续内缩" << std::endl;
            if (remainingPolygon) {
                // 阈值触发时保留当前层，后续用该剩余区域做二阶段填充。
                *remainingPolygon = currentPolygon;
            }
            if (thresholdHit) {
                *thresholdHit = true;
            }
            thresholdReached = true;
            break;
        }
        
        // 2. 面积检查
        if (nextArea < w * w * 0.5) {  // 放宽到0.5倍w²
            std::cout << "停止条件 " << layer << ": 面积太小 (" << nextArea << " < " << w*w*0.5 << ")" << std::endl;
            break;
        }
        
        // 3. 面积缩减比例检查
        double areaRatio = nextArea / currentArea;
        if (areaRatio < 0.05) {  // 从0.01调整到0.05
            std::cout << "停止条件 " << layer << ": 面积缩减比例太低 (" << areaRatio*100 << "% < 5%)" << std::endl;
            break;
        }
        
        // 4. 新检查：内层多边形顶点到外层多边形的最短距离是否小于0.5倍工作宽度
        bool tooCloseToOuter = false;
        double minDistanceToOuter = std::numeric_limits<double>::max();
        
        // 计算内层多边形每个顶点到原始多边形（最外层）的距离
        for (const auto& point : nextPolygon) {
            double dist = pointToPolygonDistance(point, polygon);  // 使用原始多边形作为参考
            
            if (dist < minDistanceToOuter) {
                minDistanceToOuter = dist;
            }
            
            // 如果距离小于0.5倍工作宽度，认为太接近了
            if (dist < 0.5 * w) {
                tooCloseToOuter = true;
                // 不立即break，继续计算最小距离用于调试
            }
        }
        
        if (tooCloseToOuter) {
            std::cout << "停止条件 " << layer << ": 内层顶点距离外层太近 (min=" << minDistanceToOuter 
                      << " < 0.5w=" << 0.5*w << ")" << std::endl;
            break;
        }
        
        // 5. 新检查：内层多边形顶点到当前外层多边形（上一层）的距离
        bool tooCloseToCurrent = false;
        double minDistanceToCurrent = std::numeric_limits<double>::max();
        
        for (const auto& point : nextPolygon) {
            double dist = pointToPolygonDistance(point, currentPolygon);
            
            if (dist < minDistanceToCurrent) {
                minDistanceToCurrent = dist;
            }
            
            // 如果距离小于1.5倍工作宽度，认为太接近了
            if (dist < 0.5 * w) {
                tooCloseToCurrent = true;
            }
        }
        
        if (tooCloseToCurrent) {
            std::cout << "停止条件 " << layer << ": 内层顶点距离当前层太近 (min=" << minDistanceToCurrent 
                      << " < 0.5w=" << 0.5*w << ")" << std::endl;
            break;
        }
        
        // 6. 新检查：多边形是否退化（边长是否太短）
        bool polygonDegenerated = false;
        int n = nextPolygon.size();
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double edgeLength = nextPolygon[i].distance(nextPolygon[j]);
            
            if (edgeLength < w * 0.3) {  // 边长小于0.3倍工作宽度认为退化
                polygonDegenerated = true;
                std::cout << "停止条件 " << layer << ": 边长短 (" << edgeLength << " < 0.3w=" << 0.3*w << ")" << std::endl;
                break;
            }
        }
        
        if (polygonDegenerated) {
            break;
        }
        
        // 7. 新检查：面积相对于原始面积是否太小
        double relativeArea = nextArea / initialArea;
        if (relativeArea < 0.02) {  // 小于原始面积的2%
            std::cout << "停止条件 " << layer << ": 相对面积太小 (" << relativeArea*100 << "% < 2%)" << std::endl;
            break;
        }
        
        // 8. 检查相邻层间距是否合理
        if (layer >= 2) {
            const std::vector<Point>& outerLayer = contours[layer-1];
            double avgDistance = 0.0;
            int count = 0;
            
            for (const auto& point : nextPolygon) {
                double dist = pointToPolygonDistance(point, outerLayer);
                avgDistance += dist;
                count++;
            }
            
            if (count > 0) {
                avgDistance /= count;
                // 平均距离应该接近工作宽度w，如果显著小于w，可能有问题
                if (avgDistance < w * 0.5) {
                    std::cout << "停止条件 " << layer << ": 层间距太小 (avg=" << avgDistance 
                              << " < 0.5w=" << 0.5*w << ")" << std::endl;
                    break;
                }
            }
        }
        
        // 所有检查通过，添加新层
        contours.push_back(nextPolygon);
        currentPolygon = nextPolygon;
        layer++;
        
        // 调试信息：每5层打印一次状态
        if (layer % 5 == 0) {
            std::cout << "已生成 " << layer << " 层，当前面积: " << nextArea 
                      << "，相对原始面积: " << (nextArea / initialArea * 100) << "%" << std::endl;
        }
    }
    
    std::cout << "总共生成 " << contours.size() << " 层平行轨迹" << std::endl;

    if (remainingPolygon && !thresholdReached) {
        *remainingPolygon = currentPolygon;
    }
    
    // 验证相邻层间距
    if (contours.size() >= 2) {
        std::cout << "相邻层间距验证:" << std::endl;
        for (size_t i = 0; i < contours.size() - 1; i++) {
            const auto& layer1 = contours[i];
            const auto& layer2 = contours[i + 1];
            
            double totalDist = 0.0;
            int count = 0;
            
            for (const auto& point : layer2) {
                totalDist += pointToPolygonDistance(point, layer1);
                count++;
            }
            
            if (count > 0) {
                double avgDist = totalDist / count;
                double ratio = avgDist / w;
                std::cout << "  层" << i+1 << "->层" << i+2 << ": 平均距离=" << avgDist 
                          << "，与w的比值=" << ratio << (ratio < 0.8 ? " (警告)" : "") << std::endl;
            }
        }
    }
    
    return contours;
}

// 计算线段与多边形的交点（用于生成轨迹连接点）
std::vector<Point> segmentPolygonIntersection(const Point& p1, const Point& p2,
                                             const std::vector<Point>& polygon) {
    std::vector<Point> intersections;
    int n = polygon.size();
    
    for (int i = 0; i < n; i++) {
        Point q1 = polygon[i];
        Point q2 = polygon[(i + 1) % n];
        
        Point r = p2 - p1;
        Point s = q2 - q1;
        
        double crossRS = r.cross(s);
        if (std::abs(crossRS) < 1e-10) continue;
        
        Point qp = q1 - p1;
        double t = qp.cross(s) / crossRS;
        double u = qp.cross(r) / crossRS;
        
        if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
            intersections.push_back(p1 + r * t);
        }
    }
    
    // 按距离p1从近到远排序
    std::sort(intersections.begin(), intersections.end(),
              [&p1](const Point& a, const Point& b) {
                  return p1.distance(a) < p1.distance(b);
              });
    
    return intersections;
}

// 生成螺旋式全覆盖路径（基于平行轨迹）
std::vector<Point> generateSpiralCoveragePath(const std::vector<Point>& polygon, double w) {
    // 生成平行轨迹（类等高线）
    std::vector<Point> remainingPolygon;
    bool thresholdTriggered = false;
    // 提高阈值占比，让规划更早切换到二阶段之字形填充。
    const double remainingAreaRatioForZigzag = 0.18;
    std::vector<std::vector<Point>> contours = generateParallelContours(
        polygon, w, &remainingPolygon, remainingAreaRatioForZigzag, 1000, &thresholdTriggered);
    //std::vector<std::vector<Point>> contours = generateParallelContoursStrict(polygon, w);
    
    if (contours.empty()) {
        return std::vector<Point>();
    }
    
    std::vector<Point> fullPath;

    // 连续等高螺旋：在相邻等高线之间按相位插值，形成无“连接缝线”的向内盘旋轨迹。
    // 相位起点取最左侧顶点附近，便于与常见覆盖起始姿态一致。
    int startVertex = 0;
    double minX = std::numeric_limits<double>::max();
    for (size_t i = 0; i < contours[0].size(); i++) {
        if (contours[0][i].x < minX) {
            minX = contours[0][i].x;
            startVertex = static_cast<int>(i);
        }
    }
    double startPhase = static_cast<double>(startVertex) / static_cast<double>(contours[0].size());

    if (contours.size() == 1) {
        for (const auto& p : contours[0]) appendPointIfNeeded(fullPath, p);
        return fullPath;
    }

    // 第一圈外扩相关代码先保留但注释，当前不做首圈外扩。
    // const double firstLoopOutwardOffset = 1.0 * w;
    // std::vector<Point> firstLoopOuterContour = contours[0];
    // if (firstLoopOutwardOffset > 1e-10) {
    //     std::vector<Point> expanded = offsetPolygonByEdge(contours[0], -firstLoopOutwardOffset);
    //     if (expanded.size() == contours[0].size()) {
    //         firstLoopOuterContour = expanded;
    //     }
    // }

    // 首圈整圈单独执行会让第一圈看起来近似闭合，这里先停用。
    // {
    //     int n0 = static_cast<int>(firstLoopOuterContour.size());
    //     int samplesPerEdge0 = std::max(8, static_cast<int>(std::ceil(12.0 / std::max(1.0, w))));
    //     int totalSteps0 = std::max(1, n0 * samplesPerEdge0);
    //     for (int step = 0; step < totalSteps0; step++) {
    //         double alpha = static_cast<double>(step) / static_cast<double>(totalSteps0);
    //         double phase = startPhase + alpha;
    //         appendPointIfNeeded(fullPath, pointOnContourByPhase(firstLoopOuterContour, phase));
    //     }
    // }

    for (size_t layer = 0; layer + 1 < contours.size(); layer++) {
        const std::vector<Point>& outer = contours[layer];
        const std::vector<Point>& inner = contours[layer + 1];
        int n = static_cast<int>(outer.size());
        int samplesPerEdge = std::max(8, static_cast<int>(std::ceil(12.0 / std::max(1.0, w))));
        int totalSteps = std::max(1, n * samplesPerEdge);

        for (int step = 0; step < totalSteps; step++) {
            double alpha = static_cast<double>(step) / static_cast<double>(totalSteps);
            double phase = startPhase + alpha;

            Point pOuter = pointOnContourByPhase(outer, phase);
            Point pInner = pointOnContourByPhase(inner, phase);
            Point p = pOuter * (1.0 - alpha) + pInner * alpha;

            appendPointIfNeeded(fullPath, p);
        }
    }

    // 将末端补到最内层轮廓起始相位点，保证螺旋自然收拢。
    // 注意这里是开口终点，不再回到外层起点，因此整体仍是一条非闭环曲线。
    appendPointIfNeeded(fullPath, pointOnContourByPhase(contours.back(), startPhase));

    // 二阶段：阈值触发后，以最后一圈轨迹作为剩余区域边界，转为同间距之字形。
    if (thresholdTriggered && contours.back().size() >= 3) {
        const std::vector<Point>& boundary = contours.back();
        Point entryPoint = fullPath.empty() ? boundary[0] : fullPath.back();
        Point tangent = estimateTailDirection(fullPath);

        // 步骤1：先沿最后一圈边界走到下一个拐点（顶点），再切换之字形。
        std::vector<Point> starter = generateBoundaryToNextCorner(boundary, entryPoint, tangent);
        if (starter.size() >= 2) {
            if (!fullPath.empty() && fullPath.back().distance(starter[0]) <= 1.2 * w) {
                fullPath.back() = starter[0];
            } else {
                appendSmoothTransition(fullPath, starter[0], starter[1], 0.7 * w);
            }

            for (size_t i = 1; i < starter.size(); i++) {
                appendPointIfNeeded(fullPath, starter[i]);
            }
            entryPoint = fullPath.back();
        }

        // 步骤2：以最后一圈等高线作为之字形边界（按闭合边界处理，路径本身不闭环）。
        std::vector<Point> remainingForZigzag = boundary;

        // 步骤2.1：到达拐点后，先走一段与相邻等高螺旋线平行的引导段（长度约w）。
        // 先尝试在剩余区域内强制生成该段，失败时回退到边界邻近平行段。
        {
            Point cornerTangent = estimateTailDirection(fullPath);
            std::vector<Point> lead = generateForcedParallelStarterInPolygon(
                remainingForZigzag, entryPoint, cornerTangent, w);
            if (lead.size() < 2) {
                lead = generateParallelLeadSegment(boundary, w, entryPoint, cornerTangent);
            }

            if (lead.size() >= 2) {
                appendPointIfNeeded(fullPath, lead[0]);
                appendPointIfNeeded(fullPath, lead[1]);
                entryPoint = fullPath.back();
            }
        }

        Point zigzagDir = estimateTailDirection(fullPath);
        if (zigzagDir.length() < 1e-10) {
            zigzagDir = tangent;
        }
        std::vector<Point> quad = orientedBoundingQuadByDirection(remainingForZigzag, zigzagDir);
        if (quad.size() != 4) {
            quad = minimumAreaBoundingQuad(remainingForZigzag);
        }
        if (quad.size() == 4) {
            std::vector<Point> zigzag = generateZigzagInsidePolygonByQuad(quad, remainingForZigzag, w, entryPoint);
            if (zigzag.size() < 2) {
                zigzag = generateZigzagInQuad(quad, w, entryPoint);
            }

            if (zigzag.size() >= 2) {
                Point tailAfterCorner = estimateTailDirection(fullPath);
                double forwardStartDist = entryPoint.distance(zigzag[0]);
                double reverseStartDist = entryPoint.distance(zigzag[zigzag.size() - 1]);
                double forwardScore = zigzagEntryScore(
                    entryPoint, tailAfterCorner, zigzag[0], zigzag[1], w);
                double reverseScore = zigzagEntryScore(
                    entryPoint, tailAfterCorner,
                    zigzag[zigzag.size() - 1], zigzag[zigzag.size() - 2], w);

                // 从当前拐点出发，先保证起点离拐点最近，再在接近时比较方向顺滑性。
                bool useReverse = false;
                if (reverseStartDist + 0.2 * w < forwardStartDist) {
                    useReverse = true;
                } else if (std::abs(reverseStartDist - forwardStartDist) <= 0.2 * w
                           && reverseScore + 1e-9 < forwardScore) {
                    useReverse = true;
                }

                if (useReverse) {
                    std::reverse(zigzag.begin(), zigzag.end());
                }

                if (entryPoint.distance(zigzag[0]) > 0.3 * w) {
                    appendSmoothTransition(fullPath, zigzag[0], zigzag[1], 0.6 * w);
                }
                for (const auto& p : zigzag) {
                    appendPointIfNeeded(fullPath, p);
                }
            }
        }
    }
    
    return fullPath;
}

// 优化路径：移除过于接近的点
std::vector<Point> optimizePath(const std::vector<Point>& path, double minDistance) {
    if (path.size() < 2) return path;
    
    std::vector<Point> optimized;
    optimized.push_back(path[0]);
    
    for (size_t i = 1; i < path.size(); i++) {
        double dist = optimized.back().distance(path[i]);
        
        if (dist >= minDistance) {
            optimized.push_back(path[i]);
        }
    }
    
    return optimized;
}

// 离散化路径（用于移动平台控制）
std::vector<Point> discretizePath(const std::vector<Point>& path, double stepSize) {
    std::vector<Point> discretized;
    
    if (path.empty() || stepSize <= 0) {
        return discretized;
    }
    
    discretized.push_back(path[0]);
    
    for (size_t i = 0; i < path.size() - 1; i++) {
        Point start = path[i];
        Point end = path[i + 1];
        
        double segmentLength = start.distance(end);
        if (segmentLength < 1e-10) continue;
        
        Point direction = (end - start).normalized();
        int steps = static_cast<int>(segmentLength / stepSize);
/*        
        for (int step = 1; step <= steps; step++) {
            double distance = step * stepSize;
            if (distance >= segmentLength - 1e-10) break;
            
            discretized.push_back(start + direction * distance);
        }
*/
       for (int step = 1; step <= steps; step++) {
            double distance = step * stepSize;
            if (distance >= segmentLength - 1e-10) break;

            Point current = start + direction * distance; 
            double dist2end = current.distance( end );
        
            if ( dist2end > stepSize )
                discretized.push_back( current );
        }
        
        discretized.push_back(end);
    }
    
    // 移除可能的重复点
    std::vector<Point> result;
    for (const auto& p : discretized) {
        if (result.empty() || !p.approxEqual(result.back())) {
            result.push_back(p);
        }
    }
    
    return result;
}

// 计算路径总长度
double calculatePathLength(const std::vector<Point>& path) {
    if (path.size() < 2) return 0.0;
    
    double totalLength = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        totalLength += path[i-1].distance(path[i]);
    }
    return totalLength;
}



/*------------------------------------------------------------------*/
/*------------------------------------------------------------------*/
/*------------------------------------------------------------------*/



void testTriangle() {
    std::cout << "\n=== 测试：三角形区域 ===" << std::endl;
    
    std::vector<Point> triangle = {
/*        Point(0, 0),
        Point(10, 0),
        Point(5, 8.66)  // 等边三角形
        */
        
        // Point(223.21, 259.81),
        // Point(123.21, 259.81),
        // Point(73.21, 173.21),
        Point(123.21, 86.60),
        Point(223.21, 86.60),
        Point(273.21, 173.21)
    };
    
    double robotWidth = 10.0;

    double area = polygonArea(triangle);
    std::cout << "三角形面积: " << area << std::endl;
    
//    std::vector<Point> path = generateContourSpiralPath(triangle, w);
//    std::vector<Point> path = generateSpiralCoveragePath(triangle, w);   
     // 生成螺旋式全覆盖路径
    std::vector<Point> path1 = generateSpiralCoveragePath(triangle, robotWidth);   
    // 优化路径
    std::vector<Point> optimizedPath1 = optimizePath(path1, robotWidth * 0.3);    
    // 离散化路径（用于控制）
//    std::vector<Point> discretizedPath1 = discretizePath(optimizedPath1, robotWidth * 0.5);   
    std::vector<Point> discretizedPath1 = discretizePath(optimizedPath1, robotWidth * 0.5);   


    
    std::cout << "生成的路径点数: " << path1.size() << std::endl;

    std::ofstream outFile("example.txt", std::ios::trunc); // 覆盖写入，避免旧轨迹干扰可视化
    if (outFile.is_open()) {
        for (int i = 0; i < path1.size(); i++) {
            outFile << path1[i].x << "            " << path1[i].y << std::endl; 
        }
        outFile.close(); // 关闭文件

    } else {
        std::cout << "Unable to open file";
    }

    std::ofstream outFile2("dist_example2.txt", std::ios::trunc); // 覆盖写入，避免旧轨迹干扰可视化
    if (outFile2.is_open()) {
        for (int i = 0; i < discretizedPath1.size(); i++) {
            outFile2 << discretizedPath1[i].x << "            " << discretizedPath1[i].y << std::endl; 
        }
        outFile2.close(); // 关闭文件

    } else {
        std::cout << "Unable to open file";
    }
    
    double pathLength = calculatePathLength(path1);
    double estimatedCoverage = pathLength * robotWidth;
    std::cout << "\n估算:" << std::endl;
    std::cout << "  路径总长度: " << pathLength << std::endl;
    std::cout << "  估算覆盖面积: " << estimatedCoverage << std::endl;
    std::cout << "  覆盖率: " << (estimatedCoverage / area) * 100.0 << "%" << std::endl;
}



// 主函数
int main() {
    std::cout << "凸区域螺旋式全覆盖路径规划" << std::endl;
    std::cout << "==========================" << std::endl;
 
    testTriangle();

    system("PAUSE");
    return 0;
}