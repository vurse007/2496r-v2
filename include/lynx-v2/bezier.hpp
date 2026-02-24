#pragma once
#include "pursuit_types.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace lynx {

// ============================================================================
// BEZIER PATH GENERATOR
// Generates dense Waypoint vectors from cubic Bezier splines
// for use with purePursuit()
//
// GLOBAL FRAME: +X East, +Y North (IMU-native)
// Headings: IMU degrees (0=North, 90=East, CW positive)
// ============================================================================

class BezierPath {
public:

    // ------------------------------------------------------------------------
    // Single cubic Bezier segment: P0 (start), P1 (ctrl1), P2 (ctrl2), P3 (end)
    // ------------------------------------------------------------------------
    struct CubicSegment {
        double x0, y0;   // start point
        double x1, y1;   // control point 1 (near start)
        double x2, y2;   // control point 2 (near end)
        double x3, y3;   // end point

        double start_velocity = 100.0;
        double end_velocity   = 100.0;

        CubicSegment(double x0, double y0,
                     double x1, double y1,
                     double x2, double y2,
                     double x3, double y3,
                     double sv = 100.0, double ev = 100.0)
            : x0(x0), y0(y0), x1(x1), y1(y1),
              x2(x2), y2(y2), x3(x3), y3(y3),
              start_velocity(sv), end_velocity(ev) {}
    };

    // ------------------------------------------------------------------------
    // Control point for chained splines
    // ------------------------------------------------------------------------
    struct ControlPoint {
        double x, y;
        double heading;   // IMU degrees (0=N, 90=E, CW+)
        double velocity;  // 0-127

        ControlPoint(double x, double y, double heading = 0.0, double velocity = 100.0)
            : x(x), y(y), heading(heading), velocity(velocity) {}
    };

private:

    // ------------------------------------------------------------------------
    // Cubic Bezier position at t
    // ------------------------------------------------------------------------
    static double bezier_x(const CubicSegment& s, double t) {
        double mt = 1.0 - t;
        return mt*mt*mt*s.x0 + 3*mt*mt*t*s.x1 + 3*mt*t*t*s.x2 + t*t*t*s.x3;
    }

    static double bezier_y(const CubicSegment& s, double t) {
        double mt = 1.0 - t;
        return mt*mt*mt*s.y0 + 3*mt*mt*t*s.y1 + 3*mt*t*t*s.y2 + t*t*t*s.y3;
    }

    // ------------------------------------------------------------------------
    // Cubic Bezier derivative (tangent) at t
    // ------------------------------------------------------------------------
    static double bezier_dx(const CubicSegment& s, double t) {
        double mt = 1.0 - t;
        return 3*mt*mt*(s.x1 - s.x0) + 6*mt*t*(s.x2 - s.x1) + 3*t*t*(s.x3 - s.x2);
    }

    static double bezier_dy(const CubicSegment& s, double t) {
        double mt = 1.0 - t;
        return 3*mt*mt*(s.y1 - s.y0) + 6*mt*t*(s.y2 - s.y1) + 3*t*t*(s.y3 - s.y2);
    }

    // ------------------------------------------------------------------------
    // Tangent vector -> IMU heading degrees (0=North, 90=East, CW+)
    // ------------------------------------------------------------------------
    static double tangent_to_imu_heading(double dx, double dy) {
        double math_deg = std::atan2(dy, dx) * 180.0 / M_PI;
        double imu_deg  = 90.0 - math_deg;
        return std::fmod(imu_deg + 360.0, 360.0);
    }

    // ------------------------------------------------------------------------
    // Arc-length table: maps t -> cumulative arc length
    // Ensures waypoints are evenly spaced in real distance, not in t
    // ------------------------------------------------------------------------
    static std::vector<std::pair<double,double>> build_arc_table(
        const CubicSegment& seg, int subdivisions = 500)
    {
        std::vector<std::pair<double,double>> table;
        table.reserve(subdivisions + 1);

        double arc    = 0.0;
        double prev_x = bezier_x(seg, 0.0);
        double prev_y = bezier_y(seg, 0.0);
        table.push_back({0.0, 0.0});

        for (int i = 1; i <= subdivisions; i++) {
            double t   = (double)i / subdivisions;
            double cx  = bezier_x(seg, t);
            double cy  = bezier_y(seg, t);
            double ddx = cx - prev_x;
            double ddy = cy - prev_y;
            arc += std::sqrt(ddx*ddx + ddy*ddy);
            table.push_back({t, arc});
            prev_x = cx;
            prev_y = cy;
        }
        return table;
    }

    // ------------------------------------------------------------------------
    // Binary search: arc length -> t parameter
    // ------------------------------------------------------------------------
    static double arc_to_t(const std::vector<std::pair<double,double>>& table, double target_arc) {
        if (target_arc <= 0.0) return 0.0;
        if (target_arc >= table.back().second) return 1.0;

        int lo = 0, hi = (int)table.size() - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (table[mid].second < target_arc) lo = mid;
            else hi = mid;
        }

        double frac = (target_arc - table[lo].second) /
                      (table[hi].second - table[lo].second);
        return table[lo].first + frac * (table[hi].first - table[lo].first);
    }

public:

    // ========================================================================
    // Generate waypoints from a single CubicSegment
    // ========================================================================
    static std::vector<Waypoint> from_segment(
        const CubicSegment& seg,
        double spacing_inches = 0.5)
    {
        auto arc_table   = build_arc_table(seg);
        double total_arc = arc_table.back().second;

        if (total_arc < 0.001) return {};

        int num_points = std::max(2, (int)std::ceil(total_arc / spacing_inches) + 1);
        std::vector<Waypoint> waypoints;
        waypoints.reserve(num_points);

        for (int i = 0; i < num_points; i++) {
            double frac = (double)i / (num_points - 1);
            double t    = arc_to_t(arc_table, frac * total_arc);

            double wx = bezier_x(seg, t);
            double wy = bezier_y(seg, t);
            double dx = bezier_dx(seg, t);
            double dy = bezier_dy(seg, t);

            double heading  = tangent_to_imu_heading(dx, dy);
            double velocity = std::clamp(
                seg.start_velocity + frac * (seg.end_velocity - seg.start_velocity),
                0.0, 127.0);

            waypoints.emplace_back(wx, wy, heading, velocity);
        }

        return waypoints;
    }

    // ========================================================================
    // Generate waypoints from a chain of ControlPoints
    //
    // tangent_scale: handle length as fraction of segment distance
    //                (0.4 = good default, larger = rounder, smaller = tighter)
    // spacing_inches: distance between output waypoints
    // ========================================================================
    static std::vector<Waypoint> from_control_points(
        const std::vector<ControlPoint>& pts,
        double tangent_scale  = 0.4,
        double spacing_inches = 0.5)
    {
        if (pts.size() < 2) return {};

        std::vector<Waypoint> all_waypoints;

        for (int i = 0; i < (int)pts.size() - 1; i++) {
            const ControlPoint& p0 = pts[i];
            const ControlPoint& p1 = pts[i + 1];

            double dist   = std::sqrt((p1.x-p0.x)*(p1.x-p0.x) + (p1.y-p0.y)*(p1.y-p0.y));
            double handle = dist * tangent_scale;

            // IMU heading -> tangent vector: dx=sin(h), dy=cos(h)
            double h0_rad = p0.heading * M_PI / 180.0;
            double h1_rad = p1.heading * M_PI / 180.0;

            // Exit handle: depart p0 along its heading
            double cx1 = p0.x + handle * std::sin(h0_rad);
            double cy1 = p0.y + handle * std::cos(h0_rad);

            // Entry handle: arrive at p1 from opposite its heading
            double cx2 = p1.x - handle * std::sin(h1_rad);
            double cy2 = p1.y - handle * std::cos(h1_rad);

            CubicSegment seg(p0.x, p0.y, cx1, cy1, cx2, cy2, p1.x, p1.y,
                             p0.velocity, p1.velocity);

            auto seg_wps = from_segment(seg, spacing_inches);

            // Drop first point of each segment after the first to avoid duplicates
            if (!all_waypoints.empty() && !seg_wps.empty()) {
                seg_wps.erase(seg_wps.begin());
            }

            for (auto& wp : seg_wps) {
                all_waypoints.push_back(wp);
            }
        }

        return all_waypoints;
    }

    // ========================================================================
    // Auto-heading path: just give x,y pairs, headings computed from geometry
    // ========================================================================
    static std::vector<Waypoint> auto_path(
        const std::vector<std::pair<double,double>>& xy_points,
        double velocity       = 100.0,
        double tangent_scale  = 0.4,
        double spacing_inches = 0.5)
    {
        if (xy_points.size() < 2) return {};

        std::vector<ControlPoint> pts;
        pts.reserve(xy_points.size());

        for (int i = 0; i < (int)xy_points.size(); i++) {
            double heading;

            if (i == 0) {
                double dx = xy_points[1].first  - xy_points[0].first;
                double dy = xy_points[1].second - xy_points[0].second;
                heading = tangent_to_imu_heading(dx, dy);
            } else if (i == (int)xy_points.size() - 1) {
                double dx = xy_points[i].first  - xy_points[i-1].first;
                double dy = xy_points[i].second - xy_points[i-1].second;
                heading = tangent_to_imu_heading(dx, dy);
            } else {
                // Normalized average of incoming and outgoing tangents
                double dx1 = xy_points[i].first   - xy_points[i-1].first;
                double dy1 = xy_points[i].second  - xy_points[i-1].second;
                double dx2 = xy_points[i+1].first  - xy_points[i].first;
                double dy2 = xy_points[i+1].second - xy_points[i].second;

                double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                if (len1 > 0.001) { dx1 /= len1; dy1 /= len1; }
                if (len2 > 0.001) { dx2 /= len2; dy2 /= len2; }

                heading = tangent_to_imu_heading(dx1 + dx2, dy1 + dy2);
            }

            pts.emplace_back(xy_points[i].first, xy_points[i].second, heading, velocity);
        }

        return from_control_points(pts, tangent_scale, spacing_inches);
    }

    // ========================================================================
    // One-liner helper — used internally by lynx::path() below
    // ========================================================================
    static std::vector<Waypoint> make(
        std::initializer_list<ControlPoint> pts,
        double tangent_scale  = 0.4,
        double spacing_inches = 0.5)
    {
        return from_control_points(std::vector<ControlPoint>(pts),
                                   tangent_scale, spacing_inches);
    }

}; // class BezierPath

// ============================================================================
// FREE FUNCTION - lynx::path({...})
//
//
// Usage:
//   chassis.purePursuit(lynx::path({
//       {x, y, heading, velocity},
//       ...
//   }), timeout);
// ============================================================================
inline std::vector<Waypoint> path(
    std::initializer_list<BezierPath::ControlPoint> pts,
    double tangent_scale  = 0.4,
    double spacing_inches = 0.5)
{
    return BezierPath::make(pts, tangent_scale, spacing_inches);
}

} // namespace lynx


// ============================================================================
// USAGE EXAMPLES
// ============================================================================
//
// --- Recommended: inline in autonomous ---
//
//   chassis.purePursuit(lynx::path({
//       {0,   0,   0,   100},   // x, y, heading (IMU deg), velocity
//       {24,  12,  45,  100},
//       {48,  0,   90,  80},
//   }), 5000, params);
//
//
// --- Custom tangent scale / spacing ---
//
//   chassis.purePursuit(lynx::path({
//       {0,  0,  0,  100},
//       {48, 0,  90, 80},
//   }, 0.6, 0.3), 5000, params);
//
//
// --- Named path (if you reuse it or want clarity) ---
//
//   auto my_path = lynx::path({
//       {0,  0,   0,   80},
//       {24, 24,  90,  100},
//       {48, 0,   180, 60},
//   });
//   chassis.purePursuit(my_path, 5000, params);
//
//
// --- Auto heading (geometry-derived, no manual heading) ---
//
//   auto p = lynx::BezierPath::auto_path({{0,0},{24,12},{48,0}}, 100.0);
//   chassis.purePursuit(p, 5000, params);
//
//
// --- Raw cubic segment (manual control handles) ---
//
//   lynx::BezierPath::CubicSegment seg(
//       0,  0,    // start
//       0,  20,   // ctrl1
//       48, 20,   // ctrl2
//       48, 0,    // end
//       100, 80   // start vel, end vel
//   );
//   chassis.purePursuit(lynx::BezierPath::from_segment(seg, 0.5), 5000, params);