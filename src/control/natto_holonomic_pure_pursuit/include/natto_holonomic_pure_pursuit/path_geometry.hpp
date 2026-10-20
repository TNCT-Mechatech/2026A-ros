#ifndef NATTO_HOLONOMIC_PURE_PURSUIT_PATH_GEOMETRY_HPP_
#define NATTO_HOLONOMIC_PURE_PURSUIT_PATH_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace holonomic_pure_pursuit::path_geometry {

struct Point {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

// Keep orientation-only poses in the original path. Only geometric distances
// ignore their zero-length segments.
class PathGeometry {
   public:
    PathGeometry () = default;
    explicit PathGeometry (std::vector<Point> points) : points_ (std::move (points)), lengths_ (points_.size (), 0.0) {
        for (size_t i = 1; i < points_.size (); ++i) {
            lengths_[i] = lengths_[i - 1] + distance (points_[i - 1], points_[i]);
        }
    }

    Point lookahead (double x, double y, size_t closest_index, double radius) const {
        if (points_.empty ()) return {};
        closest_index = std::min (closest_index, points_.size () - 1);
        const Point current{x, y, 0.0};
        if (distance (points_[closest_index], current) >= radius) return points_[closest_index];

        // Find the first segment leaving the lookahead circle, rather than a
        // vertex nearest its radius. Repeated XY poses cannot stall this search.
        for (size_t i = closest_index + 1; i < points_.size (); ++i) {
            if (distance (points_[i], current) < radius) continue;
            const auto  &a  = points_[i - 1];
            const auto  &b  = points_[i];
            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            const double ox = a.x - x;
            const double oy = a.y - y;
            const double aa = dx * dx + dy * dy;
            if (aa <= 1e-12) continue;
            const double bb           = 2.0 * (ox * dx + oy * dy);
            const double cc           = ox * ox + oy * oy - radius * radius;
            const double discriminant = std::max (0.0, bb * bb - 4.0 * aa * cc);
            const double ratio        = std::clamp ((-bb + std::sqrt (discriminant)) / (2.0 * aa), 0.0, 1.0);
            return interpolate (a, b, ratio);
        }
        // At the end (including a pure rotation), preserve the final pose/yaw.
        return points_.back ();
    }

    double curvature (double x, double y, double span) const {
        if (points_.size () < 3 || lengths_.back () <= 1e-9) return 0.0;
        double best_squared_distance = std::numeric_limits<double>::infinity ();
        double start                 = 0.0;
        for (size_t i = 1; i < points_.size (); ++i) {
            const auto  &a      = points_[i - 1];
            const auto  &b      = points_[i];
            const double length = lengths_[i] - lengths_[i - 1];
            if (length <= 1e-9) continue;
            const double dx               = b.x - a.x;
            const double dy               = b.y - a.y;
            const double ratio            = std::clamp (((x - a.x) * dx + (y - a.y) * dy) / (length * length), 0.0, 1.0);
            const double ex               = a.x + ratio * dx - x;
            const double ey               = a.y + ratio * dy - y;
            const double squared_distance = ex * ex + ey * ey;
            if (squared_distance < best_squared_distance) {
                best_squared_distance = squared_distance;
                start                 = lengths_[i - 1] + ratio * length;
            }
        }
        // Sample by physical distance, independently of the speed-dependent
        // steering lookahead and the density of rotation poses.
        span = std::min (span, lengths_.back () - start);
        if (span <= 1e-9) return 0.0;
        const Point  a       = sample (start);
        const Point  b       = sample (start + 0.5 * span);
        const Point  c       = sample (start + span);
        const double product = distance (a, b) * distance (b, c) * distance (a, c);
        if (product <= 1e-12) return 0.0;
        const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        return 2.0 * std::abs (cross) / product;
    }

   private:
    std::vector<Point>  points_;
    std::vector<double> lengths_;

    static double distance (const Point &a, const Point &b) {
        return std::hypot (b.x - a.x, b.y - a.y);
    }

    static Point interpolate (const Point &a, const Point &b, double ratio) {
        // Interpolate across +/-pi via the shortest angle, not through zero.
        const double delta_yaw = std::atan2 (std::sin (b.yaw - a.yaw), std::cos (b.yaw - a.yaw));
        const double yaw       = a.yaw + ratio * delta_yaw;
        return {a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y), std::atan2 (std::sin (yaw), std::cos (yaw))};
    }

    Point sample (double arc_length) const {
        if (arc_length >= lengths_.back ()) return points_.back ();
        const auto   next  = std::upper_bound (lengths_.begin (), lengths_.end (), arc_length);
        const size_t index = static_cast<size_t> (next - lengths_.begin ()) - 1;
        const double ratio = (arc_length - lengths_[index]) / (lengths_[index + 1] - lengths_[index]);
        return interpolate (points_[index], points_[index + 1], ratio);
    }
};

}  // namespace holonomic_pure_pursuit::path_geometry

#endif  // NATTO_HOLONOMIC_PURE_PURSUIT_PATH_GEOMETRY_HPP_
