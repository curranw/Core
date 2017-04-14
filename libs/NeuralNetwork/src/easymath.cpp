// Copyright Carrie Rebhuhn 2016
#include "easymath.h"
#include <set>
#include <vector>
#include <utility>
#include <algorithm>

namespace easymath {
std::set<XY> get_n_unique_points(double x_min, double x_max,
    double y_min, double y_max, size_t n) {
    std::set<XY> pt_set;
    while (pt_set.size() < n) {
        XY p = XY(rand(x_min, x_max), rand(y_min, y_max));
        pt_set.insert(p);
    }
    return pt_set;
}

size_t get_nearest_square(size_t n) {
    return static_cast<size_t>(std::pow(ceil(sqrt(n)), 2));
}

subscript ind2sub(const size_t &sub, const size_t &cols,
    const size_t &rows) {
    size_t row = sub / cols;
    size_t col = sub%rows;
    return std::make_pair(row, col);
}

std::vector<subscript> get_n_unique_square_subscripts(size_t n) {
    size_t square = get_nearest_square(n);
    std::vector<size_t> inds(square);
    for (size_t i = 0; i < inds.size(); i++) {
        inds[i] = i;
    }

    size_t n_surplus = square - n;
    for (size_t i = 0; i < n_surplus; i++) {
        int randn = std::rand() % static_cast<int>(inds.size());
        inds.erase(inds.begin()+randn);
    }

    size_t base = static_cast<size_t>(sqrt(square));
    std::vector<std::pair<size_t, size_t> > subs(inds.size());
    for (size_t i = 0; i < subs.size(); i++) {
        subs[i] = ind2sub(inds[i], base, base);
    }
    return subs;
}

std::vector<XY> get_n_unique_square_points(double x_min, double x_max,
    double y_min, double y_max, size_t n) {

    std::vector<std::pair<size_t, size_t> >
        subs = get_n_unique_square_subscripts(n);

    std::vector<XY> pts;
    for (size_t i = 0; i < subs.size(); i++) {
        double base = sqrt(get_nearest_square(n));
        double xval = (x_max - x_min)
            *static_cast<double>(subs[i].first) / base;
        double yval = (y_max - y_min)
            *static_cast<double>(subs[i].second) / base;
        pts.push_back(XY(xval, yval));
    }
    return pts;
}

double manhattan_distance(const XY &p1, const XY &p2) {
    XY diff = p1 - p2;
    return abs(diff.x) + abs(diff.y);
}


size_t cardinal_direction(const XY &dx_dy) {
    if (dx_dy.y >= 0) {  // Going up
        if (dx_dy.x >= 0) return 0;  // up-right
        else
            return 1;  // up-left
    } else {
        if (dx_dy.x >= 0) return 2;  // down-right
        else
            return 3;  // down-left
    }
}

double euclidean_distance(const XY &p1, const XY &p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

size_t bin(const double &n, const matrix1d& bounds) {
    for (size_t i = 0; i < bounds.size() - 1; i++)
        if (n < bounds[i + 1])
            return i;
    return bounds.size();
}

double rand(double low, double high) {
    double r = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
    return r*(high - low) + low;
}

double cross(const XY &U, const XY &V) {
    return U.x*V.y - U.y*V.x;
}

bool intersects_in_center(line_segment edge1, line_segment edge2) {
    // Detects whether line intersects, but not at origin
    XY p = edge1.first;
    XY q = edge2.first;
    XY r = edge1.second - edge1.first;
    XY s = edge2.second - edge2.first;
    XY qpdiff = q - p;
    double rscross = cross(r, s);
    double t = cross((qpdiff), s) / rscross;
    double u = cross((qpdiff), r) / rscross;

    if (rscross == 0) {
        if (cross(qpdiff, r)) {
            // need to check bounds
            if (!(0 < t && t < 1 && 0 < u && u < 1)) {
                return false;
            }
            return true;  // collinear
        } else {
            return false;  // parallel non-intersecting
        }
    } else if (0 < t && t < 1 && 0 < u && u < 1) {
        // if you care about origins, use <= rather than <
        return true;  // intersects at p+tr = q+us
    } else {
        return false;  // not parallel, don't inersect
    }
}


matrix1d operator*(const double A, matrix1d B) {
    for (auto &b : B) {
        b *= A;
    }
    return B;
}

matrix2d operator*(const double A, matrix2d B) {
    for (auto &b : B) {
        b = A*b;
    }
    return B;
}

double erfc(double x) {
    // constants
    double a1 = 0.254829592;
    double a2 = -0.284496736;
    double a3 = 1.421413741;
    double a4 = -1.453152027;
    double a5 = 1.061405429;
    double p = 0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x);

    // A&S formula 7.1.26
    double t = 1.0 / (1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 1 - sign*y;
}

size_t nChoosek(size_t n, size_t k) {
    if (k > n) return 0;
    if (k * 2 > n) k = n - k;
    if (k == 0) return 1;

    size_t result = n;
    for (size_t i = 2; i <= k; ++i) {
        result *= (n - i + 1);
        result /= i;
    }
    return result;
}

std::vector<std::pair<size_t, size_t> > all_combos_of_2(size_t n) {
    std::vector<std::pair<size_t, size_t> > v(nChoosek(n, 2));
    for (size_t i = 0, index = 0; i < n - 1; i++) {
        for (size_t j = i + 1; j < n; j++) {
            v[index++] = std::make_pair(i, j);
        }
    }
    return v;
}

bool is_endpt(const XY &p, const line_segment &l) {
    // Check if point lies on endpoint of line
    return (p == l.first || p == l.second);
}

bool pt_on_line(const XY &p, const line_segment &l, const double threshold) {
    XY l1 = l.first;
    XY l2 = l.second;
    double m = (l2.y - l1.y) / (l2.x - l1.x);
    double b = l1.y - l1.x*m;

    double y_on_line = m*p.x + b;
    double diff = fabs(p.y - y_on_line);
    bool not_on_line = diff > threshold;

    bool abv_xmin = p.x >= std::min(l1.x, l2.x);
    bool blw_xmax = p.x <= std::max(l1.x, l2.x);
    bool abv_ymin = p.y >= std::min(l1.y, l2.y);
    bool blw_ymax = p.y <= std::max(l1.y, l2.y);
    if (not_on_line) {
        return false;   // not on line
    } else {
        return (abv_xmin && blw_xmax && abv_ymin && blw_ymax);
    }
}

double randSetFanIn(double fan_in) {
    return rand(-10, 10) / sqrt(fan_in);
}
}  // namespace easymath
