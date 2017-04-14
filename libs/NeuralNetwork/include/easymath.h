// Copyright 2016 Carrie Rebhuhn
#ifndef MATH_EASYMATH_H_
#define MATH_EASYMATH_H_

#include <utility>
#include <set>
#include <vector>

#include "MatrixTypes.h"
#include "XY.h"

namespace easymath {
typedef std::pair<size_t, size_t> subscript;
typedef std::pair<XY, XY> line_segment;

//! Returns bin assignment based on bounds. Bounds must be sorted
size_t bin(const double& n, const matrix1d& bounds);

//! Calculates the manhattan distance between two points
double manhattan_distance(const XY &p1, const XY &p2);

//! Calculates the euclidean distance (l2 norm)
double euclidean_distance(const XY &p1, const XY &p2);

//! Calculates the cardinal direction of a vector
size_t cardinal_direction(const XY &dx_dy);

//! Cross product between vectors.
//! This assumes U and V are endpoints of vectors that originate at (0,0)
double cross(const XY &U, const XY &V);

//! Checks whether lines intersect in the center.
//! Coinciding endpoints excluded). Returns true if so.
bool intersects_in_center(line_segment edge1, line_segment edge2);

//! Returns a random number between some bounds.
double rand(double low, double high);

//! Error function (this exists in linux but not windows)
double erfc(double x);

size_t get_nearest_square(size_t n);

std::set<XY> get_n_unique_points(double xmin, double xmax,
    double ymin, double ymax, size_t n);

//! Gets random points, fit into a square
std::vector<XY> get_n_unique_square_points(double xmin, double xmax,
    double ymin, double ymax, size_t n);

//! Gets unique indices for a square that must contain n points
std::vector<subscript> get_n_unique_square_subscripts(size_t n);

subscript ind2sub(const size_t &sub, const size_t &cols, const size_t &rows);

size_t nChoosek(size_t n, size_t k);

std::vector<std::pair<size_t, size_t> > all_combos_of_2(size_t n);

bool is_endpt(const XY& a, const line_segment& b);
bool pt_on_line(const XY& a, const line_segment& b, const double thresh = 0.01);
double randSetFanIn(double fan_in);
}  // namespace easymath
#endif  // MATH_EASYMATH_H_
