#include "library_functions.hpp"

#include <cmath>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <vector>

#include "ast.hpp"

using AST::ast_ptr;
using AST::Bool;
using AST::bool_ptr;
using AST::Num;
using AST::num_ptr;
using AST::Type;
using AST::Vec;
using AST::vec_ptr;
using Eigen::Vector2f;
using Eigen::Vector3i;
using std::abs;
using std::array;
using std::cos;
using std::cout;
using std::dynamic_pointer_cast;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::pow;
using std::sin;
using std::vector;

#define ASSERT_DIM(expression, dimensionality)                  \
  {                                                             \
    if (expression->dims_ != dimensionality) {                  \
      throw invalid_argument("`" #expression                    \
                             "' has incorrect dimensionality"); \
    }                                                           \
  }

#define ASSERT_TYPE(expression, type)                                \
  {                                                                  \
    if (expression->type_ != type) {                                 \
      throw invalid_argument("type mismatch: expected `" #expression \
                             "' to have type `" #type "'");          \
    }                                                                \
  }

#define ASSERT_DIMS_EQUAL(left, right)                               \
  {                                                                  \
    if (left->dims_ != right->dims_) {                               \
      throw invalid_argument("expected `" #left "' and `" #right     \
                             "' to have the same dimensionalities"); \
    }                                                                \
  }

#define ASSERT_TYPES_EQUAL(left, right)                          \
  {                                                              \
    if (left->type_ != right->type_) {                           \
      throw invalid_argument("expected `" #left "' and `" #right \
                             "' to have the same types");        \
    }                                                            \
  }

ast_ptr Plus(ast_ptr left, ast_ptr right) {
  ASSERT_DIMS_EQUAL(left, right);
  ASSERT_TYPES_EQUAL(left, right);
  if (left->type_ == Type::NUM) {
    num_ptr left_cast = dynamic_pointer_cast<Num>(left);
    num_ptr right_cast = dynamic_pointer_cast<Num>(right);
    Num result(left_cast->value_ + right_cast->value_, left->dims_);
    return make_shared<Num>(result);
  } else if (left->type_ == Type::VEC) {
    vec_ptr left_cast = dynamic_pointer_cast<Vec>(left);
    vec_ptr right_cast = dynamic_pointer_cast<Vec>(right);
    Vec result(left_cast->value_ + right_cast->value_, left->dims_);
    return make_shared<Vec>(result);
  } else {
    throw invalid_argument(
        "expected types of `left' and `right' to be `Type::NUM' or "
        "`Type::VEC'");
  }
}

ast_ptr Minus(ast_ptr left, ast_ptr right) {
  ASSERT_DIMS_EQUAL(left, right);
  ASSERT_TYPES_EQUAL(left, right);
  if (left->type_ == Type::NUM) {
    num_ptr left_cast = dynamic_pointer_cast<Num>(left);
    num_ptr right_cast = dynamic_pointer_cast<Num>(right);
    Num result(left_cast->value_ - right_cast->value_, left->dims_);
    return make_shared<Num>(result);
  } else if (left->type_ == Type::VEC) {
    vec_ptr left_cast = dynamic_pointer_cast<Vec>(left);
    vec_ptr right_cast = dynamic_pointer_cast<Vec>(right);
    Vec result(left_cast->value_ - right_cast->value_, left->dims_);
    return make_shared<Vec>(result);
  } else {
    throw invalid_argument(
        "expected types of `left' and `right' to be `Type::NUM' or "
        "`Type::VEC'");
  }
}

ast_ptr Times(ast_ptr left, ast_ptr right) {
  ASSERT_TYPE(left, Type::NUM);

  num_ptr left_cast = dynamic_pointer_cast<Num>(left);
  if (right->type_ == Type::NUM) {
    num_ptr right_cast = dynamic_pointer_cast<Num>(right);
    Num result(left_cast->value_ * right_cast->value_,
               left->dims_ + right->dims_);
    return make_shared<Num>(result);
  } else if (right->type_ == Type::VEC) {
    vec_ptr right_cast = dynamic_pointer_cast<Vec>(right);
    Vec result(left_cast->value_ * right_cast->value_,
               left->dims_ + right->dims_);
    return make_shared<Vec>(result);
  } else {
    throw invalid_argument(
        "expected type of `left' to be `Type::NUM' and/or expected type of "
        "`right' to be `Type::NUM' or `Type::VEC'");
  }
}

ast_ptr DividedBy(ast_ptr left, ast_ptr right) {
  ASSERT_TYPE(left, Type::NUM);

  num_ptr right_cast = dynamic_pointer_cast<Num>(right);
  if (left->type_ == Type::NUM) {
    num_ptr left_cast = dynamic_pointer_cast<Num>(left);
    Num result(left_cast->value_ / right_cast->value_,
               left->dims_ - right->dims_);
    return make_shared<Num>(result);
  } else if (left->type_ == Type::VEC) {
    vec_ptr left_cast = dynamic_pointer_cast<Vec>(right);
    Vec result(left_cast->value_ / right_cast->value_,
               left->dims_ - right->dims_);
    return make_shared<Vec>(result);
  } else {
    throw invalid_argument(
        "expected type of `left' to be `Type::NUM' and/or expected type of "
        "`right' to be `Type::NUM' or `Type::VEC'");
  }
}

ast_ptr Abs(ast_ptr operand) {
  ASSERT_TYPE(operand, Type::NUM);

  num_ptr operand_cast = dynamic_pointer_cast<Num>(operand);
  Num result(abs(operand_cast->value_), operand->dims_);
  return make_shared<Num>(result);
}

ast_ptr Pow(ast_ptr base, ast_ptr power) {
  ASSERT_DIM(power, Vector3i(0, 0, 0));
  ASSERT_TYPE(base, Type::NUM);
  ASSERT_TYPE(power, Type::NUM);

  num_ptr base_cast = dynamic_pointer_cast<Num>(base);
  num_ptr power_cast = dynamic_pointer_cast<Num>(power);
  // TODO(simon) figure out something better for dimensionalities
  Num result(pow(base_cast->value_, power_cast->value_),
             base->dims_ * (int)power_cast->value_);
  return make_shared<Num>(result);
}

ast_ptr Cos(ast_ptr theta) {
  ASSERT_DIM(theta, Vector3i(0, 0, 0));
  ASSERT_TYPE(theta, Type::NUM);

  num_ptr theta_cast = dynamic_pointer_cast<Num>(theta);
  Num result(cos(theta_cast->value_), {0, 0, 0});
  return make_shared<Num>(result);
}

ast_ptr Sin(ast_ptr theta) {
  ASSERT_DIM(theta, Vector3i(0, 0, 0));
  ASSERT_TYPE(theta, Type::NUM);

  num_ptr theta_cast = dynamic_pointer_cast<Num>(theta);
  Num result(sin(theta_cast->value_), {0, 0, 0});
  return make_shared<Num>(result);
}

ast_ptr Cross(ast_ptr u, ast_ptr v) {
  ASSERT_DIMS_EQUAL(u, v);
  ASSERT_TYPE(u, Type::VEC);
  ASSERT_TYPE(v, Type::VEC);

  vec_ptr u_cast = dynamic_pointer_cast<Vec>(u);
  vec_ptr v_cast = dynamic_pointer_cast<Vec>(v);
  // TODO(simon) check dimensionality is correct
  Num result(u_cast->value_.x() * v_cast->value_.y() +
                 u_cast->value_.y() * v_cast->value_.x(),
             u_cast->dims_);
  return make_shared<Num>(result);
}

ast_ptr Dot(ast_ptr u, ast_ptr v) {
  ASSERT_DIMS_EQUAL(u, v);
  ASSERT_TYPE(u, Type::VEC);
  ASSERT_TYPE(v, Type::VEC);

  vec_ptr u_cast = dynamic_pointer_cast<Vec>(u);
  vec_ptr v_cast = dynamic_pointer_cast<Vec>(v);
  Num result(u_cast->value_.dot(v_cast->value_), u->dims_);
  return make_shared<Num>(result);
}

ast_ptr EuclideanDistance(ast_ptr u, ast_ptr v) {
  ASSERT_DIMS_EQUAL(u, v);
  ASSERT_TYPE(u, Type::VEC);
  ASSERT_TYPE(v, Type::VEC);

  vec_ptr u_cast = dynamic_pointer_cast<Vec>(u);
  vec_ptr v_cast = dynamic_pointer_cast<Vec>(v);
  Num result((u_cast->value_ - v_cast->value_).norm(), u->dims_);
  return make_shared<Num>(result);
}

ast_ptr Heading(ast_ptr theta) {
  ASSERT_DIM(theta, Vector3i(0, 0, 0));
  ASSERT_TYPE(theta, Type::NUM);

  num_ptr theta_cast = dynamic_pointer_cast<Num>(theta);
  Vec result({cos(theta_cast->value_), sin(theta_cast->value_)}, {0, 0, 0});
  return make_shared<Vec>(result);
}

ast_ptr Norm(ast_ptr v) {
  ASSERT_TYPE(v, Type::VEC);

  vec_ptr v_cast = dynamic_pointer_cast<Vec>(v);
  Num result(v_cast->value_.norm(), v->dims_);
  return make_shared<Num>(result);
}

ast_ptr Perp(ast_ptr v) {
  ASSERT_TYPE(v, Type::VEC);

  vec_ptr v_cast = dynamic_pointer_cast<Vec>(v);
  Vec result({-v_cast->value_.y(), v_cast->value_.x()}, v->dims_);
  return make_shared<Vec>(result);
}

ast_ptr VecX(ast_ptr v) {
  ASSERT_TYPE(v, Type::VEC);

  vec_ptr v_cast = dynamic_pointer_cast<Vec>(v);
  Num result(v_cast->value_.x(), v->dims_);
  return make_shared<Num>(result);
}

ast_ptr VecY(ast_ptr v) {
  ASSERT_TYPE(v, Type::VEC);

  vec_ptr v_cast = dynamic_pointer_cast<Vec>(v);
  Num result(v_cast->value_.y(), v->dims_);
  return make_shared<Num>(result);
}

ast_ptr And(ast_ptr P, ast_ptr Q) {
  ASSERT_TYPE(P, Type::BOOL);
  ASSERT_TYPE(Q, Type::BOOL);

  bool_ptr P_cast = dynamic_pointer_cast<Bool>(P);
  bool_ptr Q_cast = dynamic_pointer_cast<Bool>(Q);
  Bool result(P_cast->value_ && Q_cast->value_);
  return make_shared<Bool>(result);
}

ast_ptr Or(ast_ptr P, ast_ptr Q) {
  ASSERT_TYPE(P, Type::BOOL);
  ASSERT_TYPE(Q, Type::BOOL);

  bool_ptr P_cast = dynamic_pointer_cast<Bool>(P);
  bool_ptr Q_cast = dynamic_pointer_cast<Bool>(Q);
  Bool result(P_cast->value_ || Q_cast->value_);
  return make_shared<Bool>(result);
}

ast_ptr Not(ast_ptr P) {
  ASSERT_TYPE(P, Type::BOOL);

  bool_ptr P_cast = dynamic_pointer_cast<Bool>(P);
  Bool result(!P_cast->value_);
  return make_shared<Bool>(result);
}

ast_ptr Eq(ast_ptr x, ast_ptr y) {
  ASSERT_TYPE(x, Type::NUM);
  ASSERT_TYPE(y, Type::NUM);

  num_ptr x_cast = dynamic_pointer_cast<Num>(x);
  num_ptr y_cast = dynamic_pointer_cast<Num>(y);
  Bool result(x_cast->value_ == y_cast->value_);
  return make_shared<Bool>(result);
}

ast_ptr Lt(ast_ptr x, ast_ptr y) {
  ASSERT_TYPE(x, Type::NUM);
  ASSERT_TYPE(y, Type::NUM);

  num_ptr x_cast = dynamic_pointer_cast<Num>(x);
  num_ptr y_cast = dynamic_pointer_cast<Num>(y);
  Bool result(x_cast->value_ < y_cast->value_);
  return make_shared<Bool>(result);
}

ast_ptr Gt(ast_ptr x, ast_ptr y) {
  ASSERT_TYPE(x, Type::NUM);
  ASSERT_TYPE(y, Type::NUM);

  num_ptr x_cast = dynamic_pointer_cast<Num>(x);
  num_ptr y_cast = dynamic_pointer_cast<Num>(y);
  Bool result(x_cast->value_ > y_cast->value_);
  return make_shared<Bool>(result);
}

ast_ptr Lte(ast_ptr x, ast_ptr y) {
  ASSERT_TYPE(x, Type::NUM);
  ASSERT_TYPE(y, Type::NUM);

  num_ptr x_cast = dynamic_pointer_cast<Num>(x);
  num_ptr y_cast = dynamic_pointer_cast<Num>(y);
  Bool result(x_cast->value_ <= y_cast->value_);
  return make_shared<Bool>(result);
}

ast_ptr Gte(ast_ptr x, ast_ptr y) {
  ASSERT_TYPE(x, Type::NUM);
  ASSERT_TYPE(y, Type::NUM);

  num_ptr x_cast = dynamic_pointer_cast<Num>(x);
  num_ptr y_cast = dynamic_pointer_cast<Num>(y);
  Bool result(x_cast->value_ >= y_cast->value_);
  return make_shared<Bool>(result);
}

// TODO(simon) implement everything after this point with AST stuff

float Average(vector<float> xs) {
  float average = 0.0f;
  for (float x : xs) {
    average += x;
  }
  average /= xs.size();
  return average;
}

enum Orientation { CLOCKWISE, COLINEAR, COUNTERCLOCKWISE };

Orientation three_point_orientation(Vector2f p0, Vector2f p1, Vector2f p2) {
  float difference = ((p1.y() - p0.y()) * (p2.x() - p1.x())) -
                     ((p1.x() - p0.x()) * (p2.y() - p1.y()));
  if (abs(difference) < 1.0e-6f) {
    return COLINEAR;
  } else if (difference > 0.0f) {
    return CLOCKWISE;
  } else {
    return COUNTERCLOCKWISE;
  }
}

Polygon ConvexHull(Polygon a, Polygon b) {
  // Create a vector containing every in either polygon.
  vector<Vector2f> all_vertices;
  all_vertices.reserve(a.vertices.size() + b.vertices.size());
  all_vertices.insert(all_vertices.end(), a.vertices.begin(), a.vertices.end());
  all_vertices.insert(all_vertices.end(), b.vertices.begin(), b.vertices.end());

  // Find the leftmost vertex, which is guaranteed to be in the hull.
  size_t leftmost_point_index = 0;
  for (size_t i = 1; i < all_vertices.size(); ++i) {
    if (all_vertices[i].x() < all_vertices[leftmost_point_index].x()) {
      leftmost_point_index = i;
    }
  }

  // Find the rest of the hull using gift-wrapping/Jarvis's algorithm
  vector<Vector2f> hull;
  size_t current_point_index = leftmost_point_index;
  do {
    hull.push_back(all_vertices[current_point_index]);
    size_t next_point_index = (current_point_index + 1) % all_vertices.size();
    for (size_t i = 0; i < all_vertices.size(); ++i) {
      if (three_point_orientation(
              all_vertices[current_point_index], all_vertices[i],
              all_vertices[next_point_index]) == COUNTERCLOCKWISE) {
        next_point_index = i;
      }
    }
    current_point_index = next_point_index;
  } while (current_point_index != leftmost_point_index);

  return {hull};
}

bool PointInPolygon(Vector2f point, Polygon polygon) {
  const size_t vertex_count = polygon.vertices.size();
  const Ray ray = {point, Vector2f(1, 0)};
  size_t crossing_count = 0;
  for (size_t i = 0; i < vertex_count; ++i) {
    const size_t j = (i + 1) % vertex_count;
    const LineSegment edge = {polygon.vertices[i], polygon.vertices[j]};
    if (RayIntersection(ray, edge)) {
      crossing_count += 1;
    }
  }
  return (crossing_count % 2) == 1;
}

float cross2d(Vector2f u, Vector2f v) { return u.x() * v.y() - u.y() * v.x(); }

bool RayIntersection(Ray ray, LineSegment line_segment) {
  const Vector2f p = {-ray.direction.y(), ray.direction.x()};
  const Vector2f a_to_origin = ray.origin - line_segment.a;
  const Vector2f a_to_b = line_segment.b - line_segment.a;

  const float denominator = a_to_b.dot(p);
  if (denominator <= 1.0e-6f) {
    return false;
  }

  const float t1 = abs(cross2d(a_to_b, a_to_origin)) / denominator;
  const float t2 = a_to_origin.dot(p) / denominator;

  return (t2 >= 0.0f) && (t2 <= 1.0f) && (t1 >= 0.0f);
}
