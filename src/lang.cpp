#include "lang.hpp"

#include <eigen3/Eigen/Core>
#include <ostream>
#include <stdexcept>
#include <z3++.h>

using Eigen::Vector3i;
using z3::expr;

Quantity::Quantity(expr expression, Vector3i dimensionality)
    : m_expression(expression), dimensionality_(dimensionality) {
  if (!expression.is_arith()) {
    throw std::invalid_argument("quantity must be an arithmetic expression");
  }
}

Quantity::Quantity(expr expression, int length_exponent, int time_exponent,
                   int mass_exponent)
    : Quantity(expression, {length_exponent, time_exponent, mass_exponent}) {}

expr Quantity::expression() const { return m_expression; }

Vector3i Quantity::dimensionality() const { return dimensionality_; }

int Quantity::length_exponent() const { return dimensionality_[0]; }

int Quantity::time_exponent() const { return dimensionality_[1]; }

int Quantity::mass_exponent() const { return dimensionality_[2]; }

// Does this actually work?
bool Quantity::operator==(Quantity that) const {
  return has_same_dimensionality_as(that) &&
         expression().to_string() == that.expression().to_string();
}

Quantity Quantity::operator+(Quantity that) const {
  if (!has_same_dimensionality_as(that)) {
    throw std::invalid_argument(
        "added quantities must have the same dimensionalities");
  }
  return Quantity(expression() + that.expression(), dimensionality());
}

Quantity Quantity::operator-(Quantity that) const {
  if (!has_same_dimensionality_as(that)) {
    throw std::invalid_argument(
        "subtracted quantities must have the same dimensionalities");
  }
  return Quantity(expression() - that.expression(), dimensionality());
}

Quantity Quantity::operator*(Quantity that) const {
  return Quantity(expression() * that.expression(),
                  dimensionality() + that.dimensionality());
}

Quantity Quantity::operator/(Quantity that) const {
  return Quantity(expression() / that.expression(),
                  dimensionality() - that.dimensionality());
}

bool Quantity::has_same_dimensionality_as(Quantity &that) const {
  return dimensionality() == that.dimensionality();
}

bool Quantity::dimensionality_is(Vector3i other_dimensionality) const {
  return dimensionality() == other_dimensionality;
}

bool Quantity::dimensionality_is(int length_exponent, int time_exponent,
                                 int mass_exponent) const {
  return dimensionality_is({length_exponent, time_exponent, mass_exponent});
}

// TODO: make this nicer, like 5 m/s
std::ostream &operator<<(std::ostream &os, const Quantity &q) {
  os << q.expression().to_string() << " [" << q.length_exponent() << ", "
     << q.time_exponent() << ", " << q.mass_exponent() << "]";
  return os;
}