#pragma once

#include <ostream>
#include <z3++.h>

class Quantity {
public:
  Quantity(z3::expr expression, int length_exponent, int time_exponent,
           int mass_exponent);
  z3::expr expression() const;
  int length_exponent() const;
  int time_exponent() const;
  int mass_exponent() const;

  bool operator==(Quantity that) const;
  Quantity operator+(Quantity that) const;
  Quantity operator-(Quantity that) const;
  Quantity operator*(Quantity that) const;
  Quantity operator/(Quantity that) const;

  bool has_same_dimensionality_as(Quantity &that) const;
  bool dimensionality_is(int length_exponent, int time_exponent,
                         int mass_exponent) const;

private:
  const z3::expr m_expression;
  // Store in vector type maybe?
  const int m_length_exponent;
  const int m_time_exponent;
  const int m_mass_exponent;
};

std::ostream &operator<<(std::ostream &os, const Quantity &q);

/*
template <>
struct Eigen::NumTraits<Quantity> : Eigen::GenericNumTraits<Quantity> {
  typedef Quantity Real;
  typedef Quantity NonInteger;
  typedef Quantity Nested;

  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 3,
    MulCost = 3
  };
};
*/

class Vector2D {
public:
  Vector2D(Quantity x, Quantity y);
  Quantity x() const;
  Quantity y() const;

  bool operator==(Vector2D that) const;
  Vector2D operator+(Vector2D that) const;
  Vector2D operator-(Vector2D that) const;
  Quantity operator*(Vector2D that) const;

private:
  const Quantity m_x;
  const Quantity m_y;
};