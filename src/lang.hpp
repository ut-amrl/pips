#pragma once

#include <eigen3/Eigen/Core>
#include <ostream>
#include <z3++.h>

class Parameter {};

class Quantity {
public:
  Quantity(z3::expr expression, Eigen::Vector3i dimensionality);
  Quantity(z3::expr expression, int length_exponent, int time_exponent,
           int mass_exponent);
  z3::expr expression() const;
  Eigen::Vector3i dimensionality() const;
  int length_exponent() const;
  int time_exponent() const;
  int mass_exponent() const;

  bool operator==(Quantity that) const;
  Quantity operator+(Quantity that) const;
  Quantity operator-(Quantity that) const;
  Quantity operator*(Quantity that) const;
  Quantity operator/(Quantity that) const;

  bool has_same_dimensionality_as(Quantity &that) const;
  bool dimensionality_is(Eigen::Vector3i dimensionality) const;
  bool dimensionality_is(int length_exponent, int time_exponent,
                         int mass_exponent) const;

private:
  const z3::expr m_expression;
  const Eigen::Vector3i dimensionality_;
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