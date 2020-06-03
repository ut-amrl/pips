#include "lang.hpp"

#include <ostream>
#include <stdexcept>
#include <z3++.h>

using z3::expr;

Quantity::Quantity(expr expression, int length_exponent, int time_exponent,
                   int mass_exponent)
    : m_expression(expression), m_length_exponent(length_exponent),
      m_time_exponent(time_exponent), m_mass_exponent(mass_exponent) {
  if (!expression.is_arith()) {
    throw std::invalid_argument("quantity must be an arithmetic expression");
  }
}

expr Quantity::expression() const { return this->m_expression; }

int Quantity::length_exponent() const { return this->m_length_exponent; }

int Quantity::time_exponent() const { return this->m_time_exponent; }

int Quantity::mass_exponent() const { return this->m_time_exponent; }

// Does this actually work?
bool Quantity::operator==(Quantity that) const {
  return this->has_same_dimensionality_as(that) &&
         this->expression().to_string() == that.expression().to_string();
}

Quantity Quantity::operator+(Quantity that) const {
  if (!this->has_same_dimensionality_as(that)) {
    throw std::invalid_argument(
        "added quantites must have the same dimensionalities");
  }
  return Quantity(
      /* expression: */ this->expression() + that.expression(),
      /* length exponent: */ this->length_exponent(),
      /* time exponent: */ this->time_exponent(),
      /* mass exponent: */ this->mass_exponent());
}

Quantity Quantity::operator-(Quantity that) const {
  if (!this->has_same_dimensionality_as(that)) {
    throw std::invalid_argument(
        "subtracted quantites must have the same dimensionalities");
  }
  return Quantity(
      /* expression: */ this->expression() - that.expression(),
      /* length exponent: */ this->length_exponent(),
      /* time exponent: */ this->time_exponent(),
      /* mass exponent: */ this->mass_exponent());
}

Quantity Quantity::operator*(Quantity that) const {
  return Quantity(
      /* expression: */ this->expression() * that.expression(),
      /* length exponent: */ this->length_exponent() + that.length_exponent(),
      /* time exponent: */ this->time_exponent() + that.time_exponent(),
      /* mass exponent: */ this->mass_exponent() + that.mass_exponent());
}

Quantity Quantity::operator/(Quantity that) const {
  return Quantity(
      /* expression: */ this->expression() / that.expression(),
      /* length exponent: */ this->length_exponent() - that.length_exponent(),
      /* time exponent: */ this->time_exponent() - that.time_exponent(),
      /* mass exponent: */ this->mass_exponent() - that.mass_exponent());
}

bool Quantity::has_same_dimensionality_as(Quantity &that) const {
  return this->length_exponent() == that.length_exponent() &&
         this->time_exponent() == that.time_exponent() &&
         this->mass_exponent() == that.time_exponent();
}

bool Quantity::dimensionality_is(int length_exponent, int time_exponent,
                                 int mass_exponent) const {
  return this->length_exponent() == length_exponent &&
         this->time_exponent() == time_exponent &&
         this->mass_exponent() == mass_exponent;
}

// TODO: make this nicer, like 5 m/s
std::ostream &operator<<(std::ostream &os, const Quantity &q) {
  os << q.expression().to_string() << " [" << q.length_exponent() << ", "
     << q.time_exponent() << ", " << q.mass_exponent() << "]";
  return os;
}

Vector2D::Vector2D(Quantity x, Quantity y) : m_x(x), m_y(y) {}

Quantity Vector2D::x() const { return this->m_x; }

Quantity Vector2D::y() const { return this->m_y; }

// Does this actually work? It /seems/ to.
bool Vector2D::operator==(Vector2D that) const {
  return this->x() == that.x() && this->y() == that.y();
}

Vector2D Vector2D::operator+(Vector2D that) const {
  return Vector2D(this->x() + that.x(), this->y() + that.y());
}

Quantity Vector2D::operator*(Vector2D that) const {
  return this->x() * that.x() + this->y() * that.y();
}