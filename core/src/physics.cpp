#include <raylib.h>

#include "physics.hpp"

#include "particle_system.hpp"
#include "constraints.hpp"

namespace core {

// ========================================================================= //

Physics::Physics(ParticleSystem* ps, glm::vec3 gravity = Physics::kGravity)
    : ps_(ps), gravity_(gravity) {}

Physics::~Physics() {
  for (auto* c : constraints_) {
    delete c;
  }

  constraints_.clear();
}

// ------------------------------------------------------------------------- //

void Physics::set_gravity(const glm::vec3& gravity) {
  gravity_ = gravity;
}

glm::vec3& Physics::get_gravity() {
  return gravity_;
}

const glm::vec3& Physics::get_gravity() const {
  return gravity_;
}

void Physics::set_iterations(std::size_t iterations) {
  iterations_ = iterations;
}

std::size_t& Physics::get_iterations() {
  return iterations_;
}

const std::size_t& Physics::get_iterations() const {
  return iterations_;
}

std::vector<Constraint*> Physics::get_constraints() {
  return constraints_;
}

const std::vector<Constraint*> Physics::get_constraints() const {
  return constraints_;
}

// ------------------------------------------------------------------------- //

void Physics::add_constraint(Constraint* constraint) {
  if (constraint) {
    constraints_.push_back(constraint);
  }
}

void Physics::update(float dt) {
  if (dt <= 0.0f)
    return;

  const std::size_t count = ps_->get_current().size();
  auto& velocities = ps_->get_velocities();
  auto& current = ps_->get_current();
  auto& predicted = ps_->get_predicted();
  const auto& inv_masses = ps_->get_inv_masses();

  float time = static_cast<float>(GetTime());

  for (std::size_t i = 0; i < count; ++i) {
    if (inv_masses[i] <= 0.0f) {
      predicted[i] = current[i];
      continue;
    }

    glm::vec3 external_force = gravity_;

    float wind_wave = sin(time * 3.0f + current[i].x * 0.5f) * 4.0f;
    float wind_strength = 12.0f + wind_wave;
    external_force += glm::vec3(0.0f, 0.0f, wind_strength);

    external_force.y += cos(time * 5.0f) * 2.0f;

    velocities[i] += external_force * dt;

    velocities[i] *= 0.99f;

    predicted[i] = current[i] + velocities[i] * dt;
  }

  for (auto* constraint : constraints_) {
    constraint->prepare(*ps_, dt);
  }

  for (std::size_t iter = 0; iter < iterations_; ++iter) {
    for (auto* constraint : constraints_) {
      constraint->project(*ps_, iterations_);
    }
  }

  for (std::size_t i = 0; i < count; ++i) {
    if (inv_masses[i] <= 0.0f)
      continue;

    velocities[i] = (predicted[i] - current[i]) / dt;
    current[i] = predicted[i];
  }
}

// ========================================================================= //

}  // namespace core