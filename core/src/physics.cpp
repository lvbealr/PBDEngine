#include <raylib.h>

#include "physics.hpp"

#include "particle_system.hpp"
#include "constraints.hpp"
#include "self_collision.hpp"
#include "common.hpp"

namespace core {

// ========================================================================= //

Physics::Physics(ParticleSystem* ps, details::Config* config)
    : ps_(ps), config_(config), grid_(0.4f, ps->get_current().size()) {}

Physics::~Physics() {
  for (auto* c : constraints_) {
    delete c;
  }

  constraints_.clear();
}

// ------------------------------------------------------------------------- //

void Physics::set_gravity(const glm::vec3& gravity) {
  config_->gravity = gravity;
}

glm::vec3& Physics::get_gravity() {
  return config_->gravity;
}

const glm::vec3& Physics::get_gravity() const {
  return config_->gravity;
}

void Physics::set_iterations(std::size_t iterations) {
  config_->iterations = iterations;
}

std::size_t& Physics::get_iterations() {
  return config_->iterations;
}

const std::size_t& Physics::get_iterations() const {
  return config_->iterations;
}

std::vector<Constraint*>* Physics::get_constraints() {
  return &constraints_;
}

const std::vector<Constraint*>* Physics::get_constraints() const {
  return &constraints_;
}

details::Config* Physics::get_config() {
  return config_;
}

const details::Config* Physics::get_config() const {
  return config_;
}

// ------------------------------------------------------------------------- //

void Physics::add_constraint(Constraint* constraint) {
  if (constraint) {
    constraints_.push_back(constraint);
  }
}

void Physics::remove_constraint(Constraint* constraint) {
  constraints_.erase(
      std::remove(constraints_.begin(), constraints_.end(), constraint),
      constraints_.end());
}

void Physics::update(float dt) {
  if (dt <= 0.0f || config_->is_paused) {
    return;
  }

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

    glm::vec3 external_force = config_->gravity;

    if (config_->wind_enabled) {
      float wind_wave = sin(time * 3.0f + current[i].x * 0.5f) * 4.0f;
      float wind_strength = 12.0f + wind_wave;
      external_force += glm::vec3(0.0f, 0.0f, wind_strength);
      external_force.y += cos(time * 5.0f) * 2.0f;
    }

    velocities[i] += external_force * dt;

    velocities[i] *= 0.99f;

    predicted[i] = current[i] + velocities[i] * dt;
  }

  for (auto* constraint : constraints_) {
    constraint->prepare(*ps_, dt);
  }

  grid_.build(ps_->get_predicted());

  for (std::size_t iter = 0; iter < config_->iterations; ++iter) {
    for (auto* constraint : constraints_) {
      constraint->project(*ps_, config_->iterations);
    }

    resolve_sphere_collision();
    resolve_self_collision();
  }

  for (std::size_t i = 0; i < count; ++i) {
    if (inv_masses[i] <= 0.0f) {
      continue;
    }

    velocities[i] = (predicted[i] - current[i]) / dt;

    if (predicted[i].y <= 0.06f) {
      if (velocities[i].y < 0.0f) {
        velocities[i].y = 0;
      }

      velocities[i].x *= 0.5f;
      velocities[i].z *= 0.5f;

      if (glm::length(glm::vec2(velocities[i].x, velocities[i].z)) < 0.1f) {
        velocities[i].x = 0.0f;
        velocities[i].z = 0.0f;
      }

      predicted[i].y = 0.05f;
    }

    current[i] = predicted[i];
  }
}

void Physics::resolve_self_collision() {
  auto& predicted = ps_->get_predicted();
  const auto& inv_masses = ps_->get_inv_masses();

  float spacing = config_->cloth_spacing; // TODO:
  float min_dist = spacing * 0.8f;
  float min_dist_sq = min_dist * min_dist;

  for (std::size_t i = 0; i < predicted.size(); ++i) {
    if (inv_masses[i] == 0.0f)
      continue;

    grid_.query_neighbours(predicted[i], neighbour_buffer_);

    for (std::size_t j : neighbour_buffer_) {
      if (i <= j) {
        continue;
      }

      glm::vec3 dir = predicted[i] - predicted[j];
      float dist_sq = glm::dot(dir, dir);

      if (dist_sq < min_dist_sq && dist_sq > 1e-8f) {
        float dist = std::sqrt(dist_sq);
        float w_sum = inv_masses[i] + inv_masses[j];
        glm::vec3 correction =
            (dir / dist) * ((min_dist - dist) / w_sum) * 0.5f;

        predicted[i] += inv_masses[i] * correction;
        predicted[j] -= inv_masses[j] * correction;
      }
    }
  }
}

void Physics::resolve_sphere_collision() {
  if (!config_->sphere_active) {
    return;
  }

  auto& predicted = ps_->get_predicted();
  const glm::vec3 center = config_->sphere_pos;
  const float radius = config_->sphere_radius;
  const float radius_sq = radius * radius;

  for (size_t i = 0; i < predicted.size(); ++i) {
    glm::vec3 dir = predicted[i] - center;
    float dist_sq = glm::dot(dir, dir);

    if (dist_sq < radius_sq) {
      float dist = std::sqrt(dist_sq);
      if (dist < 1e-6f) {
        continue;
      }

      predicted[i] = center + (dir / dist) * radius;
    }
  }
}

// ========================================================================= //

}  // namespace core