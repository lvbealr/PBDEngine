#include "constraints.hpp"
#include "particle_system.hpp"

namespace core {

DistanceConstraint::DistanceConstraint(std::size_t i1, std::size_t i2,
                                       float stiffness = 1.0f)
    : indices_({i1, i2}), stiffness_(stiffness) {}

void DistanceConstraint::prepare(const ParticleSystem& ps, float dt) {
  if (rest_dist_ <= 0.0f) {
    const auto& pos = ps.get_current();
    rest_dist_ = glm::distance(pos[indices_[0]], pos[indices_[1]]);
  }
}

void DistanceConstraint::project(ParticleSystem& ps, std::size_t iterations) {
  auto& predicted = ps.get_predicted();
  const auto& inv_masses = ps.get_inv_masses();

  float w1 = inv_masses[indices_[0]];
  float w2 = inv_masses[indices_[1]];
  float w_sum = w1 + w2;

  if (w_sum <= 0.0f)
    return;

  glm::vec3 dir = predicted[indices_[0]] - predicted[indices_[1]];
  float current_length = glm::length(dir);

  if (current_length < 1e-6f) // TODO:
    return;

  float k_prime =
      1.0f - std::pow((1.0f - stiffness_), 1.0f / (float)iterations);

  float constraint_error = current_length - rest_dist_;
  glm::vec3 correction = (dir / current_length) * (constraint_error / w_sum);

  predicted[indices_[0]] -= w1 * correction * k_prime;
  predicted[indices_[1]] += w2 * correction * k_prime;
}

std::vector<std::size_t>& DistanceConstraint::get_indices() {
  return indices_;
}

const std::vector<std::size_t>& DistanceConstraint::get_indices() const {
  return indices_;
}

ConstraintType DistanceConstraint::get_type() const {
  return ConstraintType::Distance;
}

// ========================================================================= //

PlaneCollisionConstraint::PlaneCollisionConstraint(std::size_t i, float floor)
    : indices_({i}), i_(i), floor_(floor) {}

void PlaneCollisionConstraint::project(ParticleSystem& ps,
                                       std::size_t iterations) {
  auto& predicted = ps.get_predicted();

  if (predicted[i_].y >= floor_) {
    return;
  }

  predicted[i_].y = floor_;
}

std::vector<std::size_t>& PlaneCollisionConstraint::get_indices() {
  return indices_;
}

const std::vector<std::size_t>& PlaneCollisionConstraint::get_indices() const {
  return indices_;
}

ConstraintType PlaneCollisionConstraint::get_type() const {
  return ConstraintType::Collision;
}

// ========================================================================= //

SphereCollisionConstraint::SphereCollisionConstraint(std::size_t i,
                                                     glm::vec3 center,
                                                     float radius)
    : indices_({i}), i_(i), center_(center), radius_(radius) {}

void SphereCollisionConstraint::project(ParticleSystem& ps,
                                        std::size_t iterations) {
  auto& predicted = ps.get_predicted();

  glm::vec3 dir = predicted[i_] - center_;
  float dist = glm::length(dir);

  if (dist < radius_) {
    if (dist < 1e-6f) // TODO:
      return;

    predicted[i_] = center_ + (dir / dist) * radius_;
  }
}

void SphereCollisionConstraint::set_center(glm::vec3 center) {
  center_ = center;
}

std::vector<std::size_t>& SphereCollisionConstraint::get_indices() {
  return indices_;
}

const std::vector<std::size_t>& SphereCollisionConstraint::get_indices() const {
  return indices_;
}

ConstraintType SphereCollisionConstraint::get_type() const {
  return ConstraintType::Collision;
}

// ========================================================================= //

AttachmentConstraint::AttachmentConstraint(std::size_t i, glm::vec3 target_pos)
    : i_(i), target_pos_(target_pos) {}

void AttachmentConstraint::project(ParticleSystem& ps, std::size_t iterations) {
  ps.get_predicted()[i_] = target_pos_;
}

void AttachmentConstraint::set_target(glm::vec3& new_pos) {
  target_pos_ = new_pos;
}

// ========================================================================= //

BendingConstraint::BendingConstraint(std::size_t i1, std::size_t i2,
                                     std::size_t i3, float stiffness)
    : indices_({i1, i2, i3}), stiffness_(stiffness) {}

void BendingConstraint::project(ParticleSystem& ps, std::size_t iterations) {
  auto& predicted = ps.get_predicted();
  const auto& inv_masses = ps.get_inv_masses();

  glm::vec3 p1 = predicted[indices_[0]];
  glm::vec3 p2 = predicted[indices_[1]];
  glm::vec3 p3 = predicted[indices_[2]];

  float w1 = inv_masses[indices_[0]];
  float w2 = inv_masses[indices_[1]];
  float w3 = inv_masses[indices_[2]];

  float w_sum = w1 + 2.0f * w2 + w3;
  if (w_sum <= 0.0f) {
    return;
  }

  glm::vec3 center = (p1 + p3) * 0.5f;
  glm::vec3 dir = center - p2;

  float k_prime =
      1.0f - std::pow((1.0f - stiffness_), 1.0f / (float)iterations);

  glm::vec3 correction = dir * k_prime;

  if (w1 > 0.0f) {
    predicted[indices_[0]] -= w1 / w_sum * correction * 0.5f;
  }

  if (w2 > 0.0f) {
    predicted[indices_[1]] += w2 / w_sum * correction * 2.0f;
  }

  if (w3 > 0.0f) {
    predicted[indices_[2]] -= w3 / w_sum * correction * 0.5f;
  }
}

std::vector<std::size_t>& BendingConstraint::get_indices() {
  return indices_;
}

const std::vector<std::size_t>& BendingConstraint::get_indices() const {
  return indices_;
}

ConstraintType BendingConstraint::get_type() const {
  return ConstraintType::Bending;
}

// ========================================================================= //

}  // namespace core
