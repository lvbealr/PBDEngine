#ifndef PHYSICS_H_
#define PHYSICS_H_

#include <vector>
#include <glm/glm.hpp>

#include "particle_system.hpp"
#include "constraints.hpp"

namespace core {

// ========================================================================= //

class Physics {
 private:
  static inline constexpr glm::vec3 kGravity = glm::vec3(0.0f, -9.81f, 0.0f);
  static inline constexpr std::size_t kPosIterations = 30;

 public:
  Physics() = default;
  Physics(ParticleSystem* ps, glm::vec3 gravity);
  Physics(Physics& /* unused */) = default;
  Physics(Physics&& /* unused */) noexcept = default;
  ~Physics();

  Physics& operator=(Physics& /* unused */) = default;
  Physics& operator=(Physics&& /* unused */) noexcept = default;

 public:
  void set_gravity(const glm::vec3& gravity);
  glm::vec3& get_gravity();
  const glm::vec3& get_gravity() const;

  void set_iterations(std::size_t iterations);
  std::size_t& get_iterations();
  const std::size_t& get_iterations() const;

  std::vector<Constraint*> get_constraints();
  const std::vector<Constraint*> get_constraints() const;

 public:
  void add_constraint(Constraint* constraint);
  void update(float dt);

 private:
  void resolve_self_collisions(ParticleSystem& ps);

 private:
  ParticleSystem* ps_;
  glm::vec3 gravity_ = kGravity;
  std::size_t iterations_ = kPosIterations;

  std::vector<Constraint*> constraints_;
};

// ========================================================================= //

}  // namespace core

#endif // PHYSICS_H_