#ifndef PHYSICS_H_
#define PHYSICS_H_

#include <vector>
#include <glm/glm.hpp>

#include "particle_system.hpp"
#include "constraints.hpp"
#include "common.hpp"
#include "self_collision.hpp"

namespace core {

// ========================================================================= //

class Physics {
 public:
  Physics(ParticleSystem* ps, details::Config* config);
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

  std::vector<Constraint*>* get_constraints();
  const std::vector<Constraint*>* get_constraints() const;

  details::Config* get_config();
  const details::Config* get_config() const;

 public:
  void add_constraint(Constraint* constraint);
  void remove_constraint(Constraint* constraint);
  void update(float dt);

 private:
  void resolve_self_collision();
  void resolve_sphere_collision();

 private:
  ParticleSystem* ps_;
  details::Config* config_;
  std::vector<Constraint*> constraints_;

  SpatialHashGrid grid_;
  std::vector<std::size_t> neighbour_buffer_;
};

// ========================================================================= //

}  // namespace core

#endif // PHYSICS_H_