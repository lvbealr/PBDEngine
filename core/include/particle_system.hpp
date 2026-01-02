#ifndef PARTICLE_SYSTEM_H_
#define PARTICLE_SYSTEM_H_

#include <vector>
#include <glm/glm.hpp>

namespace core {

// ========================================================================= //

class ParticleSystem {
 public:
  ParticleSystem() = default;
  ~ParticleSystem() = default;

 public:
  void set_current(std::vector<glm::vec3>& current);
  std::vector<glm::vec3>& get_current();
  const std::vector<glm::vec3>& get_current() const;

  void set_predicted(std::vector<glm::vec3>& predicted);
  std::vector<glm::vec3>& get_predicted();
  const std::vector<glm::vec3>& get_predicted() const;

  void set_velocities(std::vector<glm::vec3>& velocities);
  std::vector<glm::vec3>& get_velocities();
  const std::vector<glm::vec3>& get_velocities() const;

  void set_inv_masses(std::vector<float>& inv_masses);
  std::vector<float>& get_inv_masses();
  const std::vector<float>& get_inv_masses() const;

 public:
  void validate();
  void reserve(std::size_t capacity);
  void clear();
  std::size_t count();

 public:
  int add_particle(glm::vec3 pos, float mass);

 private:
  std::vector<glm::vec3> current_;
  std::vector<glm::vec3> predicted_;
  std::vector<glm::vec3> velocities_;
  std::vector<float> inv_masses_;
};

// ========================================================================= //

}  // namespace core

#endif // PARTICLE_SYSTEM_H_