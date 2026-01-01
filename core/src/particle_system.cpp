#include "particle_system.hpp"

// ============================== ParticleSystem =========================== //
// --------------------------------  get/set  ------------------------------ //

namespace core {

void ParticleSystem::set_current(std::vector<glm::vec3>& current) {
  current_ = current;
}

std::vector<glm::vec3>& ParticleSystem::get_current() {
  return current_;
}

const std::vector<glm::vec3>& ParticleSystem::get_current() const {
  return current_;
}

void ParticleSystem::set_predicted(std::vector<glm::vec3>& predicted) {
  predicted_ = predicted;
}

std::vector<glm::vec3>& ParticleSystem::get_predicted() {
  return predicted_;
}

const std::vector<glm::vec3>& ParticleSystem::get_predicted() const {
  return predicted_;
}

void ParticleSystem::set_velocities(std::vector<glm::vec3>& velocities) {
  velocities_ = velocities;
}

std::vector<glm::vec3>& ParticleSystem::get_velocities() {
  return velocities_;
}

const std::vector<glm::vec3>& ParticleSystem::get_velocities() const {
  return velocities_;
}

void ParticleSystem::set_inv_masses(std::vector<float>& inv_masses) {
  inv_masses_ = inv_masses;
}

std::vector<float>& ParticleSystem::get_inv_masses() {
  return inv_masses_;
}

const std::vector<float>& ParticleSystem::get_inv_masses() const {
  return inv_masses_;
}

// ------------------------------------------------------------------------- //

void ParticleSystem::validate() {
  size_t size = current_.size();
  assert(predicted_.size() == size && velocities_.size() == size
         && inv_masses_.size() == size);
}

void ParticleSystem::reserve(std::size_t capacity) {
  current_.reserve(capacity);
  predicted_.reserve(capacity);
  velocities_.reserve(capacity);
  inv_masses_.reserve(capacity);
}

void ParticleSystem::clear() {
  current_.clear();
  predicted_.clear();
  velocities_.clear();
  inv_masses_.clear();
}

// ------------------------------------------------------------------------- //

int ParticleSystem::add_particle(glm::vec3 pos, float mass) {
  float inv_mass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;

  current_.push_back(pos);
  predicted_.push_back(pos);
  velocities_.push_back(glm::vec3(0.0f));
  inv_masses_.push_back(inv_mass);

  return current_.size() - 1;
}

}  // namespace core