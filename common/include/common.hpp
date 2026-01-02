#ifndef COMMON_H_
#define COMMON_H_

#include <glm/glm.hpp>
#include <cstddef>

namespace details {
struct Config {
  glm::vec3 gravity = glm::vec3{0.0f, -9.81f, 0.0f};
  std::size_t iterations = 10;
  bool is_paused = false;
  bool show_grid = true;
  bool show_constraints = true;
  bool visible = false;
  bool wind_enabled = false;
  float wind_strength = 12.0f;
  bool show_fps = false;
  bool sphere_active = false;
  glm::vec3 sphere_pos = {0.0f, 5.0f, 0.0f};
  float sphere_radius = 1.0f;
  float cloth_spacing = 0.4f;
};
}  // namespace details

#endif // COMMON_H_