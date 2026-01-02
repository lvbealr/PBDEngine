#ifndef UI_H_
#define UI_H_

#include <glm/glm.hpp>
#include <raylib.h>

#include "common.hpp"

namespace graphics {

// ========================================================================= //

struct Config {
  glm::vec3 gravity = glm::vec3{0.0f, -9.81f, 0.0f};
  std::size_t iterations = 50;
  bool is_paused = false;
  bool show_grid = true;
  bool show_constraints = true;
  bool visible = false;
  bool wind_enabled = false;
};

// ========================================================================= //

class UI {
 public:
  static bool draw_interface(::details::Config& config);

 private:
  static bool button(int x, int y, int w, int h, const std::string& text);
  static void slider(int x, int y, int w, int h, const std::string& label,
                     float& value, float min, float max, Color color);
  static void checkbox(int x, int y, int size, const std::string& label,
                       bool& checked);
};

// ========================================================================= //

}  // namespace graphics

#endif // UI_H_