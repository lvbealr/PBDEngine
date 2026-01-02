#include "graphics/include/UI.hpp"
#include "raylib.h"

namespace graphics {

// ========================================================================= //

bool UI::button(int x, int y, int w, int h, const std::string& text) {
  Vector2 mouse = GetMousePosition();
  Rectangle rect = {static_cast<float>(x), static_cast<float>(y),
                    static_cast<float>(w), static_cast<float>(h)};
  bool hover = CheckCollisionPointRec(mouse, rect);
  bool clicked = hover && IsMouseButtonPressed(MOUSE_LEFT_BUTTON);

  DrawRectangleRec(rect, hover ? LIGHTGRAY : GRAY);
  DrawRectangleLinesEx(rect, 2, WHITE);

  int fontSize = 20;
  int textWidth = MeasureText(text.c_str(), fontSize);
  DrawText(text.c_str(), x + (w - textWidth) / 2, y + (h - fontSize) / 2,
           fontSize, BLACK);

  return clicked;
}

// ------------------------------------------------------------------------- //

void UI::slider(int x, int y, int w, int h, const std::string& label,
                float& value, float min, float max, Color color = WHITE) {
  Vector2 mouse = GetMousePosition();
  Rectangle rect = {static_cast<float>(x), static_cast<float>(y),
                    static_cast<float>(w), static_cast<float>(h)};

  if (CheckCollisionPointRec(mouse, rect)
      && IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
    value = min + (mouse.x - x) / static_cast<float>(w) * (max - min);

    if (value < min) {
      value = min;
    }

    if (value > max) {
      value = max;
    }
  }

  DrawRectangleRec(rect, DARKGRAY);
  float fillWidth = (value - min) / (max - min) * w;
  DrawRectangleRec({static_cast<float>(x), static_cast<float>(y), fillWidth,
                    static_cast<float>(h)},
                   BLUE);
  DrawRectangleLinesEx(rect, 1, WHITE);

  std::string valText = label + ": " + std::to_string(value).substr(0, 4);
  DrawText(valText.c_str(), x, y - 20, 18, color);
}

// ------------------------------------------------------------------------- //

void UI::checkbox(int x, int y, int size, const std::string& label,
                  bool& checked) {
  Vector2 mouse = GetMousePosition();
  Rectangle rect = {static_cast<float>(x), static_cast<float>(y),
                    static_cast<float>(size), static_cast<float>(size)};

  if (CheckCollisionPointRec(mouse, rect)
      && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    checked = !checked;
  }

  DrawRectangleRec(rect, checked ? BLUE : DARKGRAY);
  DrawRectangleLinesEx(rect, 2, WHITE);
  DrawText(label.c_str(), x + size + 10, y + 2, 18, WHITE);
}

// ------------------------------------------------------------------------- //

bool UI::draw_interface(Config& config) {
  bool reset = false;
  DrawRectangle(5, 5, 260, 400, Fade(BLACK, 0.7f));
  DrawRectangleLines(5, 5, 260, 400, WHITE);

  DrawText("SETTINGS", 20, 20, 20, YELLOW);

  slider(20, 70, 200, 15, "Gravity X", config.gravity.x, -10.0f, 10.0f, RED);
  slider(20, 110, 200, 15, "Gravity Y", config.gravity.y, -20.0f, 0.0f, GREEN);
  slider(20, 150, 200, 15, "Gravity Z", config.gravity.z, -10.0f, 10.0f, BLUE);

  float iter_f = static_cast<float>(config.iterations);
  slider(20, 200, 200, 15, "PBD Iterations", iter_f, 1.0f, 50.0f);
  config.iterations = static_cast<int>(iter_f);

  checkbox(20, 240, 20, "Pause Simulation", config.is_paused);
  checkbox(20, 280, 20, "Show Grid", config.show_grid);

  if (button(20, 330, 220, 40, "RESET SIM")) {
    reset = true;
  }

  return reset;
}

// ========================================================================= //

}  // namespace graphics