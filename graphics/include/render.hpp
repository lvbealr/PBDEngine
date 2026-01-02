#ifndef RENDER_H_
#define RENDER_H_

#include <glm/glm.hpp>
#include <raylib.h>
#include <string>
#include <functional>

namespace graphics {

namespace details {

// ========================================================================= //

struct Settings {
  Vector3 position;
  Vector3 target;
  Vector3 up;
  float fovY;
  CameraMode mode;
  CameraProjection projection;
};

// ========================================================================= //

}  // namespace details

// ========================================================================= //

class Camera {
 public:
  Camera(details::Settings& settings);
  ~Camera() = default;

  Camera(const Camera& other) = default;
  Camera(Camera&& other) noexcept = default;

  Camera& operator=(const Camera& other) = default;
  Camera& operator=(Camera&& other) noexcept = default;

 public:
  void update();
  operator ::Camera3D() const;

 public:
  void set(details::Settings& settings);
  details::Settings& get();
  const details::Settings& get() const;

 public:
  Vector3 get_forward();
  const Vector3 get_forward() const;

  Vector3 get_right();
  const Vector3 get_right() const;

  void move_to(Vector3& position, float smoothness);
  void set_target(Vector3& target);

  void look_at(Vector3& target);
  void set_position(Vector3& position);

  void set_fovY(float fovY);

  void zoom(float amount);

 private:
  details::Settings settings_;
};

// ========================================================================= //

class Controller {
 private:
  static constexpr float kMouseSensitivity = 0.5f;
  static constexpr float kWheelSensitivity = 1.0f;

 public:
  Controller(Camera& camera);
  ~Controller() = default;

 public:
  void update(float dt);
  void set_sensitivity(float mouse, float wheel);

 private:
  void handle_orbit(float dt);
  void handle_movement(float dt);
  void handle_zoom();
  void handle_reset();

 private:
  Camera& camera_;
  details::Settings default_;
  float mouse_sensitivity_ = kMouseSensitivity;
  float wheel_sensitivity_ = kWheelSensitivity;
};

// ========================================================================= //

class Render {
 public:
  template <typename... Flags>
  Render(const int width, const int height, const std::string& title,
         Flags... flags) {
    unsigned int combined = 0;

    if constexpr (sizeof...(Flags) > 0) {
      combined = (flags | ...);
      SetConfigFlags(combined);
    }

    InitWindow(width, height, title.data());
    SetTargetFPS(60); // TODO:
  }
  ~Render();

 public:
  void setup(details::Settings& settings);
  Controller& get_controller();
  const Controller& get_controller() const;

  Camera& get_camera();
  const Camera& get_camera() const;

 public:
  bool should_close() const;

 public:
  void update();
  void draw(std::function<void()> scene_func, std::function<void()> ui_func);

 private:
  std::unique_ptr<Camera> camera_;
  std::unique_ptr<Controller> controller_;
};

// ========================================================================= //

}  // namespace graphics

#endif // RENDER_H_