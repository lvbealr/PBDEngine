#include <glm/glm.hpp>
#include <raylib.h>
#include <rlgl.h>

#include "graphics/include/render.hpp"

namespace graphics {

namespace details {

static inline glm::vec3 GLM(const Vector3& v) {
  return glm::vec3(v.x, v.y, v.z);
}

static inline Vector3 RAY(const glm::vec3& v) {
  return Vector3({v.x, v.y, v.z});
}

}  // namespace details

// ========================================================================= //

Camera::Camera(details::Settings& settings) : settings_(settings) {}

void Camera::update() {}

Camera::operator Camera3D() const {
  return {settings_.position, settings_.target, settings_.up, settings_.fovY,
          static_cast<int>(settings_.projection)};
}

void Camera::set(details::Settings& settings) {
  settings_ = settings;
}

details::Settings& Camera::get() {
  return settings_;
}

const details::Settings& Camera::get() const {
  return settings_;
}

Vector3 Camera::get_forward() {
  return details::RAY(glm::normalize(details::GLM(settings_.target)
                                     - details::GLM(settings_.position)));
}

const Vector3 Camera::get_forward() const {
  return details::RAY(glm::normalize(details::GLM(settings_.target)
                                     - details::GLM(settings_.position)));
}

Vector3 Camera::get_right() {
  Vector3 forward = get_forward();
  return details::RAY(glm::normalize(
      glm::cross(details::GLM(forward), details::GLM(settings_.up))));
}

const Vector3 Camera::get_right() const {
  Vector3 forward = get_forward();
  return details::RAY(glm::normalize(
      glm::cross(details::GLM(forward), details::GLM(settings_.up))));
}

void Camera::move_to(Vector3& position, float smoothness = 1.0) {
  settings_.position.x +=
      (settings_.target.x - settings_.position.x) * smoothness;
  settings_.position.y +=
      (settings_.target.y - settings_.position.y) * smoothness;
  settings_.position.z +=
      (settings_.target.z - settings_.position.z) * smoothness;
}

void Camera::set_target(Vector3& target) {
  settings_.target = target;
}

void Camera::look_at(Vector3& target) {
  Camera::set_target(target);
  // TODO: optional
}

void Camera::set_position(Vector3& position) {
  settings_.position = position;
}

void Camera::set_fovY(float fovY) {
  settings_.fovY = fovY;
}

void Camera::zoom(float amount) {
  glm::vec3 pos = details::GLM(settings_.position);
  glm::vec3 target = details::GLM(settings_.target);
  glm::vec3 direction = pos - target;

  float distance = glm::length(direction);

  float new_distance = distance + amount;

  if (new_distance < 1.0f) {
    new_distance = 1.0f;
  }

  if (new_distance > 100.0f) {
    new_distance = 100.0f;
  }

  glm::vec3 new_pos = target + glm::normalize(direction) * new_distance;
  settings_.position = details::RAY(new_pos);
}

// ========================================================================= //

Controller::Controller(Camera& camera)
    : camera_(camera), default_(camera_.get()) {}

void Controller::update(float dt) {
  handle_zoom();
  handle_orbit(dt);
  handle_movement(dt);
  handle_reset();
}

void Controller::set_sensitivity(float mouse, float wheel) {
  mouse_sensitivity_ = mouse;
  wheel_sensitivity_ = wheel;
}

void Controller::handle_orbit(float dt) {
  float speed = 1.0f * dt;
  float dx = 0.0f;
  float dy = 0.0f;

  if (IsKeyDown(KEY_RIGHT)) {
    dx += speed;
  }

  if (IsKeyDown(KEY_LEFT)) {
    dx -= speed;
  }

  if (IsKeyDown(KEY_UP)) {
    dy -= speed;
  }

  if (IsKeyDown(KEY_DOWN)) {
    dy += speed;
  }

  if (dx != 0.0f || dy != 0.0f) {
    details::Settings setup = camera_.get();

    glm::vec3 relPos =
        details::GLM(setup.position) - details::GLM(setup.target);
    float radius = glm::length(relPos);

    float theta = atan2(relPos.x, relPos.z);
    float phi = acos(relPos.y / radius);

    theta += dx;
    phi += dy;

    const float epsilon = 0.1f;
    if (phi < epsilon) {
      phi = epsilon;
    }

    if (phi > PI - epsilon) {
      phi = PI - epsilon;
    }

    relPos.x = radius * sin(phi) * sin(theta);
    relPos.y = radius * cos(phi);
    relPos.z = radius * sin(phi) * cos(theta);

    setup.position =
        details::RAY(glm::vec3(details::GLM(setup.target)) + relPos);
    camera_.set(setup);
  }
}

void Controller::handle_movement(float dt) {
  float moveSpeed = 10.0f * dt;
  glm::vec3 offset(0.0f);

  glm::vec3 forward = details::GLM(camera_.get_forward());
  forward.y = 0;
  forward = glm::normalize(forward);

  glm::vec3 right = details::GLM(camera_.get_right());

  if (IsKeyDown(KEY_W)) {
    offset += forward * moveSpeed;
  }

  if (IsKeyDown(KEY_S)) {
    offset -= forward * moveSpeed;
  }

  if (IsKeyDown(KEY_D)) {
    offset += right * moveSpeed;
  }

  if (IsKeyDown(KEY_A)) {
    offset -= right * moveSpeed;
  }

  if (glm::length(offset) > 0) {
    details::Settings setup = camera_.get();

    setup.position = details::RAY(details::GLM(setup.position) + offset);
    setup.target = details::RAY(details::GLM(setup.target) + offset);

    camera_.set(setup);
  }
}

void Controller::handle_zoom() {
  float wheel = GetMouseWheelMove();

  if (wheel != 0) {  // TODO:
    camera_.zoom(-wheel * wheel_sensitivity_);
  }
}

void Controller::handle_reset() {
  if (IsKeyPressed(KEY_R)) {
    camera_.set(default_);
  }
}

// ========================================================================= //

Render::~Render() {
  if (IsWindowReady()) {
    CloseWindow();
  }
}

void Render::setup(details::Settings& settings) {
  camera_ = std::make_unique<Camera>(settings);
  controller_ = std::make_unique<Controller>(*camera_);
}

Controller& Render::get_controller() {
  return *controller_;
}

const Controller& Render::get_controller() const {
  return *controller_;
}

Camera& Render::get_camera() {
  return *camera_;
}

const Camera& Render::get_camera() const {
  return *camera_;
}

bool Render::should_close() const {
  return WindowShouldClose();
}

void Render::update() {
  float dt = GetFrameTime();

  if (controller_) {
    controller_->update(dt);
  }
}

void Render::draw(std::function<void()> scene_func,
                  std::function<void()> ui_func = nullptr) {
  int width = GetRenderWidth();
  int height = GetRenderHeight();

  BeginDrawing();
  ClearBackground({30, 30, 30, 255});  // TODO:

  if (camera_) {
    rlViewport(0, 0, width, height);
    BeginMode3D(*camera_);

    if (scene_func) {
      scene_func();
    }

    EndMode3D();
  }

  if (ui_func) {
    ui_func();
  }

  EndDrawing();
}

// ========================================================================= //

}  // namespace graphics