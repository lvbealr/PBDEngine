#include <limits>
#include <glm/glm.hpp>
#include <raylib.h>
#include <raymath.h>

#include "interaction/include/interaction.hpp"

namespace interaction {

InteractionSystem::InteractionSystem(core::Physics* physics,
                                     core::ParticleSystem* ps)
    : physics_(physics), ps_(ps), mouse_constraint_(nullptr) {}

void InteractionSystem::update(const Camera3D& camera) {
  if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    try_grab(camera);
  }

  if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && grabbed_idx_ != -1) {
    move_grabbed(camera);
  }

  if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
    release();
  }
}

bool InteractionSystem::is_grabbing() const {
  return grabbed_idx_ != kNotGrabbing;
}

void InteractionSystem::try_grab(const Camera3D& camera) {
  Ray ray = GetMouseRay(GetMousePosition(), camera);
  float closest_dist = std::numeric_limits<float>::max();
  const auto& positions = ps_->get_current();

  for (size_t i = 0; i < positions.size(); ++i) {
    RayCollision col = GetRayCollisionSphere(
        ray, {positions[i].x, positions[i].y, positions[i].z}, 0.4f);

    if (col.hit && col.distance < closest_dist) {
      closest_dist = col.distance;
      grabbed_idx_ = static_cast<int>(i);
      last_dist_ = col.distance;
    }
  }

  if (grabbed_idx_ != -1) {
    mouse_constraint_ = new core::AttachmentConstraint(
        grabbed_idx_, ps_->get_current()[grabbed_idx_]);
    physics_->add_constraint(mouse_constraint_);
  }
}

void InteractionSystem::move_grabbed(const Camera3D& camera) {
  Ray ray = GetMouseRay(GetMousePosition(), camera);

  Vector3 target =
      Vector3Add(camera.position, Vector3Scale(ray.direction, last_dist_));

  glm::vec3 new_pos = {target.x, target.y, target.z};
  mouse_constraint_->set_target(new_pos);
}

void InteractionSystem::release() {
  if (mouse_constraint_) {
    physics_->remove_constraint(mouse_constraint_);
    delete mouse_constraint_;
    mouse_constraint_ = nullptr;
  }

  grabbed_idx_ = kNotGrabbing;
}

}  // namespace interaction