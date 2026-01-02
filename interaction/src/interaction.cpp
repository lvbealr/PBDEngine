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

  handle_pin_input();

  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
    process_cutting(camera);
  }

  if (IsKeyPressed(KEY_LEFT_SHIFT)) {
    physics_->get_config()->sphere_active =
        !physics_->get_config()->sphere_active;
  }

  if (physics_->get_config()->sphere_active) {
    Ray ray = GetMouseRay(GetMousePosition(), camera);
    Vector3 target =
        Vector3Add(camera.position, Vector3Scale(ray.direction, 10.0f));
    physics_->get_config()->sphere_pos = {target.x, target.y, target.z};
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

void InteractionSystem::process_cutting(const Camera3D& camera) {
  Ray ray = GetMouseRay(GetMousePosition(), camera);
  const auto& positions = ps_->get_current();
  auto* constraints = physics_->get_constraints();

  const float cutting_threshold =
      physics_->get_config()->cloth_spacing; // TODO:

  for (auto it = constraints->begin(); it != constraints->end();) {
    if ((*it)->get_type() == core::ConstraintType::Collision) {
      ++it;
      continue;
    }

    auto& indices = (*it)->get_indices();
    bool needs_removal = false;

    if (indices.size() >= 2) {
      glm::vec3 center(0.0f);
      for (auto idx : indices) {
        center += positions[idx];
      }

      center /= static_cast<float>(indices.size());

      RayCollision col = GetRayCollisionSphere(
          ray, {center.x, center.y, center.z}, cutting_threshold);

      if (col.hit) {
        needs_removal = true;
      }
    }

    if (needs_removal) {
      delete *it;
      it = constraints->erase(it);
    } else {
      ++it;
    }
  }
}

void InteractionSystem::handle_pin_input() {
  if (IsKeyPressed(KEY_F)) {
    if (grabbed_idx_ != kNotGrabbing) {
      process_pinning();
    }
  }
}

int InteractionSystem::get_grabbed_index() const {
  return grabbed_idx_;
}

void InteractionSystem::process_pinning() {
  auto& inv_masses = ps_->get_inv_masses();

  if (inv_masses[grabbed_idx_] == 0.0f) {
    inv_masses[grabbed_idx_] = 1.0f;
  } else {
    inv_masses[grabbed_idx_] = 0.0f;
    ps_->get_velocities()[grabbed_idx_] = glm::vec3(0.0f);
  }
}

}  // namespace interaction