#ifndef INTERACTION_H_
#define INTERACTION_H_

#include "core/include/constraints.hpp"
#include "core/include/physics.hpp"
#include "core/include/particle_system.hpp"

#include <raylib.h>

namespace interaction {

class InteractionSystem {
 private:
  static constexpr int kNotGrabbing = -1;

 public:
  InteractionSystem(core::Physics* physics, core::ParticleSystem* ps);

 public:
  void update(const Camera3D& camera);
  bool is_grabbing() const;

 private:
  void try_grab(const Camera3D& camera);
  void move_grabbed(const Camera3D& camera);
  void release();

 private:
  core::Physics* physics_;
  core::ParticleSystem* ps_;
  int grabbed_idx_ = kNotGrabbing;
  float last_dist_ = 0.0f;
  core::AttachmentConstraint* mouse_constraint_;
};

}  // namespace interaction

#endif // INTERACTION