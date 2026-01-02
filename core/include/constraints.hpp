#ifndef CONSTRAINTS_H_
#define CONSTRAINTS_H_

#include "particle_system.hpp"

namespace core {

// ========================================================================= //

enum class ConstraintType {
  Distance,
  Bending,
  Volume,
  Collision,
  Attachment
};

// ========================================================================= //

class Constraint {
 public:
  virtual ~Constraint() = default;
  virtual void prepare(const ParticleSystem& ps, float dt) {}
  virtual void project(ParticleSystem& ps, std::size_t iterations) = 0;
  virtual std::vector<std::size_t>& get_indices() = 0;
  virtual const std::vector<std::size_t>& get_indices() const = 0;
  virtual ConstraintType get_type() const = 0;
};

// ========================================================================= //

class DistanceConstraint : public Constraint {
 public:
  DistanceConstraint(std::size_t i1, std::size_t i2, float stiffness);

  void prepare(const ParticleSystem& ps, float dt) override;
  void project(ParticleSystem& ps, std::size_t iterations) override;
  std::vector<std::size_t>& get_indices() override;
  const std::vector<std::size_t>& get_indices() const override;
  ConstraintType get_type() const override;

 private:
  std::vector<std::size_t> indices_;
  float rest_dist_;
  float stiffness_;
};

// ========================================================================= //

class PlaneCollisionConstraint : public Constraint {
 public:
  PlaneCollisionConstraint(std::size_t i, float floor);

  void project(ParticleSystem& ps, std::size_t iterations) override;
  std::vector<std::size_t>& get_indices() override;
  const std::vector<std::size_t>& get_indices() const override;
  ConstraintType get_type() const override;

 private:
  std::vector<std::size_t> indices_;
  std::size_t i_;
  float floor_;
};

// ========================================================================= //

class SphereCollisionConstraint : public Constraint {
 public:
  SphereCollisionConstraint(std::size_t i, glm::vec3 center, float radius);

  void project(ParticleSystem& ps, std::size_t iterations) override;

  void set_center(glm::vec3 center);

  std::vector<std::size_t>& get_indices() override;
  const std::vector<std::size_t>& get_indices() const override;
  ConstraintType get_type() const override;

 private:
  std::vector<std::size_t> indices_;
  std::size_t i_;
  glm::vec3 center_;
  float radius_;
};

// ========================================================================= //

class AttachmentConstraint : public Constraint {
  AttachmentConstraint(std::size_t i, glm::vec3 target_pos);

  void project(ParticleSystem& ps, std::size_t iterations) override;
  void set_target(glm::vec3& new_pos);

 private:
  std::size_t i_;
  glm::vec3 target_pos_;
};

// ========================================================================= //

class BendingConstraint : public Constraint {
 public:
  BendingConstraint(std::size_t i1, std::size_t i2, std::size_t i3,
                    float stiffness);

  void project(ParticleSystem& ps, std::size_t iterations) override;

  std::vector<std::size_t>& get_indices() override;
  const std::vector<std::size_t>& get_indices() const override;
  ConstraintType get_type() const override;

 private:
  std::vector<std::size_t> indices_;
  float stiffness_;
};

// ========================================================================= //

}  // namespace core

#endif  // CONSTRAINTS_H_