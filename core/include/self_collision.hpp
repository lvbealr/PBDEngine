#ifndef SELF_COLLISION_H_
#define SELF_COLLISION_H_

#include <cstddef>
#include <unordered_map>
#include <vector>
#include <glm/glm.hpp>

class SpatialHashGrid {
 public:
  explicit SpatialHashGrid(float cell_size);

 public:
  void build(const std::vector<glm::vec3>& positions);
  std::vector<std::size_t> get_neighbours(const glm::vec3& position) const;

 private:
  std::uint64_t hash(const glm::vec3& position) const;

 private:
  float cell_size_;
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> grid_;
};

#endif // SELF_COLLISION_H_