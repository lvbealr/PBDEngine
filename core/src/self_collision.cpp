#include <glm/glm.hpp>
#include <vector>

#include "include/self_collision.hpp"

// ========================================================================= //

SpatialHashGrid::SpatialHashGrid(float cell_size) : cell_size_(cell_size) {}

void SpatialHashGrid::build(const std::vector<glm::vec3>& positions) {
  grid_.clear();
  for (std::size_t i = 0; i < positions.size(); ++i) {
    uint64_t key = hash(positions[i]);
    grid_[key].push_back(i);
  }
}

std::vector<std::size_t> SpatialHashGrid::get_neighbours(
    const glm::vec3& position) const {
  std::vector<std::size_t> neighbours;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        glm::vec3 neighbour_pos = position + glm::vec3(x, y, z) * cell_size_;
        uint64_t key = hash(neighbour_pos);

        auto it = grid_.find(key);
        if (it != grid_.end()) {
          neighbours.insert(neighbours.end(), it->second.begin(),
                            it->second.end());
        }
      }
    }
  }

  return neighbours;
}

std::uint64_t SpatialHashGrid::hash(const glm::vec3& position) const {
  std::int64_t ix =
      static_cast<std::int64_t>(std::floor(position.x / cell_size_));
  std::int64_t iy =
      static_cast<std::int64_t>(std::floor(position.y / cell_size_));
  std::int64_t iz =
      static_cast<std::int64_t>(std::floor(position.z / cell_size_));

  return (static_cast<std::uint64_t>(ix) * 73856093)
         ^ (static_cast<std::uint64_t>(iy) * 19349663)
         ^ (static_cast<std::uint64_t>(iz) * 83492791);
}

// ========================================================================= //