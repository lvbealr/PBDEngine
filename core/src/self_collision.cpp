#include <glm/glm.hpp>
#include <vector>

#include "include/self_collision.hpp"

// ========================================================================= //

SpatialHashGrid::SpatialHashGrid(float cell_size, std::size_t num_particles)
    : cell_size_(cell_size), table_size_(num_particles * 2) {
  cell_starts_.resize(table_size_);
  cell_counts_.resize(table_size_);
  particle_indices_.resize(num_particles);
}

void SpatialHashGrid::build(const std::vector<glm::vec3>& positions) {
  if (positions.empty()) {
    return;
  }

  if (particle_indices_.size() != positions.size()) {
    table_size_ = positions.size() * 2;
    cell_starts_.resize(table_size_);
    cell_counts_.resize(table_size_);
    particle_indices_.resize(positions.size());
  }

  std::fill(cell_counts_.begin(), cell_counts_.end(), 0);

  for (const auto& pos : positions) {
    int ix = static_cast<int>(std::floor(pos.x / cell_size_));
    int iy = static_cast<int>(std::floor(pos.y / cell_size_));
    int iz = static_cast<int>(std::floor(pos.z / cell_size_));
    cell_counts_[get_hash(ix, iy, iz)]++;
  }

  uint32_t start = 0;
  for (std::size_t i = 0; i < table_size_; ++i) {
    cell_starts_[i] = start;
    start += cell_counts_[i];
    cell_counts_[i] = 0;
  }

  for (std::size_t i = 0; i < positions.size(); ++i) {
    int ix = static_cast<int>(std::floor(positions[i].x / cell_size_));
    int iy = static_cast<int>(std::floor(positions[i].y / cell_size_));
    int iz = static_cast<int>(std::floor(positions[i].z / cell_size_));
    uint32_t h = get_hash(ix, iy, iz);
    particle_indices_[cell_starts_[h] + cell_counts_[h]] = i;
    cell_counts_[h]++;
  }
}

void SpatialHashGrid::query_neighbours(const glm::vec3& position,
                                       std::vector<std::size_t>& out) const {
  out.clear();
  int ix = static_cast<int>(std::floor(position.x / cell_size_));
  int iy = static_cast<int>(std::floor(position.y / cell_size_));
  int iz = static_cast<int>(std::floor(position.z / cell_size_));

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        uint32_t h = get_hash(ix + x, iy + y, iz + z);
        uint32_t start = cell_starts_[h];
        uint32_t count = cell_counts_[h];

        for (uint32_t i = 0; i < count; ++i) {
          out.push_back(particle_indices_[start + i]);
        }
      }
    }
  }
}

uint32_t SpatialHashGrid::get_hash(int ix, int iy, int iz) const {
  if (table_size_ == 0) {
    return 0;
  }

  uint32_t h = (static_cast<uint32_t>(ix) * 73856093)
               ^ (static_cast<uint32_t>(iy) * 19349663)
               ^ (static_cast<uint32_t>(iz) * 83492791);

  return h % table_size_;
}

// ========================================================================= //