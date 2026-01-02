#ifndef SELF_COLLISION_H_
#define SELF_COLLISION_H_

#include <cstddef>
#include <vector>
#include <glm/glm.hpp>

class SpatialHashGrid {
 public:
  SpatialHashGrid(float cell_size, std::size_t num_particles);

 public:
  void build(const std::vector<glm::vec3>& positions);
  void query_neighbours(const glm::vec3& position,
                        std::vector<std::size_t>& out) const;

 private:
  std::uint32_t get_hash(int ix, int iy, int iz) const;

 private:
  float cell_size_;
  std::size_t table_size_;

  std::vector<uint32_t> cell_starts_;
  std::vector<uint32_t> cell_counts_;
  std::vector<std::size_t> particle_indices_;
};

#endif  // SELF_COLLISION_H_