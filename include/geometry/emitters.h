#pragma once

#include <rng/sampling.h>

#include <vector>

struct EmitterInfo {
  glm::vec3 wi;  // direction vector from look_from to point on surface
  float pdf;     // pdf of sampling point on light in area measure
  float dist;    // distance to point on light surface
  float G;       // geometry term for point on light
};

class Emitter {
public:
  Emitter() = default;
  ~Emitter() = default;

  /*
    input point to sample from (look_from) and 2 random numbers. Return emission from light
    and fill EmitterInfo
  */
  virtual std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                                   pcg32_random_t& pcg_rng) const
      = 0;
  virtual float surf_pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                         const glm::vec3& dir) const
      = 0;

  virtual bool is_background() const { return false; }
};

class GroupOfEmitters {
private:
  std::vector<Emitter*> list_of_emitters;
  size_t num_total_lights = 0;

public:
  GroupOfEmitters(const std::vector<Emitter*>& list_lights) : list_of_emitters(list_lights) {
    num_total_lights = list_of_emitters.size();
  }
  ~GroupOfEmitters() = default;

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const;

  size_t num_lights() const { return num_total_lights; }
};
