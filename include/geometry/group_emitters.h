#pragma once

#include <geometry/surface.h>
#include <rng/sampling.h>

#include <algorithm>
#include <vector>

class GroupOfEmitters {
private:
  std::vector<Surface*> list_of_emitters;
  size_t num_total_lights = 0;

public:
  GroupOfEmitters(const std::vector<Surface*>& list_lights) : list_of_emitters(list_lights) {
    num_total_lights = list_of_emitters.size();
  }
  ~GroupOfEmitters() {};

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const {
    float rand = rand_float(pcg_rng);

    // choose random light object
    float sx = rand * list_of_emitters.size();
    const int index_obj = std::clamp((int)sx, 0, (int)list_of_emitters.size() - 1);

    // probability of choosing the light object
    const float prob_obj = 1 / list_of_emitters.size();

    auto emitCol_emitInfo = list_of_emitters[index_obj]->sample(look_from, pcg_rng);

    // pdf of child sampled and probability of choosing the light
    emitCol_emitInfo.second.pdf *= prob_obj;

    return emitCol_emitInfo;
  }

  size_t num_lights() const { return num_total_lights; }
};
