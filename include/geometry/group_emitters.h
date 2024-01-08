#pragma once

#include <geometry/surface.h>

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
  ~GroupOfEmitters(){};

  glm::vec3 sample(const glm::vec3& look_from, EmitterInfo& emit_info, float rand1,
                   float rand2) const {
    // choose random light object
    float sx = rand1 * list_of_emitters.size();
    const int index_obj = std::clamp((int)sx, 0, (int)list_of_emitters.size() - 1);
    rand1 = sx - index_obj;  // remap the random number for reuse

    // probability of choosing the light object
    const float prob_obj = 1 / list_of_emitters.size();

    glm::vec3 emit_col = list_of_emitters[index_obj]->sample(look_from, emit_info, rand1, rand2);

    // pdf of child sampled and probability of choosing the light
    emit_info.pdf *= prob_obj;

    return emit_col;
  }

  size_t num_lights() const { return num_total_lights; }
};
