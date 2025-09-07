#include <color_utils.h>
#include <fmt/core.h>

#include <cmath>

static inline glm::vec3 agxDefaultContrastApprox(glm::vec3 x) {
  glm::vec3 x2 = x * x;
  glm::vec3 x4 = x2 * x2;

  return glm::vec3(+15.5f) * x4 * x2 - glm::vec3(40.14f) * x4 * x + glm::vec3(31.96f) * x4
         - glm::vec3(6.868f) * x2 * x + glm::vec3(0.4298f) * x2 + glm::vec3(0.1191f) * x
         - glm::vec3(0.00232f);
}

static inline glm::vec3 agx(glm::vec3 val) {
  const glm::mat3 agx_mat = glm::mat3(0.842479062253094, 0.0423282422610123, 0.0423756549057051,
                                      0.0784335999999992, 0.878468636469772, 0.0784336,
                                      0.0792237451477643, 0.0791661274605434, 0.879142973793104);

  const float min_ev = -12.47393f;
  const float max_ev = 4.026069f;

  // Input transform (inset)
  val = agx_mat * val;

  // Log2 space encoding
  val = glm::clamp(glm::log2(val), min_ev, max_ev);
  val = (val - min_ev) / (max_ev - min_ev);

  // Apply sigmoid function approximation
  val = agxDefaultContrastApprox(val);

  return val;
}

static inline glm::vec3 agxEotf(glm::vec3 val) {
  const glm::mat3 agx_mat_inv
      = glm::mat3(1.19687900512017, -0.0528968517574562, -0.0529716355144438, -0.0980208811401368,
                  1.15190312990417, -0.0980434501171241, -0.0990297440797205, -0.0989611768448433,
                  1.15107367264116);

  // Inverse input transform (outset)
  val = agx_mat_inv * val;
  // sRGB IEC 61966-2-1 2.2 Exponent Reference EOTF Display
  // NOTE: We're linearizing the output here. Comment/adjust when
  // *not* using a sRGB render target
  // val = glm::pow(val, glm::vec3(2.2));

  if (val.r < 0.f) {
    val.r = 0.f;
  }
  if (val.g < 0.f) {
    val.g = 0.f;
  }
  if (val.b < 0.f) {
    val.b = 0.f;
  }

  val = glm::pow(val, glm::vec3(2.2f));

  return val;
}

// default agx look used
static inline glm::vec3 agxLook(glm::vec3 val) {
  // Default
  glm::vec3 offset = glm::vec3(0.0);
  glm::vec3 slope = glm::vec3(1.0);
  glm::vec3 power = glm::vec3(1.0);
  float sat = 1.0;

  // ASC CDL
  val = glm::pow(val * slope + offset, power);
  float luma = luminance(val);

  return luma + sat * (val - luma);
}

/*
 * minimal agx implementation source:
 * https://iolite-engine.com/blog_posts/minimal_agx_implementation
 * original agx source: https://github.com/sobotka/AgX
 */
void agx(std::vector<glm::vec3>& input_col) {
  for (glm::vec3& col : input_col) {
    col = agx(col);
    col = agxLook(col);
    col = agxEotf(col);
  }
}