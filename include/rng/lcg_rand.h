#include <cstdint>
#include <limits>

namespace lcg {
  // linear congruential generator
  // https://www.ams.org/journals/mcom/1999-68-225/S0025-5718-99-00996-5/S0025-5718-99-00996-5.pdf
  int32_t next_hash(int32_t s) { return s * 594156893; }

  // multi-linear congruential seed function (give space/time coordinates)
  // use to get first hash
  int32_t hash(int32_t k, int32_t x, int32_t y) {
    int32_t s = next_hash(k * 37769685);
    s ^= next_hash(x * 26757677);
    s ^= next_hash(y * 20501397);
    return next_hash(s);
  }

  // input hash is moved to the next hash
  int32_t rande(int32_t& s) {
    s = next_hash(s);
    return s >> 1;
  }

  // input hash is moved to the next hash
  // input hash and get uniform [0,1]
  float unitrand(int32_t& s) {
    return static_cast<float>(rande(s) + (std::numeric_limits<int32_t>::max() >> 1))
           / std::numeric_limits<int32_t>::max();
  }

  // input hash is moved to the next hash
  float boxrand(int32_t& s) {
    return static_cast<float>(rande(s)) / std::numeric_limits<int32_t>::max();
  }
}