#include "lib.hpp"

auto main() -> int
{
  library lib;

  return lib.name == "sandbox" ? 0 : 1;
}
