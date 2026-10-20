#include "r3v_ros/r3v_handles.hpp"

#include <stdexcept>

namespace r3v_ros
{

std::string lastRendererError()
{
  const char * message = rr::r3v_renderer_last_error();
  return message != nullptr ? message : "";
}

void checkStatus(rr::R3vStatus status, const std::string & what)
{
  if (status != rr::R3vStatus::R3V_OK) {
    throw std::runtime_error(
            what + " failed (status " + std::to_string(static_cast<int>(status)) + "): " +
            lastRendererError());
  }
}

}  // namespace r3v_ros
