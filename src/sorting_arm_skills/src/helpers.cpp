#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

sorting_arm_interfaces::msg::SkillResult to_msg(const SkillResult& result) {
  sorting_arm_interfaces::msg::SkillResult msg;
  msg.ok = result.ok;
  msg.native_code = result.native_code;
  msg.phase = result.phase;
  msg.message = result.message;
  return msg;
}

}  // namespace sorting_arm
