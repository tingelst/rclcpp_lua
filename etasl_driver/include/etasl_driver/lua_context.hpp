#ifndef ETASL_DRIVER__LUA_CONTEXT_HPP_
#define ETASL_DRIVER__LUA_CONTEXT_HPP_

#define SOL_ALL_SAFETIES_ON 1
#include <sol/sol.hpp>

#include "expressiongraph/context.hpp"

#include "etasl_driver/visibility_control.h"

namespace etasl_driver
{
using namespace KDL;
class LuaContext
{
public:
  LuaContext(std::shared_ptr<Context> ctx);

  int executeString(const std::string& cmds);
  int executeFile(const std::string& filename);

private:
  std::shared_ptr<Context> ctx_;
  sol::state lua_;
};

}  // namespace etasl_driver

#endif  // ETASL_DRIVER__LUA_CONTEXT_HPP_
