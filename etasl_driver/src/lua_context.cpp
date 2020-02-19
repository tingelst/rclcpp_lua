#include "etasl_driver/lua_context.hpp"

namespace etasl_driver
{
LuaContext::LuaContext(std::shared_ptr<Context> ctx) : ctx_(ctx)
{
  lua_.open_libraries(sol::lib::base, sol::lib::package);
  lua_["time"] = ctx->getScalarExpr("time");
  lua_["ctx"] = ctx_;
}

int LuaContext::executeString(const std::string& cmds)
{
  return 0;
}

int LuaContext::executeFile(const std::string& filename)
{
  return 0;
}

}  // namespace etasl_driver