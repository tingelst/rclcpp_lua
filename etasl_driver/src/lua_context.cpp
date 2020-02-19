#include "etasl_driver/lua_context.hpp"

namespace etasl_driver
{
LuaContext::LuaContext(std::shared_ptr<Context> ctx) : ctx_(ctx)
{
  lua_.open_libraries(sol::lib::base, sol::lib::package);

  // expressiongraph_lua::register_kdl(lua_);
  // expressiongraph_context_lua::register_context(lua_);

  lua_["time"] = ctx->getScalarExpr("time");
  lua_["ctx"] = ctx_;
}

int LuaContext::executeString(const std::string& cmds)
{
	auto result = lua_.safe_script(cmds, sol::script_pass_on_error);
	if (!result.valid()) {
		sol::error err = result;
		std::cerr << "The code has failed to run!\n" << err.what() << "\nPanicking and exiting..." << std::endl;
		return 1;
	}
  else {
    return 0;
  }
}

int LuaContext::executeFile(const std::string& filename)
{
  lua_.script_file(filename);
  return 0;
}

}  // namespace etasl_driver