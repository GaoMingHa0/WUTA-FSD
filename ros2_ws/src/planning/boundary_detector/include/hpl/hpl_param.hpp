#pragma once
#include <string>

namespace hpl {

class ParamNode {
public:
  explicit ParamNode(const std::string& /*ns*/) {}

  template<typename T>
  void declParam(T& /*var*/, const std::string& /*name*/) {}

  template<typename T>
  void declParam(T& /*var*/, const std::string& /*name*/, const T& /*default_val*/) {}
};

}  // namespace hpl
