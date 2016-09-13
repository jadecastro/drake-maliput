// Copyright 2016 Toyota Research Institute.  All rights reserved.

#pragma once

#include <memory>

namespace maliput {
namespace monolane {

// Workaround for not having C++14 yet.
template <class T, class... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace monolane
}  // namespace maliput
