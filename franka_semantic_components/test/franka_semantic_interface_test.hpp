// Copyright (c) 2023 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Original authors: Subhas Das, Denis Stogl
 */

// modified ros2 control semantic control interface to add command interface access.
// https://github.com/ros-controls/ros2_control/blob/humble/controller_interface/include/semantic_components/semantic_component_interface.hpp

#pragma once

#include <memory>
#include <string>

#include "franka_semantic_components/franka_semantic_component.hpp"
#include "gmock/gmock.h"

// implementing and friending so we can access member variables
class TestableSemanticComponentInterface
    : public franka_semantic_components::FrankaSemanticComponentInterface {
  FRIEND_TEST(SemanticComponentInterfaceTest, validate_default_names);
  FRIEND_TEST(SemanticComponentInterfaceTest, validate_command_interface_default_names);
  FRIEND_TEST(SemanticComponentInterfaceTest, validate_custom_state_interface_names);
  FRIEND_TEST(SemanticComponentInterfaceTest, validate_custom_command_interface_names);
  FRIEND_TEST(SemanticComponentInterfaceTest, validate_state_interfaces);
  FRIEND_TEST(SemanticComponentInterfaceTest, validate_command_interfaces);

 public:
  // Use generation of interface names
  explicit TestableSemanticComponentInterface(const std::string& name,
                                              size_t state_interface_size,
                                              size_t command_interface_size)
      : franka_semantic_components::FrankaSemanticComponentInterface(name,
                                                                     state_interface_size,
                                                                     command_interface_size) {}
  // Use custom interface names
  explicit TestableSemanticComponentInterface(size_t state_interface_size,
                                              size_t command_interface_size)
      : franka_semantic_components::FrankaSemanticComponentInterface("TestFrankaSemanticComponent",
                                                                     state_interface_size,
                                                                     command_interface_size) {
    // generate the interface_names_
    for (auto i = 0u; i < state_interface_size; ++i) {
      state_interface_names_.emplace_back(std::string("TestFrankaSemanticComponent") + "/i" +
                                          std::to_string(i + 5));
    }

    for (auto i = 0u; i < command_interface_size; ++i) {
      command_interface_names_.emplace_back(std::string("TestFrankaSemanticComponentCommand") +
                                            "/i" + std::to_string(i + 5));
    }
  }

  virtual ~TestableSemanticComponentInterface() = default;

  std::string test_name_ = "TestFrankaSemanticComponent";
};

class SemanticComponentInterfaceTest : public ::testing::Test {
 public:
  void TearDown();

 protected:
  const std::string component_name_ = "test_component";
  const size_t size_ = 5;
  std::unique_ptr<TestableSemanticComponentInterface> semantic_component_;
};
