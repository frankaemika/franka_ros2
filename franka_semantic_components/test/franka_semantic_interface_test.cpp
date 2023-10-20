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

// modified ros2 control semantic control interface to add command interface access.
// https://github.com/ros-controls/ros2_control/blob/humble/controller_interface/include/semantic_components/semantic_component_interface.hpp

#include "franka_semantic_interface_test.hpp"

#include <memory>
#include <string>
#include <vector>

void SemanticComponentInterfaceTest::TearDown() {
  semantic_component_.reset(nullptr);
}

TEST_F(SemanticComponentInterfaceTest, validate_default_names) {
  // create 'test_component' with 5 interfaces using default naming
  // e.g. test_component_1, test_component_2 so on...
  semantic_component_ =
      std::make_unique<TestableSemanticComponentInterface>(component_name_, size_, 0);

  // validate the component name
  ASSERT_EQ(semantic_component_->name_, component_name_);

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(semantic_component_->state_interface_names_.capacity(), size_);
  ASSERT_EQ(semantic_component_->state_interfaces_.capacity(), size_);

  // validate the interface_names_
  std::vector<std::string> interface_names = semantic_component_->get_state_interface_names();
  ASSERT_EQ(interface_names, semantic_component_->state_interface_names_);

  ASSERT_EQ(interface_names.size(), size_);
  ASSERT_EQ(interface_names[0], component_name_ + "/1");
  ASSERT_EQ(interface_names[1], component_name_ + "/2");
  ASSERT_EQ(interface_names[2], component_name_ + "/3");
  ASSERT_EQ(interface_names[3], component_name_ + "/4");
  ASSERT_EQ(interface_names[4], component_name_ + "/5");
}

TEST_F(SemanticComponentInterfaceTest, validate_command_interface_default_names) {
  // create 'test_component' with 5 interfaces using default naming
  // e.g. test_component_1, test_component_2 so on...
  semantic_component_ =
      std::make_unique<TestableSemanticComponentInterface>(component_name_, 0, size_);

  // validate the component name
  ASSERT_EQ(semantic_component_->name_, component_name_);

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(semantic_component_->command_interface_names_.capacity(), size_);
  ASSERT_EQ(semantic_component_->command_interfaces_.capacity(), size_);

  // validate the interface_names_
  std::vector<std::string> command_interface_names =
      semantic_component_->get_command_interface_names();
  ASSERT_EQ(command_interface_names, semantic_component_->command_interface_names_);

  ASSERT_EQ(command_interface_names.size(), size_);
  ASSERT_EQ(command_interface_names[0], component_name_ + "/1");
  ASSERT_EQ(command_interface_names[1], component_name_ + "/2");
  ASSERT_EQ(command_interface_names[2], component_name_ + "/3");
  ASSERT_EQ(command_interface_names[3], component_name_ + "/4");
  ASSERT_EQ(command_interface_names[4], component_name_ + "/5");
}

TEST_F(SemanticComponentInterfaceTest, validate_custom_state_interface_names) {
  // create a component with 5 interfaces using custom naming
  // as defined in the constructor
  semantic_component_ = std::make_unique<TestableSemanticComponentInterface>(size_, 0);

  // validate the component name
  ASSERT_EQ(semantic_component_->name_, semantic_component_->test_name_);

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(semantic_component_->state_interface_names_.capacity(), size_);
  ASSERT_EQ(semantic_component_->state_interfaces_.capacity(), size_);

  // validate the interface_names_
  std::vector<std::string> interface_names = semantic_component_->get_state_interface_names();
  ASSERT_TRUE(std::equal(semantic_component_->state_interface_names_.begin(),
                         semantic_component_->state_interface_names_.end(), interface_names.begin(),
                         interface_names.end()));

  ASSERT_EQ(interface_names.size(), size_);
  ASSERT_EQ(interface_names[0], semantic_component_->test_name_ + "/i5");
  ASSERT_EQ(interface_names[1], semantic_component_->test_name_ + "/i6");
  ASSERT_EQ(interface_names[2], semantic_component_->test_name_ + "/i7");
  ASSERT_EQ(interface_names[3], semantic_component_->test_name_ + "/i8");
  ASSERT_EQ(interface_names[4], semantic_component_->test_name_ + "/i9");
}

TEST_F(SemanticComponentInterfaceTest, validate_custom_command_interface_names) {
  // create a component with 5 interfaces using custom naming
  // as defined in the constructor
  semantic_component_ = std::make_unique<TestableSemanticComponentInterface>(0, size_);

  // validate the component name
  ASSERT_EQ(semantic_component_->name_, semantic_component_->test_name_);

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(semantic_component_->command_interface_names_.capacity(), size_);
  ASSERT_EQ(semantic_component_->command_interfaces_.capacity(), size_);

  // validate the interface_names_
  std::vector<std::string> command_interface_names =
      semantic_component_->get_command_interface_names();
  ASSERT_TRUE(std::equal(semantic_component_->command_interface_names_.begin(),
                         semantic_component_->command_interface_names_.end(),
                         command_interface_names.begin(), command_interface_names.end()));

  ASSERT_EQ(command_interface_names.size(), size_);
  ASSERT_EQ(command_interface_names[0], semantic_component_->test_name_ + "Command" + "/i5");
  ASSERT_EQ(command_interface_names[1], semantic_component_->test_name_ + "Command" + "/i6");
  ASSERT_EQ(command_interface_names[2], semantic_component_->test_name_ + "Command" + "/i7");
  ASSERT_EQ(command_interface_names[3], semantic_component_->test_name_ + "Command" + "/i8");
  ASSERT_EQ(command_interface_names[4], semantic_component_->test_name_ + "Command" + "/i9");
}

TEST_F(SemanticComponentInterfaceTest, validate_state_interfaces) {
  // create 'test_component' with 5 interfaces using default naming
  // e.g. test_component_1, test_component_2 so on...
  semantic_component_ =
      std::make_unique<TestableSemanticComponentInterface>(component_name_, size_, 0);

  // generate the interface_names_
  std::vector<std::string> interface_names = semantic_component_->get_state_interface_names();

  // validate assign_loaned_state_interfaces
  // create interfaces and assign values to it
  std::vector<double> interface_values = {1.1, 3.3, 5.5};

  // assign 1.1 to interface_1, 3.3 to interface_3 and 5.5 to interface_5
  hardware_interface::StateInterface interface_1{component_name_, "1", &interface_values[0]};
  hardware_interface::StateInterface interface_3{component_name_, "3", &interface_values[1]};
  hardware_interface::StateInterface interface_5{component_name_, "5", &interface_values[2]};

  // create local state interface vector
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;

  // insert the interfaces in jumbled sequence
  temp_state_interfaces.emplace_back(interface_5);
  temp_state_interfaces.emplace_back(interface_1);
  temp_state_interfaces.emplace_back(interface_3);

  // now call the function to make them in order like interface_names
  semantic_component_->assign_loaned_state_interfaces(temp_state_interfaces);

  // validate the count of state_interfaces_
  ASSERT_EQ(semantic_component_->state_interfaces_.size(), 3u);

  // validate the values of state_interfaces_ which should be
  // in order as per interface_names_
  std::vector<double> temp_values;
  ASSERT_FALSE(semantic_component_->get_values(temp_values));

  // reserve memory for the vector and call get_values
  temp_values.reserve(3);
  ASSERT_TRUE(semantic_component_->get_values(temp_values));
  ASSERT_EQ(temp_values, interface_values);

  // release the state_interfaces_
  semantic_component_->release_interfaces();

  // validate the count of state_interfaces_
  ASSERT_EQ(semantic_component_->state_interfaces_.size(), 0u);

  // validate that release_interfaces() does not touch interface_names_
  ASSERT_TRUE(std::equal(semantic_component_->state_interface_names_.begin(),
                         semantic_component_->state_interface_names_.end(), interface_names.begin(),
                         interface_names.end()));
}

TEST_F(SemanticComponentInterfaceTest, validate_command_interfaces) {
  // create 'test_component' with 5 interfaces using default naming
  // e.g. test_component_1, test_component_2 so on...
  semantic_component_ =
      std::make_unique<TestableSemanticComponentInterface>(component_name_, size_, size_);

  // generate the interface_names_
  std::vector<std::string> command_interface_names =
      semantic_component_->get_command_interface_names();

  // validate assign_loaned_state_interfaces
  // create interfaces and assign values to it
  std::vector<double> interface_values = {1.1, 3.3, 5.5};

  // assign 1.1 to interface_1, 3.3 to interface_3 and 5.5 to interface_5
  hardware_interface::CommandInterface interface_1{component_name_, "1", &interface_values[0]};
  hardware_interface::CommandInterface interface_3{component_name_, "3", &interface_values[1]};
  hardware_interface::CommandInterface interface_5{component_name_, "5", &interface_values[2]};

  // create local state interface vector
  std::vector<hardware_interface::LoanedCommandInterface> temp_command_interfaces;

  // insert the interfaces in jumbled sequence
  temp_command_interfaces.emplace_back(interface_5);
  temp_command_interfaces.emplace_back(interface_1);
  temp_command_interfaces.emplace_back(interface_3);

  // now call the function to make them in order like interface_names
  semantic_component_->assign_loaned_command_interfaces(temp_command_interfaces);

  // validate the count of command_interfaces_
  ASSERT_EQ(semantic_component_->command_interfaces_.size(), 3u);

  // validate the values of command_interfaces_ which should be
  // in order as per interface_names_
  std::vector<double> temp_values;
  ASSERT_FALSE(semantic_component_->set_values(temp_values));

  temp_values = {0.0, 1.0, 2.0};
  ASSERT_TRUE(semantic_component_->set_values(temp_values));

  ASSERT_EQ(temp_values, interface_values);

  // release the command_interfaces_
  semantic_component_->release_interfaces();

  // validate the count of command_interfaces_
  ASSERT_EQ(semantic_component_->command_interfaces_.size(), 0u);

  // validate that release_interfaces() does not touch interface_names_
  ASSERT_TRUE(std::equal(semantic_component_->command_interface_names_.begin(),
                         semantic_component_->command_interface_names_.end(),
                         command_interface_names.begin(), command_interface_names.end()));
}