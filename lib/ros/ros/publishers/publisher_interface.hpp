// Copyright 2022 Husarion sp. z o.o.
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

#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>

class PublisherInterface {
 public:
  explicit PublisherInterface(const char* topic) : topic_(topic) {}

  /// Initialize publisher on the given node.
  virtual rcl_ret_t init(rcl_node_t& node, rcl_allocator_t& allocator) = 0;

  /// Publish one message. Called every publishLoop() cycle.
  virtual void publish() = 0;

  /// Cleanup. Called during destroyEntities().
  virtual void fini(rcl_node_t& node) = 0;

  /// Topic name for logging / debug.
  virtual const char* topicName() const { return topic_; }

 protected:
  const char* topic_;
};
