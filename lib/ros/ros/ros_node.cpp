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

#include "ros_node.hpp"

bool RosNode::pingAgent() {
  return rmw_uros_ping_agent(cfg_.ping_timeout_ms, cfg_.ping_attempts) ==
         RMW_RET_OK;
}

bool RosNode::createEntities() {
  allocator_ = rcl_get_default_allocator();
  init_options_ = rcl_get_zero_initialized_init_options();

  RC_RETURN(rcl_init_options_init(&init_options_, allocator_));
  RC_RETURN(rcl_init_options_set_domain_id(&init_options_, cfg_.domain_id));
  rclc_support_init_with_options(&support_, 0, NULL, &init_options_,
                                 &allocator_);
  rclc_node_init_default(&node_, cfg_.node_name, ns_, &support_);

  // ── Register all publishers ──────────────────────────────
  for (uint8_t i = 0; i < cfg_.pub_count; ++i) {
    RC_RETURN(cfg_.publishers[i]->init(node_, allocator_));
  }

  // ── Register all subscriptions ──────────────────────────
  for (uint8_t i = 0; i < cfg_.sub_count; ++i) {
    auto& s = cfg_.subscriptions[i];
    if (s.best_effort) {
      rclc_subscription_init_best_effort(&s.sub, &node_, s.type_support,
                                         s.topic_name);
    } else {
      rclc_subscription_init_default(&s.sub, &node_, s.type_support,
                                     s.topic_name);
    }
  }

  // ── Register all services ────────────────────────────────
  for (uint8_t i = 0; i < cfg_.srv_count; ++i) {
    auto& s = cfg_.services[i];
    rclc_service_init_default(&s.srv, &node_, s.type_support, s.topic_name);
  }

  // ── Count executor handles ───────────────────────────────
  uint8_t exec_count = cfg_.sub_count + cfg_.srv_count;

  // ── Create executor ──────────────────────────────────────
  executor_ = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor_, &support_.context, exec_count, &allocator_);

  for (uint8_t i = 0; i < cfg_.sub_count; ++i) {
    auto& s = cfg_.subscriptions[i];
    rclc_executor_add_subscription(&executor_, &s.sub, s.msg, s.callback,
                                   ON_NEW_DATA);
  }
  for (uint8_t i = 0; i < cfg_.srv_count; ++i) {
    auto& s = cfg_.services[i];
    rclc_executor_add_service(&executor_, &s.srv, s.request, s.response,
                              s.callback);
  }

  rmw_uros_sync_session(1000);
  return true;
}

void RosNode::destroyEntities() {
  auto* ctx = rcl_context_get_rmw_context(&support_.context);
  rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);

  for (uint8_t i = 0; i < cfg_.pub_count; ++i) cfg_.publishers[i]->fini(node_);
  for (uint8_t i = 0; i < cfg_.sub_count; ++i)
    RC_SKIP(rcl_subscription_fini(&cfg_.subscriptions[i].sub, &node_));
  for (uint8_t i = 0; i < cfg_.srv_count; ++i)
    RC_SKIP(rcl_service_fini(&cfg_.services[i].srv, &node_));

  rclc_executor_fini(&executor_);
  RC_SKIP(rcl_node_fini(&node_));
  rclc_support_fini(&support_);
  RC_SKIP(rcl_init_options_fini(&init_options_));
}

void RosNode::loop() {
  switch (state_) {
    case WAITING:
      if (pingAgent()) state_ = AGENT_AVAILABLE;
      break;
    case AGENT_AVAILABLE:
      if (createEntities())
        state_ = CONNECTED;
      else {
        destroyEntities();
        state_ = WAITING;
      }
      break;
    case CONNECTED:
      if (!pingAgent()) state_ = DISCONNECTED;
      break;
    case DISCONNECTED:
      destroyEntities();
      state_ = WAITING;
      break;
  }
}

void RosNode::publishLoop() {
  if (state_ != CONNECTED) return;
  for (uint8_t i = 0; i < cfg_.pub_count; ++i) cfg_.publishers[i]->publish();
  rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(0));
}

void RosNode::transportInit(const SerialConfig& config) {
  rmw_uros_set_custom_transport(
      /* Enable XRCE framing */
      true,
      /* Arguments for callbacks - pass config pointer */
      (void*)&config,

      /* Open transport callback */
      [](struct uxrCustomTransport* transport) -> bool {
        const SerialConfig* cfg = (const SerialConfig*)transport->args;
        cfg->serial->setRx(cfg->rxPin);
        cfg->serial->setTx(cfg->txPin);
        cfg->serial->setTimeout(cfg->timeout_ms);
        cfg->serial->begin(cfg->baudrate);
        return cfg->serial->operator bool();
      },

      /* Close transport callback */
      [](struct uxrCustomTransport* transport) -> bool {
        const SerialConfig* cfg = (const SerialConfig*)transport->args;
        cfg->serial->end();
        return true;
      },

      /* Write transport callback */
      [](struct uxrCustomTransport* transport, const uint8_t* buf, size_t len,
         uint8_t* errcode) -> unsigned int {
        const SerialConfig* cfg = (const SerialConfig*)transport->args;
        return cfg->serial->write(buf, len);
      },

      /* Read transport callback */
      [](struct uxrCustomTransport* transport, uint8_t* buf, size_t len,
         int timeout, uint8_t* errcode) -> unsigned int {
        const SerialConfig* cfg = (const SerialConfig*)transport->args;
        cfg->serial->setTimeout(timeout);
        return cfg->serial->readBytes((char*)buf, len);
      });
}
