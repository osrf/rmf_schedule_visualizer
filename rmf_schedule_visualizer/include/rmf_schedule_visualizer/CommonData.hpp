/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RMF_SCHEDULE_VISUALIZER__COMMONDATA_HPP
#define RMF_SCHEDULE_VISUALIZER__COMMONDATA_HPP

#include <rmf_traffic/Time.hpp>
#include <string>
#include <memory>

namespace rmf_schedule_visualizer {

struct RequestParam
{
  std::string map_name;
  rmf_traffic::Time start_time;
  rmf_traffic::Time finish_time;
};

using RequestParamPtr = std::shared_ptr<RequestParam>;

} // namespace rmf_schedule_visualizer

#endif //RMF_SCHEDULE_VISUALIZER__COMMONDATA_HPP
