/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <string>
#include <vector>
#include <iostream>

#include "Server.hpp"


int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " PORT_NUMBER" << std::endl;
    return 1;
  }

  std::string port_number_str = argv[1];
  const uint16_t port = std::stoul(port_number_str);

  auto server = rmf_visualizer::planning::Server::make();
  if (!server)
    return 1;
  
  server->run(port);
  return 0;
}
