// Copyright 2020 Gergely Kovács
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

#include <iostream>
#include <vector>
#include <string>

#include "CortexClient.hpp"

namespace ros2_cortex
{

template<typename T>
struct Callback;

template<typename Ret, typename ... Params>
struct Callback<Ret(Params...)>
{
  template<typename ... Args>
  static Ret callback(Args... args)
  {
    return func(args ...);
  }
  static std::function<Ret(Params...)> func;
};

template<typename Ret, typename ... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

typedef void (* data_callback_t)(sFrameOfData *);
typedef void (* error_msg__callback_t)(int i_level, char * sz_msg);

class SimpleFodPrinter : public CortexClient
{
public:
  explicit SimpleFodPrinter(const std::string & file_name)
  : CortexClient{file_name}
  {
    Callback<void(sFrameOfData *)>::func = std::bind(&SimpleFodPrinter::dataHandlerFunc_, this,
        std::placeholders::_1);
    data_callback_t data_func =
      static_cast<data_callback_t>(Callback<void(sFrameOfData *)>::callback);
    setDataHandlerFunc(data_func);

    Callback<void(int i_level, char * sz_msg)>::func = std::bind(
      &SimpleFodPrinter::errorMsgHandlerFunc_, this, std::placeholders::_1, std::placeholders::_2);
    error_msg__callback_t error_msg_func = static_cast<error_msg__callback_t>(
      Callback<void(int i_level, char * sz_msg)>::callback);
    setErrorMsgHandlerFunc(error_msg_func);
  }

  void dataHandlerFunc_(sFrameOfData * fod)
  {
    cortex_mock_.copyFrame(fod, &current_fod_);
    std::cout << "Frame " << current_fod_.iFrame << std::endl;
    std::cout << "Number of unidentified markers " << current_fod_.nUnidentifiedMarkers <<
      std::endl;
  }

  const std::vector<std::string> verb_levels = {"None", "Error", "Warning", "Info", "Debug"};

  void errorMsgHandlerFunc_(int i_level, char * error_msg)
  {
    std::cerr << verb_levels[i_level] << ": " << error_msg << std::endl;
  }
};

}  // namespace ros2_cortex

int main(int argc, char const * argv[])
{
  ros2_cortex::SimpleFodPrinter printer("CaptureWithPlots1.json");
  printer.run();

  return 0;
}
