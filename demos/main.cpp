// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sys/stat.h>
#include <unistd.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "hardware_map.hpp"

int main()
{
  auto init_result = initialize_target();

  if (!init_result) {
    hal::halt();
  }

  auto hardware_map = init_result.value();
  auto is_finished = application(hardware_map);

  if (!is_finished) {
    hardware_map.reset();
  } else {
    hal::halt();
  }

  return 0;
}

namespace boost {
void throw_exception(std::exception const& e)
{
  std::abort();
}
}  // namespace boost

extern "C"
{
  /// Dummy implementation of getpid
  int _getpid_r()
  {
    return 1;
  }

  /// Dummy implementation of kill
  int _kill_r(int, int)
  {
    return -1;
  }

  /// Dummy implementation of fstat, makes the assumption that the "device"
  /// representing, in this case STDIN, STDOUT, and STDERR as character devices.
  int _fstat_r([[maybe_unused]] int file, struct stat* status)
  {
    status->st_mode = S_IFCHR;
    return 0;
  }

  int _write_r([[maybe_unused]] int file,
               [[maybe_unused]] const char* ptr,
               int length)
  {
    return length;
  }

  int _read_r([[maybe_unused]] FILE* file,
              [[maybe_unused]] char* ptr,
              int length)
  {
    return length;
  }

  // Dummy implementation of _lseek
  int _lseek_r([[maybe_unused]] int file,
               [[maybe_unused]] int ptr,
               [[maybe_unused]] int dir)
  {
    return 0;
  }

  // Dummy implementation of close
  int _close_r([[maybe_unused]] int file)
  {
    return -1;
  }

  // Dummy implementation of isatty
  int _isatty_r([[maybe_unused]] int file)
  {
    return 1;
  }
}
