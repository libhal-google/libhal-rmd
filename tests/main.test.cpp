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

namespace hal::rmd {
extern void drc_test();
extern void drc_adaptors_test();
extern void mc_x_test();
}  // namespace hal::rmd

int main()
{
  hal::rmd::drc_test();
  hal::rmd::drc_adaptors_test();
  hal::rmd::mc_x_test();
}
