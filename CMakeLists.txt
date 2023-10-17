# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.15)

project(libhal-rmd LANGUAGES CXX)

libhal_test_and_make_library(
  LIBRARY_NAME libhal-rmd

  SOURCES
  src/drc.cpp
  src/drc_adaptors.cpp
  src/mc_x.cpp
  src/mc_x_adaptors.cpp

  TEST_SOURCES
  tests/drc.test.cpp
  tests/mc_x.test.cpp
  tests/drc_motor.test.cpp
  tests/main.test.cpp

  PACKAGES
  libhal
  libhal-util
  libhal-canrouter

  LINK_LIBRARIES
  libhal::libhal
  libhal::util
  libhal::canrouter

  TEST_PACKAGES
  libhal-mock

  TEST_LINK_LIBRARIES
  libhal::mock
)
