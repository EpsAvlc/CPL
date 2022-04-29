cmake_minimum_required(VERSION 3.14.0)
project(convex_polygon_library VERSION 0.1.0)

set(UNIT_TEST True)

set(CMAKE_CXX_STANDARD 11)
if(UNIT_TEST)
  enable_testing()
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
  )

  include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include/
  )

  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  
  add_executable(cpl_test test/test.cc)

  target_link_libraries(
    cpl_test 
    gtest_main
  )

  include(GoogleTest)
  gtest_discover_tests(cpl_test)
endif(UNIT_TEST)


