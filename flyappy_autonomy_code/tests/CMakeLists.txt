find_package(GTest REQUIRED)

include(GoogleTest)

add_executable(test_my_feature test_my_feature.cpp)
target_link_libraries(test_my_feature
  flyappy
  gtest
  gtest_main
)
gtest_discover_tests(test_my_feature)
