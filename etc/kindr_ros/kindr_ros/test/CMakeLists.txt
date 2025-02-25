ament_add_gtest(${PROJECT_NAME}-test
  test_main.cpp
  RosGeometryMsgPhysicalQuantitiesTest.cpp
  RosGeometryMsgRotationTest.cpp
  RosGeometryMsgPoseTest.cpp
  RosTfPoseTest.cpp
  TfConventionTest.cpp
)


ament_target_dependencies(${PROJECT_NAME}-test
  ${dependencies}
)
