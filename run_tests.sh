cd /sim_ws
colcon build
colcon test --ctest-args tests
colcon test-result --all --verbose