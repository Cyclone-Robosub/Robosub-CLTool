source ./script/build.sh
colcon test --packages-select pwm_cltool --pytest-with-coverage --pytest-args --cov-report=term --event-handlers console_direct+