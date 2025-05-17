pushd lib/crs_common
source script/build.sh      # this runs in the lib/crs_common workspace
popd
colcon build --packages-select pwm_cltool
source install/setup.bash      # or setup.zsh