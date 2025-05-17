## Set up
You must be on Ubuntu or some compatible Linux distro.
Dependencies:
ROS2
python3
git
cmake
make/ninja

When you first clone the repo, you should run:
```
. script/setup.sh
```

## build
Run at the project root:
```
. script/build.sh
```
## Test
You must have colcon mixins installed.
Run the following:
```
. script/test.sh
```
Or :
```
. script/test_detailed.sh
```
The test logs will be at the `log` directory and `build/pwm_cltool/coverage.html` will contain the coverage report. 
Currently the test corresponds to the old msg formats so they will fail but they still runs.