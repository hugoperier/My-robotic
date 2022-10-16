
## PermissionError: [Errno 13] Permission denied: '/dev/i2c-1'
## Permission error that occurs while using adafruit package to control osoyoo motor
sudo chmod a+rw /dev/i2c-*

## Environment variable checks

# check for MYROBOTICS_ROOT which describe the root directory of the project's configuration path
if [ -z "$MYROBOTICS_ROOT" ]; then
    echo "MYROBOTICS_ROOT is not set"
    exit 1
fi