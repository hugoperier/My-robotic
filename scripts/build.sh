## Build python3 package and install it using pip3
## Package is identified as "modules"

python3 setup.py sdist bdist_wheel
sudo pip3 install -e .