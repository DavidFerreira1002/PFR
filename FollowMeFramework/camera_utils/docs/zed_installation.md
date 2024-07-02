
## installation of StereoLabs ZED SDK

- if you want to use the ZED camera you need to:
   - download the sdk from: https://www.stereolabs.com/developers/release/
   - run the downloaded file and follow the instruction:
   ```
   chmod +x NAME_OF_FILE.run
   ./NAME_OF_FILE.run
   ```
- Once that the installation is finished you should have the /usr/local/zed directory

- if the python packages is not automatically unstalled you should:
  - activate your virtual environment (ore install it on your system) and run:
  - open the directory where the zed sdk is installed and then run the python script get_python_api.py from the virtual environment:
  ```
  cd /usr/local/zed && sudo python3 get_python_api.py
  ```
  - after this a new .whl will appear in the /usr/local/zed directory (e.g., pyzed-3.6-SOMETHING-ELSE.whl). run the following command
  ```
  pip install --ignore-installed /usr/local/zed/WHL_FILE_NAME.whl
  ```
# Issues

1) ZED command ``` sudo python3 get_python_api.py ``` gives the following error:
    ```
    -> Downloading to '/usr/local/zed'
    Error: you must install Cuda.
    ```
    SOLUTION:  you need to install Cuda toolkit
   - install Nvidia drivers by typing in the terminal:
    ```
    ubuntu-drivers autoinstall
    ```
    - download the .run cuda installer from https://developer.nvidia.com/cuda-downloads (for last version) or https://developer.nvidia.com/cuda-toolkit-archive (for old versions) selecting your current platform (linux - ubuntu, ecc):
    - make the .run file executable and run it:
    ```
    chmod +x CUDA_FILE.run
    sudo ./CUDA_FILE.run --toolkit --silent --override
    ```
    - Once that you have installed cuda repeat the Python packages installation step

2) ZED command ``` sudo python3 get_python_api.py ``` gives the following error:
    ```
    ERROR: Could not install packages due to an OSError: [Errno 13] Permission denied: '/home/USER/PROJECT_PATH/venv/lib/python3.7/site-packages/pyzed'
    Check the permissions.
    ```
    you should give the permission access to your user by typing:
    ```
    sudo chown -R user:user YOUR_VENV_ABSOLUTE_PATH
    ```
    where user = YOUR_USER_NAME e.g., user:user = federico:federico
  
    after this you can repeat the Python packages installation step

3) in latest version, you may encounter the following error when installing the ZED SDK (i.e. while running ```./NAME_OF_FILE.run```):
    ```
    Verifying archive integrity...  100%   MD5 checksums are OK. All good. Uncompressing 'ZED camera SDK by Stereolabs (Use 'sudo apt install zstd' if zstd is not found)'./ZED_SDK_Ubuntu20_cuda11.7_v3.8.0.zstd.run: 1: eval: zstd: not found
    ... Decompression failed
    .... Extraction failed.
    100%  Signal caught, cleaning up
    ```
    following the instructions, you should install zstd:
    ```
    sudo apt install zstd
    ```
    and then run again ./NAME_OF_FILE.run
