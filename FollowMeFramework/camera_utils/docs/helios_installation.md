# LucidVision HELIOS installation instructions

## download arena sdk for linux and arena_api for python from:
https://thinklucid.com/downloads-hub/

## install Arena SDK for linux:
follow Readme instructions in the arena sdk directory. They should be similar to:
```
Installing Arena SDK for Linux:

1. Extract the tarball to your desired location:

    $ tar -xvzf ArenaSDK_Linux.tar.gz
    
    where ArenaSDK_Linux.tar.gz is the tarball name.

2. Run the Arena_SDK.conf file

    $ cd /path/to/ArenaSDK_Linux
    $ sudo sh Arena_SDK_Linux_x64.conf
    
    This will make the Arena SDK shared library files accessible by the run-time linker (ld.so or ld-linux.so).
```
## Arena Python API installation
follow instructions in readme, select the installers for your platform and python versions. They should be similar to:
```
### arena_api installation instructions
  '''pip install arena_api-<>.<>.<>-py3-none-any.whl'''

### Examples prerequisites
  - Make sure arena_api package is installed
  - Some examples require other pip packages which can be
    found in examples/requirements.txt. To install these packages run
      '''pip install -r examples/requirements_lin.txt''' for linux (non arm)
```

Be sure to select the right modules, they are platform speceific.

You can now use the LucidVision Helios wrapper.
