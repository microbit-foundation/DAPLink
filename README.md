# v0234

This branch has been updated to ensure we can easily build v0234 in the future.
It has also been git tagged to `v0234-mb-patched`:
https://github.com/microbit-foundation/DAPLink/releases/tag/v0234-mb-patched

## Modifications

You can see the differences using GH:
https://github.com/microbit-foundation/DAPLink/compare/microbit-0234...v0234-mb-patched

List of changes:
- This README has been updated with additional information
- A future commit has been applied manually to fix an issue building with
  a newer version of uVision
    * More info in the commit message
    * https://github.com/ARMmbed/DAPLink/pull/49
    * https://github.com/ARMmbed/DAPLink/commit/28a6acc01e8904dce2247431104c2b64eddb1e9b
    * Applied in e8be53c3f106aea5fd11c3d096afcda4ec8a5672
    * Patch file `28a6acc01e.patch` from original commit was also added
- Added a `requirements_v0234_frozen.txt` file with pinned Python dependencies
  to get project-generator working correctly
    * Beacuse the Python dependencies were not pinned and the nested
      dependencies also had unpinned versions, it took a bit of effort to
      figure out the right combination
- In case there are still issues with the pinned Python dependencies the
  uVision project files have been added as well
    * File were added as they were created by project-generator, before being
      opened with uVision
    * These files will be overwritten when project-generator is used


## Set up build environment

With Python 2.7:

```
git clone https://github.com/microbit-foundation/DAPLink.git
cd DAPLink
git checkout -b v0234 origin/v0234
pip install virtualenv
virtualenv venv
cd venv/Scripts/ && activate.bat & cd ../..
pip install pip==9.0.3
pip install -r requirements_v0234_frozen.txt
```

Note: The `pip install pip==9.0.3` command might give an error on Windows when
it tries to remove the pip.exe file (which is locked by the terminal command
you are executing). That should be fine, just check you are running v9 with
`pip --version`.


To generate the project files:

```
pgen export -t uvision -p microbit_if
pgen export -t uvision -p kl26z_bl
```

In this version of DAPLink the uVision project doesn't execute the Python
tools, so there is no need to launch uVision from within the virtual
environment.


## uVision version used

This branch was built using:
```
IDE-Version:
µVision V5.26.2.0
Copyright (C) 2018 ARM Ltd and ARM Germany GmbH. All rights reserved.

Tool Version Numbers:
Toolchain:         MDK-ARM Professional     Version: 5.26.2.0
Toolchain Path:    C:\Keil_v5\ARM\ARMCC\Bin
C Compiler:        Armcc.exe                V5.06 update 6 (build 750)
Assembler:         Armasm.exe               V5.06 update 6 (build 750)
Linker/Locator:    ArmLink.exe              V5.06 update 6 (build 750)
Library Manager:   ArmAr.exe                V5.06 update 6 (build 750)
Hex Converter:     FromElf.exe              V5.06 update 6 (build 750)
CPU DLL:           SARMCM3.DLL              V5.26.2.0
Dialog DLL:        DARMCM1.DLL              V1.19.1.0
Target DLL:        CMSIS_AGDI.dll           V1.30.6.0
Dialog DLL:        TARMCM1.DLL              V1.14.0.0
```


## Original README

Below this line is the original README.md:

-----

# DAPLink

## Setup
Skip any step where a compatible tool already exists

1. Install [Python 2.7.x](https://www.python.org/downloads/) and make sure it's added to path
2. Install [pip](https://pip.pypa.io/en/latest/installing.html) and make sure <python_install_loc>\Scripts it's added to path
3. Install DAPLink dependencies using pip (see command below)
4. Install [Keil MDK-ARM](https://www.keil.com/download/product/)

```
daplink> pip install -r requirements.txt
```

## Develop
1. Generate project files. This should be done everytime you pull new changes
```
> pgen export
```
For adding new targets start from template and use these docs...

## Contribute
Check out the issue tracker.

##ToDo
- Create a test
- Document how to use
- Document how to contribute
