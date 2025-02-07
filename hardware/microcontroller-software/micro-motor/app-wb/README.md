# Installation of toolchain
follow the [instructions for modm installation, including the ARM Cortex-M toolchain](https://modm.io/guide/installation/)

# Compile the firmware
You can compile the firmware with
```
scons -j8
```

And you can program the motorcontroller with:
```
scons program
```

# Changes from modm
MODM generated files are committed directly, if you want to update the generated files, call `lbuild build` and you should see the required files.

