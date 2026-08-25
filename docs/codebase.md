# **osiris-firmware** 

The UCSD Rocket Propulsion Lab firmware codebase developed in collaboration by the Osiris and Software teams. It was created the summer of 2026.
At this time (Summer 2026), this repo exclusively consists of *esp-idf* projects and components designed for esp32 microcontrollers- primarily the esp32p4 and esp32s3.

The codebase is split into **projects/** and **components/**.

## Structure

***Projects*** are unique firmware builds intended for particular tasks including but not limited to:

> - flight-controller
> - bench testing
> - sensor integration testing
> - simulated flight logic testing
> - telemetry nodes (ground station)

***Components*** are custom libraries that modularize sensor code, re-usable logic, data management, etc. 
The purpose is to decluter projects, make them easier to interpret, decouple specific sensor/model interfaces from implementation, and make components re-usable across multiple projects.

***CMakeLists.txt*** files in component and project directories connect components into whatever project that needs them.

Refer to *projects/project-example/* and *components/rpl_component_example/* for their respective structures and CMakeLists.txt.

It is recommended to copy the examples as boilerplates to keep things consistent.

## Building Firmware

**esp-idf v6** is currently expected. Activate the esp-idf python environment, navigate to the project folder you intend to build, and:

Check for necessary configurations.
> idf.py menuconfig
Custom macros should live in a submenu titled *RPL*. There must be documentation in every project directory describing these settings as well as any other necessary configuration.

Set build target (p4/s3/c6). For example:
> idf.py set-target esp32p4

Build firmware.
> idf.py build
You may need to delete the project build/ between major changes.

Flash to esp32.
> idf.py flash
*Auto-scans port*

If you'd like to open the serial monitor:
> idf.py monitor

Commands can be combined:
> idf.py build flash monitor


*Hand written by Edgar VM, Osiris Avionics Lead - Aug 24, 2026*