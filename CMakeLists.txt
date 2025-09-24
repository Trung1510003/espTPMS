# The following lines of boilerplate have to be in your project's
# CMakeLists.txt file. You can add more components to the build by
# referencing them with the EXTRA_COMPONENTS_DIRS variable.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(testBLE_Sensor)

# Add component directories
set(EXTRA_COMPONENT_DIRS 
    main
    components/bt_hci
)

# Set component requirements
set(COMPONENT_REQUIRES 
    bt
    nvs_flash
    driver
    esp_timer
    freertos
)

# Set component private requirements
set(COMPONENT_PRIV_REQUIRES 
    bt
    nvs_flash
)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED YES)

# Add source files and include director