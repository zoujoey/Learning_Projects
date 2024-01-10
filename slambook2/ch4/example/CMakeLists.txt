# Set the option for Ubuntu 20.04
option(USE_UBUNTU_20 "Set to ON if you are using Ubuntu 20.04" ON)

# Find the Pangolin package
find_package(Pangolin REQUIRED)

# Check if the USE_UBUNTU_20 option is enabled
if(USE_UBUNTU_20)
    message("You are using Ubuntu 20.04, fmt::fmt will be linked")
    # Find the fmt package
    find_package(fmt REQUIRED)
    # Set FMT_LIBRARIES to fmt::fmt
    set(FMT_LIBRARIES fmt::fmt)
endif()

# Include directories for Pangolin
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

# Create the executable trajectoryError from trajectoryError.cpp
add_executable(trajectoryError trajectoryError.cpp)

# Link the executable against Pangolin and fmt::fmt if USE_UBUNTU_20 is ON
# Link the executable against Pangolin, fmt::fmt (if USE_UBUNTU_20 is ON), and Sophus::Sophus
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES} ${FMT_LIBRARIES} Sophus::Sophus)