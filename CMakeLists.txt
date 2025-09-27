cmake_minimum_required(VERSION 3.16)
project(ZenoTrack C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(zenotrack
    src/main.c
    src/second.c
)
target_include_directories(zenotrack PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Prefer pkg-config (Linux/macOS)
find_package(PkgConfig)
if (PkgConfig_FOUND)
    pkg_check_modules(SDL2 IMPORTED_TARGET sdl2)
    pkg_check_modules(FFMPEG IMPORTED_TARGET libavformat libavcodec libswscale libavutil)
endif()

if (TARGET PkgConfig::sdl2 AND
    TARGET PkgConfig::libavformat AND TARGET PkgConfig::libavcodec AND
    TARGET PkgConfig::libswscale AND TARGET PkgConfig::libavutil)
    message(STATUS "Using pkg-config packages")
    target_link_libraries(zenotrack PRIVATE
        PkgConfig::sdl2
        PkgConfig::libavformat
        PkgConfig::libavcodec
        PkgConfig::libswscale
        PkgConfig::libavutil
    )
else()
    message(STATUS "Falling back to CMake config packages (e.g., vcpkg)")
    find_package(SDL2 CONFIG REQUIRED)
    if (TARGET SDL2::SDL2main)
        target_link_libraries(zenotrack PRIVATE SDL2::SDL2main)
    endif()
    target_link_libraries(zenotrack PRIVATE SDL2::SDL2)

    # vcpkg provides unofficial-ffmpeg
    find_package(unofficial-ffmpeg CONFIG REQUIRED COMPONENTS avformat avcodec swscale avutil)
    target_link_libraries(zenotrack PRIVATE
        unofficial::ffmpeg::avformat
        unofficial::ffmpeg::avcodec
        unofficial::ffmpeg::swscale
        unofficial::ffmpeg::avutil
    )
endif()

# Math library on Unix
if (UNIX AND NOT APPLE)
    target_link_libraries(zenotrack PRIVATE m)
endif()

# Optimization flags
if (MSVC)
    target_compile_options(zenotrack PRIVATE /O2)
else()
    target_compile_options(zenotrack PRIVATE -O3 -ffast-math)
    include(CheckCCompilerFlag)
    check_c_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
    if (COMPILER_SUPPORTS_MARCH_NATIVE)
        target_compile_options(zenotrack PRIVATE -march=native)
    endif()
endif()

install(TARGETS zenotrack RUNTIME DESTINATION bin)
