cmake_minimum_required(VERSION 3.2)
set(PROJECT_NAME "Khaybulin_Daniel_ASearch")
project(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX)
    if (${MINGW})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra -static -static-libgcc -static-libstdc++")
    else ()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra")
    endif()
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -static -static-libgcc -static-libstdc++")
endif()

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

enable_testing()

add_subdirectory(Src)
add_subdirectory(Tests)
