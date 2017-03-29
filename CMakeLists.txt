cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(4D_Segmentation)

list(APPEND CMAKE_MODULE_PATH "/home/wine/FindTBB")
#SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA 8.0 REQUIRED)

find_package(TBB COMPONENTS tbbmalloc tbbmalloc_proxy tbb_preview)

add_subdirectory(src)

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${TBB_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${4D_Segmentation_SOURCE_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${TBB_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(${TBB_DEFINITIONS})

file(GLOB_RECURSE 4D_Segmentation_HEADERS include/*.h)
file(GLOB_RECURSE 4D_Segmentation_SOURCES src/*.cpp)
set (4D_Segmentation_INCLUDE_DIRS "")
foreach (_headerFile ${4D_Segmentation_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND 4D_Segmentation_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES 4D_Segmentation_INCLUDE_DIRS)

include_directories(${4D_Segmentation_INCLUDE_DIRS})

add_library (4D_Segmentation src/4D_Segmentation.cpp src/RegionTree.cpp src/GraphSegmentation.cpp src/OpticalFlow.cpp ${4D_Segmentation_HEADERS})
target_link_libraries (4D_Segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES})

add_executable (Test_Seg src/TestVideoSegmentation.cpp ${4D_Segmentation_HEADERS})
target_link_libraries (Test_Seg 4D_Segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS} tbb)
