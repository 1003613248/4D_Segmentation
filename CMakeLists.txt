cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(4D_Segmentation)

list(APPEND CMAKE_MODULE_PATH "/home/wine/FindTBB")
#SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#FindOpenGL()
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA 8.0 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(TBB COMPONENTS tbbmalloc tbbmalloc_proxy tbb_preview)

add_subdirectory(src)
set(CUDA_HEADS cuda_common/inc)
#add_subdirectory(${CUDA_HEADS})

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})

include_directories(${TBB_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${4D_Segmentation_SOURCE_DIR}/include)
include_directories(${CUDA_HEADS})
include_directories(${CUDA_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OPENGL_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${TBB_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(${TBB_DEFINITIONS})
add_definitions(${CUDA_DEFINITIONS})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -gencode arch=compute_30,code=sm_30)

file( GLOB  cu  *.cu)
cuda_add_executable( vector src/vector_add.cu ${cu} )
#target_link_libraries(vector_add ${CUDA_LIBRARIES})
#cuda_add_executable( cuFilter src/recursiveGaussian_cuda.cu  src/cuFilter.cpp ${cu} ${CUDA_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )


file(GLOB_RECURSE 4D_Segmentation_HEADERS include/*.h)
file(GLOB_RECURSE 4D_Segmentation_SOURCES src/*.cpp)
set (4D_Segmentation_INCLUDE_DIRS "")
foreach (_headerFile ${4D_Segmentation_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND 4D_Segmentation_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES 4D_Segmentation_INCLUDE_DIRS)

include_directories(${4D_Segmentation_INCLUDE_DIRS})

add_library (4D_Segmentation src/4D_Segmentation.cpp src/GraphSegmentation.cpp src/RegionTree.cpp  src/OpticalFlow.cpp  ${4D_Segmentation_HEADERS})
target_link_libraries (4D_Segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES} ${OpenGL_LIBRARIES} ${CUDA_LIBs} ${GLUT_LIBRARIES})

add_executable (Test_Seg src/TestVideoSegmentation.cpp ${4D_Segmentation_HEADERS})
target_link_libraries (Test_Seg 4D_Segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS} tbb)

add_executable (pcd2ppm src/pcd2ppm.cpp)
target_link_libraries(pcd2ppm ${PCL_LIBRARIES} ${CUDA_LIBRARIES})
add_executable (pcdDepth2ppm src/pcdDepth2ppm.cpp)
target_link_libraries(pcdDepth2ppm ${PCL_LIBRARIES} ${CUDA_LIBRARIES})
#cuda_add_library  (recursiveCUDA src/recursiveGaussian_cuda.cu ${cu} ${CUDA_HEADS})
#target_link_libraries(recursiveCUDA ${CUDA_LIBRARIES})
#add_library (recursiveCPP src/cuFilter.cpp ${4D_Segmentation_HEADERS} ${cu} ${CUDA_HEADS})
#target_link_libraries(recursiveCPP ${CUDA_LIBRARIES})
cuda_add_executable (cuFilter src/cuFilter.cpp src/recursiveGaussian_cuda.cu ${CUDA_HEADS} ${cu})
target_link_libraries(cuFilter ${cu} ${CUDA_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
