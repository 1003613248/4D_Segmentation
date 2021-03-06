cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

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

#cuda_add_library (4D_Segmentation src/4D_Segmentation.cpp src/recursiveGaussian_cuda.cu src/GraphSegmentation.cpp src/RegionTree.cpp  src/OpticalFlow.cpp ${CUDA_HEADS} ${cu}  ${4D_Segmentation_HEADERS})
#target_link_libraries (4D_Segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIBRARIES} ${OpenGL_LIBRARIES} ${CUDA_LIBS} ${GLUT_LIBRARIES} ${cu})

add_executable (jpg2ppm src/jpg2ppm.cpp)
target_link_libraries (jpg2ppm ${OpenCV_LIBS})
#must be OpenCV_LIBS !

cuda_add_executable (Test_Seg src/TestVideoSegmentation.cpp src/4D_Segmentation.cpp src/recursiveGaussian_cuda.cu src/GraphSegmentation.cpp src/RegionTree.cpp  src/OpticalFlow.cpp ${cu} ${4D_Segmentation_HEADERS} ${CUDA_HEADS})
target_link_libraries (Test_Seg ${cu} ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} tbb ${CUDA_LIBRARIES} ${GLUT_LIBRARIES})
#target_link_libraries (Test_Seg 4D_Segmentation ${cu} ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} tbb ${CUDA_LIBRARIES} ${GLUT_LIBRARIES})

#add_executable (pcd2ppm src/pcd2ppm.cpp)
#target_link_libraries(pcd2ppm ${PCL_LIBRARIES} ${CUDA_LIBRARIES})
add_executable (pcdDepth2ppm src/pcdDepth2ppm.cpp)
target_link_libraries(pcdDepth2ppm ${PCL_LIBRARIES} ${CUDA_LIBRARIES})
#cuda_add_library  (recursiveCUDA src/recursiveGaussian_cuda.cu ${cu} ${CUDA_HEADS})
#target_link_libraries(recursiveCUDA ${CUDA_LIBRARIES})
#add_library (recursiveCPP src/cuFilter.cpp ${4D_Segmentation_HEADERS} ${cu} ${CUDA_HEADS})
#target_link_libraries(recursiveCPP ${CUDA_LIBRARIES})
cuda_add_executable (cuFilter src/cuFilter.cpp src/recursiveGaussian_cuda.cu ${CUDA_HEADS} ${cu})
target_link_libraries(cuFilter ${CUDA_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

cuda_add_executable (mst mst/kernels.cuh mst/segmentationTree.cu ${CUDA_HEADS} )
target_link_libraries (mst ${cu} ${OPENGL_LIBRARIES} ${CUDA_LIBRARIES} ${GLUT_LIBRARIES}) 
cuda_add_executable (hso hso/flowGold.cpp hso/flowCUDA.cu hso/main.cpp ${CUDA_HEADS})
target_link_libraries (hso ${cu} ${OPENGL_LIBRARIES} ${CUDA_LIBRARIES} ${GLUT_LIBRARIES}) 
cuda_add_executable (hist hist/histogram_gold.cpp hist/histogram256.cu hist/main.cpp hist/histogram64.cu ${CUDA_HEADS})
target_link_libraries (hist ${cu} ${OPENGL_LIBRARIES} ${CUDA_LIBRARIES} ${GLUT_LIBRARIES}) 
