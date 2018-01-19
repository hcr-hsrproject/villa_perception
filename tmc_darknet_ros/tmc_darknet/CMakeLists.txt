# This package has no tests
cmake_minimum_required(VERSION 2.8.3)
project(tmc_darknet)

if(DEFINED ENV{GPU_ENABLED})
  set(USE_GPU true)
else()
  set(USE_GPU false)
endif()
set(USE_CUDNN false)

set(CMAKE_C_FLAGS_RELEASE "-Ofast -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG")
add_compile_options(-Wall -Wextra)

if(USE_GPU)
  set(CUDA_NVCC_FLAGS_RELEASE --use_fast_math)
  add_definitions(-DGPU)
  if(USE_CUDNN)
    add_definitions(-DCUDNN)
  endif()
  find_package(CUDA REQUIRED)
endif()

find_package(catkin REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
)

include_directories(
  SYSTEM src
  include/${PROJECT_NAME}
)

if(USE_GPU)
  include_directories(
  SYSTEM ${CUDA_INCLUDE_DIRS}
  )
  cuda_add_library(${PROJECT_NAME}
    src/activation_kernels.cu
    src/activation_layer.c
    src/activations.c
    src/avgpool_layer.c
    src/avgpool_layer_kernels.cu
    src/batchnorm_layer.c
    src/blas.c
    src/blas_kernels.cu
    src/box.c
    src/col2im.c
    src/col2im_kernels.cu
    src/connected_layer.c
    src/convolutional_kernels.cu
    src/convolutional_layer.c
    src/cost_layer.c
    src/crnn_layer.c
    src/crop_layer.c
    src/crop_layer_kernels.cu
    src/cuda.c
    src/data.c
    src/detection_layer.c
    src/dropout_layer.c
    src/dropout_layer_kernels.cu
    src/gemm.c
    src/gru_layer.c
    src/im2col.c
    src/im2col_kernels.cu
    src/image.c
    src/layer.c
    src/list.c
    src/local_layer.c
    src/matrix.c
    src/maxpool_layer.c
    src/maxpool_layer_kernels.cu
    src/network.c
    src/network_kernels.cu
    src/normalization_layer.c
    src/option_list.c
    src/parser.c
    src/region_layer.c
    src/reorg_layer.c
    src/rnn_layer.c
    src/route_layer.c
    src/shortcut_layer.c
    src/softmax_layer.c
    src/tree.c
    src/utils.c
    )
  cuda_add_cublas_to_target(${PROJECT_NAME})
  target_link_libraries(${PROJECT_NAME}
    ${CUDA_LIBRARIES}
    ${CUDA_curand_LIBRARY})
  if(USE_CUDNN)
    target_link_libraries(${PROJECT_NAME} cudnn)
  endif()
else()
  add_library(${PROJECT_NAME}
    src/activation_layer.c
    src/activations.c
    src/avgpool_layer.c
    src/batchnorm_layer.c
    src/blas.c
    src/box.c
    src/col2im.c
    src/connected_layer.c
    src/convolutional_layer.c
    src/cost_layer.c
    src/crnn_layer.c
    src/crop_layer.c
    src/cuda.c
    src/data.c
    src/detection_layer.c
    src/dropout_layer.c
    src/gemm.c
    src/gru_layer.c
    src/im2col.c
    src/image.c
    src/layer.c
    src/list.c
    src/local_layer.c
    src/matrix.c
    src/maxpool_layer.c
    src/network.c
    src/normalization_layer.c
    src/option_list.c
    src/parser.c
    src/region_layer.c
    src/reorg_layer.c
    src/rnn_layer.c
    src/route_layer.c
    src/shortcut_layer.c
    src/softmax_layer.c
    src/tree.c
    src/utils.c
    )
  target_link_libraries(${PROJECT_NAME} m)
endif()

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
