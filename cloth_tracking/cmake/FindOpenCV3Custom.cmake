# CMake find module for OpenCV3, as installed on sand by cli

find_path( OPENCV3CUSTOM_INCLUDE_DIRS opencv2/shape.hpp
          PATHS ~/opencv/install/include )

message(STATUS "OpenCV3Custom Include: ${OPENCV3CUSTOM_INCLUDE_DIRS}")

set(OPENCV3_LIBS
    opencv_core
    opencv_shape
    opencv_imgproc
    opencv_video
    opencv_videostab
    opencv_highgui
    opencv_features2d
    opencv_objdetect
    opencv_imgcodecs
)

set( OpenCV_LIBS "" )
foreach(__CVLIB ${OPENCV3_LIBS})
    find_library( OpenCV_${__CVLIB}_LIBRARY NAMES
            ${__CVLIB}
            PATHS ~/opencv/install/lib NO_DEFAULT_PATH)

    message( STATUS " ---- ${OpenCV_${__CVLIB}_LIBRARY}" )

    if(OpenCV_${__CVLIB}_LIBRARY)
        set(OpenCV_LIBS ${OpenCV_LIBS} ${OpenCV_${__CVLIB}_LIBRARY})
    endif(OpenCV_${__CVLIB}_LIBRARY)
endforeach(__CVLIB)

             
message( STATUS "OpenCV3Custom Libs: ${OpenCV_LIBS}" )
set( OPENCV3CUSTOM_LIBRARIES ${OpenCV_LIBS} )
             

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( OpenCV3Custom DEFAULT_MSG
                                   OPENCV3CUSTOM_LIBRARIES OPENCV3CUSTOM_INCLUDE_DIRS )
mark_as_advanced( OPENCV3CUSTOM_INCLUDE_DIRS OPENCV3CUSTOM_LIBRARIES )

