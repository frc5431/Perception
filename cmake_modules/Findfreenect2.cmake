find_path(freenect2_INCLUDE_DIR NAMES libfreenect2.hpp
        HINTS
        /usr/local/include/libfreenect2/
        /usr/include/libfreenect2
        /usr/local/include/
        /usr/include/
        }
)
 
find_library(freenect2_LIBRARY NAMES freenect2)

if(freenect2_INCLUDE_DIR AND freenect2_LIBRARY)
  set(freenect2_FOUND TRUE)
endif()

if(freenect2_LIBRARY)
    set(freenect2_LIBRARY ${freenect2_LIBRARY})
endif()

if (freenect2_FOUND)
  mark_as_advanced(freenect2_INCLUDE_DIRS freenect2_LIBRARY freenect2_LIBRARIES)
endif()