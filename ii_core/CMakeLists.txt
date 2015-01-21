SET(II_CORE_LIBS
    convex_hull.h
    convex_hull.cpp
    
    my_texture.h
    my_texture.cpp
    
    my_rend.h
    my_rend.cpp
    
    optimize.h
    optimize.cpp
    
    scene_cal.h
    scene_cal.cpp
    
    shadow_cal.h
    shadow_cal.cpp)
ADD_LIBRARY(iiCore ${II_CORE_LIBS})
TARGET_LINK_LIBRARIES(iiCore ${CORELIBS})
