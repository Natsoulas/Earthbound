add_executable(Earthbound main.cpp)

target_link_libraries(Earthbound 
    PRIVATE 
    Eigen3::Eigen
)

if(MSVC)
    target_compile_options(Earthbound PRIVATE /W4)
else()
    target_compile_options(Earthbound PRIVATE -Wall -Wextra -Wpedantic)
endif() 