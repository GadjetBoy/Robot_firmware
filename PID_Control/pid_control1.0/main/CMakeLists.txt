idf_component_register(SRCS "pid_control1.0.c"
                    INCLUDE_DIRS ".")
target_link_libraries(${COMPONENT_LIB} PRIVATE m)