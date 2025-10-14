cmake_minimum_required(VERSION 3.11)

project(EmulatedControllerTest)

add_prx_module(${PROJECT_NAME}
    emulated_controller_test.c
    exports.exp
    imports.S
)

set_property(SOURCE imports.S PROPERTY LANGUAGE C)
# set_target_properties(EmulatedControllerTest PROPERTIES USE_KERNEL_LIBS "1")

target_compile_definitions(
    # If the debug configuration pass the DEBUG define to the compiler
    ${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:DEBUG>
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    pspdebug
    pspdisplay
#    pspctrl
    pspge
)

set_target_properties(EmulatedControllerTest PROPERTIES OUTPUT_NAME "emu_ctrl_test")
# target_include_directories(EmulatedControllerTest PUBLIC ${CMAKE_SOURCE_DIR}/include)
