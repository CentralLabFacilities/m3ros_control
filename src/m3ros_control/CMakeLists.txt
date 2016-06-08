###############################################################################
# Protobuff stuff
file(GLOB ControllerProtoFiles "${PROJECT_PROTO_DIR}/${PROJECT_NAME}/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${CMAKE_CURRENT_BINARY_DIR} ${ControllerProtoFiles})
add_custom_target(${PROJECT_NAME}_cpp_protobuf_output  ALL DEPENDS ${ProtoSources}) # force the generation
PROTOBUF_GENERATE_PYTHON(PySources ${PROJECT_PYTHON_DIR}/${PROJECT_NAME} ${ControllerProtoFiles})
add_custom_target(${PROJECT_NAME}_python_protobuf_output ALL DEPENDS ${PySources}) # force the generation
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_PYTHON_DIR}/${PROJECT_NAME}) # To create a directory at build time
set(ProtoSources ${ProtoSources} PARENT_SCOPE)


