ivw_module(Robotics)

set(HEADER_FILES
    include/inviwo/robotics/roboticsmodule.h
    include/inviwo/robotics/roboticsmoduledefine.h
    include/inviwo/robotics/processors/roboticsProcessor.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/roboticsmodule.cpp
    src/processors/roboticsProcessor.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(PYTHON_FILES
    # Add pythons
)
ivw_group("Python Files" ${PYTHON_FILES})

set(SHADER_FILES
	# Add shaders
    pyProcessors/robotics.py
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/robotics-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES} ${PYTHON_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)