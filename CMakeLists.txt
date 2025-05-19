ivw_module(Template)

set(HEADER_FILES
    include/inviwo/template/templatemodule.h
    include/inviwo/template/templatemoduledefine.h
    include/inviwo/template/processors/templateProcessor.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/templatemodule.cpp
    src/processors/templateProcessor.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(PYTHON_FILES
    # Add pythons
)
ivw_group("Python Files" ${PYTHON_FILES})

set(SHADER_FILES
	# Add shaders
    pyProcessors/template.py
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/template-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES} ${PYTHON_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)