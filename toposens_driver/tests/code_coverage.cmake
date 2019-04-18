# Set up program paths and compiler settings

find_program(GCOV_PATH gcov)
find_program(LCOV_PATH lcov)
find_program(GENHTML_PATH genhtml)

set(run TRUE)

if(NOT GCOV_PATH OR NOT LCOV_PATH OR NOT GENHTML_PATH)
    set(run FALSE)
    message(WARNING "To run code coverage, you must have gcov, lcov, and genhtml! One or all of them were not found...")
endif()

set(CMAKE_CXX_FLAGS_COVERAGE
        "-g -O0 --coverage -fprofile-arcs -ftest-coverage"
        CACHE STRING "Flags used by the C++ compiler during coverage builds."
        FORCE
        )

set(CMAKE_C_FLAGS_COVERAGE
        "-g -O0 --coverage -fprofile-arcs -ftest-coverage"
        CACHE STRING "Flags used by the C compiler during coverage builds."
        FORCE
        )

set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
        ""
        CACHE STRING "Flags used for linking binaries during coverage builds."
        FORCE
        )

set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE
        ""
        CACHE STRING "Flags used by the shared libraries linker during coverage builds."
        FORCE
        )

mark_as_advanced(
        CMAKE_CXX_FLAGS_COVERAGE
        CMAKE_C_FLAGS_COVERAGE
        CMAKE_EXE_LINKER_FLAGS_COVERAGE
        CMAKE_SHARED_LINKER_FLAGS_COVERAGE
)

# Set up coverage targets
set(COVERAGE_DIR ${CMAKE_BINARY_DIR}/coverage)

add_custom_target(${PROJECT_NAME}_coverage_dir
        COMMAND ${CMAKE_COMMAND} -E make_directory ${COVERAGE_DIR}
        )

add_custom_target(${PROJECT_NAME}_coverage_prep
        # Clean up code counters
        COMMAND ${LCOV_PATH} --quiet --directory ${CMAKE_BINARY_DIR} --zerocounters

        WORKING_DIRECTORY ${COVERAGE_DIR}
        DEPENDS ${PROJECT_NAME}_coverage_dir
        )

function(coverage_add_target tgt)
    if(${run})
        add_dependencies(${tgt} ${PROJECT_NAME}_coverage_prep)

        add_custom_target(${PROJECT_NAME}_coverage
                # Set directories and capture code counters
                COMMAND ${LCOV_PATH}
                --directory ${CMAKE_BINARY_DIR}
                --base-directory ${PROJECT_SOURCE_DIR}
                --capture
                --output-file ${PROJECT_NAME}.info

                # Extract code counters, excluding Google Test and header files
                COMMAND ${LCOV_PATH}
                --extract ${PROJECT_NAME}.info '${PROJECT_SOURCE_DIR}/src/*'
                --output-file ${PROJECT_NAME}.info.cleaned


                # Generate HTML report
                COMMAND ${GENHTML_PATH} -o ${COVERAGE_DIR}
                --show-details
                --legend ${PROJECT_NAME}.info.cleaned

                # Clean up auxiliary files
                COMMAND ${CMAKE_COMMAND} -E remove ${PROJECT_NAME}.info ${PROJECT_NAME}.info.cleaned
                COMMAND ${CMAKE_COMMAND} -E echo "Coverage report found in: ${COVERAGE_DIR}"

                WORKING_DIRECTORY ${COVERAGE_DIR}
                DEPENDS ${tgt}
                )
    endif()

endfunction()
