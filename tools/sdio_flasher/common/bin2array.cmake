# Creates C resources file from files in given directory recursively
function(create_resources dir output)
    message(STATUS "[bin2array] Creating resources from directory: ${dir}")
    message(STATUS "[bin2array] Output file: ${output}")

    # Check if directory exists
    if(NOT EXISTS ${dir})
        message(WARNING "[bin2array] Directory does not exist: ${dir}")
        return()
    endif()

    # Create empty output file
    file(WRITE ${output} "#include <stdint.h>\n\n")
    # Collect input files.
    # Only *.bin — globbing everything also picks up git placeholders
    # (.gitignore/.gitkeep), and an EMPTY file such as .gitkeep makes the
    # unquoted REGEX REPLACE below collapse to 5 arguments and fail the build.
    file(GLOB bin_paths ${dir}/*.bin)

    # Iterate through input files
    foreach(bin ${bin_paths})
        message(STATUS "[bin2array] Processing file: ${bin}")
        # Get short filenames, by discarding relative path
        get_filename_component(name ${bin} NAME)
        message(STATUS "[bin2array]   File name: ${name}")
        # Replace filename spaces & extension separator for C compatibility
        string(REGEX REPLACE "[\\./-]" "_" filename ${name})
        message(STATUS "[bin2array]   Variable name: ${filename}")

        # Check if file exists and is readable
        if(NOT EXISTS ${bin})
            message(WARNING "[bin2array]   File does not exist: ${bin}")
            continue()
        endif()

        # Read hex data from file
        file(READ ${bin} filedata HEX)
        # Skip empty files: an empty ${filedata} would collapse the argument
        # list of the REGEX REPLACE below and abort configuration.
        if(filedata STREQUAL "")
            message(WARNING "[bin2array]   Skipping empty file: ${bin}")
            continue()
        endif()
        # Convert hex data for C compatibility (quote filedata — see above)
        string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," filedata "${filedata}")
        # Compute MD5 hash of the file
        file(MD5 "${bin}" md5_hash)
        message(STATUS "[bin2array]   MD5: ${md5_hash}")
        # Append data to output file
        file(APPEND ${output}
            "const uint8_t  ${filename}[] = {${filedata}};\n"
            "const uint32_t ${filename}_size = sizeof(${filename});\n"
            "const uint8_t  ${filename}_md5[] = \"${md5_hash}\";\n"
        )
    endforeach()
endfunction()

# Main execution when run as script
if(INPUT_DIR AND OUTPUT_FILE)
    create_resources(${INPUT_DIR} ${OUTPUT_FILE})
endif()
