cmake_minimum_required(VERSION 3.10)

if(NOT DEFINED NM_TOOL OR NOT DEFINED ELF_FILE OR NOT DEFINED SYMBOLS_FILE)
    message(FATAL_ERROR "Missing required parameters: NM_TOOL, ELF_FILE, SYMBOLS_FILE")
endif()

# Generate kernel symbols
execute_process(
    COMMAND ${NM_TOOL} --numeric-sort ${ELF_FILE}
    OUTPUT_VARIABLE KERNEL_NM_OUTPUT
    RESULT_VARIABLE KERNEL_NM_RESULT
)

if(NOT KERNEL_NM_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to run nm on kernel: ${KERNEL_NM_RESULT}")
endif()

# Generate user app symbols if UAPP_ELF_FILE is provided
set(UAPP_NM_OUTPUT "")
if(DEFINED UAPP_ELF_FILE)
    execute_process(
        COMMAND ${NM_TOOL} --numeric-sort ${UAPP_ELF_FILE}
        OUTPUT_VARIABLE UAPP_NM_OUTPUT
        RESULT_VARIABLE UAPP_NM_RESULT
    )

    if(NOT UAPP_NM_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to run nm on user app: ${UAPP_NM_RESULT}")
    endif()
endif()

file(WRITE ${SYMBOLS_FILE} "#include \"symbol.h\"\n\n")
file(APPEND ${SYMBOLS_FILE} "/* Auto-generated symbol table - do not edit */\n\n")
file(APPEND ${SYMBOLS_FILE} "__attribute__((section(\".symbols\"))) kernel_symbol_t kernel_symbols[] = {\n")

# Process kernel symbols
string(REPLACE "\n" ";" KERNEL_LINES "${KERNEL_NM_OUTPUT}")
foreach(LINE ${KERNEL_LINES})
    if(LINE STREQUAL "")
        continue()
    endif()

    # Format: address type symbol
    string(REGEX MATCH "^([0-9a-fA-F]+) ([a-zA-Z]) (.+)$" MATCH "${LINE}")
    if(MATCH)
        set(ADDR "${CMAKE_MATCH_1}")
        set(TYPE "${CMAKE_MATCH_2}")
        set(SYMBOL "${CMAKE_MATCH_3}")

        # Only keep code (T) and data (D) symbols, but filter out __user_stacks_start
        if((TYPE STREQUAL "T" OR TYPE STREQUAL "t" OR
           TYPE STREQUAL "D" OR TYPE STREQUAL "d" OR
           TYPE STREQUAL "B" OR TYPE STREQUAL "b" OR
           TYPE STREQUAL "R" OR TYPE STREQUAL "r") AND
           NOT SYMBOL STREQUAL "__user_stacks_start")
            file(APPEND ${SYMBOLS_FILE} "    {0x${ADDR}, \"${SYMBOL}\"},\n")
        endif()
    endif()
endforeach()

# Process user app symbols if available
if(NOT UAPP_NM_OUTPUT STREQUAL "")
    set(UAPP_BASE_ADDR "0x00000")

    string(REPLACE "\n" ";" UAPP_LINES "${UAPP_NM_OUTPUT}")
    foreach(LINE ${UAPP_LINES})
        if(LINE STREQUAL "")
            continue()
        endif()

        # Format: address type symbol
        string(REGEX MATCH "^([0-9a-fA-F]+) ([a-zA-Z]) (.+)$" MATCH "${LINE}")
        if(MATCH)
            set(ADDR "${CMAKE_MATCH_1}")
            set(TYPE "${CMAKE_MATCH_2}")
            set(SYMBOL "${CMAKE_MATCH_3}")

            # Only keep code (T) and data (D) symbols
            if(TYPE STREQUAL "T" OR TYPE STREQUAL "t" OR
               TYPE STREQUAL "D" OR TYPE STREQUAL "d" OR
               TYPE STREQUAL "B" OR TYPE STREQUAL "b" OR
               TYPE STREQUAL "R" OR TYPE STREQUAL "r")
                # Convert hex address to decimal, add base address, convert back to hex
                math(EXPR ADJUSTED_ADDR "${UAPP_BASE_ADDR} + 0x${ADDR}" OUTPUT_FORMAT HEXADECIMAL)
                file(APPEND ${SYMBOLS_FILE} "    {${ADJUSTED_ADDR}, \"[uapp] ${SYMBOL}\"},\n")
            endif()
        endif()
    endforeach()
endif()

file(APPEND ${SYMBOLS_FILE} "};\n")