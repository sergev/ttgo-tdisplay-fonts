cmake_minimum_required(VERSION 3.5)
idf_component_register(
    SRCS
        "st7789.c"
        "main.c"

    INCLUDE_DIRS
        "."
)
