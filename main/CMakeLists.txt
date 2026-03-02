set(srcs "main.c"
        "board.c"
        "sensors/aht20.c"
        "sensors/max30101.c"
        "sensors/SGP30.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  "." "sensors")
