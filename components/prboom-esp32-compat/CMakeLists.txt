idf_component_register(
    SRCS 
        "gamepad.c"
        "i_main.c"
        "i_network.c"
        "i_sound.c"
        "i_system.c"
        "i_video.c"
        "psxcontroller.c"
        "spi_lcd.c"
    INCLUDE_DIRS "include"
    REQUIRES prboom
)
