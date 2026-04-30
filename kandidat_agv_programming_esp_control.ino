#include "src/board/board_esp32_wroom32.hpp"

void setup()
{
    board_esp32_wroom32::init();
}

void loop()
{
    board_esp32_wroom32::tick(millis());
}
