#include "ui.hpp"

#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_event.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/libs/fsdrv/lv_fsdrv.h"
#include "liblvgl/draw/lv_image_dsc.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_fs.h"
#include "liblvgl/misc/lv_timer.h"
#include "liblvgl/widgets/button/lv_button.h"
#include "liblvgl/widgets/image/lv_image.h"
#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/widgets/led/lv_led.h"
#include "liblvgl/widgets/slider/lv_slider.h"
#include "liblvgl/widgets/tabview/lv_tabview.h"
#include "liblvgl/widgets/textarea/lv_textarea.h"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include "pros/imu.hpp"
#include "subsystems.hpp"

#include <atomic>
#include <cstdint>

namespace{

// Team palette: cyan on charcoal, with earthy brown structure.
constexpr std::uint32_t CYAN_HEX = 0x59D9F5;
constexpr std::uint32_t CYAN_DARK_HEX = 0x1A8199;
constexpr std::uint32_t BROWN_HEX = 0x6B4226;
constexpr std::uint32_t BROWN_LIGHT_HEX = 0x9A6741;
constexpr std::uint32_t CHARCOAL_HEX = 0x171412;
constexpr std::uint32_t PANEL_HEX = 0x241C18;
constexpr std::uint32_t TEXT_HEX = 0xEAFBFF;
constexpr std::uint32_t MUTED_HEX = 0xA9B8BA;
constexpr std::uint32_t RED_HEX = 0xE05A5A;
constexpr std::uint32_t BLUE_HEX = 0x397BDB;

constexpr lv_color_t CYAN = LV_COLOR_MAKE(0x59, 0xD9, 0xF5);
constexpr lv_color_t CYAN_DARK = LV_COLOR_MAKE(0x1A, 0x81, 0x99);
constexpr lv_color_t BROWN = LV_COLOR_MAKE(0x6B, 0x42, 0x26);
constexpr lv_color_t BROWN_LIGHT = LV_COLOR_MAKE(0x9A, 0x67, 0x41);
constexpr lv_color_t CHARCOAL = LV_COLOR_MAKE(0x17, 0x14, 0x12);
constexpr lv_color_t PANEL = LV_COLOR_MAKE(0x24, 0x1C, 0x18);
constexpr lv_color_t TEXT = LV_COLOR_MAKE(0xEA, 0xFB, 0xFF);
constexpr lv_color_t MUTED = LV_COLOR_MAKE(0xA9, 0xB8, 0xBA);
constexpr lv_color_t RED = LV_COLOR_MAKE(0xE0, 0x5A, 0x5A);
constexpr lv_color_t BLUE = LV_COLOR_MAKE(0x39, 0x7B, 0xDB);

constexpr std::size_t AUTON_COUNT = 5;
constexpr lv_coord_t FIELD_INSET = 2;
const char* const AUTON_NAMES[AUTON_COUNT] = {
    "Left Side Quals",
    "Right Side Elims",
    "AWP Solo",
    "Do Nothing",
    "Test Auton",
};



}