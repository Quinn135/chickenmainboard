#pragma once

// The core's stm32h7xx_hal_conf.h pulls this in (via __has_include) right
// before stm32h7xx_hal_conf_default.h, so anything defined here takes effect
// ahead of the framework defaults. It has to live in include/ to be found -
// PlatformIO puts include/ and src/ on the include path, but not bare lib/.

// FDCAN for the ESC bus on PI9/PH13. The H7 default conf already enables this,
// but keep it explicit so the build doesn't depend on that staying true.
#ifndef HAL_FDCAN_MODULE_ENABLED
#define HAL_FDCAN_MODULE_ENABLED
#endif
