#!/bin/sh

git submodule add https://github.com/bdmihai/stm32-doc doc/stm32
git submodule add https://github.com/bdmihai/stm32-qbs qbs/modules/stm32
git submodule add https://github.com/bdmihai/stm32-startup source/startup
git submodule add https://github.com/bdmihai/stm32-hal source/hal
git submodule add https://github.com/bdmihai/stm32-cmsis source/cmsis
git submodule add https://github.com/bdmihai/stm32-freertos source/freertos
git submodule add https://github.com/bdmihai/stm32-boot source/boot 
git submodule add https://github.com/bdmihai/stm32-runtime source/runtime