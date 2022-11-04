# SPDX-License-Identifier: Apache-2.0

# Configures binary tools as mwdt binutils

find_program(CMAKE_ELF2BIN ${CROSS_COMPILE}elf2bin   PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
find_program(CMAKE_OBJDUMP ${CROSS_COMPILE}elfdumpac PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
find_program(CMAKE_AS      ${CROSS_COMPILE}ccac      PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
find_program(CMAKE_AR      ${CROSS_COMPILE}arac      PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
find_program(CMAKE_RANLIB  ${CROSS_COMPILE}arac      PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
find_program(CMAKE_READELF ${CROSS_COMPILE}elfdumpac PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
find_program(CMAKE_NM      ${CROSS_COMPILE}nmac      PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
find_program(CMAKE_STRIP   ${CROSS_COMPILE}stripac   PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
find_program(CMAKE_SIZE    ${CROSS_COMPILE}sizeac    PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
find_program(CMAKE_ELF2HEX ${CROSS_COMPILE}elf2hex   PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)

SET(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> -rq <TARGET> <LINK_FLAGS> <OBJECTS>")
SET(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> -rq <TARGET> <LINK_FLAGS> <OBJECTS>")
SET(CMAKE_CXX_ARCHIVE_FINISH "<CMAKE_AR> -sq <TARGET>")
SET(CMAKE_C_ARCHIVE_FINISH "<CMAKE_AR> -sq <TARGET>")

find_program(CMAKE_GDB     ${CROSS_COMPILE}mdb     PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)

# MWDT binutils don't support required features like section renaming, so we
# temporarily had to use GNU objcopy instead
if(CONFIG_ISA_ARCV3)
  set(ARCMWDT_Z_HELPER_TARGET       arc64-zephyr-elf)
else()
  set(ARCMWDT_Z_HELPER_TARGET         arc-zephyr-elf)
endif()

set(ARCMWDT_Z_HELPER_PATH ${ZEPHYR_SDK_INSTALL_DIR}/${ARCMWDT_Z_HELPER_TARGET}/bin)

find_program(CMAKE_OBJCOPY ${ARCMWDT_Z_HELPER_TARGET}-objcopy PATHS ${ARCMWDT_Z_HELPER_PATH})
if(NOT CMAKE_OBJCOPY)
  message(FATAL_ERROR "Zephyr unable to find any GNU objcopy from Zephyr SDK. Please check that \
  you have specified ZEPHYR_SDK_INSTALL_DIR correctly. Current value: '${ZEPHYR_SDK_INSTALL_DIR}'")
endif()

include(${ZEPHYR_BASE}/cmake/bintools/arcmwdt/target_bintools.cmake)
