# GCC toolchain prefix
SET(TOOLCHAIN_PREFIX "/home/iwasz/local/share/msp430-gcc")
SET(TARGET_TRIPLET "msp430-elf")
SET(SUPPORT_FILES "/home/iwasz/local/share/msp430-gcc/include")

SET(TOOLCHAIN_BIN_DIR ${TOOLCHAIN_PREFIX}/bin)
SET(TOOLCHAIN_INC_DIR ${TOOLCHAIN_PREFIX}/${TARGET_TRIPLET}/include)
SET(TOOLCHAIN_LIB_DIR ${TOOLCHAIN_PREFIX}/${TARGET_TRIPLET}/lib)

SET(CMAKE_SYSTEM_NAME Generic)

SET(CMAKE_C_COMPILER ${TOOLCHAIN_BIN_DIR}/${TARGET_TRIPLET}-gcc)
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN_DIR}/${TARGET_TRIPLET}-g++)
SET(CMAKE_ASM_COMPILER ${TOOLCHAIN_BIN_DIR}/${TARGET_TRIPLET}-as)
SET(CMAKE_OBJCOPY ${TOOLCHAIN_BIN_DIR}/${TARGET_TRIPLET}-objcopy)
SET(CMAKE_OBJDUMP ${TOOLCHAIN_BIN_DIR}/${TARGET_TRIPLET}-objdump)

SET(CMAKE_C_FLAGS "-std=gnu99 -Wall" CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -fdata-sections -ffunction-sections -MD -Wall" CACHE INTERNAL "cxx compiler flags")

SET(CMAKE_EXE_LINKER_FLAGS "-T ${TOOLCHAIN_PREFIX}/include/${DEVICE}.ld -T ${USBLIB}/USB_API/msp430USB.ld -Wl,--gc-sections" CACHE INTERNAL "exe link flags")

INCLUDE_DIRECTORIES(${SUPPORT_FILES})
LINK_DIRECTORIES(${SUPPORT_FILES})

ADD_DEFINITIONS (-mmcu=${DEVICE})

SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "-mmcu=${DEVICE}")
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-mmcu=${DEVICE}")

enable_language(ASM-ATT)