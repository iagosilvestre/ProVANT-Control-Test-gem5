#/*
#-----------------------------------------------------------------------------
#   Filename:    CMakeLists.txt
#   Gerador de Makefile para io-board.
#-----------------------------------------------------------------------------
#      _____________  /\   ____________
#     /\_  _ __ ___ \ \/ / __ _ _ __ / |_
#    / /_)/ '__/ _ \ \  / / _` | '_ \| __|
#   / ___/| | | (_) \ \/ / (_| | | | | |_
#   \/    |_|  \___/ \__/ \__,_|_| |_|\__|
#
#-----------------------------------------------------------------------------
#
#   Compilando em release/debug (chamar pelo terminal)
#   cmake -DCMAKE_BUILD_TYPE=Debug ..
#
#   Para obter informacoes sobre um modulo (ex, FindBoost)
#   cmake --help-module FindBoost
#*/

# GO FROM THAT http://www.cmake.org/cmake/help/examples.html 

cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0037 OLD)

# CONFIGURATIONS FOR PROJECT  ------------------------------------------------
set(PROJECT_NAME 	remote-controlled-flight)

set(APPDIR  "${CMAKE_CURRENT_LIST_DIR}")
set(OUTDIR  "${CMAKE_CURRENT_LIST_DIR}/buildtest")
set(CORE    "${CMAKE_CURRENT_LIST_DIR}/../../core")
set(BASE    "${CMAKE_CURRENT_LIST_DIR}/../../base")

# CMSIS Device Drivers directory
set(CMSISDIR     "${CORE}/cmsis")
set(CMSISSRCDIR  "${CMSISDIR}/src")
set(CMSISINCDIR  "${CMSISDIR}/inc")

# CMSIS DSP directory
set(DSPDIR       "${CORE}/DSP_Lib")
set(DSPSRC       "${DSPDIR}/Source")
set(DSPLIB       "${DSPDIR}/Lib/")

# FreeRTOS
set(FRTDIR       "${BASE}/FreeRTOSV7.5.2/FreeRTOS")
set(FRTSRCDIR    "${FRTDIR}/Source")
set(FRTINCDIR    "${FRTSRCDIR}/include")
set(FRTMEMDIR    "${FRTSRCDIR}/portable/MemMang/")
set(FRTPORDIR    "${FRTSRCDIR}/portable/GCC/ARM_CM4F")

# CONFIGURATIONS FOR TOOLCHAIN
INCLUDE(CMakeForceCompiler)
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc   GNU)
CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)

# Adds .c files to the sources list. 
# CMake MUST be rerun when a new file is added since the wildcard expansion occurs 
# only during Makefile generation time.
FILE(GLOB SRCS 
        ${APPDIR}/main.c
        ${APPDIR}/pv_module_rc.c
        ${APPDIR}/pv_module_io.c

        ${CORE}/system/*.c
        ${BASE}/hal/*.c
        ${BASE}/modules/rc/*.c
        ${BASE}/modules/io/*.c
        ${BASE}/modules/datapr/*.c
        ${BASE}/fsm/*.c

        ${FRTSRCDIR}/list.c
        ${FRTSRCDIR}/queue.c
        ${FRTSRCDIR}/tasks.c
        ${FRTSRCDIR}/timers.c
        ${FRTPORDIR}/port.c
        ${FRTMEMDIR}/heap_2.c

        ${BASE}/lib/provant/*.c
        ${BASE}/lib/trace/*.c

        ${CMSISDIR}/startup_stm32f4xx.s 
        )


FILE(GLOB HDRS
        ${APPDIR}/*.h    

        ${CMSISINCDIR}/*.h
        ${CMSISINCDIR}/peripherals/*.h
        ${CMSISINCDIR}/core/*.h

        ${BASE}/lib/provant/*.h
        ${BASE}/lib/trace/inc/*.h
        ${BASE}/lib/trace/config/*.h

        ${CORE}/system/*.h
        ${BASE}/hal/*.h
        ${BASE}/modules/rc/*.h
        ${BASE}/modules/io/*.h
        ${BASE}/modules/datapr/*.h
        ${BASE}/fsm/*.h

        ${FRTSRCDIR}/*.h
        ${FRTINCDIR}/*.h
        ${FRTPORDIR}/*.h

        ${DSPSRC}/*.h
        )


# CONFIGURATIONS FOR COMPILER  -----------------------------------------------
project(${PROJECT_NAME})

set(FLAGS "-g -T${CORE}/system/stm32_flash.ld -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork")
set(FLAGS "${FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nano.specs -Wl,--gc-sections -std=c99")
set(FLAGS "${FLAGS} -DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT")
set(FLAGS "${FLAGS} -Og -O0 -ffunction-sections -fdata-sections -fno-exceptions")
set(FLAGS "${FLAGS} -Wall  -Wunused-value -Wunused-variable -Wunused-but-set-variable ")#-ftime-report")
        
set(CMAKE_C_FLAGS   ${CMAKE_C_FLAGS} ${FLAGS})
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${FLAGS})

include_directories(
        ${BASE}/lib/provant
        ${BASE}/lib/trace/inc
        ${BASE}/lib/trace/config    
        ${APPDIR}

        ${CMSISINCDIR}
        ${CMSISINCDIR}/peripherals
        ${CMSISINCDIR}/core                

        ${CORE}/system
        ${BASE}/hal
        ${BASE}/modules/rc
        ${BASE}/modules/io
        ${BASE}/modules/datapr
        ${BASE}/fsm

        ${FRTSRCDIR}
        ${FRTINCDIR}
        ${FRTPORDIR}

        ${DSPSRC}       
)

link_directories(
        ${CMSISDIR}
        ${DSPLIB}
)

# DEBUG MESSAGES --------------------------------------------------------------
# message(status "** Sources:      ${SRCS}")
# message(status "** Headers:      ${HDRS}")

# BUILDING IT -----------------------------------------------------------------
set_source_files_properties(${HDRS} ${SRCS} PROPERTIES LANGUAGE C)

add_executable(${PROJECT_NAME} ${SRCS})

target_link_libraries(${PROJECT_NAME}
                        c
                        m
                        stm32f4
                        stc++
                        nosys
                        arm_cortexM4lf_math
                     )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BUILD_DIR})