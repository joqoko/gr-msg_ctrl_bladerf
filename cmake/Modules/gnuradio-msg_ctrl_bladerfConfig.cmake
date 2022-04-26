find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_MSG_CTRL_BLADERF gnuradio-msg_ctrl_bladerf)

FIND_PATH(
    GR_MSG_CTRL_BLADERF_INCLUDE_DIRS
    NAMES gnuradio/msg_ctrl_bladerf/api.h
    HINTS $ENV{MSG_CTRL_BLADERF_DIR}/include
        ${PC_MSG_CTRL_BLADERF_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_MSG_CTRL_BLADERF_LIBRARIES
    NAMES gnuradio-msg_ctrl_bladerf
    HINTS $ENV{MSG_CTRL_BLADERF_DIR}/lib
        ${PC_MSG_CTRL_BLADERF_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-msg_ctrl_bladerfTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_MSG_CTRL_BLADERF DEFAULT_MSG GR_MSG_CTRL_BLADERF_LIBRARIES GR_MSG_CTRL_BLADERF_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_MSG_CTRL_BLADERF_LIBRARIES GR_MSG_CTRL_BLADERF_INCLUDE_DIRS)
