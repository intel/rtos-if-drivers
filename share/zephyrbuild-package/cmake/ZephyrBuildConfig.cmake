# Intel specific definitions

# To ensure final path is absolute and does not contain ../.. in variable.
get_filename_component(APPLICATION_PROJECT_DIR
                       ${CMAKE_CURRENT_LIST_DIR}/../../..
                       ABSOLUTE
)

# Add this project to list of board roots
list(APPEND BOARD_ROOT ${APPLICATION_PROJECT_DIR})
list(APPEND SOC_ROOT ${APPLICATION_PROJECT_DIR})
list(APPEND DTS_ROOT ${APPLICATION_PROJECT_DIR})
list(APPEND MODULE_EXT_ROOT ${APPLICATION_PROJECT_DIR})

# To support driver development in the internal tree for boards that are
# already upstream, allow having an app-independent overlay file for
# specific boards under overlay/<board>.overlay. Do this without losing
# support for app-specific overlays, although in a limited manner (for
# the most common <board>.overlay file convention in the root directory
# of the application.
set(INTEL_OVERLAY ${APPLICATION_PROJECT_DIR}/overlay/${BOARD}.overlay)
set(BOARD_OVERLAY ${APPLICATION_SOURCE_DIR}/boards/${BOARD}.overlay)

if(EXISTS ${INTEL_OVERLAY})
  if (EXISTS ${BOARD_OVERLAY})
    set(DTC_OVERLAY_FILE "${INTEL_OVERLAY} ${BOARD_OVERLAY}")
  else()
    set(DTC_OVERLAY_FILE ${INTEL_OVERLAY})
  endif()
endif()

list(APPEND SYSCALL_INCLUDE_DIRS ${APPLICATION_PROJECT_DIR}/include)
