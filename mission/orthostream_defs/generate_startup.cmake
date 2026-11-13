######################################################################
#
# OrthoStream: generation of the cFE startup script (ADR-009)
#
# Writes cfe_es_startup.scr for a processor from its application list.
# Applications that are not in the list are not started.
#
######################################################################

function (generate_cfs_startup_script CFS_INSTALL_DIR)

    set (STARTUP_FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CFS_INSTALL_DIR}/cfe_es_startup.scr")

    # Startup lines of the known applications, in start order:
    # type, object file, entry point, name, priority, stack size, load address, exception action
    # Libraries come first, so that they are initialised before the
    # applications that use them.
    set (STARTUP_ost_common "CFE_LIB, ost_common, OST_Common_Init, OST_COMMON, 0, 0, 0x0, 0;")
    set (STARTUP_sch_lab "CFE_APP, sch_lab, SCH_LAB_AppMain, SCH_LAB, 35, 32768, 0x0, 0;")
    set (STARTUP_ci_lab  "CFE_APP, ci_lab,  CI_LAB_AppMain,  CI_LAB,  40, 32768, 0x0, 0;")
    set (STARTUP_to_lab  "CFE_APP, to_lab,  TO_LAB_AppMain,  TO_LAB,  45, 32768, 0x0, 0;")
    set (STARTUP_ORDER ost_common sch_lab ci_lab to_lab)

    file (WRITE ${STARTUP_FILE} "")
    foreach (APP ${STARTUP_ORDER})
        list (FIND ARGN ${APP} SHOULD_START)
        if (SHOULD_START GREATER -1)
            file (APPEND ${STARTUP_FILE} "${STARTUP_${APP}}\n")
        endif ()
    endforeach ()

    # Every startup script ends with an end-of-file marker
    file (APPEND ${STARTUP_FILE} "!\n")

endfunction (generate_cfs_startup_script)
