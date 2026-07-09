# Compatibility shim for CMake 4.x + OpenMPI on restricted environments.
#
# VTK, pulled in by Ubuntu's PCL package, calls find_package(MPI). CMake 4.x
# can mis-parse OpenMPI wrapper output when the wrapper prints network probing
# warnings, then fails configuration before this project code is compiled.
# OpenMPI's pkg-config files provide the same compile/link metadata without
# running those wrapper probes.

find_package(PkgConfig QUIET)

if(PkgConfig_FOUND)
  pkg_check_modules(OMPI_C QUIET ompi-c)
  pkg_check_modules(OMPI_CXX QUIET ompi-cxx)
endif()

set(MPI_C_FOUND FALSE)
set(MPI_CXX_FOUND FALSE)
set(MPI_FOUND FALSE)

if(OMPI_C_FOUND)
  set(MPI_C_FOUND TRUE)
  set(MPI_C_INCLUDE_DIRS ${OMPI_C_INCLUDE_DIRS})
  set(MPI_C_LIBRARIES ${OMPI_C_LINK_LIBRARIES})
  if(NOT MPI_C_LIBRARIES)
    set(MPI_C_LIBRARIES ${OMPI_C_LDFLAGS})
  endif()

  if(NOT TARGET MPI::MPI_C)
    add_library(MPI::MPI_C INTERFACE IMPORTED)
    set_target_properties(MPI::MPI_C PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${MPI_C_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${MPI_C_LIBRARIES}"
    )
  endif()
endif()

if(OMPI_CXX_FOUND)
  set(MPI_CXX_FOUND TRUE)
  set(MPI_CXX_INCLUDE_DIRS ${OMPI_CXX_INCLUDE_DIRS})
  set(MPI_CXX_LIBRARIES ${OMPI_CXX_LINK_LIBRARIES})
  if(NOT MPI_CXX_LIBRARIES)
    set(MPI_CXX_LIBRARIES ${OMPI_CXX_LDFLAGS})
  endif()

  if(NOT TARGET MPI::MPI_CXX)
    add_library(MPI::MPI_CXX INTERFACE IMPORTED)
    set_target_properties(MPI::MPI_CXX PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${MPI_CXX_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${MPI_CXX_LIBRARIES}"
    )
  endif()
endif()

if(MPI_C_FOUND OR MPI_CXX_FOUND)
  set(MPI_FOUND TRUE)
endif()

find_program(MPIEXEC_EXECUTABLE NAMES mpiexec mpirun)
set(MPIEXEC_NUMPROC_FLAG "-n")
set(MPIEXEC_MAX_NUMPROCS 1)
set(MPIEXEC_PREFLAGS "")
set(MPIEXEC_POSTFLAGS "")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MPI
  REQUIRED_VARS MPI_FOUND
  HANDLE_COMPONENTS
)
