# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(fast_lio_sam_CONFIG_INCLUDED)
  return()
endif()
set(fast_lio_sam_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(fast_lio_sam_SOURCE_PREFIX /home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM)
  set(fast_lio_sam_DEVEL_PREFIX /home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/build/devel)
  set(fast_lio_sam_INSTALL_PREFIX "")
  set(fast_lio_sam_PREFIX ${fast_lio_sam_DEVEL_PREFIX})
else()
  set(fast_lio_sam_SOURCE_PREFIX "")
  set(fast_lio_sam_DEVEL_PREFIX "")
  set(fast_lio_sam_INSTALL_PREFIX /usr/local)
  set(fast_lio_sam_PREFIX ${fast_lio_sam_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'fast_lio_sam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(fast_lio_sam_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/build/devel/include;/usr/include/eigen3;/usr/include/pcl-1.8;/usr/include;/usr/include/ni;/usr/include/openni2;/usr/include/vtk-6.3;/usr/include/freetype2;/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi;/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent;/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent/include;/usr/lib/x86_64-linux-gnu/openmpi/include;/usr/include/python2.7;/usr/include/x86_64-linux-gnu;/usr/include/hdf5/openmpi;/usr/include/libxml2;/usr/include/jsoncpp;/usr/include/tcl " STREQUAL " ")
  set(fast_lio_sam_INCLUDE_DIRS "")
  set(_include_dirs "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/build/devel/include;/usr/include/eigen3;/usr/include/pcl-1.8;/usr/include;/usr/include/ni;/usr/include/openni2;/usr/include/vtk-6.3;/usr/include/freetype2;/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi;/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent;/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/opal/mca/event/libevent2022/libevent/include;/usr/lib/x86_64-linux-gnu/openmpi/include;/usr/include/python2.7;/usr/include/x86_64-linux-gnu;/usr/include/hdf5/openmpi;/usr/include/libxml2;/usr/include/jsoncpp;/usr/include/tcl")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'claydergc <dev@livoxtech.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${fast_lio_sam_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'fast_lio_sam' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'fast_lio_sam' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/${idir}'.  ${_report}")
    endif()
    _list_append_unique(fast_lio_sam_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "optimized;/usr/lib/x86_64-linux-gnu/libpcl_common.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_common.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_octree.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_octree.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_io.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_io.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_search.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_search.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_filters.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_filters.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_features.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_features.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_ml.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_ml.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_segmentation.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_segmentation.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_visualization.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_visualization.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_surface.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_surface.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_registration.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_registration.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_keypoints.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_keypoints.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_tracking.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_tracking.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_recognition.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_recognition.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_stereo.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_stereo.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_apps.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_apps.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_outofcore.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_outofcore.so;optimized;/usr/lib/x86_64-linux-gnu/libpcl_people.so;debug;/usr/lib/x86_64-linux-gnu/libpcl_people.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/libOpenNI.so;/usr/lib/libOpenNI2.so;optimized;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;debug;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;/usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtksys-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkalglib-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libfreetype.so;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libvtkftgl-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOXML-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOCore-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libexpat.so;/usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkParallelCore-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkParallelMPI-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersPython-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libpython2.7.so;/usr/lib/x86_64-linux-gnu/libvtkWrappingPython27Core-6.3.so.6.3.0;/usr/lib/libvtkWrappingTools-6.3.a;/usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersSMP-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkverdict-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOImage-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkmetaio-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libtiff.so;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQtSQL-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOSQL-6.3.so.6.3.0;sqlite3;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQtWebkit-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkViewsQt-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkGeovisCore-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libproj.so;/usr/lib/x86_64-linux-gnu/libvtkIOAMR-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5.so;/usr/lib/x86_64-linux-gnu/libsz.so;/usr/lib/x86_64-linux-gnu/libdl.so;/usr/lib/x86_64-linux-gnu/libm.so;/usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so;/usr/lib/x86_64-linux-gnu/libvtkIOEnSight-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOExodus-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkexoIIc-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libnetcdf_c++.so;/usr/lib/x86_64-linux-gnu/libnetcdf.so;/usr/lib/x86_64-linux-gnu/libvtkIOExport-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PS-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libgl2ps.so;/usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOMovie-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libtheoraenc.so;/usr/lib/x86_64-linux-gnu/libtheoradec.so;/usr/lib/x86_64-linux-gnu/libogg.so;/usr/lib/x86_64-linux-gnu/libvtkIOGDAL-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOGeoJSON-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOImport-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOInfovis-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libxml2.so;/usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOMINC-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOParallel-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIONetCDF-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libjsoncpp.so;/usr/lib/x86_64-linux-gnu/libvtkIOMySQL-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOODBC-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOParallelXML-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOVPIC-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkVPIC-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOVideo-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkxdmf2-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkImagingMath-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkImagingStencil-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkInteractionImage-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkLocalExample-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkParallelMPI4Py-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingExternal-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingImage-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingLIC-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingParallelLIC-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeAMR-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkTestingRendering-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-6.3.so.6.3.0;/usr/lib/x86_64-linux-gnu/libvtkWrappingJava-6.3.so.6.3.0")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND fast_lio_sam_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND fast_lio_sam_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT fast_lio_sam_NUM_DUMMY_TARGETS)
      set(fast_lio_sam_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::fast_lio_sam::wrapped-linker-option${fast_lio_sam_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR fast_lio_sam_NUM_DUMMY_TARGETS "${fast_lio_sam_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::fast_lio_sam::wrapped-linker-option${fast_lio_sam_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND fast_lio_sam_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND fast_lio_sam_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND fast_lio_sam_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/build/devel/lib;/home/lory/catkin_ws/devel/lib;/opt/ros/melodic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(fast_lio_sam_LIBRARY_DIRS ${lib_path})
      list(APPEND fast_lio_sam_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'fast_lio_sam'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND fast_lio_sam_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(fast_lio_sam_EXPORTED_TARGETS "fast_lio_sam_generate_messages_cpp;fast_lio_sam_generate_messages_eus;fast_lio_sam_generate_messages_lisp;fast_lio_sam_generate_messages_nodejs;fast_lio_sam_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${fast_lio_sam_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "geometry_msgs;nav_msgs;roscpp;rospy;std_msgs;message_runtime")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 fast_lio_sam_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${fast_lio_sam_dep}_FOUND)
      find_package(${fast_lio_sam_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${fast_lio_sam_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(fast_lio_sam_INCLUDE_DIRS ${${fast_lio_sam_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(fast_lio_sam_LIBRARIES ${fast_lio_sam_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${fast_lio_sam_dep}_LIBRARIES})
  _list_append_deduplicate(fast_lio_sam_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(fast_lio_sam_LIBRARIES ${fast_lio_sam_LIBRARIES})

  _list_append_unique(fast_lio_sam_LIBRARY_DIRS ${${fast_lio_sam_dep}_LIBRARY_DIRS})
  list(APPEND fast_lio_sam_EXPORTED_TARGETS ${${fast_lio_sam_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "fast_lio_sam-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${fast_lio_sam_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
