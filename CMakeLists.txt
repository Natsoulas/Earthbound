cmake_minimum_required(VERSION 3.5)
project(Earthbound)

# Enable testing
include(CTest)
enable_testing()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add local Eigen
add_subdirectory(lib/eigen)

# Find GTest package with proper paths for macOS
if(APPLE)
    set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/homebrew/Cellar/googletest/1.15.0")
endif()
find_package(GTest REQUIRED)

# Create erfa_export.h if it doesn't exist
file(WRITE ${CMAKE_SOURCE_DIR}/include/astro/erfa/erfa_export.h
"#ifndef ERFA_EXPORT_H
#define ERFA_EXPORT_H

#ifdef _WIN32
    #ifdef BUILDING_ERFA
        #define ERFA_EXPORT __declspec(dllexport)
    #else
        #define ERFA_EXPORT __declspec(dllimport)
    #endif
#else
    #define ERFA_EXPORT
#endif

#endif // ERFA_EXPORT_H
")

# Create ERFA library from source
add_library(erfa STATIC
    include/astro/erfa/a2af.c
    include/astro/erfa/a2tf.c
    include/astro/erfa/ab.c
    include/astro/erfa/af2a.c
    include/astro/erfa/anp.c
    include/astro/erfa/anpm.c
    include/astro/erfa/apcg13.c
    include/astro/erfa/apcg.c
    include/astro/erfa/apci13.c
    include/astro/erfa/apci.c
    include/astro/erfa/apco13.c
    include/astro/erfa/apco.c
    include/astro/erfa/apcs13.c
    include/astro/erfa/apcs.c
    include/astro/erfa/aper13.c
    include/astro/erfa/aper.c
    include/astro/erfa/apio13.c
    include/astro/erfa/apio.c
    include/astro/erfa/atci13.c
    include/astro/erfa/atciq.c
    include/astro/erfa/atciqn.c
    include/astro/erfa/atciqz.c
    include/astro/erfa/atco13.c
    include/astro/erfa/atic13.c
    include/astro/erfa/aticq.c
    include/astro/erfa/aticqn.c
    include/astro/erfa/atio13.c
    include/astro/erfa/atioq.c
    include/astro/erfa/atoc13.c
    include/astro/erfa/atoi13.c
    include/astro/erfa/atoiq.c
    include/astro/erfa/bi00.c
    include/astro/erfa/bp00.c
    include/astro/erfa/bp06.c
    include/astro/erfa/bpn2xy.c
    include/astro/erfa/c2i00a.c
    include/astro/erfa/c2i00b.c
    include/astro/erfa/c2i06a.c
    include/astro/erfa/c2ibpn.c
    include/astro/erfa/c2ixy.c
    include/astro/erfa/c2ixys.c
    include/astro/erfa/c2s.c
    include/astro/erfa/c2t00a.c
    include/astro/erfa/c2t00b.c
    include/astro/erfa/c2t06a.c
    include/astro/erfa/c2tcio.c
    include/astro/erfa/c2teqx.c
    include/astro/erfa/c2tpe.c
    include/astro/erfa/c2txy.c
    include/astro/erfa/cal2jd.c
    include/astro/erfa/cp.c
    include/astro/erfa/cpv.c
    include/astro/erfa/cr.c
    include/astro/erfa/d2dtf.c
    include/astro/erfa/d2tf.c
    include/astro/erfa/dat.c
    include/astro/erfa/dtdb.c
    include/astro/erfa/dtf2d.c
    include/astro/erfa/eceq06.c
    include/astro/erfa/ee00.c
    include/astro/erfa/ee00a.c
    include/astro/erfa/ee00b.c
    include/astro/erfa/ee06a.c
    include/astro/erfa/eect00.c
    include/astro/erfa/eo06a.c
    include/astro/erfa/epb.c
    include/astro/erfa/epb2jd.c
    include/astro/erfa/epj.c
    include/astro/erfa/epj2jd.c
    include/astro/erfa/epv00.c
    include/astro/erfa/eqeq94.c
    include/astro/erfa/era00.c
    include/astro/erfa/fad03.c
    include/astro/erfa/fae03.c
    include/astro/erfa/faf03.c
    include/astro/erfa/faju03.c
    include/astro/erfa/fal03.c
    include/astro/erfa/falp03.c
    include/astro/erfa/fama03.c
    include/astro/erfa/fame03.c
    include/astro/erfa/fane03.c
    include/astro/erfa/faom03.c
    include/astro/erfa/fapa03.c
    include/astro/erfa/fasa03.c
    include/astro/erfa/faur03.c
    include/astro/erfa/fave03.c
    include/astro/erfa/fk52h.c
    include/astro/erfa/fk5hip.c
    include/astro/erfa/fk5hz.c
    include/astro/erfa/fw2m.c
    include/astro/erfa/fw2xy.c
    include/astro/erfa/g2icrs.c
    include/astro/erfa/gc2gd.c
    include/astro/erfa/gc2gde.c
    include/astro/erfa/gd2gc.c
    include/astro/erfa/gd2gce.c
    include/astro/erfa/gmst00.c
    include/astro/erfa/gmst06.c
    include/astro/erfa/gmst82.c
    include/astro/erfa/gst00a.c
    include/astro/erfa/gst00b.c
    include/astro/erfa/gst06.c
    include/astro/erfa/gst06a.c
    include/astro/erfa/gst94.c
    include/astro/erfa/h2fk5.c
    include/astro/erfa/hfk5z.c
    include/astro/erfa/icrs2g.c
    include/astro/erfa/ir.c
    include/astro/erfa/jd2cal.c
    include/astro/erfa/jdcalf.c
    include/astro/erfa/ld.c
    include/astro/erfa/ldn.c
    include/astro/erfa/ldsun.c
    include/astro/erfa/lteceq.c
    include/astro/erfa/ltecm.c
    include/astro/erfa/lteqec.c
    include/astro/erfa/ltp.c
    include/astro/erfa/ltpb.c
    include/astro/erfa/ltpecl.c
    include/astro/erfa/ltpequ.c
    include/astro/erfa/num00a.c
    include/astro/erfa/num00b.c
    include/astro/erfa/num06a.c
    include/astro/erfa/numat.c
    include/astro/erfa/nut00a.c
    include/astro/erfa/nut00b.c
    include/astro/erfa/nut06a.c
    include/astro/erfa/nut80.c
    include/astro/erfa/nutm80.c
    include/astro/erfa/obl06.c
    include/astro/erfa/obl80.c
    include/astro/erfa/p06e.c
    include/astro/erfa/p2pv.c
    include/astro/erfa/p2s.c
    include/astro/erfa/pap.c
    include/astro/erfa/pas.c
    include/astro/erfa/pb06.c
    include/astro/erfa/pdp.c
    include/astro/erfa/pfw06.c
    include/astro/erfa/plan94.c
    include/astro/erfa/pm.c
    include/astro/erfa/pmat00.c
    include/astro/erfa/pmat06.c
    include/astro/erfa/pmat76.c
    include/astro/erfa/pmp.c
    include/astro/erfa/pmpx.c
    include/astro/erfa/pmsafe.c
    include/astro/erfa/pn.c
    include/astro/erfa/pn00.c
    include/astro/erfa/pn00a.c
    include/astro/erfa/pn00b.c
    include/astro/erfa/pn06.c
    include/astro/erfa/pn06a.c
    include/astro/erfa/pnm00a.c
    include/astro/erfa/pnm00b.c
    include/astro/erfa/pnm06a.c
    include/astro/erfa/pnm80.c
    include/astro/erfa/pom00.c
    include/astro/erfa/ppp.c
    include/astro/erfa/ppsp.c
    include/astro/erfa/pr00.c
    include/astro/erfa/prec76.c
    include/astro/erfa/pv2p.c
    include/astro/erfa/pv2s.c
    include/astro/erfa/pvdpv.c
    include/astro/erfa/pvm.c
    include/astro/erfa/pvmpv.c
    include/astro/erfa/pvppv.c
    include/astro/erfa/pvstar.c
    include/astro/erfa/pvtob.c
    include/astro/erfa/pvu.c
    include/astro/erfa/pvup.c
    include/astro/erfa/pvxpv.c
    include/astro/erfa/pxp.c
    include/astro/erfa/refco.c
    include/astro/erfa/rm2v.c
    include/astro/erfa/rv2m.c
    include/astro/erfa/rx.c
    include/astro/erfa/rxp.c
    include/astro/erfa/rxpv.c
    include/astro/erfa/rxr.c
    include/astro/erfa/ry.c
    include/astro/erfa/rz.c
    include/astro/erfa/s00.c
    include/astro/erfa/s00a.c
    include/astro/erfa/s00b.c
    include/astro/erfa/s06.c
    include/astro/erfa/s06a.c
    include/astro/erfa/s2c.c
    include/astro/erfa/s2p.c
    include/astro/erfa/s2pv.c
    include/astro/erfa/s2xpv.c
    include/astro/erfa/sepp.c
    include/astro/erfa/seps.c
    include/astro/erfa/sp00.c
    include/astro/erfa/starpm.c
    include/astro/erfa/starpv.c
    include/astro/erfa/sxp.c
    include/astro/erfa/sxpv.c
    include/astro/erfa/taitt.c
    include/astro/erfa/taiut1.c
    include/astro/erfa/taiutc.c
    include/astro/erfa/tcbtdb.c
    include/astro/erfa/tcgtt.c
    include/astro/erfa/tdbtcb.c
    include/astro/erfa/tdbtt.c
    include/astro/erfa/tf2a.c
    include/astro/erfa/tf2d.c
    include/astro/erfa/tr.c
    include/astro/erfa/trxp.c
    include/astro/erfa/trxpv.c
    include/astro/erfa/tttai.c
    include/astro/erfa/tttcg.c
    include/astro/erfa/tttdb.c
    include/astro/erfa/ttut1.c
    include/astro/erfa/ut1tai.c
    include/astro/erfa/ut1tt.c
    include/astro/erfa/ut1utc.c
    include/astro/erfa/utctai.c
    include/astro/erfa/utcut1.c
    include/astro/erfa/xy06.c
    include/astro/erfa/xys00a.c
    include/astro/erfa/xys00b.c
    include/astro/erfa/xys06a.c
    include/astro/erfa/zp.c
    include/astro/erfa/zpv.c
    include/astro/erfa/zr.c
)

target_include_directories(erfa PUBLIC
    ${CMAKE_SOURCE_DIR}/include/astro/erfa
    ${CMAKE_SOURCE_DIR}/include/astro
)

# Set ERFA include directory
set(ERFA_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/astro/erfa)

# Add the executable target
add_executable(Earthbound src/main.cpp)

# Include directories for Earthbound
target_include_directories(Earthbound PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
    ${ERFA_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/lib/eigen
)

# Link libraries for Earthbound
target_link_libraries(Earthbound PRIVATE
    Eigen3::Eigen
    erfa
)

# Add test directory
add_subdirectory(test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -march=native")

if(WIN32)
    target_compile_definitions(erfa PRIVATE BUILDING_ERFA)
    target_compile_definitions(Earthbound PRIVATE USING_ERFA) 
endif()