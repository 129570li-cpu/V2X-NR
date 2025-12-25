set(name traci-applications)

set(source_files
    model/traffic-control-app.cc
    helper/traffic-control-app-helper.cc
)

set(header_files
    model/traffic-control-app.h
    helper/traffic-control-app-helper.h
)

set(libraries_to_link
    ${libcore}
    ${libwifi}
    ${libapplications}
    ${libmobility}
    ${libinternet}
    ${libconfig-store}
    ${libstats}
    ${libtraci}
)

build_lib(
    LIBNAME ${name}
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK ${libraries_to_link}
)

# Add example
build_lib_example(
    NAME ns3-sumo-coupling-simple
    SOURCE_FILES examples/ns3-sumo-coupling-simple.cc
    LIBRARIES_TO_LINK
        ${libtraci-applications}
        ${libtraci}
        ${libcore}
        ${libinternet}
        ${libwifi}
        ${libmobility}
        ${libnetwork}
        ${libwave}
        ${libnetanim}
)
