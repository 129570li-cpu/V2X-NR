set(name traci)

set(source_files
    model/traci-client.cc
    model/sumo-socket.cc
    model/sumo-storage.cc
    model/sumo-TraCIAPI.cc
)

set(header_files
    model/traci-client.h
    model/sumo-TraCIAPI.h
    model/sumo-config.h
    model/sumo-socket.h
    model/sumo-storage.h
    model/sumo-TraCIConstants.h
    model/sumo-TraCIDefs.h
)

set(libraries_to_link
    ${libcore}
    ${libmobility}
    ${libinternet}
)

build_lib(
    LIBNAME ${name}
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK ${libraries_to_link}
)
