set(name location-service)

set(source_files
    model/location-service.cc
    model/god.cc
)

set(header_files
    model/location-service.h
    model/god.h
)

set(libraries_to_link
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libmobility}
)

build_lib(
    LIBNAME ${name}
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK ${libraries_to_link}
)
