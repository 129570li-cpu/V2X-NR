set(name gpsr)

set(source_files
    model/gpsr.cc
    model/gpsr-ptable.cc
    model/gpsr-rqueue.cc
    model/gpsr-packet.cc
    helper/gpsr-helper.cc
)

set(header_files
    model/gpsr.h
    model/gpsr-ptable.h
    model/gpsr-rqueue.h
    model/gpsr-packet.h
    helper/gpsr-helper.h
)

set(libraries_to_link
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libmobility}
    ${liblocation-service}
    ${libwifi}
    ${libapplications}
)

build_lib(
    LIBNAME ${name}
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK ${libraries_to_link}
)

build_lib_example(
    NAME gpsr-test
    SOURCE_FILES examples/gpsr-test.cc
    LIBRARIES_TO_LINK
        ${libgpsr}
        ${libcore}
        ${libnetwork}
        ${libinternet}
        ${libmobility}
        ${libwifi}
        ${libapplications}
        ${liblocation-service}
)
