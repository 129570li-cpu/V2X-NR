set(name elite)

set(source_files
    model/elite-controller.cc
    model/elite-q-table.cc
    model/elite-reward-model.cc
    model/elite-trainer.cc
    model/sumo-road-graph-loader.cc
    model/twin-environment.cc
)

set(header_files
    model/elite-controller.h
    model/elite-q-table.h
    model/elite-reward-model.h
    model/elite-trainer.h
    model/sumo-road-graph-loader.h
    model/twin-environment.h
)

set(xml2_libraries)
if(${LIBXML2_FOUND})
  set(xml2_libraries
      ${LIBXML2_LIBRARIES}
  )
endif()

set(libraries_to_link
    ${libcore}
    ${libmobility}
    ${libnetwork}
    ${xml2_libraries}
)

build_lib(
    LIBNAME ${name}
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK ${libraries_to_link}
)

build_lib_example(
    NAME sl-80211p-elite-example
    SOURCE_FILES examples/sl-80211p-elite-example.cc
    LIBRARIES_TO_LINK
        ${libelite}
        ${libgpsr}
        ${libtraci}
        ${libwifi}
        ${libapplications}
        ${libinternet}
)
