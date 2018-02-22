rock_library(indra_heads_protocol
    SOURCES Protocol.cpp Driver.cpp
    HEADERS Protocol.hpp Driver.hpp RequestedConfiguration.hpp Response.hpp
    DEPS_PKGCONFIG eigen3 iodrivers_base)

rock_executable(indra_heads_protocol_cmd
    SOURCES Main.cpp
    DEPS indra_heads_protocol)
