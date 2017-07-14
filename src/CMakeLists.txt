rock_library(indra_heads_protocol
    SOURCES Protocol.cpp Driver.cpp
    HEADERS Protocol.hpp Driver.hpp RequestedConfiguration.hpp
    DEPS_PKGCONFIG eigen3 iodrivers_base)

