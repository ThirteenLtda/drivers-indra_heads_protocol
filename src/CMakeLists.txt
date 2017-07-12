rock_library(indra_heads_protocol
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(indra_heads_protocol_bin Main.cpp
    DEPS indra_heads_protocol)

