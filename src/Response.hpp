#ifndef INDRA_HEADS_PROTOCOL_RESPONSE_HPP
#define INDRA_HEADS_PROTOCOL_RESPONSE_HPP

#include <indra_heads_protocol/Protocol.hpp>

namespace indra_heads_protocol
{
    struct Response
    {
        CommandIDs command_id;
        ResponseStatus status;
    };
}

#endif