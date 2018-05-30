#ifndef INDRA_HEADS_DRIVER_HPP
#define INDRA_HEADS_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include <indra_heads_protocol/RequestedConfiguration.hpp>
#include <indra_heads_protocol/Response.hpp>

namespace indra_heads_protocol
{
    class Driver : public iodrivers_base::Driver
    {
        std::vector<uint8_t> mWriteBuffer;
        std::vector<uint8_t> mReadBuffer;
        RequestedConfiguration mRequestedConfiguration;

    protected:
        int extractPacket(uint8_t const* buffer, size_t buffer_size) const;
        bool mStabilized = true;

    public:
        /** Exception thrown from the getters that allow to access the command
         * details
         */
        class InvalidState : public std::runtime_error { };

        Driver();

        /** Write a request
         *
         * Build the request packet itself using the functions
         * in indra_heads_protocol::requests
         */
        template<typename T>
        void sendRequest(T const& packet)
        {
            mWriteBuffer = requests::packetize(packet);
            writePacket(mWriteBuffer.data(), mWriteBuffer.size());
        }

        /** Read a command and return which command was received
         *
         * This internally updates the requested configuration that can be
         * accessed with getRequestedCommand()
         */
        CommandIDs readRequest();

        /** Send a response packet
         */
        void writeResponse(Response response);

        /** Read a response packet and return the status
         */
        Response readResponse();

        /** Returns the current requested configuration
         */
        RequestedConfiguration getRequestedConfiguration() const;
    };
}

#endif
