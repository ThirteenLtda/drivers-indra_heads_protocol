#include <indra_heads_protocol/Driver.hpp>
#include <iostream>

using namespace std;
using namespace indra_heads_protocol;

void usage()
{
    std::cout 
        << "usage: indra_heads_protocol_cmd URI CMD [ARGS]\n"
        << "\n"
        << "indra_heads_protocol_cmd URI stop\n"
        << "  stops all operations of the stabilization platform\n"
        << "\n"
        << "indra_heads_protocol_cmd URI self-test\n"
        << "  perform a self-test\n"
        << "\n"
        << "indra_heads_protocol_cmd URI target LATITUDE LONGITUDE ALTITUDE\n"
        << "  provides the pointing target\n"
        << "\n"
        << "indra_heads_protocol_cmd URI stabilize\n"
        << "  enable stabilization"
        << std::endl;
}
void verify_argc_atleast(int expected, int actual)
{
    if (expected > actual) {
        usage();
        exit(1);
    }
}
void verify_argc(int expected, int actual)
{
    if (expected != actual) {
        usage();
        exit(1);
    }
}

template<typename T>
ResponseStatus request(Driver& driver, T const& packet)
{
    driver.sendRequest(packet);
    while(true)
    {
        Response response = driver.readResponse();
        if (response.command_id == packet.command_id)
            return response.status;
    }
}

int main(int argc, char** argv)
{
    verify_argc_atleast(2, argc);

    string uri = argv[1];
    Driver driver;
    driver.setReadTimeout(base::Time::fromSeconds(10));
    driver.setWriteTimeout(base::Time::fromSeconds(10));
    driver.openURI(uri);

    string cmd = argv[2];

    if (cmd == "stop") {
        verify_argc(3, argc);
        request(driver, requests::Deploy());
    }
    else if (cmd == "self-test") {
        verify_argc(3, argc);
        request(driver, requests::BITE());
    }
    else if (cmd == "target") {
        verify_argc(6, argc);
        double latitude  = stod(argv[3]);
        double longitude = stod(argv[4]);
        double altitude  = stod(argv[5]);
        request(driver, requests::StabilizationTarget(latitude, longitude, altitude));
    }
    return 0;
}