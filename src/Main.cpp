#include <indra_heads_protocol/Driver.hpp>
#include <iodrivers_base/IOStream.hpp>
#include <iostream>
#include <string>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>

using namespace std;
using namespace indra_heads_protocol;

void usage()
{
    std::cout
        << "usage: indra_heads_protocol_cmd PORT\n"
        << std::endl;
}

void commands()
{
    std::cout
        << "\n"
        << "stop\n"
        << "  stops all operations of the stabilization platform\n"
        << "\n"
        << "self-test\n"
        << "  perform a self-test\n"
        << "\n"
        << "target LATITUDE LONGITUDE ALTITUDE\n"
        << "  provides the pointing target\n"
        << "\n"
        << "stabilize\n"
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

static const int STATUS_TIMEOUT = 10;

template<typename T>
int request(Driver& driver, T const& packet)
{
    driver.sendRequest(packet);
    while(true)
    {
        try {
            Response response = driver.readResponse();
            if (response.command_id == packet.command_id)
                return response.status;
        }
        catch(iodrivers_base::TimeoutError) {
            return STATUS_TIMEOUT;
        }
    }
}

void displayResponse(int status)
{
    if (status == STATUS_OK)
        std::cout << "OK" << std::endl;
    else if (status == STATUS_FAILED)
        std::cout << "Failed" << std::endl;
    else if (status == STATUS_UNSUPPORTED)
        std::cout << "Unsupported" << std::endl;
    else if (status == STATUS_TIMEOUT)
        std::cout << "Timeout" << std::endl;
}

Rates rate_from_arg(std::string const& arg) {
    if (arg == "disable") {
        return RATE_DISABLED;
    }
    else if (arg == "10") {
        return RATE_10HZ;
    }
    else if (arg == "20") {
        return RATE_20HZ;
    }
    else if (arg == "50") {
        return RATE_50HZ;
    }
    else {
        throw std::invalid_argument("unknown data rate " + arg + " known values are disable, 10, 20 and 50");
    }
}

sockaddr_in getipa(const char* hostname, int port){
	sockaddr_in ipa;
	ipa.sin_family = AF_INET;
	ipa.sin_port = htons(port);

	struct hostent* localhost = gethostbyname(hostname);
	if (!localhost)
        throw std::runtime_error("cannot resolve requested hostname");


	char* addr = localhost->h_addr_list[0];
	memcpy(&ipa.sin_addr.s_addr, addr, sizeof(ipa));

	return ipa;
}

string ask(std::string prompt)
{
    string cmd;
    std::cout << prompt << " " << std::flush;
    std::cin >> cmd;
    return cmd;
}

int main(int argc, char** argv)
{
    verify_argc_atleast(2, argc);

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int enable = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    {
        std::cerr << "setsockopt(SO_REUSEADDR) failed" << std::endl;
        std::cerr << strerror(errno) << std::endl;
        return 1;
    }
    sockaddr_in server_addr = getipa("0.0.0.0", std::stol(argv[1]));
    int result = bind(server_fd, (sockaddr*)&server_addr, sizeof(server_addr));
    if (result < 0)
    {
        std::cerr << strerror(errno) << std::endl;
        return 1;
    }

    int client_fd = -1;
    while (client_fd < 0)
    {
        std::cout << "Waiting for connection on port " << argv[1] << std::endl;
        if (listen(server_fd, 1) == -1)
        {
            std::cerr << strerror(errno) << std::endl;
            continue;
        }

        client_fd = accept(server_fd, nullptr, nullptr);
        if (client_fd == -1)
        {
            std::cerr << strerror(errno) << std::endl;
            usleep(100000);
        }
    }

    close(server_fd);

    Driver driver;
    driver.setMainStream(new iodrivers_base::FDStream(client_fd, true));
    driver.setReadTimeout(base::Time::fromSeconds(10));
    driver.setWriteTimeout(base::Time::fromSeconds(10));

    while(true)
    {
        string cmd = ask("Command ?");
        if (cmd == "stop") {
            displayResponse(request(driver, requests::Stop()));
        }
        else if (cmd == "self-test") {
            displayResponse(request(driver, requests::BITE()));
        }
        else if (cmd == "rate-imu") {
            string rate = ask("Rate ?");
            Rates target_rate = rate_from_arg(rate);
            displayResponse(request(driver, requests::StatusRefreshRateIMU(target_rate)));
        }
        else if (cmd == "rate-pt") {
            string rate = ask("Rate ?");
            Rates target_rate = rate_from_arg(argv[3]);
            displayResponse(request(driver, requests::StatusRefreshRatePT(target_rate)));
        }
        else if (cmd == "target") {
            string latitude_s  = ask("Lat  ?");
            string longitude_s = ask("Long ?");
            string altitude_s  = ask("Alt  ?");
            double latitude  = stod(latitude_s);
            double longitude = stod(longitude_s);
            double altitude  = stod(altitude_s);
            displayResponse(request(driver, requests::StabilizationTarget(latitude, longitude, altitude)));
        }
        else {
            commands();
        }
    }
    return 0;
}