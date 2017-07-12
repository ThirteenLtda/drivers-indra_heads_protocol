#include <boost/test/unit_test.hpp>
#include <indra_heads_protocol/Dummy.hpp>

using namespace indra_heads_protocol;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    indra_heads_protocol::DummyClass dummy;
    dummy.welcome();
}
