#include <boost/test/unit_test.hpp>
#include <gazebo_underwater/Dummy.hpp>

using namespace gazebo_underwater;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    gazebo_underwater::DummyClass dummy;
    dummy.welcome();
}
