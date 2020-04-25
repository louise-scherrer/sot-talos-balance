#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_dummy) {
  BOOST_CHECK(true);
  std::cout << "It works!" << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
