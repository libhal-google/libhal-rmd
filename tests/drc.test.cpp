#include <libhal-rmd/drc.hpp>

#include <libhal-mock/can.hpp>

#include <boost/ut.hpp>

namespace hal::rmd {
namespace {
constexpr can::id_t expected_id = 0x140;
constexpr can::id_t expected_gear_ratio = 6.0f;

template<size_t N>
constexpr auto prefilled_messages(hal::byte p_command_byte = 0x00)
{
  std::array<can::message_t, N> prefilled_expected_messages{};
  for (auto& message : prefilled_expected_messages) {
    message.id = expected_id;
    message.length = 8;
    message.payload.fill(0x00);
    message.payload[0] = p_command_byte;
  }
  return prefilled_expected_messages;
};
}  // namespace

void drc_test()
{
  using namespace boost::ut;

  "drc::create()"_test = []() {
    // Setup
    mock::can mock_can;
    auto router = hal::can_router::create(mock_can).value();
    auto expected = prefilled_messages<2>();
    expected[0].payload[0] = hal::value(drc::system::off);
    expected[1].payload[0] = hal::value(drc::system::running);

    // Exercise
    auto driver = drc::create(router, expected_gear_ratio, expected_id).value();

    // Verify
    expect(that % 2 == mock_can.spy_send.call_history().size());
    expect(that % expected[0] == mock_can.spy_send.history<0>(0));
    expect(that % expected[1] == mock_can.spy_send.history<0>(1));
  };

  "drc::create() failure"_test = []() {
    // Setup
    mock::can mock_can;
    auto router = hal::can_router::create(mock_can).value();
    mock_can.spy_send.trigger_error_on_call(1);

    constexpr can::message_t expected1{
      .id = expected_id,
      .payload = { hal::value(drc::system::off),
                   0x00,
                   0x00,
                   0x00,
                   0x00,
                   0x00,
                   0x00,
                   0x00 },
      .length = 8,
    };

    // Exercise
    auto driver = drc::create(router, expected_gear_ratio, expected_id);

    // Verify
    expect(that % 1 == mock_can.spy_send.call_history().size());
    expect(that % expected1 == mock_can.spy_send.history<0>(0));
  };

  "drc::velocity_control()"_test = []() {
    // Setup
    mock::can mock_can;
    auto router = hal::can_router::create(mock_can).value();
    auto driver = drc::create(router, expected_gear_ratio, expected_id).value();
    mock_can.reset();
    auto expected = prefilled_messages<7>(hal::value(drc::actuate::speed));
    std::array injected_rpm{
      0.0_rpm, 10.0_rpm, 10.0_rpm, 123.0_rpm, 0.0_rpm, 1024.0_rpm,
    };

    using payload_t = decltype(expected[0].payload);

    // Setup: values verified to be correct
    expected[1].payload = payload_t{
      0xa2, 0x0, 0x0, 0x0, 0xa0, 0x8c, 0x0, 0x0,
    };
    expected[2].payload = payload_t{
      0xa2, 0x0, 0x0, 0x0, 0xa0, 0x8c, 0x0, 0x0,
    };
    expected[3].payload = payload_t{
      0xa2, 0x0, 0x0, 0x0, 0xb0, 0xc1, 0x6, 0x0,
    };
    expected[4].payload = payload_t{
      0xa2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    };
    expected[5].payload = payload_t{
      0xa2, 0x0, 0x0, 0x0, 0x0, 0x40, 0x38, 0x0,
    };

    // Exercise
    auto result0 = driver.velocity_control(injected_rpm[0]);
    auto result1 = driver.velocity_control(injected_rpm[1]);
    auto result2 = driver.velocity_control(injected_rpm[2]);
    auto result3 = driver.velocity_control(injected_rpm[3]);
    auto result4 = driver.velocity_control(injected_rpm[4]);
    auto result5 = driver.velocity_control(injected_rpm[5]);

    // Verify
    expect(bool{ result0 });
    expect(bool{ result1 });
    expect(bool{ result2 });
    expect(bool{ result3 });
    expect(bool{ result4 });
    expect(bool{ result5 });

    expect(that % 6 == mock_can.spy_send.call_history().size());
    expect(that % expected[0] == mock_can.spy_send.history<0>(0));
    expect(that % expected[1] == mock_can.spy_send.history<0>(1));
    expect(that % expected[2] == mock_can.spy_send.history<0>(2));
    expect(that % expected[3] == mock_can.spy_send.history<0>(3));
    expect(that % expected[4] == mock_can.spy_send.history<0>(4));
    expect(that % expected[5] == mock_can.spy_send.history<0>(5));
  };

  "drc::velocity_control() fails out of bounds"_test = []() {
    // Setup
    mock::can mock_can;
    auto router = hal::can_router::create(mock_can).value();
    auto driver = drc::create(router, expected_gear_ratio, expected_id).value();
    mock_can.reset();

    attempt_all(
      [&driver]() -> status {
        // Exercise
        return driver.velocity_control(1000000.0_rpm);
      },
      // Verify
      [](match<std::errc, std::errc::result_out_of_range>) { expect(true); },
      []() { expect(false); });

    expect(that % 0 == mock_can.spy_send.call_history().size());
  };

  "drc::position_control()"_test = []() {
    // Setup
    mock::can mock_can;
    auto router = hal::can_router::create(mock_can).value();
    auto driver = drc::create(router, expected_gear_ratio, expected_id).value();
    mock_can.reset();
    auto expected = prefilled_messages<6>(hal::value(drc::actuate::position_2));
    using payload_t = decltype(expected[0].payload);

    // Setup: values verified to be correct
    expected[0].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0x0, 0x0, 0x0, 0x0,
    };
    expected[1].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0x78, 0x69, 0x0, 0x0,
    };
    expected[2].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0xf0, 0xd2, 0x0, 0x0,
    };
    expected[3].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0x20, 0x1c, 0x0, 0x0,
    };
    expected[4].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0xd8, 0xdc, 0xff, 0xff,
    };
    expected[5].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0x40, 0xc6, 0xf9, 0xff,
    };

    // Exercise
    auto result0 = driver.position_control(0.0_deg, 10.0_rpm);
    auto result1 = driver.position_control(45.0_deg, 10.0_rpm);
    auto result2 = driver.position_control(90.0_deg, 10.0_rpm);
    auto result3 = driver.position_control(12.0_deg, 10.0_rpm);
    auto result4 = driver.position_control(-15.0_deg, 10.0_rpm);
    auto result5 = driver.position_control(-680.0_deg, 10.0_rpm);

    // Verify
    expect(bool{ result0 });
    expect(bool{ result1 });
    expect(bool{ result2 });
    expect(bool{ result3 });
    expect(bool{ result4 });
    expect(bool{ result5 });

    expect(that % expected[0] == mock_can.spy_send.history<0>(0));
    expect(that % expected[1] == mock_can.spy_send.history<0>(1));
    expect(that % expected[2] == mock_can.spy_send.history<0>(2));
    expect(that % expected[3] == mock_can.spy_send.history<0>(3));
    expect(that % expected[4] == mock_can.spy_send.history<0>(4));
    expect(that % expected[5] == mock_can.spy_send.history<0>(5));
  };

  "drc::position_control() fails"_test = []() {
    // Setup
    mock::can mock_can;
    auto router = hal::can_router::create(mock_can).value();
    auto driver = drc::create(router, expected_gear_ratio, expected_id).value();

    // Exercise
    attempt_all(
      [&driver]() -> status {
        return driver.position_control(1000000.0_deg, 10.0_rpm);
      },
      // Verify
      [](match<std::errc, std::errc::result_out_of_range>) { expect(true); },
      []() { expect(false); });

    attempt_all(
      [&driver]() -> status {
        // Exercise
        return driver.position_control(0.0_deg, 1000000.0_rpm);
      },
      // Verify
      [](match<std::errc, std::errc::result_out_of_range>) { expect(true); },
      []() { expect(false); });
  };

  "drc::feedback_request()"_test = []() {
    // Setup
    mock::can mock_can;
    auto router = hal::can_router::create(mock_can).value();
    auto driver = drc::create(router, expected_gear_ratio, expected_id).value();
    mock_can.reset();

    auto expected = prefilled_messages<8>();

    expected[0].payload[0] = hal::value(drc::read::pid_data);
    expected[1].payload[0] = hal::value(drc::read::acceleration_data);
    expected[2].payload[0] = hal::value(drc::read::encoder_data);
    expected[3].payload[0] = hal::value(drc::read::multi_turns_angle);
    expected[4].payload[0] = hal::value(drc::read::single_circle_angle);
    expected[5].payload[0] = hal::value(drc::read::status_1_and_error_flags);
    expected[6].payload[0] = hal::value(drc::read::status_2);
    expected[7].payload[0] = hal::value(drc::read::status_3);

    // Exercise
    auto result0 = driver.feedback_request(drc::read::pid_data);
    auto result1 = driver.feedback_request(drc::read::acceleration_data);
    auto result2 = driver.feedback_request(drc::read::encoder_data);
    auto result3 = driver.feedback_request(drc::read::multi_turns_angle);
    auto result4 = driver.feedback_request(drc::read::single_circle_angle);
    auto result5 = driver.feedback_request(drc::read::status_1_and_error_flags);
    auto result6 = driver.feedback_request(drc::read::status_2);
    auto result7 = driver.feedback_request(drc::read::status_3);

    // Verify
    expect(bool{ result0 });
    expect(bool{ result1 });
    expect(bool{ result2 });
    expect(bool{ result3 });
    expect(bool{ result4 });
    expect(bool{ result5 });
    expect(bool{ result6 });
    expect(bool{ result7 });

    expect(that % 8 == mock_can.spy_send.call_history().size());
    expect(that % expected[0] == mock_can.spy_send.history<0>(0));
    expect(that % expected[1] == mock_can.spy_send.history<0>(1));
    expect(that % expected[2] == mock_can.spy_send.history<0>(2));
    expect(that % expected[3] == mock_can.spy_send.history<0>(3));
    expect(that % expected[4] == mock_can.spy_send.history<0>(4));
    expect(that % expected[5] == mock_can.spy_send.history<0>(5));
    expect(that % expected[6] == mock_can.spy_send.history<0>(6));
    expect(that % expected[7] == mock_can.spy_send.history<0>(7));
  };

  "drc::feedback_request() fails"_test = []() {
    // Setup
    mock::can mock_can;
    auto router = hal::can_router::create(mock_can).value();
    auto driver = drc::create(router, expected_gear_ratio, expected_id).value();

    mock_can.spy_send.reset();
    mock_can.spy_send.trigger_error_on_call(1);

    // Exercise
    auto result = driver.feedback_request(drc::read::pid_data);

    // Verify
    expect(!bool{ result });
    expect(that % 1 == mock_can.spy_send.call_history().size());
  };

  "drc::system_control()"_test = []() {
    // Setup
    mock::can mock_can;
    auto router = hal::can_router::create(mock_can).value();
    auto driver = drc::create(router, expected_gear_ratio, expected_id).value();
    mock_can.reset();

    auto expected = prefilled_messages<4>();

    expected[0].payload[0] = hal::value(drc::system::clear_error_flag);
    expected[1].payload[0] = hal::value(drc::system::off);
    expected[2].payload[0] = hal::value(drc::system::stop);
    expected[3].payload[0] = hal::value(drc::system::running);

    // Exercise
    auto result0 = driver.system_control(drc::system::clear_error_flag);
    auto result1 = driver.system_control(drc::system::off);
    auto result2 = driver.system_control(drc::system::stop);
    auto result3 = driver.system_control(drc::system::running);

    // Verify
    expect(bool{ result0 });
    expect(bool{ result1 });
    expect(bool{ result2 });
    expect(bool{ result3 });

    expect(that % 4 == mock_can.spy_send.call_history().size());
    expect(that % expected[0] == mock_can.spy_send.history<0>(0));
    expect(that % expected[1] == mock_can.spy_send.history<0>(1));
    expect(that % expected[2] == mock_can.spy_send.history<0>(2));
    expect(that % expected[3] == mock_can.spy_send.history<0>(3));
  };

  "drc::system_control() fails"_test = []() {
    // Setup
    mock::can mock_can;
    auto router = hal::can_router::create(mock_can).value();
    auto driver = drc::create(router, expected_gear_ratio, expected_id).value();

    mock_can.spy_send.reset();
    mock_can.spy_send.trigger_error_on_call(1);

    // Exercise
    auto result = driver.system_control(drc::system::clear_error_flag);

    // Verify
    expect(!bool{ result });
    expect(that % 1 == mock_can.spy_send.call_history().size());
  };
};
}  // namespace hal::rmd