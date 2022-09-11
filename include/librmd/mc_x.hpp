// auto timeout = create_timeout(counter, 100ms);
// motor.velocity(10.0f, timeout);

// while(motor.is_finished()) {
//   if (!timeout) {
//     continue; // or
//     co_yield;
//   }
//   break; // or
//   return hal::new_error();
// }
// auto current = motor.get_current(timeout);

// // motor.request(x_series::current);

// = ===========================================================================

// // MAIN API

// class mx_300 {
//   status request(request_enum req) {

//   }
//   result<message_t> response() {

//   }
// };