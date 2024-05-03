#include <Arduino.h>
#include <engine.h>
#include <engine_config.pb.h>
#include <engine_state.pb.h>
#include <iirfilter.h>

Engine::Engine() : state(&state_) {
  config = {
      .sensor_pin = 15,
      .rotation_sample_rate = 4,
      .time_sample_rate_ms = 10,
      .counts_per_rotation = 16,
      // Automatically Generated Coefficients (DO NOT MODIFY)
      .time_filter_b_coeffs_count = 2,
      .time_filter_b_coeffs = {0.24523727525278557, 0.24523727525278557},
      .time_filter_a_coeffs_count = 2,
      .time_filter_a_coeffs = {1.0, -0.5095254494944288},
      // Automatically Generated Coefficients (DO NOT MODIFY)
      .rotation_filter_b_coeffs_count = 5,
      .rotation_filter_b_coeffs = {0.8677114646, -3.305398989, 4.8804516238,
                                   -3.305398989, 0.8677114646},
      .rotation_filter_a_coeffs_count = 5,
      .rotation_filter_a_coeffs = {1.0, -3.5518051128, 4.8720546544,
                                   -3.0589928651, 0.7438198987},
  };

  rotation_filter = new IIRFilter(config.rotation_filter_b_coeffs,
                                  config.rotation_filter_a_coeffs,
                                  config.rotation_filter_b_coeffs_count,
                                  config.rotation_filter_a_coeffs_count);

  time_filter = new IIRFilter(
      config.time_filter_b_coeffs, config.time_filter_a_coeffs,
      config.time_filter_b_coeffs_count, config.time_filter_a_coeffs_count);
}

Engine::~Engine() {
  delete rotation_filter;
  delete time_filter;
}

void Engine::on_sensor_pulse() {
  u32 cur_time_us = micros();
  if (cur_time_us - last_pulse_time_us > 100) {
    if (state_.pulse_count % config.rotation_sample_rate == 0) {
      state_.pulse_time_diff_us = cur_time_us - last_sampled_pulse_time_us;
      state_.filt_pulse_time_diff_us =
          rotation_filter->update(state_.pulse_time_diff_us);
      last_sampled_pulse_time_us = cur_time_us;
    }
    ++state_.pulse_count;
  }
  last_pulse_time_us = cur_time_us;
}

void Engine::update() {
  noInterrupts();
  float cur_pulse_time_diff_us = state_.pulse_time_diff_us;
  float cur_filt_pulse_time_diff_us = state_.filt_pulse_time_diff_us;
  interrupts();

  // TODO: Fix edge case of no movement
  if (state_.pulse_time_diff_us != 0) {
    state_.rpm = ((float)config.rotation_sample_rate) /
                 config.counts_per_rotation / cur_pulse_time_diff_us *
                 US_PER_SECOND * SECONDS_PER_MINUTE;
    state_.filt_rpm = ((float)config.rotation_sample_rate) /
                      config.counts_per_rotation / cur_filt_pulse_time_diff_us *
                      US_PER_SECOND * SECONDS_PER_MINUTE;
    state_.filt_rpm = time_filter->update(state_.filt_rpm);
  }
}
