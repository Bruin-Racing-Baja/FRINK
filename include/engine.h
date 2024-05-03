#ifndef ENGINE_H
#define ENGINE_H

#include <Arduino.h>
#include <constants.h>
#include <engine_config.pb.h>
#include <engine_state.pb.h>
#include <iirfilter.h>
#include <types.h>

class Engine {
public:
  EngineConfig config = {
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

  const EngineState *const state;

  Engine();
  ~Engine();

  void on_sensor_pulse();
  void update();

private:
  EngineState state_;
  u32 last_pulse_time_us = 0;
  u32 last_sampled_pulse_time_us = 0;
  IIRFilter *time_filter;
  IIRFilter *rotation_filter;
};

#endif
