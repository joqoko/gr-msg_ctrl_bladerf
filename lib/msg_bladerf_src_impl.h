/* -*- c++ -*- */
/*
 * Copyright 2022 skysense.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_MSG_CTRL_BLADERF_MSG_BLADERF_SRC_IMPL_H
#define INCLUDED_MSG_CTRL_BLADERF_MSG_BLADERF_SRC_IMPL_H

#include <gnuradio/msg_ctrl_bladerf/msg_bladerf_src.h>
#include <gnuradio/thread/thread.h>
#include <libbladeRF.h>
#include <volk/volk.h>
#include <chrono>

namespace gr {
  namespace msg_ctrl_bladerf {

    class msg_bladerf_src_impl : public msg_bladerf_src
    {
      typedef struct S 
      {
        bladerf_channel ch0;
        bladerf_channel ch1;
        bladerf_frequency freq;
        bladerf_bandwidth bw;
        bladerf_sample_rate fs;
        bladerf_gain gain0;
        bladerf_gain gain1;
        bladerf_gain_mode gainmode0;
        bladerf_gain_mode gainmode1;
        bool biastee_rx;
      } bladerf_config;

     private:
      int d_display_level = 0;
      int d_channel;
      bool d_external_ref;
      int d_verbose;
      int d_external_freq;
      struct bladerf* d_dev;
      bladerf_config d_conf;
    
      gr::thread::mutex d_mutex; /**< mutex to protect set/work access */
    
      /* Scaling factor used when converting from int16_t to float */
      const float SCALING_FACTOR = 2048.0f;
      const int MAX_CONSECUTIVE_FAILURES = 3;
      const int SAMPLES_PER_BUFFER = 4096;
      const int NUMBER_OF_BUFFERS = 512;
      const int NUMBER_OF_TRANSFERS = 32;
      const int STREAM_TIMEOUT_MS = 3000;
      const int EXT_CLOCK_DWELL_US = 2000;
    
      // Private and hidden
      int16_t* _16ibuf;     /**< raw samples from bladeRF */
      gr_complex* _32fcbuf; /**< intermediate buffer to gnuradio */
      int _failures;
      int _counter = 0;
      bool _running;

     public:
      msg_bladerf_src_impl(uint32_t samp_rate, uint64_t freq, uint32_t bw, int32_t gain0, int32_t gm0, int32_t gain1, int32_t gm1, int32_t biastee_rx, int32_t external_ref, int32_t external_freq, int32_t verbose, int display_level);
      ~msg_bladerf_src_impl();

      void set_freq(uint64_t freq);
      void set_gain(int32_t gain, int32_t channel);
      void set_gainmode(int32_t gainmode, int32_t channel);
      void set_biastee(int32_t enable);
      void set_bw(uint32_t bw);
      void set_external_ref(int32_t external_ref);
      void reconfig(pmt::pmt_t msg);

      // Where all the action really happens
      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };

  } // namespace msg_ctrl_bladerf
} // namespace gr

#endif /* INCLUDED_MSG_CTRL_BLADERF_MSG_BLADERF_SRC_IMPL_H */
