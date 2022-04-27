/* -*- c++ -*- */
/*
 * Copyright 2022 skysense.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "msg_bladerf_src_impl.h"

namespace gr {
  namespace msg_ctrl_bladerf {

    using output_type = gr_complex;
    msg_bladerf_src::sptr
    msg_bladerf_src::make(uint32_t samp_rate, uint64_t freq, uint32_t bw, int32_t gain0, int32_t gm0, int32_t gain1, int32_t gm1, int32_t biastee_rx, int32_t external_ref, int32_t external_freq, int32_t verbose)
    {
      return gnuradio::make_block_sptr<msg_bladerf_src_impl>(
        samp_rate, freq, bw, gain0, gm0, gain1, gm1, biastee_rx, external_ref, external_freq, verbose);
    }


    /*
     * The private constructor
     */
    msg_bladerf_src_impl::msg_bladerf_src_impl(uint32_t samp_rate, uint64_t freq, uint32_t bw, int32_t gain0, int32_t gm0, int32_t gain1, int32_t gm1, int32_t biastee_rx, int32_t external_ref, int32_t external_freq, int32_t verbose)
      : gr::sync_block("msg_bladerf_src",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(2, 2, sizeof(output_type)))
    {
      if (verbose != 0) {
          bladerf_log_set_verbosity(BLADERF_LOG_LEVEL_VERBOSE);
          d_verbose = 1;
      } else {
          bladerf_log_set_verbosity(BLADERF_LOG_LEVEL_CRITICAL);
          d_verbose = 0;
      }
      bladerf_set_usb_reset_on_open(true);
    
      _16ibuf = NULL;
      _32fcbuf = NULL;
    
      if (external_ref == 0) {
          d_external_ref = false;
      } else {
          d_external_ref = true;
      }
      // if (d_external_ref) { d_external_freq = external_freq;} else { d_external_freq  =
      // 0; }
      d_external_freq = external_freq;
    
      int status = bladerf_open(&d_dev, "*:instance=0");
      if (status != 0) {
          fprintf(stderr, "Unable to open device: %s\n", bladerf_strerror(status));
          exit(0);
      }
      // Set external frequency in any case
    
      status = bladerf_set_pll_refclk(d_dev, (uint64_t)d_external_freq);
      if (status != 0) {
          fprintf(stderr,
                  "Unable to set external reference clock frequency: %s\n",
                  bladerf_strerror(status));
          exit(0);
      }
      usleep(EXT_CLOCK_DWELL_US); // Sleep for 200 ms
    
      /*
      Setup the ADF4002 synth for external reference frequency
      */
    
      if (d_external_ref) {
    
          fprintf(stderr, "%s\n", "Starting CLOCK setup.");
    
          // Use VCTCXO no matter what
          status = bladerf_set_clock_select(d_dev, CLOCK_SELECT_ONBOARD);
          if (status != 0) {
              fprintf(
                  stderr, "Unable to set clock reference: %s\n", bladerf_strerror(status));
              exit(0);
          }
    
          status = bladerf_set_pll_enable(d_dev, true);
          if (status != 0) {
              fprintf(stderr, "Unable to enable PLL: %s\n", bladerf_strerror(status));
              exit(0);
          }
    
          uint64_t _FREQ;
          status = bladerf_get_pll_refclk(d_dev, &_FREQ);
          if (status == 0) {
              fprintf(stderr, "Current external reference clock frequency: %lu\n", _FREQ);
          } else {
              fprintf(stderr, "Unable to get reference clock frequency: %lu\n", _FREQ);
              exit(0);
          }
          usleep(500000);
    
    
          bool _LOCKED;
          status = bladerf_get_pll_lock_state(d_dev, &_LOCKED);
          if (status != 0) {
              fprintf(stderr,
                      "Unable to get ADF4002 lock state: %s\n",
                      bladerf_strerror(status));
              exit(0);
          }
    
          if (_LOCKED) {
              fprintf(stderr, "%s\n", "ADF4002 locked");
          } else {
              fprintf(stderr, "%s\n", "ADF4002 unlocked");
          }
    
          fprintf(stderr, "%s\n", "CLOCK setup done.");
    
      } else {
          fprintf(stderr, "%s\n", "REF_IN disabled.");
          status = bladerf_set_pll_enable(d_dev, false);
          if (status != 0) {
              fprintf(stderr, "Unable to enable PLL: %s\n", bladerf_strerror(status));
              exit(0);
          }
      }
    
    
      const bladerf_range* bw_range;
    
      status = bladerf_get_bandwidth_range(d_dev, BLADERF_CHANNEL_RX(0), &bw_range);
      if (status != 0) {
          fprintf(stderr, "%s\n", "bladerf_get_bandwidth_range failed");
      }
    
#ifdef SAPP_DEBUG_PRINT
      fprintf(stderr,
              "Analog BW can be set between %ld and %ld Hz\n",
              bw_range->min,
              bw_range->max);
#endif
    
      // Don't use external clock for now, just external reference
      /*
      if (d_external_clock ){
        status = bladerf_set_clock_select(d_dev, CLOCK_SELECT_EXTERNAL);
      }
      else {
        status = bladerf_set_clock_select(d_dev, CLOCK_SELECT_ONBOARD);
      }
      if (status != 0) {
        fprintf(stderr, "Unable to set clock reference: %s\n", bladerf_strerror(status));
        exit(0);
      }
      */
    
      d_conf = {
          .ch0 = BLADERF_CHANNEL_RX(0),
          .ch1 = BLADERF_CHANNEL_RX(1),
          .freq = freq,
          .bw = bw,
          .fs = samp_rate,
          .gain0 = gain0,
          .gain1 = gain1,
      };
    
      switch (gm0) {
      case 0:
          d_conf.gainmode0 = BLADERF_GAIN_MGC;
          break;
      case 1:
          d_conf.gainmode0 = BLADERF_GAIN_FASTATTACK_AGC;
          break;
      case 2:
          d_conf.gainmode0 = BLADERF_GAIN_SLOWATTACK_AGC;
          break;
      case 3:
          d_conf.gainmode0 = BLADERF_GAIN_HYBRID_AGC;
          break;
      default:
          fprintf(stderr, "Failed to set gain mode: %d\n", gm0);
          exit(0);
          break;
      }
    
      switch (gm1) {
      case 0:
          d_conf.gainmode1 = BLADERF_GAIN_MGC;
          break;
      case 1:
          d_conf.gainmode1 = BLADERF_GAIN_FASTATTACK_AGC;
          break;
      case 2:
          d_conf.gainmode1 = BLADERF_GAIN_SLOWATTACK_AGC;
          break;
      case 3:
          d_conf.gainmode1 = BLADERF_GAIN_HYBRID_AGC;
          break;
      default:
          fprintf(stderr, "Failed to set gain mode: %d\n", gm1);
          exit(0);
          break;
      }
    
      if (biastee_rx) {
          d_conf.biastee_rx = true;
      } else {
          d_conf.biastee_rx = false;
      }
    
      if (bw == 0) {
          d_conf.bw = d_conf.fs;
      }
    
      /*
      Block settings
      */
      set_max_noutput_items(SAMPLES_PER_BUFFER);
      set_output_multiple(2);
    
      size_t alignment = volk_get_alignment();
    
      _16ibuf = reinterpret_cast<int16_t*>(
          volk_malloc(2 * SAMPLES_PER_BUFFER * sizeof(int16_t), alignment));
      _32fcbuf = reinterpret_cast<gr_complex*>(
          volk_malloc(SAMPLES_PER_BUFFER * sizeof(gr_complex), alignment));
      /*
      Open device and make settings
      */
      bladerf_sample_rate _actual;
      status = bladerf_set_sample_rate(d_dev, d_conf.ch0, samp_rate, &_actual);
      if (status != 0) {
          fprintf(stderr,
                  "Problem while setting sampling rate: %s\n",
                  bladerf_strerror(status));
      }
    
      status = bladerf_set_bias_tee(d_dev, d_conf.ch0, d_conf.biastee_rx);
      if (status != 0) {
          fprintf(stderr, "Problem while enabling biastee: %s\n", bladerf_strerror(status));
      }
      /* Configure RX sample stream */
      status = bladerf_sync_config(d_dev,                   // device
                                   BLADERF_RX_X2,           // channel layout
                                   BLADERF_FORMAT_SC16_Q11, // format
                                   NUMBER_OF_BUFFERS,       // # buffers
                                   SAMPLES_PER_BUFFER,      // buffer size
                                   NUMBER_OF_TRANSFERS,     // # transfers
                                   STREAM_TIMEOUT_MS);      // timeout (ms)
      if (status < 0) {
          fprintf(stderr, "Couldn't configure RX streams: %s\n", bladerf_strerror(status));
      }
    
      /* Enable RX 0 */
      status = bladerf_enable_module(d_dev, d_conf.ch0, true);
      if (status < 0) {
          fprintf(stderr, "Couldn't enable RX module: %s\n", bladerf_strerror(status));
          exit(-1);
      }
    
      /* Enable RX 1 */
      status = bladerf_enable_module(d_dev, d_conf.ch1, true);
      if (status < 0) {
          fprintf(stderr, "Couldn't enable RX module: %s\n", bladerf_strerror(status));
          exit(-1);
      }
      // status = bladerf_set_rx_mux(d_dev,BLADERF_RX_MUX_12BIT_COUNTER);
      status = bladerf_set_rx_mux(d_dev, BLADERF_RX_MUX_BASEBAND);
      if (status < 0) {
          fprintf(stderr, "Couldn't set RX MUX mode: %s\n", bladerf_strerror(status));
          exit(-1);
      }
    
      set_freq(d_conf.freq);
      set_gainmode(gm0, 0);
      set_gain(d_conf.gain0, 0);
      set_gainmode(gm1, 1);
      set_gain(d_conf.gain1, 1);
      set_bw(d_conf.bw);
    
      /*
       */
      int const alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
      set_alignment(std::max(1, alignment_multiple));
      set_max_noutput_items(SAMPLES_PER_BUFFER);
      set_output_multiple(2);
      // message_port_register_out(pmt::mp("status"));
      message_port_register_in(pmt::mp("config_in"));
      set_msg_handler(pmt::mp("config_in"),
        [this](const pmt::pmt_t& msg) { reconfig(msg); });
    }
    /*
     * Our virtual destructor.
     */
    msg_bladerf_src_impl::~msg_bladerf_src_impl()
    {
      int status;
    
      /* Enable RX 0 */
      status = bladerf_enable_module(d_dev, d_conf.ch0, false);
      if (status < 0) {
          fprintf(stderr, "Couldn't disable RX module: %s\n", bladerf_strerror(status));
          exit(-1);
      }
    
      /* Enable RX 1 */
      status = bladerf_enable_module(d_dev, d_conf.ch1, false);
      if (status < 0) {
          fprintf(stderr, "Couldn't disable RX module: %s\n", bladerf_strerror(status));
          exit(-1);
      }
    
      bladerf_close(d_dev);
      volk_free(_16ibuf);
      volk_free(_32fcbuf);
    }

    void msg_bladerf_src_impl::reconfig(pmt::pmt_t msg)
    {
      // here we reconfig recevier according to different command
      if(pmt::dict_has_key(msg, pmt::string_to_symbol("cmd")) && pmt::dict_has_key(msg, pmt::string_to_symbol("value")))
      {
        std::string cmd = pmt::symbol_to_string(pmt::dict_ref(msg, pmt::string_to_symbol("cmd"), pmt::PMT_NIL));
        if(cmd.compare("freq") == 0)
        {
          uint64_t freq = pmt::to_long(pmt::dict_ref(msg, pmt::string_to_symbol("value"), pmt::PMT_NIL));
          set_freq(freq);
          if(d_display_level == 1)
          {
            std::cout << "block::msg_balderf_src: new center_frequency is set to [" << freq << "]" << std::endl;
          }
        }
        else if(cmd.compare("external_ref") == 0)
        {
          int external_ref = pmt::to_long(pmt::dict_ref(msg, pmt::string_to_symbol("value"), pmt::PMT_NIL));
          set_external_ref(external_ref);
          if(d_display_level == 1)
          {
            std::cout << "block::msg_balderf_src: new external_ref is set to [" << external_ref << "]" << std::endl;
          }
        }
        else if(cmd.compare("gain") == 0)
        {
          if(pmt::is_pair(pmt::dict_ref(msg, pmt::string_to_symbol("value"), pmt::PMT_NIL)))
          {
            pmt::pmt_t gain_setting = pmt::dict_ref(msg, pmt::string_to_symbol("value"), pmt::PMT_NIL);
            int gain = pmt::to_long(pmt::car(gain_setting)); 
            int channel = pmt::to_long(pmt::cdr(gain_setting)); 
            set_gain(gain, channel);
            if(d_display_level == 1)
            {
              std::cout << "block::msg_balderf_src: new gain at channel [" << channel << "] is set to [" << gain << "]" << std::endl;
            }
          }
          else
          {
            std::cout << "WARNING - block::msg_balderf_src: unknown values for setting gain. nothing will be changed. please check" << std::endl;
          }
        }
        else if(cmd.compare("biastee") == 0)
        {
          int enable = pmt::to_long(pmt::dict_ref(msg, pmt::string_to_symbol("biastee"), pmt::PMT_NIL));
          set_biastee(enable);
          if(d_display_level == 1)
          {
            std::cout << "block::msg_balderf_src: new biastee is set to [" << enable << "]" << std::endl;
          }
        }
        else if(cmd.compare("gainmode") == 0)
        {
          if(pmt::is_pair(pmt::dict_ref(msg, pmt::string_to_symbol("value"), pmt::PMT_NIL)))
          {
            pmt::pmt_t gainmode_setting = pmt::dict_ref(msg, pmt::string_to_symbol("value"), pmt::PMT_NIL);
            int gainmode = pmt::to_long(pmt::car(gainmode_setting)); 
            int channel = pmt::to_long(pmt::cdr(gainmode_setting)); 
            set_gainmode(gainmode, channel);
            if(d_display_level == 1)
            {
              std::cout << "block::msg_balderf_src: new gain mode at channel [" << channel << "] is set to [" << gainmode << "]" << std::endl;
            }
          }
          else
          {
            std::cout << "WARNING - block::msg_balderf_src: unknown values for setting gain. nothing will be changed. please check" << std::endl;
          }
        }
        else if(cmd.compare("bw") == 0)
        {
          uint32_t bw = pmt::to_long(pmt::dict_ref(msg, pmt::string_to_symbol("value"), pmt::PMT_NIL));
          set_bw(bw);
          if(d_display_level == 1)
          {
            std::cout << "block::msg_balderf_src: new bandwidth is set to [" << bw << "]" << std::endl;
          }
        }
        else
        {
          std::cout << "WARNING - block::msg_balderf_src: unknown cmd type. nothing will be changed. please check" << std::endl;
        }
      }
      else
      {
        std::cout << "WARNING - block::msg_balderf_src: unknown msg input. nothing will be changed. please check" << std::endl;
      }
    }

    void msg_bladerf_src_impl::set_freq(uint64_t freq)
    {
      int status;
      gr::thread::scoped_lock guard(d_mutex);

      d_conf.freq = freq;

      status = bladerf_set_frequency(d_dev, d_conf.ch0, d_conf.freq);
      if (status != 0) {
          fprintf(stderr,
                  "Failed to set frequency = %lu: %s\n",
                  d_conf.freq,
                  bladerf_strerror(status));
          exit(0);
      }
      // There's only one RX LO anyway
      /*
      status = bladerf_set_frequency(d_dev, d_conf.ch1 , d_conf.freq );
      if (status != 0) {
        fprintf(stderr, "Failed to set frequency = %lu: %s\n",
          d_conf.freq , bladerf_strerror(status));
        exit(0);
      }
      */
      return;
    }

    void msg_bladerf_src_impl::set_external_ref(int external_ref)
    {
      int status;
      if (external_ref == 0) {
          status = bladerf_set_pll_enable(d_dev, false);
          if (status != 0) {
              fprintf(stderr, "Unable to enable PLL: %s\n", bladerf_strerror(status));
              exit(0);
          }
          usleep(EXT_CLOCK_DWELL_US);
      } else {
          status = bladerf_set_pll_enable(d_dev, true);
          if (status != 0) {
              fprintf(stderr, "Unable to enable PLL: %s\n", bladerf_strerror(status));
              exit(0);
          }
          usleep(EXT_CLOCK_DWELL_US);
      }
    }


    void msg_bladerf_src_impl::set_gain(int gain, int channel)
    {
      int status;
      gr::thread::scoped_lock guard(d_mutex);

      if (gain > 60) {
          gain = 60;
      }
      if (gain < 0) {
          gain = 0;
      }

      if (channel == 0) {
          d_conf.gain0 = gain;
          status = bladerf_set_gain(d_dev, d_conf.ch0, d_conf.gain0);
          if (status != 0) {
              fprintf(stderr, "Failed to set gain: %s\n", bladerf_strerror(status));
              exit(0);
          }
      } else if (channel == 1) {
          d_conf.gain1 = gain;
          status = bladerf_set_gain(d_dev, d_conf.ch1, d_conf.gain1);
          if (status != 0) {
              fprintf(stderr, "Failed to set gain: %s\n", bladerf_strerror(status));
              exit(0);
          }
      } else {
          fprintf(stderr, "set_gain() invalid channel: %d\n", channel);
          exit(0);
      }
      return;
    }

    void msg_bladerf_src_impl::set_biastee(int enable)
    {
      int status;
      gr::thread::scoped_lock guard(d_mutex);

      bool state;
      if (enable != 0) {
          state = true;
      } else {
          state = false;
      }

      d_conf.biastee_rx = state;
      status = bladerf_set_bias_tee(d_dev, d_conf.ch0, d_conf.biastee_rx);
      if (status != 0) {
          fprintf(stderr, "Problem while enabling biastee: %s\n", bladerf_strerror(status));
          exit(0);
      }
      return;
    }


    void msg_bladerf_src_impl::set_gainmode(int gainmode, int channel)
    {
      int status;
      gr::thread::scoped_lock guard(d_mutex);

      bladerf_gain_mode gm;

      switch (gainmode) {
      case 0:
          gm = BLADERF_GAIN_MGC;
          break;
      case 1:
          gm = BLADERF_GAIN_FASTATTACK_AGC;
          break;
      case 2:
          gm = BLADERF_GAIN_SLOWATTACK_AGC;
          break;
      case 3:
          gm = BLADERF_GAIN_HYBRID_AGC;
          break;
      default:
          fprintf(stderr, "Failed to set default gain mode: %d\n", gm);
          exit(0);
          break;
      }

      switch (channel) {
      case 0:
          d_conf.gainmode0 = gm;
          status = bladerf_set_gain_mode(d_dev, d_conf.ch0, d_conf.gainmode0);
          if (status != 0) {
              fprintf(stderr,
                      "Failed to set default gain mode: %s\n",
                      bladerf_strerror(status));
              exit(0);
          }
          break;
      case 1:
          d_conf.gainmode1 = gm;
          status = bladerf_set_gain_mode(d_dev, d_conf.ch1, d_conf.gainmode1);
          if (status != 0) {
              fprintf(stderr,
                      "Failed to set default gain mode: %s\n",
                      bladerf_strerror(status));
              exit(0);
          }
          break;
      default:
          fprintf(stderr, "set_gainmode() invalid channel: %d\n", channel);
          exit(0);
          break;
      }
      return;
    }

    void msg_bladerf_src_impl::set_bw(uint32_t bw)
    {
      int status;
      // gr::thread::scoped_lock guard(d_mutex);

      /*
      status = bladerf_enable_module(d_dev, d_conf.ch0 , false );
      if ( status < 0 ) {
          fprintf(stderr, "Couldn't enable RX module: %s\n", bladerf_strerror(status) ) ;
          exit(-1);
      }
      status = bladerf_enable_module(d_dev, d_conf.ch1 , false );
      if ( status < 0 ) {
          fprintf(stderr, "Couldn't enable RX module: %s\n", bladerf_strerror(status) ) ;
          exit(-1);
      }
      */
      _running = false;
      #ifdef SAPP_DEBUG_PRINT
      std::chrono::high_resolution_clock::time_point t1 =
          std::chrono::high_resolution_clock::now();
      #endif

      d_conf.bw = bw;
      uint32_t actual;

      status = bladerf_set_bandwidth(d_dev, d_conf.ch0, d_conf.bw, &actual);
      if (status != 0) {
          fprintf(stderr,
                  "Failed to set bandwidth = %u: %s\n",
                  d_conf.bw,
                  bladerf_strerror(status));
          exit(0);
      }

      _running = true;


      #ifdef SAPP_DEBUG_PRINT
      std::chrono::high_resolution_clock::time_point t2 =
          std::chrono::high_resolution_clock::now();
      auto duration =
          std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
      std::cout << "Changing BW took " << duration << " us." << std::endl;
      std::cout << "New BW is " << actual << " Hz." << std::endl;
      #endif
      /*
      status = bladerf_enable_module(d_dev, d_conf.ch0 , true );
      if ( status < 0 ) {
          fprintf(stderr, "Couldn't enable RX module: %s\n", bladerf_strerror(status) ) ;
          exit(-1);
      }
      status = bladerf_enable_module(d_dev, d_conf.ch1 , true );
      if ( status < 0 ) {
          fprintf(stderr, "Couldn't enable RX module: %s\n", bladerf_strerror(status) ) ;
          exit(-1);
      }
      */
      return;
    }

    int
    msg_bladerf_src_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      int status;
      size_t nstreams = 2;
    
      gr::thread::scoped_lock guard(d_mutex);
    
      if (_running == false) {
          return 0;
      }
      // grab samples into temp buffer
      status = bladerf_sync_rx(
          d_dev, static_cast<void*>(_16ibuf), noutput_items, NULL, STREAM_TIMEOUT_MS);
      if (status != 0) {
          fprintf(stderr, "%s: %s\n", "bladeRF stream error", bladerf_strerror(status));
          ++_failures;
          if (_failures >= MAX_CONSECUTIVE_FAILURES) {
              fprintf(stderr, "%s\n", "Consecutive error limit hit. Shutting down.");
              return WORK_DONE;
          }
      } else {
          _failures = 0;
      }
    
      // Use this cast to be able to include the division by SCALING_FACTOR into the call
      volk_16i_s32f_convert_32f(
          reinterpret_cast<float*>(_32fcbuf), _16ibuf, SCALING_FACTOR, 2 * noutput_items);
    
      // Deinterleave
      // gr_complex *out1 = (gr_complex *) output_items[0];
      // gr_complex *out2 = (gr_complex *) output_items[1];
      auto out0 = static_cast<output_type*>(output_items[0]);
      auto out1 = static_cast<output_type*>(output_items[1]);
      int k = 0;
      for (uint32_t i = 0; i < noutput_items / nstreams; i++) {
          out0[i] = _32fcbuf[k];
          out1[i] = _32fcbuf[k + 1];
          k += 2;
      }
      /*
      //Deinterleave alternative (identical performance)
      gr_complex **out = reinterpret_cast<gr_complex **>(&output_items[0]);
      gr_complex const *deint_in = _32fcbuf;
    
      for (size_t i = 0; i < (noutput_items/nstreams); ++i) {
        memcpy(out[0]++, deint_in++, sizeof(gr_complex));
        memcpy(out[1]++, deint_in++, sizeof(gr_complex));
      }
      */
      return noutput_items / nstreams;
    }

  } /* namespace msg_ctrl_bladerf */
} /* namespace gr */
