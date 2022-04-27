/* -*- c++ -*- */
/*
 * Copyright 2022 skysense.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_MSG_CTRL_BLADERF_MSG_BLADERF_SRC_H
#define INCLUDED_MSG_CTRL_BLADERF_MSG_BLADERF_SRC_H

#include <gnuradio/msg_ctrl_bladerf/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace msg_ctrl_bladerf {

    /*!
     * \brief <+description of block+>
     * \ingroup msg_ctrl_bladerf
     *
     */
    class MSG_CTRL_BLADERF_API msg_bladerf_src : virtual public gr::sync_block
    {
     public:
      typedef std::shared_ptr<msg_bladerf_src> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of msg_ctrl_bladerf::msg_bladerf_src.
       *
       * To avoid accidental use of raw pointers, msg_ctrl_bladerf::msg_bladerf_src's
       * constructor is in a private implementation
       * class. msg_ctrl_bladerf::msg_bladerf_src::make is the public interface for
       * creating new instances.
       */
      static sptr make(uint32_t samp_rate, uint64_t freq, uint32_t bw, int32_t gain0, int32_t gm0, int32_t gain1, int32_t gm1, int32_t biastee_rx, int32_t external_ref, int32_t external_freq, int32_t verbose, int display_level);

      virtual void set_freq(uint64_t freq) = 0;
      virtual void set_gain(int32_t gain, int32_t channel) = 0;
      virtual void set_gainmode(int32_t gainmode, int32_t channel) = 0;
      virtual void set_biastee(int32_t enable) = 0;
      virtual void set_bw(uint32_t bw) = 0;
      virtual void set_external_ref(int32_t external_ref) = 0;

    };

  } // namespace msg_ctrl_bladerf
} // namespace gr

#endif /* INCLUDED_MSG_CTRL_BLADERF_MSG_BLADERF_SRC_H */
