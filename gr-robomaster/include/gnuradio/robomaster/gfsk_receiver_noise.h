/* -*- c++ -*- */
/*
 * Copyright 2026 liu.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ROBOMASTER_GFSK_RECEIVER_NOISE_H
#define INCLUDED_ROBOMASTER_GFSK_RECEIVER_NOISE_H

#include <gnuradio/robomaster/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace robomaster {

    /*!
     * \brief GFSK receiver for interference wave packets.
     * \ingroup robomaster
     *
     */
    class ROBOMASTER_API gfsk_receiver_noise : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<gfsk_receiver_noise> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of robomaster::gfsk_receiver_noise.
       */
      static sptr make(float threshold, int sps, int packet_bytes);
    };

  } // namespace robomaster
} // namespace gr

#endif /* INCLUDED_ROBOMASTER_GFSK_RECEIVER_NOISE_H */
