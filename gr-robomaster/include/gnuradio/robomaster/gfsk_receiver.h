/* -*- c++ -*- */
/*
 * Copyright 2026 liu.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_ROBOMASTER_GFSK_RECEIVER_H
#define INCLUDED_ROBOMASTER_GFSK_RECEIVER_H

#include <gnuradio/robomaster/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace robomaster {

    /*!
     * \brief <+description of block+>
     * \ingroup robomaster
     *
     */
    class ROBOMASTER_API gfsk_receiver : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<gfsk_receiver> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of robomaster::gfsk_receiver.
       *
       * To avoid accidental use of raw pointers, robomaster::gfsk_receiver's
       * constructor is in a private implementation
       * class. robomaster::gfsk_receiver::make is the public interface for
       * creating new instances.
       */
      static sptr make(float threshold, int sps, int packet_bytes);
    };

  } // namespace robomaster
} // namespace gr

#endif /* INCLUDED_ROBOMASTER_GFSK_RECEIVER_H */
