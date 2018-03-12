/* -*- c++ -*- */
/*
 * Copyright 2014 Communications Engineering Lab, KIT.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_RADAR_USRP_ECHOTIMER_CC_H
#define INCLUDED_RADAR_USRP_ECHOTIMER_CC_H

#include <radar/api.h>
#include <radar/usrp.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr { namespace radar {

    /*!
     * \brief <+description of block+>
     * \ingroup radar
     *
     */
      class RADAR_API usrp_echotimer_cc : virtual public gr::tagged_stream_block
      {
      public:
          typedef boost::shared_ptr<usrp_echotimer_cc> sptr;

          static sptr make(
              usrp::sptr tx_usrp,
              usrp::sptr rx_usrp,
              int num_delay_samps,
              std::string wire_tx,
              float timeout_tx,
              float wait_tx,
              std::string wire_rx,
              float timeout_rx,
              float wait_rx,
              const std::string& len_key="packet_len"
          );

          virtual void set_num_delay_samps(int num_samps) = 0;
          virtual void set_rx_gain(float gain) = 0;
          virtual void set_tx_gain(float gain) = 0;
      };

}} // namespace gr::radar

#endif /* INCLUDED_RADAR_USRP_ECHOTIMER_CC_H */

