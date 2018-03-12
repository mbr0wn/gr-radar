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

#ifndef INCLUDED_RADAR_USRP_ECHOTIMER_CC_IMPL_H
#define INCLUDED_RADAR_USRP_ECHOTIMER_CC_IMPL_H

#include <radar/usrp_echotimer_cc.h>
#include <radar/usrp.h>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/usrp/multi_usrp.hpp>

namespace gr { namespace radar {

    class usrp_echotimer_cc_impl : public usrp_echotimer_cc
    {
    protected:
        int calculate_output_stream_length(const gr_vector_int &ninput_items);

    public:
        usrp_echotimer_cc_impl(
            uhd::usrp::multi_usrp::sptr tx_usrp,
            uhd::usrp::multi_usrp::sptr rx_usrp,
            int num_delay_samps,
            std::string wire_tx,
            float timeout_tx,
            float wait_tx,
            std::string wire_rx,
            float timeout_rx,
            float wait_rx,
            const std::string& len_key
        );
        ~usrp_echotimer_cc_impl();

        void send();
        void receive();
        void set_num_delay_samps(int num_samps);
        void set_rx_gain(float gain);
        void set_tx_gain(float gain);

        int work(int noutput_items,
                gr_vector_int &ninput_items,
                gr_vector_const_void_star &input_items,
                gr_vector_void_star &output_items);

    private:
        uhd::usrp::multi_usrp::sptr d_usrp_tx;
        uhd::usrp::multi_usrp::sptr d_usrp_rx;

        int d_samp_rate;
        int d_num_delay_samps;
        std::vector<gr_complex> d_out_buffer;

        std::string d_args_tx, d_args_rx;
        std::string d_wire_tx, d_wire_rx;
        uhd::tx_streamer::sptr d_tx_stream;
        uhd::rx_streamer::sptr d_rx_stream;
        uhd::tx_metadata_t d_metadata_tx;
        uhd::rx_metadata_t d_metadata_rx;
        double d_lo_offset_tx, d_lo_offset_rx;
        float d_timeout_tx, d_timeout_rx;
        float d_wait_tx, d_wait_rx;
        float d_gain_tx, d_gain_rx;

        uhd::time_spec_t d_time_now_tx, d_time_now_rx;

        gr::thread::thread d_thread_recv;
        gr_complex *d_out_recv;
        int d_noutput_items_recv;
        pmt::pmt_t d_time_key, d_time_val, d_srcid;

        gr::thread::thread d_thread_send;
        gr_complex *d_in_send;
        int d_noutput_items_send;

    };

}} // namespace gr::radar

#endif /* INCLUDED_RADAR_USRP_ECHOTIMER_CC_IMPL_H */

