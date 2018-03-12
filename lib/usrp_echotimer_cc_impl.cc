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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "usrp_echotimer_cc_impl.h"
#include <iostream>
#include <chrono>
#include <thread>

namespace gr { namespace radar {

    usrp_echotimer_cc::sptr
    usrp_echotimer_cc::make(
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
    ) {
        return gnuradio::get_initial_sptr(new usrp_echotimer_cc_impl(
            tx_usrp,
            rx_usrp,
            num_delay_samps,
            wire_tx,
            timeout_tx,
            wait_tx,
            wire_rx,
            timeout_rx,
            wait_rx,
            len_key
        ));
    }

    usrp_echotimer_cc_impl::usrp_echotimer_cc_impl(
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
    ) : gr::tagged_stream_block("usrp_echotimer_cc",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex)), len_key),
        d_usrp_tx(tx_usrp),
        d_usrp_rx(rx_usrp),
        d_samp_rate(tx_usrp->get_tx_rate()),
    {
        d_center_freq = center_freq;
        d_num_delay_samps = num_delay_samps;
        d_out_buffer.resize(0);

        //***** Setup USRP TX *****//
        d_wire_tx = wire_tx;
        d_timeout_tx = timeout_tx; // timeout for sending
        d_wait_tx = wait_tx; // secs to wait befor sending

        // Setup USRP TX: timestamp
        if (d_time_source_tx != "gpsdo"){
            d_usrp_tx->set_time_now(uhd::time_spec_t(0.0)); // Do set time on startup if not gpsdo is activated.
        }

        // Setup transmit streamer
        uhd::stream_args_t stream_args_tx("fc32", d_wire_tx); // complex floats
        d_tx_stream = d_usrp_tx->get_tx_stream(stream_args_tx);

        //***** Setup USRP RX *****//
        d_args_rx = args_rx;
        d_wire_rx = wire_rx;
        d_clock_source_rx = clock_source_rx;
        d_time_source_rx = time_source_rx;
        d_antenna_rx = antenna_rx;
        d_lo_offset_rx = lo_offset_rx;
        d_gain_rx = gain_rx;
        d_timeout_rx = timeout_rx; // timeout for receiving
        d_wait_rx = wait_rx; // secs to wait befor receiving

        // Setup receive streamer
        uhd::stream_args_t stream_args_rx("fc32", d_wire_rx); // complex floats
        std::vector<size_t> channel_nums; channel_nums.push_back(0); // define channel!
        stream_args_rx.channels = channel_nums;
        d_rx_stream = d_usrp_rx->get_rx_stream(stream_args_rx);

        //***** Misc *****//
        // Setup rx_time pmt
        d_time_key = pmt::string_to_symbol("rx_time");
        d_srcid = pmt::string_to_symbol("usrp_echotimer");

        // Setup thread priority
        //uhd::set_thread_priority_safe(); // necessary? doesnt work...

        // Sleep to get sync done
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // FIXME: necessary?
    }

    usrp_echotimer_cc_impl::~usrp_echotimer_cc_impl()
    {
    }

    int
    usrp_echotimer_cc_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
        int noutput_items = ninput_items[0];
        return noutput_items ;
    }

    void
    usrp_echotimer_cc_impl::set_num_delay_samps(int num_samps){
        d_num_delay_samps = num_samps;
    }

    void
    usrp_echotimer_cc_impl::set_rx_gain(float gain){
        d_usrp_rx->set_rx_gain(gain);
    }

    void
    usrp_echotimer_cc_impl::set_tx_gain(float gain){
        d_usrp_tx->set_tx_gain(gain);
    }

    void
    usrp_echotimer_cc_impl::send()
    {
        // Setup metadata for first package
        d_metadata_tx.start_of_burst = true;
        d_metadata_tx.end_of_burst = false;
        d_metadata_tx.has_time_spec = true;
        d_metadata_tx.time_spec = d_time_now_tx+uhd::time_spec_t(d_wait_tx); // Timespec needed?

        // Send input buffer
        size_t num_acc_samps = 0; // Number of accumulated samples
        size_t num_tx_samps, total_num_samps;
        total_num_samps = d_noutput_items_send;
        // Data to USRP
        num_tx_samps = d_tx_stream->send(
                d_in_send,
                total_num_samps,
                d_metadata_tx,
                total_num_samps/(float)d_samp_rate+d_timeout_tx
        );
        // Get timeout
        if (num_tx_samps < total_num_samps) {
            std::cerr << "Send timeout..." << std::endl;
        }

        //send a mini EOB packet
        d_metadata_tx.start_of_burst = false;
        d_metadata_tx.end_of_burst = true;
        d_metadata_tx.has_time_spec = false;
        d_tx_stream->send("", 0, d_metadata_tx);
    }

    void
    usrp_echotimer_cc_impl::receive()
    {
        // Setup RX streaming
        size_t total_num_samps = d_noutput_items_recv;
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
        stream_cmd.num_samps = total_num_samps;
        stream_cmd.stream_now = false;
        stream_cmd.time_spec = d_time_now_rx+uhd::time_spec_t(d_wait_rx);
        d_rx_stream->issue_stream_cmd(stream_cmd);

        size_t num_rx_samps;
        // Receive a packet
        num_rx_samps = d_rx_stream->recv(
            d_out_recv,
            total_num_samps,
            d_metadata_rx,
            total_num_samps/(float)d_samp_rate+d_timeout_rx
        );

        // Save timestamp
        d_time_val = pmt::make_tuple
            (pmt::from_uint64(d_metadata_rx.time_spec.get_full_secs()),
             pmt::from_double(d_metadata_rx.time_spec.get_frac_secs()));

        // Handle the error code
        if (d_metadata_rx.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            throw std::runtime_error(str(boost::format("Receiver error %s") % d_metadata_rx.strerror()));
        }

        if (num_rx_samps < total_num_samps) std::cerr << "Receive timeout before all samples received..." << std::endl;
    }

    int
    usrp_echotimer_cc_impl::work (int noutput_items,
            gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items)
    {
        gr_complex *in = (gr_complex *) input_items[0]; // remove const
        gr_complex *out = (gr_complex *) output_items[0];

        // Set output items on packet length
        noutput_items = ninput_items[0];

        // Resize output buffer
        if(d_out_buffer.size()!=noutput_items) d_out_buffer.resize(noutput_items);

        // Get time from USRP TX
        d_time_now_tx = d_usrp_tx->get_time_now();
        d_time_now_rx = d_time_now_tx;

        // Send thread
        d_in_send = in;
        d_noutput_items_send = noutput_items;
        d_thread_send = gr::thread::thread(boost::bind(&usrp_echotimer_cc_impl::send, this));

        // Receive thread
        d_out_recv = &d_out_buffer[0];
        d_noutput_items_recv = noutput_items;
        d_thread_recv = gr::thread::thread(boost::bind(&usrp_echotimer_cc_impl::receive, this));

        // Wait for threads to complete
        d_thread_send.join();
        d_thread_recv.join();

        // Shift of number delay samples (fill with zeros)
        memcpy(out,&d_out_buffer[0]+d_num_delay_samps,(noutput_items-d_num_delay_samps)*sizeof(gr_complex)); // push buffer to output
        memset(out+(noutput_items-d_num_delay_samps),0,d_num_delay_samps*sizeof(gr_complex)); // set zeros

        // Setup rx_time tag
        add_item_tag(0, nitems_written(0), d_time_key, d_time_val, d_srcid);

        // Tell runtime system how many output items we produced.
        return noutput_items;
    }

}} /* namespace gr::radar */

