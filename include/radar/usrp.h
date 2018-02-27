/* -*- c++ -*- */
/*
 * Copyright 2018 Ettus Research, a National Instruments Company
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <radar/api.h>
#include <uhd/usrp/multi_usrp.hpp>

namespace gr { namespace radar {

    /*! Convenience encapsulation of a multi_usrp object for purpose of radar
     *
     * For an active radar setup, you'll want two of these. Make sure they are
     * actually physically connected to the same reference (unless they're the
     * same device).
     *
     */
    class RADAR_API usrp {
    public:
        typedef boost::shared_ptr<usrp> sptr;

        /*!
         * \param dev_args Args used in multi_usrp::make() call
         * \param is_tx Set to true if this is the transmitter
         * \param rate Sampling rate [Hz]
         * \param center_freq Center frequency [Hz]
         * \param gain Gain [dB]
         * \param lo_offset LO Offset [Hz]
         * \param clock_source Clock source ("external", "mimo", ...)
         * \param time_source Time/PPS source ("external", "mimo", ...)
         * \param antenna Antenna port ("TX/RX", "RX2", ...)
         * \param channels List of channels used in this setup
         * \param subdev_spec Subdev spec
         */
        static sptr make(
            const std::string& dev_args,
            const bool is_tx,
            const float rate,
            const float center_freq,
            const float gain,
            const float lo_offset,
            const std::string& clock_source,
            const std::string& time_source,
            const std::string& antenna,
            const std::vector<size_t> channels,
            const std::string& subdev_spec
        );

        //! Return a pointer to the underlying multi_usrp device
        virtual ::uhd::usrp::multi_usrp::sptr get_device() = 0;
    };

}} /* namespace gr::radar */

