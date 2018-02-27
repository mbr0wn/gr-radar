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

#include <radar/usrp.h>
#include <boost/make_shared.hpp>

namespace gr { namespace radar {

    /*! Convenience encapsulation of a multi_usrp object for purpose of radar
     *
     * For an active radar setup, you'll want two of these. Make sure they are
     * actually physically connected to the same reference (unless they're the
     * same device).
     */
    class usrp_impl : public usrp
    {
    public:
        usrp_impl(
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
        ) : _dev(::uhd::usrp::multi_usrp::make(dev_args)),
            _channels(channels)
        {
            const auto tune_request = ::uhd::tune_request_t(
                center_freq,
                lo_offset
            );
            _dev->set_clock_source(clock_source);
            _dev->set_time_source(time_source);
            if (is_tx) {
                _dev->set_tx_rate(rate);
                if (!subdev_spec.empty()) {
                    _dev->set_tx_subdev_spec(subdev_spec);
                }
                for (const size_t chan : channels) {
                    _dev->set_tx_freq(tune_request, chan);
                    _dev->set_tx_gain(gain, chan);
                    _dev->set_tx_antenna(antenna, chan);
                }
            } else {
                _dev->set_rx_rate(rate);
                if (!subdev_spec.empty()) {
                    _dev->set_rx_subdev_spec(subdev_spec);
                }
                for (const size_t chan : channels) {
                    _dev->set_rx_freq(tune_request, chan);
                    _dev->set_rx_gain(gain, chan);
                    _dev->set_rx_antenna(antenna, chan);
                }
            }
        }

        ::uhd::usrp::multi_usrp::sptr get_device()
        {
            return _dev;
        }

        std::vector<size_t> get_channels() const
        {
            return _channels;
        }

    private:
        ::uhd::usrp::multi_usrp::sptr _dev;
        const std::vector<size_t> _channels;
    };

    usrp::sptr usrp::make(
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
    ) {
        return boost::make_shared<usrp_impl>(
            dev_args,
            is_tx,
            rate,
            center_freq,
            gain,
            lo_offset,
            clock_source,
            time_source,
            antenna,
            channels,
            subdev_spec
        );
    }

}} /* namespace gr::radar */

