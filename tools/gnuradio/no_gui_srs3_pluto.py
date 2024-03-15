#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: SRS-3 test flow without GUI
# GNU Radio version: v3.11.0.0git-352-g6efdc9d8

import signal
import sys
from argparse import ArgumentParser

from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import iio
from gnuradio import network
from gnuradio import satlab
from gnuradio.eng_arg import eng_float
from gnuradio.eng_arg import intx
from gnuradio.fft import window
from gnuradio.filter import firdes


class no_gui_srs3_flow(gr.top_block):

    def __init__(self, udp_addr='localhost', verbose=0):
        gr.top_block.__init__(self, "SRS-3 test flow without GUI", catch_exceptions=True)

        ##################################################
        # Parameters
        ##################################################
        self.udp_addr = udp_addr
        self.verbose = verbose

        ##################################################
        # Variables
        ##################################################
        self.rx_rate_kbps = rx_rate_kbps = 512
        self.tx_rate_kbps = tx_rate_kbps = 128
        self.rx_sps = rx_sps = 5
        self.rx_size = rx_size = 256
        self.rx_rs = rx_rs = True
        self.rx_rate = rx_rate = rx_rate_kbps * 1000
        self.rx_id = rx_id = 0
        self.rx_decrypt = rx_decrypt = False
        self.rx_crc = rx_crc = True
        self.rx_cc = rx_cc = True
        self.rx_auth = rx_auth = False
        self.tx_size = tx_size = 256
        self.tx_rs = tx_rs = True
        self.tx_rate = tx_rate = tx_rate_kbps * 1000
        self.tx_rand = tx_rand = True
        self.tx_preamble = tx_preamble = 8
        self.tx_key = tx_key = "0000000000000000000000000000000000000000000000000000000000000000"
        self.tx_id = tx_id = 0
        self.tx_freq = tx_freq = 2105.35e6
        self.tx_encrypt = tx_encrypt = False
        self.tx_crc = tx_crc = True
        self.tx_cc = tx_cc = True
        self.tx_bt = tx_bt = 0.90
        self.tx_auth = tx_auth = False
        self.samp_rate = samp_rate = 2.56e6
        self.rx_rand = rx_rand = True
        self.rx_key = rx_key = "0000000000000000000000000000000000000000000000000000000000000000"
        self.rx_freq = rx_freq = 2278.6e6
        self.rx_chan_samp_rate = rx_chan_samp_rate = rx_sps * rx_rate
        self.frame_size = frame_size = satlab.frame_size(rx_size, rx_crc, rx_rs, rx_cc, rx_id, rx_decrypt, rx_auth)

        ##################################################
        # Blocks
        ##################################################
        self.satlab_srs3_tx_0 = satlab.srs3_tx(
            samp_rate=samp_rate,
            rate=tx_rate,
            size=tx_size,
            idnum=tx_id,
            cc=tx_cc,
            rs=tx_rs,
            crc=tx_crc,
            rand=tx_rand,
            key=bytes.fromhex(tx_key) if len(tx_key) == 64 else '\x00' * 32,
            encrypt=tx_encrypt,
            auth=tx_auth,
            bt=tx_bt,
            preamble=tx_preamble,
            idleframes=0,
            max_in_flight=100,
        )
        self.satlab_srs3_rx_0 = satlab.srs3_rx(
            samp_rate=samp_rate,
            rate=rx_rate,
            size=rx_size,
            idnum=rx_id,
            cc=rx_cc,
            rs=rx_rs,
            crc=rx_crc,
            rand=rx_rand,
            key=bytes.fromhex(rx_key) if len(rx_key) == 64 else '\x00' * 32,
            auth=rx_auth,
            decrypt=rx_decrypt,
        )
        self.network_socket_pdu_0_1_0 = network.socket_pdu('UDP_CLIENT', udp_addr, '52004', 1500, False)
        self.network_socket_pdu_0_1 = network.socket_pdu('UDP_SERVER', '', '52002', 1500, False)

        self.iio_pluto_source_0 = iio.fmcomms2_source_fc32('ip:192.168.2.1' if 'ip:192.168.2.1' else iio.get_pluto_uri(), [True, True], 262144)
        self.iio_pluto_source_0.set_len_tag_key('packet_len')
        self.iio_pluto_source_0.set_frequency(int(rx_freq))
        self.iio_pluto_source_0.set_samplerate(int(samp_rate))
        self.iio_pluto_source_0.set_gain_mode(0, 'manual')
        self.iio_pluto_source_0.set_gain(0, 20)
        self.iio_pluto_source_0.set_quadrature(True)
        self.iio_pluto_source_0.set_rfdc(True)
        self.iio_pluto_source_0.set_bbdc(True)
        self.iio_pluto_source_0.set_filter_params('Auto', '', 0, 0)

        self.iio_pluto_sink_0 = iio.fmcomms2_sink_fc32('ip:192.168.2.1' if 'ip:192.168.2.1' else iio.get_pluto_uri(), [True, True], (int((samp_rate / tx_rate) * ((satlab.frame_size(tx_size, tx_crc, tx_rs, tx_cc, tx_id, tx_encrypt, tx_auth, True, True) + tx_preamble) * 8 + 10))), False)
        self.iio_pluto_sink_0.set_len_tag_key('packet_len')
        self.iio_pluto_sink_0.set_bandwidth(20000000)
        self.iio_pluto_sink_0.set_frequency(int(tx_freq))
        self.iio_pluto_sink_0.set_samplerate(int(samp_rate))
        self.iio_pluto_sink_0.set_attenuation(0, 0)
        self.iio_pluto_sink_0.set_filter_params('Auto', '', 0, 0)

        self.blocks_message_debug_0 = blocks.message_debug(True)

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.network_socket_pdu_0_1, 'pdus'), (self.satlab_srs3_tx_0, 'csp_in'))
        self.msg_connect((self.satlab_srs3_rx_0, 'csp_out'), (self.blocks_message_debug_0, 'print'))
        self.msg_connect((self.satlab_srs3_rx_0, 'csp_out'), (self.network_socket_pdu_0_1, 'pdus'))
        self.msg_connect((self.satlab_srs3_rx_0, 'csp_out'), (self.network_socket_pdu_0_1_0, 'pdus'))
        self.connect((self.iio_pluto_source_0, 0), (self.satlab_srs3_rx_0, 0))
        self.connect((self.satlab_srs3_tx_0, 0), (self.iio_pluto_sink_0, 0))

    def get_udp_addr(self):
        return self.udp_addr

    def set_udp_addr(self, udp_addr):
        self.udp_addr = udp_addr

    def get_verbose(self):
        return self.verbose

    def set_verbose(self, verbose):
        self.verbose = verbose

    def get_rx_rate_kbps(self):
        return self.rx_rate_kbps

    def set_rx_rate_kbps(self, rx_rate_kbps):
        self.rx_rate_kbps = rx_rate_kbps
        self.set_rx_rate(self.rx_rate_kbps * 1000)

    def get_tx_rate_kbps(self):
        return self.tx_rate_kbps

    def set_tx_rate_kbps(self, tx_rate_kbps):
        self.tx_rate_kbps = tx_rate_kbps
        self.set_tx_rate(self.tx_rate_kbps * 1000)

    def get_rx_sps(self):
        return self.rx_sps

    def set_rx_sps(self, rx_sps):
        self.rx_sps = rx_sps
        self.set_rx_chan_samp_rate(self.rx_sps * self.rx_rate)

    def get_rx_size(self):
        return self.rx_size

    def set_rx_size(self, rx_size):
        self.rx_size = rx_size
        self.set_frame_size(satlab.frame_size(self.rx_size, self.rx_crc, self.rx_rs, self.rx_cc, self.rx_id, self.rx_decrypt, self.rx_auth))
        self.satlab_srs3_rx_0.set_size(self.rx_size)

    def get_rx_rs(self):
        return self.rx_rs

    def set_rx_rs(self, rx_rs):
        self.rx_rs = rx_rs
        self.set_frame_size(satlab.frame_size(self.rx_size, self.rx_crc, self.rx_rs, self.rx_cc, self.rx_id, self.rx_decrypt, self.rx_auth))
        self.satlab_srs3_rx_0.set_rs(self.rx_rs)

    def get_rx_rate(self):
        return self.rx_rate

    def set_rx_rate(self, rx_rate):
        self.rx_rate = rx_rate
        self.set_rx_chan_samp_rate(self.rx_sps * self.rx_rate)
        self.satlab_srs3_rx_0.set_rate(self.rx_rate)

    def get_rx_id(self):
        return self.rx_id

    def set_rx_id(self, rx_id):
        self.rx_id = rx_id
        self.set_frame_size(satlab.frame_size(self.rx_size, self.rx_crc, self.rx_rs, self.rx_cc, self.rx_id, self.rx_decrypt, self.rx_auth))
        self.satlab_srs3_rx_0.set_id(self.rx_id)

    def get_rx_decrypt(self):
        return self.rx_decrypt

    def set_rx_decrypt(self, rx_decrypt):
        self.rx_decrypt = rx_decrypt
        self.set_frame_size(satlab.frame_size(self.rx_size, self.rx_crc, self.rx_rs, self.rx_cc, self.rx_id, self.rx_decrypt, self.rx_auth))
        self.satlab_srs3_rx_0.set_decrypt(self.rx_decrypt)

    def get_rx_crc(self):
        return self.rx_crc

    def set_rx_crc(self, rx_crc):
        self.rx_crc = rx_crc
        self.set_frame_size(satlab.frame_size(self.rx_size, self.rx_crc, self.rx_rs, self.rx_cc, self.rx_id, self.rx_decrypt, self.rx_auth))
        self.satlab_srs3_rx_0.set_crc(self.rx_crc)

    def get_rx_cc(self):
        return self.rx_cc

    def set_rx_cc(self, rx_cc):
        self.rx_cc = rx_cc
        self.set_frame_size(satlab.frame_size(self.rx_size, self.rx_crc, self.rx_rs, self.rx_cc, self.rx_id, self.rx_decrypt, self.rx_auth))
        self.satlab_srs3_rx_0.set_cc(self.rx_cc)

    def get_rx_auth(self):
        return self.rx_auth

    def set_rx_auth(self, rx_auth):
        self.rx_auth = rx_auth
        self.set_frame_size(satlab.frame_size(self.rx_size, self.rx_crc, self.rx_rs, self.rx_cc, self.rx_id, self.rx_decrypt, self.rx_auth))
        self.satlab_srs3_rx_0.set_auth(self.rx_auth)

    def get_tx_size(self):
        return self.tx_size

    def set_tx_size(self, tx_size):
        self.tx_size = tx_size
        self.satlab_srs3_tx_0.set_size(self.tx_size)

    def get_tx_rs(self):
        return self.tx_rs

    def set_tx_rs(self, tx_rs):
        self.tx_rs = tx_rs
        self.satlab_srs3_tx_0.set_rs(self.tx_rs)

    def get_tx_rate(self):
        return self.tx_rate

    def set_tx_rate(self, tx_rate):
        self.tx_rate = tx_rate
        self.satlab_srs3_tx_0.set_rate(self.tx_rate)

    def get_tx_rand(self):
        return self.tx_rand

    def set_tx_rand(self, tx_rand):
        self.tx_rand = tx_rand
        self.satlab_srs3_tx_0.set_rand(self.tx_rand)

    def get_tx_preamble(self):
        return self.tx_preamble

    def set_tx_preamble(self, tx_preamble):
        self.tx_preamble = tx_preamble
        self.satlab_srs3_tx_0.set_preamble(self.tx_preamble)

    def get_tx_key(self):
        return self.tx_key

    def set_tx_key(self, tx_key):
        self.tx_key = tx_key
        self.satlab_srs3_tx_0.set_key(bytes.fromhex(self.tx_key) if len(self.tx_key) == 64 else '\x00' * 32)

    def get_tx_id(self):
        return self.tx_id

    def set_tx_id(self, tx_id):
        self.tx_id = tx_id
        self.satlab_srs3_tx_0.set_id(self.tx_id)

    def get_tx_freq(self):
        return self.tx_freq

    def set_tx_freq(self, tx_freq):
        self.tx_freq = tx_freq
        self.iio_pluto_sink_0.set_frequency(int(self.tx_freq))

    def get_tx_encrypt(self):
        return self.tx_encrypt

    def set_tx_encrypt(self, tx_encrypt):
        self.tx_encrypt = tx_encrypt
        self.satlab_srs3_tx_0.set_encrypt(self.tx_encrypt)

    def get_tx_crc(self):
        return self.tx_crc

    def set_tx_crc(self, tx_crc):
        self.tx_crc = tx_crc
        self.satlab_srs3_tx_0.set_crc(self.tx_crc)

    def get_tx_cc(self):
        return self.tx_cc

    def set_tx_cc(self, tx_cc):
        self.tx_cc = tx_cc
        self.satlab_srs3_tx_0.set_cc(self.tx_cc)

    def get_tx_bt(self):
        return self.tx_bt

    def set_tx_bt(self, tx_bt):
        self.tx_bt = tx_bt
        self.satlab_srs3_tx_0.set_bt(self.tx_bt)

    def get_tx_auth(self):
        return self.tx_auth

    def set_tx_auth(self, tx_auth):
        self.tx_auth = tx_auth
        self.satlab_srs3_tx_0.set_auth(self.tx_auth)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.iio_pluto_sink_0.set_samplerate(int(self.samp_rate))
        self.iio_pluto_source_0.set_samplerate(int(self.samp_rate))

    def get_rx_rand(self):
        return self.rx_rand

    def set_rx_rand(self, rx_rand):
        self.rx_rand = rx_rand
        self.satlab_srs3_rx_0.set_rand(self.rx_rand)

    def get_rx_key(self):
        return self.rx_key

    def set_rx_key(self, rx_key):
        self.rx_key = rx_key
        self.satlab_srs3_rx_0.set_key(bytes.fromhex(self.rx_key) if len(self.rx_key) == 64 else '\x00' * 32)

    def get_rx_freq(self):
        return self.rx_freq

    def set_rx_freq(self, rx_freq):
        self.rx_freq = rx_freq
        self.iio_pluto_source_0.set_frequency(int(self.rx_freq))

    def get_rx_chan_samp_rate(self):
        return self.rx_chan_samp_rate

    def set_rx_chan_samp_rate(self, rx_chan_samp_rate):
        self.rx_chan_samp_rate = rx_chan_samp_rate

    def get_frame_size(self):
        return self.frame_size

    def set_frame_size(self, frame_size):
        self.frame_size = frame_size


def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "-d", "--udp-addr", dest="udp_addr", type=str, default='localhost',
        help="Set host name where telemetry send to [default=%(default)r]")
    parser.add_argument(
        "-v", "--verbose", dest="verbose", type=intx, default=0,
        help="Set Enable Noisy Output [default=%(default)r]")
    return parser


def main(top_block_cls=no_gui_srs3_flow, options=None):
    if options is None:
        options = argument_parser().parse_args()
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print("Error: failed to enable real-time scheduling.")
    tb = top_block_cls(udp_addr=options.udp_addr, verbose=options.verbose)

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    tb.wait()


if __name__ == '__main__':
    main()
