#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: GFSK_RX_RED
# GNU Radio version: 3.10.9.2

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio import analog
import math
from gnuradio import blocks
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import iio
from gnuradio import robomaster
from gnuradio import zeromq



class GFSK_RX_RED(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "GFSK_RX_RED", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("GFSK_RX_RED")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "GFSK_RX_RED")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)

        ##################################################
        # Variables
        ##################################################
        self.sps = sps = 47
        self.samp_rate = samp_rate = 1000000
        self.symb_rate = symb_rate = samp_rate/sps
        self.pam2 = pam2 = [-1, 1]

        ##################################################
        # Blocks
        ##################################################

        self.zeromq_pub_sink_0_0_1 = zeromq.pub_sink(gr.sizeof_char, 1, 'tcp://127.0.0.1:6667', 100, False, (-1), '', True, True)
        self.zeromq_pub_sink_0_0 = zeromq.pub_sink(gr.sizeof_char, 1, 'tcp://127.0.0.1:6666', 100, False, (-1), '', True, True)
        self.zeromq_pub_sink_0 = zeromq.pub_sink(gr.sizeof_char, 1, 'tcp://127.0.0.1:5555', 100, False, (-1), '', True, True)
        self.robomaster_gfsk_receiver_noise_0_1 = robomaster.gfsk_receiver_noise(900, 47, 27)
        self.robomaster_gfsk_receiver_noise_0 = robomaster.gfsk_receiver_noise(900, 47, 27)
        self.robomaster_gfsk_receiver_0 = robomaster.gfsk_receiver(1100, 47, 27)
        self.low_pass_filter_0_1_1 = filter.fir_filter_fff(
            1,
            firdes.low_pass(
                1,
                samp_rate,
                410000,
                40000,
                window.WIN_BLACKMAN,
                6.76))
        self.low_pass_filter_0_1 = filter.fir_filter_fff(
            1,
            firdes.low_pass(
                1,
                samp_rate,
                480000,
                40000,
                window.WIN_BLACKMAN,
                6.76))
        self.low_pass_filter_0 = filter.fir_filter_fff(
            1,
            firdes.low_pass(
                1,
                samp_rate,
                250000,
                50000,
                window.WIN_BLACKMAN,
                6.76))
        self.iio_fmcomms2_source_0 = iio.fmcomms2_source_fc32('ip:192.168.2.1', [False, False, True, True], 65536)
        self.iio_fmcomms2_source_0.set_len_tag_key('packet_len')
        self.iio_fmcomms2_source_0.set_frequency(432600000)
        self.iio_fmcomms2_source_0.set_samplerate((samp_rate*2))
        if False:
            self.iio_fmcomms2_source_0.set_gain_mode(0, 'slow_attack')
            self.iio_fmcomms2_source_0.set_gain(0, 64)
        if True:
            self.iio_fmcomms2_source_0.set_gain_mode(1, 'slow_attack')
            self.iio_fmcomms2_source_0.set_gain(1, 64)
        self.iio_fmcomms2_source_0.set_quadrature(True)
        self.iio_fmcomms2_source_0.set_rfdc(True)
        self.iio_fmcomms2_source_0.set_bbdc(True)
        self.iio_fmcomms2_source_0.set_filter_params('Auto', '', 0, 0)
        self.freq_xlating_fir_filter_xxx_0_0_0_2 = filter.freq_xlating_fir_filter_ccc(2, firdes.low_pass(1.0,samp_rate*2,270000,40000), 600000, (samp_rate*2))
        self.freq_xlating_fir_filter_xxx_0_0_0_1 = filter.freq_xlating_fir_filter_ccc(2, firdes.low_pass(1.0,samp_rate*2,420000,40000), (-100000), (samp_rate*2))
        self.freq_xlating_fir_filter_xxx_0_0_0 = filter.freq_xlating_fir_filter_ccc(2, firdes.low_pass(1.0,samp_rate*2,480000,40000), (-400000), (samp_rate*2))
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_gr_complex*1, '/home/tdt/radio_data/8.4.bin', False)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.analog_quadrature_demod_cf_0_0_1 = analog.quadrature_demod_cf(1.5)
        self.analog_quadrature_demod_cf_0_0 = analog.quadrature_demod_cf(1.5)
        self.analog_quadrature_demod_cf_0 = analog.quadrature_demod_cf(1.5)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_quadrature_demod_cf_0, 0), (self.low_pass_filter_0, 0))
        self.connect((self.analog_quadrature_demod_cf_0_0, 0), (self.low_pass_filter_0_1, 0))
        self.connect((self.analog_quadrature_demod_cf_0_0_1, 0), (self.low_pass_filter_0_1_1, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0_0_0, 0), (self.analog_quadrature_demod_cf_0_0, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0_0_0_1, 0), (self.analog_quadrature_demod_cf_0_0_1, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0_0_0_2, 0), (self.analog_quadrature_demod_cf_0, 0))
        self.connect((self.iio_fmcomms2_source_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.iio_fmcomms2_source_0, 0), (self.freq_xlating_fir_filter_xxx_0_0_0, 0))
        self.connect((self.iio_fmcomms2_source_0, 0), (self.freq_xlating_fir_filter_xxx_0_0_0_1, 0))
        self.connect((self.iio_fmcomms2_source_0, 0), (self.freq_xlating_fir_filter_xxx_0_0_0_2, 0))
        self.connect((self.low_pass_filter_0, 0), (self.robomaster_gfsk_receiver_0, 0))
        self.connect((self.low_pass_filter_0_1, 0), (self.robomaster_gfsk_receiver_noise_0, 0))
        self.connect((self.low_pass_filter_0_1_1, 0), (self.robomaster_gfsk_receiver_noise_0_1, 0))
        self.connect((self.robomaster_gfsk_receiver_0, 0), (self.zeromq_pub_sink_0, 0))
        self.connect((self.robomaster_gfsk_receiver_noise_0, 0), (self.zeromq_pub_sink_0_0, 0))
        self.connect((self.robomaster_gfsk_receiver_noise_0_1, 0), (self.zeromq_pub_sink_0_0_1, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "GFSK_RX_RED")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_symb_rate(self.samp_rate/self.sps)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_symb_rate(self.samp_rate/self.sps)
        self.freq_xlating_fir_filter_xxx_0_0_0.set_taps(firdes.low_pass(1.0,self.samp_rate*2,480000,40000))
        self.freq_xlating_fir_filter_xxx_0_0_0_1.set_taps(firdes.low_pass(1.0,self.samp_rate*2,420000,40000))
        self.freq_xlating_fir_filter_xxx_0_0_0_2.set_taps(firdes.low_pass(1.0,self.samp_rate*2,270000,40000))
        self.iio_fmcomms2_source_0.set_samplerate((self.samp_rate*2))
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, 250000, 50000, window.WIN_BLACKMAN, 6.76))
        self.low_pass_filter_0_1.set_taps(firdes.low_pass(1, self.samp_rate, 480000, 40000, window.WIN_BLACKMAN, 6.76))
        self.low_pass_filter_0_1_1.set_taps(firdes.low_pass(1, self.samp_rate, 410000, 40000, window.WIN_BLACKMAN, 6.76))

    def get_symb_rate(self):
        return self.symb_rate

    def set_symb_rate(self, symb_rate):
        self.symb_rate = symb_rate

    def get_pam2(self):
        return self.pam2

    def set_pam2(self, pam2):
        self.pam2 = pam2




def main(top_block_cls=GFSK_RX_RED, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
