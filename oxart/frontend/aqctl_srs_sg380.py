#!/usr/bin/env python3

import argparse
import logging

from oxart.devices.srs_synth.driver import SRSSynth
from sipyco.pc_rpc import simple_server_loop
import sipyco.common_args as sca

logger = logging.getLogger(__name__)


def get_argparser():
    parser = argparse.ArgumentParser(description="ARTIQ controller for SRS Synth")
    parser.add_argument("--addr", help="device's ip address")
    sca.simple_network_args(parser, 4300)
    sca.verbosity_args(parser)
    return parser


def main():
    args = get_argparser().parse_args()
    sca.init_logger_from_args(args)

    logger.info('Initialising SRS synth {}'.format(args.addr))

    dev = SRSSynth(args.addr)
    logger.info(dev.identity())

    try:
        simple_server_loop({"Synth": dev}, args.bind, args.port)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
