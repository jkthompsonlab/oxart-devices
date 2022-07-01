#!/usr/bin/env python3

import argparse
import logging

from oxart.devices.picoscope.driver import PicoScope
from sipyco.pc_rpc import simple_server_loop
import sipyco.common_args as sca

logger = logging.getLogger(__name__)


def get_argparser():
    parser = argparse.ArgumentParser(description="ARTIQ controller for Picoscope")
    sca.simple_network_args(parser, 4005)
    sca.verbosity_args(parser)
    return parser


def main():
    args = get_argparser().parse_args()
    sca.init_logger_from_args(args)

    logger.info('Initialising Picoscope')

    dev = PicoScope()
    logger.info(dev.ping())

    try:
        simple_server_loop({"Picoscope": dev}, args.bind, args.port)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
