#!/usr/bin/env python3

import argparse
import logging

# from oxart.devices.signalhound.api_driver import Signalhound
from oxart.devices.signalhound.driver import Signalhound
from sipyco.pc_rpc import simple_server_loop
import sipyco.common_args as sca

logger = logging.getLogger(__name__)


def get_argparser():
    parser = argparse.ArgumentParser(description="ARTIQ controller for Signalhound")
    sca.simple_network_args(parser, 5025)
    sca.verbosity_args(parser)
    return parser


def main():
    args = get_argparser().parse_args()
    sca.init_logger_from_args(args)

    logger.info('Initialising Signalhound')

    dev = Signalhound()
    logger.info(dev.ping())

    try:
        simple_server_loop({"Spectrum_analyser": dev}, args.bind, args.port)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
