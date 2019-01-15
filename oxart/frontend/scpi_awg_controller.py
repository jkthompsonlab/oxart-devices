#!/usr/bin/env python3.5

import argparse
import sys

from oxart.devices.scpi_device.awg import SCPIAWG
from artiq.protocols.pc_rpc import simple_server_loop
from artiq.tools import simple_network_args, init_logger
from oxart.tools import add_common_args


def get_argparser():
    parser = argparse.ArgumentParser(description="ARTIQ controller for SCPI AWGs")
    parser.add_argument("-i", "--ipaddr", default=None,
                        help="IP address of device")
    parser.add_argument("-s", "--serialnumber", default=None,
                        help="Serial number of device to check identity")
    simple_network_args(parser, 4004)
    add_common_args(parser)
    return parser


def main():
    args = get_argparser().parse_args()
    init_logger(args)

    if args.ipaddr is None:
        print("You need to specify -i/--ipaddr"
            "Use --help for more information.")
        sys.exit(1)

    dev = SCPIAWG(addr=args.ipaddr, serial_number=args.serialnumber)

    try:
        simple_server_loop({"scpi_awg": dev}, args.bind, args.port)
    finally:
        dev.close()

if __name__ == "__main__":
    main()
