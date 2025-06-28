#!/usr/bin/env python3
# coding: UTF-8
import rospy
import sys
import yaml
import argparse
import os


def main(args):
    with open(os.path.join(args.config), "r") as yml:
        config = yaml.safe_load(yml)

    # set controller_list
    rospy.set_param(args.param_name, config)


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default="")
    parser.add_argument("--param_name", type=str, default="")
    return parser.parse_args(args)


if __name__ == "__main__":
    args = parse_arguments(rospy.myargv()[1:])
    main(args)
