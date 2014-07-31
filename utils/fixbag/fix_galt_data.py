#!/usr/bin/env python3

import sys
import os


def main():
    input_path = 'test_in'
    output_path = 'test_out'

    input_path = os.path.normpath(os.path.expanduser(input_path))
    output_path = os.path.normpath(os.path.expanduser(output_path))

    print("input path: ", input_path)
    print("output path: ", output_path)

    if not (os.path.isdir(input_path) and os.path.isdir(output_path)):
        print('input and output must be folder')
        sys.exit(1)



if __name__ == '__main__':
    sys.exit(main())
