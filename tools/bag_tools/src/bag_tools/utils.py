from __future__ import absolute_import, print_function, division
import argparse as ap
import os
import errno


def get_io_parser():
    """
    :return: parent parser that handles inputs and outputs
    """
    parser = ap.ArgumentParser(add_help=False)
    parser.add_argument('input', nargs='+', help='Input directory or bagfiles.')
    parser.add_argument('-o', '--output-dir',
                        help=('Output directory of bagfiles.'
                              'If you did not specify one, the resulting '
                              'bagfiles will put in the same folder with '
                              'a suffix, '
                              'usually _fixed).'))
    parser.add_argument('-r', '--recursive', action='store_true',
                        help=('Search recursively in input directory for '
                              'bagfiles.'))

    return parser


def get_common_options_parser():
    """
    :return: parent parser that handles common options
    """
    parser = ap.ArgumentParser(add_help=False)
    parser.add_argument('-f', '--force-yes', action='store_true',
                        help='Force yes, without prompting')
    parser.add_argument('-q', '--quiet', action='store_true',
                        help='Suppress command-line output'
                             ' and log to $ROS_HOME')
    parser.add_argument('-m', '--move-fixed', action='store_true',
                        help='Move fixed bag to output dir')

    return parser


def get_all_files_with_ext_in_dir(directory, extension, recursive=False):
    """
    :param directory: absolute path of directory
    :param extension: file extension, either (.xxx) or just (xxx)
    :param recursive: true will search for files in all sub folders
    :return: list of files
    """
    all_files = []

    # Check if this directory exists
    if not os.path.isdir(directory):
        raise IOError('Directory not found: ', directory)

    # Check if this is a valid extension
    if not extension.startswith('.'):
        extension = '.' + extension

    # Go through this directory and find all files with this extension
    for root, dirs, files in os.walk(directory):
        files = [file for file in files if file.endswith(extension)]
        if files:
            all_files.extend([os.path.join(root, file) for file in files])
        # stop if we are not looking for files recursively
        if not recursive:
            break

    return all_files


def get_all_bagfiles_in_dir(directory, recursive=False):
    return get_all_files_with_ext_in_dir(directory, '.bag', recursive)


class UserChoice:
    YES, NO, ALL, ABORT = range(4)


def query_user_choice(message):
    choice = raw_input(
        'Do you wish to {0:s}? [Y]es, [N]o, [A]ll, [A]bort)'.format(
            message)).lower()
    if choice in ('y', 'yes'):
        return UserChoice.YES
    elif choice in ('n', 'no'):
        return UserChoice.NO
    elif choice in ('a', 'all'):
        return UserChoice.ALL
    else:
        return UserChoice.ABORT


def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        print("{0:s} already exists.".format(path))
        if exception.errno != errno.EEXIST:
            raise
