#!/usr/bin/env python
from __future__ import print_function

import codecs
import collections
import fnmatch
import functools
import os.path
import subprocess
import sys


def find_all(a_str, sub):
    for i, line in enumerate(a_str.splitlines()):
        column = 0
        while True:
            column = line.find(sub, column)
            if column == -1:
                break
            yield i, column
            column += len(sub)


command = ['git', 'ls-files', '-s']
proc = subprocess.Popen(command, stdout=subprocess.PIPE)
output, err = proc.communicate()
lines = [x.split() for x in output.decode('utf-8').splitlines()]
EXECUTABLE_BIT = {
    s[3].strip(): int(s[0]) for s in lines
}
files = [s[3].strip() for s in lines]
files.sort()

file_types = ('.h', '.c', '.cpp', '.tcc', '.yaml', '.yml', '.ini', '.txt', '.ico',
              '.py', '.html', '.js', '.md', '.sh', '.css', '.proto', '.conf', '.cfg')
cpp_include = ('*.h', '*.c', '*.cpp', '*.tcc')
ignore_types = ('.ico',)

LINT_FILE_CHECKS = []
LINT_CONTENT_CHECKS = []


def run_check(lint_obj, fname, *args):
    include = lint_obj['include']
    exclude = lint_obj['exclude']
    func = lint_obj['func']
    if include is not None:
        for incl in include:
            if fnmatch.fnmatch(fname, incl):
                break
        else:
            return None
    for excl in exclude:
        if fnmatch.fnmatch(fname, excl):
            return None
    return func(*args)


def run_checks(lints, fname, *args):
    for lint in lints:
        add_errors(fname, run_check(lint, fname, *args))


def _add_check(checks, func, include=None, exclude=None):
    checks.append({
        'include': include,
        'exclude': exclude or [],
        'func': func,
    })


def lint_file_check(**kwargs):
    def decorator(func):
        _add_check(LINT_FILE_CHECKS, func, **kwargs)
        return func
    return decorator


def lint_content_check(**kwargs):
    def decorator(func):
        _add_check(LINT_CONTENT_CHECKS, func, **kwargs)
        return func
    return decorator


def lint_content_find_check(find, **kwargs):
    decor = lint_content_check(**kwargs)

    def decorator(func):
        def new_func(content):
            for line, col in find_all(content, find):
                err = func()
                return "{err} See line {line}:{col}.".format(err=err, line=line+1, col=col+1)
        return decor(new_func)
    return decorator


@lint_file_check(include=['*.ino'])
def lint_ino(fname):
    return "This file extension (.ino) is not allowed. Please use either .cpp or .h"


@lint_file_check(exclude=['*{}'.format(f) for f in file_types] + [
    '.clang-*', '.dockerignore', '.editorconfig', '*.gitignore', 'LICENSE', 'pylintrc',
    'MANIFEST.in', 'docker/Dockerfile*', 'docker/rootfs/*', 'script/*'
])
def lint_ext_check(fname):
    return "This file extension is not a registered file type. If this is an error, please " \
           "update the script/ci-custom.py script."


@lint_file_check(exclude=[
    'docker/rootfs/*', 'script/*', 'setup.py'
])
def lint_executable_bit(fname):
    ex = EXECUTABLE_BIT[fname]
    if ex != 100644:
        return 'File has invalid executable bit {}. If running from a windows machine please ' \
               'see disabling executable bit in git.'.format(ex)
    return None


@lint_content_find_check('\t', exclude=[
    'esphome/dashboard/static/ace.js', 'esphome/dashboard/static/ext-searchbox.js',
    'script/.neopixelbus.patch',
])
def lint_tabs():
    return "File contains tab character. Please convert tabs to spaces."


@lint_content_find_check('\r')
def lint_newline():
    return "File contains windows newline. Please set your editor to unix newline mode."


@lint_content_check()
def lint_end_newline(content):
    if content and not content.endswith('\n'):
        return "File does not end with a newline, please add an empty line at the end of the file."
    return None


@lint_content_find_check('"esphome.h"', include=cpp_include, exclude=['tests/custom.h'])
def lint_esphome_h():
    return ("File contains reference to 'esphome.h' - This file is "
            "auto-generated and should only be used for *custom* "
            "components. Please replace with references to the direct files.")


@lint_content_check(include=['*.h'])
def lint_pragma_once(content):
    if '#pragma once' not in content:
        return ("Header file contains no 'pragma once' header guard. Please add a "
                "'#pragma once' line at the top of the file.")
    return None


errors = collections.defaultdict(list)


def add_errors(fname, errs):
    if not isinstance(errs, list):
        errs = [errs]
    errs = [x for x in errs if x is not None]
    for err in errs:
        if not isinstance(err, str):
            raise ValueError("Error is not instance of string!")
    if not errs:
        return
    errors[fname].extend(errs)


for fname in files:
    _, ext = os.path.splitext(fname)
    run_checks(LINT_FILE_CHECKS, fname, fname)
    if ext in ('.ico',):
        continue
    try:
        with codecs.open(fname, 'r', encoding='utf-8') as f_handle:
            content = f_handle.read()
    except UnicodeDecodeError:
        add_errors(fname, "File is not readable as UTF-8. Please set your editor to UTF-8 mode.")
        continue
    run_checks(LINT_CONTENT_CHECKS, fname, content)

for f, errs in sorted(errors.items()):
    print("\033[0;32m************* File \033[1;32m{}\033[0m".format(f))
    for err in errs:
        print(err)
    print()

sys.exit(len(errors))
