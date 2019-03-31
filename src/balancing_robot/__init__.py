# #!/usr/bin/env python3
# -*- coding: utf-8 -*-
# noinspection PyUnresolvedReferences

from __future__ import print_function
from pkg_resources import get_distribution, DistributionNotFound

__all__ = ['Robot', 'Controller']
__author__ = 'Magnus Kvendseth Øye'

try:
    # Change here if project is renamed and does not equal the package name
    dist_name = __name__
    __version__ = get_distribution(dist_name).version
except DistributionNotFound:
    __version__ = '0.0.1'