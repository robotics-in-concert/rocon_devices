#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
#
#################################################################################

##############################################################################
# Exceptions
##############################################################################


class PhueException(Exception):
    def __init__(self, error_type, message):
        self.error_type = error_type
        self.message = message


class PhueRegistrationException(PhueException):
    pass
