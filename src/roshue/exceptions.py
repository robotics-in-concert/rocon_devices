#!/usr/bin/env python
##############################################################################
# Exceptions
##############################################################################


class PhueException(Exception):
    def __init__(self, error_type, message):
        self.error_type = error_type
        self.message = message


class PhueRegistrationException(PhueException):
    pass
