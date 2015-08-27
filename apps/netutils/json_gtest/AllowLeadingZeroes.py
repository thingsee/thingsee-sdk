"""pymodel config"""

import SpewJson

# Big enough to produce otherwise correct object
SpewJson.max_length = 200
# Allow objects to close before maximum length is reached
SpewJson.min_length_is_max_length = False
# Allow integer part of number value to start with 0 (e.g. 01)
SpewJson.error_leading_zero = True
