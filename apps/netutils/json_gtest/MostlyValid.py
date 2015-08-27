"""pymodel config"""

import SpewJson

# Big enough to rarely hit on incorrect object
SpewJson.max_length = 2000
# Allow objects to close before maximum length is reached
SpewJson.min_length_is_max_length = False

