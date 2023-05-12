#!/usr/bin/env python3

# Licensed under the Apache License, Version 2.0 or the MIT License.
# SPDX-License-Identifier: Apache-2.0 OR MIT
# Copyright Tock Contributors 2023.

'''
Check if all of the available capsules are documented in the README.
'''


import os
import re

SKIP = ['/mod.rs',
        'src/lib.rs',
        '/test',
        'src/driver.rs',
        'src/rf233_const.rs']


documented_capsules = []
implemented_capsules = []

# Find all documented capsules
with open('capsules/README.md') as f:
	for l in f:
		items = re.findall(r".*\((src/.*?)\).*", l)
		if len(items) > 0:
			documented_capsules.extend(f'capsules/{item}' for item in items)
# Find all capsule source files.
for subdir, dirs, files in os.walk(os.fsencode('capsules/src/')):
	for file in files:
		filepath = os.fsdecode(os.path.join(subdir, file))

		# Get just the part after /src, effectively.
		folders = filepath.split('/')
		filepath = '/'.join(folders[:3])

		# Skip some noise.
		for skip in SKIP:
			if skip in filepath:
				break
		else:
			implemented_capsules.append(filepath)


# Calculate what doesn't seem to be documented.
missing = list(set(implemented_capsules) - set(documented_capsules))

# Calculate what has been removed
removed = list(set(documented_capsules) - set(implemented_capsules))


print('The following capsules do not seem to be documented:')
for m in sorted(missing):
	print(f' - {m}')


print('The following capsules seem to have been removed:')
for m in sorted(removed):
	print(f' - {m}')

