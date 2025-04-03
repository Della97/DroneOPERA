from hwcounter import Timer, count, count_end
from time import sleep
from math import sqrt

with Timer() as t:
	sleep(1)

print(f'elapsed cycles: {t.cycles}')

with Timer() as t:
	sleep(0.5)

print(f'elapsed cycles: {t.cycles}')