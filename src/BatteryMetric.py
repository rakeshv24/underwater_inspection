# NOTE! the input distribution should be a continuous distribution
# scipy.stats.<distribution>

class Batter():
	# constructor
	def __init__(self, battery, distribution):
		# the battery here is defined as the max distance the AUV can travel
		# if we start at 50 and move a distance of 2, then the battery level would be 48
		self._battery = battery

		# to simulate noise, we use a scipy random distribution specified on contruction
		self._distribution = distribution

	# getters/setters
	@property
	def battery(self):
		return self._battery

	# member methods
	def depleteBattery(self, dist):
		# depletes the battery by the distance
		# checking if we have a random distribution
		if self._distribution is None:
			self._battery -= dist
		else:
			self._battery -= dist * self._distribution.rvs()
