class DetClass():


	def __init__(self, msg_det, is_processed):
		self._msg_det = msg_det
		self._is_processed = is_processed


	@property
	def msg_det(self):
		# геттер для атрибута msg_det
		return self._msg_det
	

	@property
	def is_processed(self):
		# геттер для атрибута is_processed
		return self._is_processed


	@msg_det.setter
	def msg_det(self, new_msg_det):
		# сеттер для атрибута msg_det
		self._msg_det = new_msg_det


	@is_processed.setter
	def is_processed(self, new_is_processed):
		# сеттер для атрибута is_processed
		self._is_processed = new_is_processed
