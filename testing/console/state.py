
from helper import eprint

class StateManager:
	'''Responsible for managing navigation between screens and routing data between the different
		handler classes'''

	def __init__(self, navtree):
		self.tree = navtree
		self.path = []
		
		# This is the variable that is updated in response to keypresses
		# Starts out at the root of the tree
		self.current_state = self.tree

	def respond_keypress(self, keypress):
		'''Updates state according to keypresses'''
		## NOTE I actually think this should just forward the keypress to the screen if it is NOT a navigation screen

		##eprint(self.current_state)
		response, node = self.current_state.keypress(keypress)
		
		if (response==None):
			return
		elif (response=='back'):
			## handle back up hierarchy
			if (len(self.path)==0):
				## At root node
				return 
			else:
				self.current_state = self.path.pop()

		elif (response=='forward'):
			## descend hierarchy
			self.path.append(self.current_state)
			self.current_state = node
			return
