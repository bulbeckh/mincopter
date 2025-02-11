
import curses

from draw import draw_none, draw_navigation, draw_logs

class NoneHandler:
	'''The most basic handler class. Does not do anything'''
	def __init__(self, queues=None):
		self.serq = queues[0]
		self.cmdq = queues[1]
		return

	def run(self):
		return

	def draw(self, stdscr):
		draw_none(stdscr, None)
		return

	def keypress(self, key):
		if key==curses.KEY_BACKSPACE:
			return ('back', None)
		else:
			return (None, None)

	def parse(self, msg):
		return

class Log:
	'''Represents a single log entry in the LogHandler'''
	def __init__(self, lognum : int, logsize : int, content : bytes):
		self.lognum = lognum
		self.logsize = logsize
		self.content = content

class LogHandler:
	'''Responsible for parsing and storing log messages'''
	def __init__(self, queues=None):
		self.n_message=0
		self.logs = []

		self.serq = queues[0]
		self.cmdq = queues[1]

		self.running = False

	def parse(self, msg):
		'''msg is the string sent from the mincopter

		For logs, this is of the format "SL00-%d-%d"
		'''
		eprint('parse method called from loghandler')
	
		if (msg[0:4] != 'SL00'):
			eprint(f'non-std message found: {msg}')
			return

		temp = msg.split('-')
		log_num = temp[1]
		log_size = temp[2]
		
		## Add a new entry into the list
		self.logs.append(Log(log_num, log_size, None))

	def run(self):
		'''Commence communication between console and mincopter to request logs and
		parse responses'''

		eprint("Sending showlogs commands")
		send_command('showlogs')
		
		while(True):
			resp = ser.readline().decode('utf-8')
			if resp=="END0":
				return
			else:
				self.parse(resp)

		return

	def keypress(self, key):
		if key==curses.KEY_BACKSPACE:
			return ('back', None)
		elif key==curses.KEY_ENTER or key==10 or key==13:
			## Run node
			self.cmdq.put_nowait('showlogs')
			return (None, None)
		else:
			return (None, None)

	def draw(self, stdscr):
		'''On first call to draw, start the communication process'''
		ctx = {
			'numlogs': len(self.logs)
		}
		draw_logs(stdscr, ctx)

class NavHandler:
	'''Responsible for navigation to a different screen'''
	def __init__(self, navtree, queues=None):
		self.tree = navtree
		self.line = 0

		self.serq = queues[0]
		self.cmdq = queues[1]
	
		## These are the rows of the navigation
		self.rows = list(navtree.keys())

	def keypress(self, key):
		if key==curses.KEY_UP:
			self.line = max(self.line-1, 0)
			return (None, None)
		elif key==curses.KEY_DOWN:
			self.line = min(self.line+1, len(self.rows)-1)
			return (None, None)
		elif key==curses.KEY_BACKSPACE:
			return ('back', None)
		elif key==curses.KEY_ENTER or key==10 or key==13:
			## TODO Fix this - horrible
			return ('forward', self.tree[list(self.tree.keys())[self.line]])
		else:
			return (None, None)

	def draw(self, stdscr):
		ctx = {
			'line': self.line,
			'rows': self.rows
		}
		draw_navigation(stdscr, ctx)

	def parse(self, msg):
		return
