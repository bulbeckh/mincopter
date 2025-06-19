import curses

''' Drawing
Functions for updating the screen with the content from a manager
'''

def draw_none(stdscr, ctx):
	stdscr.clear()
	stdscr.addstr(0,100, 'THIS IS A NONE SCREEN', curses.A_REVERSE)
	stdscr.addstr(0,100, 'THIS IS A NONE SCREEN')
	stdscr.refresh()
	return

def draw_navigation(stdscr, ctx):
	'''Draw a navigation screen, given a navigation context (ctx) that contains elements'''
	stdscr.clear()

	height, width = stdscr.getmaxyx()

	## Add heading
	stdscr.addstr(0, 0, "Item", curses.A_BOLD)

	## Add elements to screen
	for i in range(0,len(ctx['rows'])):
		if i == ctx['line']:
			stdscr.addstr(i+1,0, ctx['rows'][i].ljust(width), curses.A_REVERSE)
		else:
			stdscr.addstr(i+1,0, ctx['rows'][i])

	stdscr.refresh()

def draw_logs(stdscr, ctx):
	'''Draw a list of logs on the screen'''
	stdscr.clear()

	height, width = stdscr.getmaxyx()
	
	stdscr.addstr(0, 0, f'Number of logs: {ctx["numlogs"]}', curses.A_BOLD)

	stdscr.refresh()

