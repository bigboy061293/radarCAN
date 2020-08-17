import time
import curses
import threading
from curses import wrapper
from time import sleep

x = 0

def inputThread(stdscr):
    global x
    while True:
        c = stdscr.getch()
        curses.flushinp()
        if c == ord('a'):
            x -= 1
        elif c == ord('d'):
            x += 1
        sleep(0.05)
        stdscr.addstr("inputThread:" + str(x) + "\n" + "c:" + str(c))

def main(stdscr):
    curses.initscr()
    stdscr.clear()

    #t = threading.Thread(name ='daemon', target=inputThread, args=(stdscr,))
    #t.setDaemon(True)
    #t.start()

    while True:
        stdscr.clear()
        stdscr.addstr("vlllll \n")
        sleep(0.5)
	stdscr.refresh()
wrapper(main)
