#!/usr/bin/env python

from FSM import State, FiniteStateMachine
import time

stateMachine = None
firstState = None
secondState = None
thirdState = None

def firstStateEnter():
	print 'first state enter'

def firstStateUpdate():
	print 'first state update'
	stateMachine.transitionTo(secondState)
	print 'first state update done'

def firstStateExit():
	print 'first state exit'

def secondStateEnter():
	print 'second state enter'

def secondStateUpdate():
	print 'second state update'
	stateMachine.transitionTo(thirdState)
	print 'second state update done'

def thirdStateUpdate():
	print 'third state update'

firstState = State(firstStateEnter, firstStateUpdate, firstStateExit)
secondState = State(secondStateEnter, secondStateUpdate, None)
thirdState = State(None, thirdStateUpdate, None)

stateMachine = FiniteStateMachine(firstState)

while True:
	print 'state machine pre-update'
	stateMachine.update()
	print 'state machine post-update'
	time.sleep(0.5)
