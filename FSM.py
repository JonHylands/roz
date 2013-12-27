#!/usr/bin/env python

import time

def millis():
	return int(round(time.time() * 1000))

class State:

	def __init__(self, enterFunction, updateFunction, exitFunction):
		self.userEnter = enterFunction
		self.userUpdate = updateFunction
		self.userExit = exitFunction

	def enter(self):
		self.startTimeMillis = millis()
		if self.userEnter is not None:
			self.userEnter()

	def update(self):
		if self.userUpdate is not None:
			self.userUpdate()

	def exit(self):
		if self.userExit is not None:
			self.userExit()

	def elapsedTimeMillis(self):
		return millis() - self.startTimeMillis

class FiniteStateMachine:

	def __init__(self, startState):
		self.currentState = startState
		self.nextState = startState
		self.needToTriggerEnter = True
		self.cycleCount = 0

	def transitionTo(self, newState):
		self.nextState = newState

	def getCycleCount(self):
		return self.cycleCount

	def getCurrentStateMillis(self):
		return self.currentState.elapsedTimeMillis()

	def update(self):
		if self.needToTriggerEnter:
			self.currentState.enter()
			self.needToTriggerEnter = False
		if self.currentState != self.nextState:
			self.currentState.exit()
			self.currentState = self.nextState
			self.currentState.enter()
			self.cycleCount = True
		self.currentState.update()
