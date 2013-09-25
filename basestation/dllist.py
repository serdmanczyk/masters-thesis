#!/usr/bin/env python
# http://fw-geekycoder.blogspot.com/2012/04/how-to-implement-doubly-linked-list-in.html
# replaced Node with Element not to confuse with 'node' in Xbee

class Element(object):
	def __init__(self):
		self.next = None
		self.previous = None
		self.element = None

class LinkedList(object):
	def __init__(self):
		self.n = 0
		self.last = Element()
		self.first = self.last
	
	def append(self, element):
		self.last.element = element
		self.last.next = Element()
		tmp = self.last
		self.last = self.last.next
		self.last.previous = tmp
		self.n += 1
		return tmp
	
	def popfront(self):
		if self.n == 0: return None
		e = self.first
		self.first = self.first.next
		self.n -= 1
		return e

	def front(self):
		if self.n == 0: return None
		return self.first

	def popback(self):
		if self.n == 0: return None
		e = self.last.previous
		self.last = self.last.previous
		self.last.next = Element()
		self.n -= 1
		return e

	def back(self):
		if self.n == 0: return None
		return self.last.previous

	def size(self):
		return self.n
	
	def elements(self):
		i = self.first
		while i.element:
			yield i.element
			i = i.next
	
if __name__ == "__main__":
	l = LinkedList()
	
	assert None == l.front()
	assert None == l.back()
	
	derf = l.append(1)
	derf = l.append(2)
	derf = l.append(3)

	assert 1 == l.popfront().element
	assert 2 == l.popfront().element
	assert 3 == l.popfront().element
	
	l.append(1)
	l.append(2)
	l.append(3)
	
	assert 3 == l.popback().element
	assert 2 == l.popback().element
	assert 1 == l.popback().element
	
	l.append(1)
	assert 1 == l.back().element
	
	l.append(1)
	assert 1 == l.front().element

	mine = LinkedList()