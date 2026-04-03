CXX      ?= c++
CXXFLAGS := -std=c++14 -Wall -Wextra -O2

test_handlers: test_handlers.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

test: test_handlers
	./test_handlers

clean:
	rm -f test_handlers

.PHONY: test clean
