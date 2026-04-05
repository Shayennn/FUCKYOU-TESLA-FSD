CXX      ?= c++
CXXFLAGS := -std=c++14 -Wall -Wextra -O2

TEST_BINS := test_handlers test_flood_logic

test_handlers: test_handlers.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

test_flood_logic: test_flood_logic.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

test: $(TEST_BINS)
	./test_handlers
	./test_flood_logic

clean:
	rm -f $(TEST_BINS)

.PHONY: test clean
