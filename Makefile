CC := g++
CFLAGS := -Wall -Wextra -W -std=c++17
CPPFLAGS := -Isrc

BIN := raytrace
SRCDIR := src

SRC := $(filter-out $(SRCDIR)/main.cpp,$(wildcard $(SRCDIR)/*.cpp))
OBJ := $(SRC:.cpp=.o)

$(BIN): $(SRCDIR)/main.cpp $(OBJ)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^

%.o: %.cpp
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

.PHONY: test clean
test: $(BIN)
	./$(BIN) scenes/basic.ray

clean:
	rm -f $(OBJ) $(BIN)
