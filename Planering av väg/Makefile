SRC_DIR := .
OBJ_DIR := obj
SRC_FILES := $(wildcard $(SRC_DIR)/*.cc)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cc, $(OBJ_DIR)/%.o,$(SRC_FILES))
CCC = g++
CFLAGS = -std=c++17 -g -O2 -Weffc++ -Wall -Wextra  -Wno-unused-parameter
SFML_FLAGS = -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio

.PHONY:
run:    $(OBJ_DIR)/main.out Makefile
		$(OBJ_DIR)/main.out

$(OBJ_DIR)/main.out:    $(OBJ_FILES)
	$(CCC) $(OBJ_FILES) $(SFML_FLAGS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cc
	$(CCC) $(CFLAGS) -c -o $@ $<

.PHONY:
städa: clean
clean:
		rm -f $(OBJ_DIR)/*.out
		rm -f $(OBJ_DIR)/*.o
