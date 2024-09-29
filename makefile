TARGET = ./intersection.x
CXX = g++

DEBUG_FLAGS = -D _DEBUG -ggdb3 -std=c++17 -O0 -Wall -Wextra -Weffc++ -Wc++14-compat \
		-Wmissing-declarations -Wcast-align -Wcast-qual -Wchar-subscripts  -Wconversion \
		-Wctor-dtor-privacy -Wempty-body -Wfloat-equal -Wformat-nonliteral -Wformat-security -Wformat-signedness \
		-Wformat=2 -Winline -Wlogical-op -Wnon-virtual-dtor -Wopenmp-simd -Woverloaded-virtual -Wpacked -Wpointer-arith \
		-Winit-self -Wredundant-decls -Wshadow -Wsign-conversion -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=2 \
		-Wsuggest-attribute=noreturn -Wsuggest-final-methods -Wsuggest-final-types -Wsuggest-override -Wswitch-default \
		-Wswitch-enum -Wsync-nand -Wundef -Wunreachable-code -Wunused -Wuseless-cast -Wvariadic-macros -Wno-literal-suffix \
		-Wno-missing-field-initializers -Wno-narrowing -Wno-old-style-cast -Wno-varargs -Wstack-protector -fcheck-new \
		-fsized-deallocation -fstack-protector -fstrict-overflow -fno-omit-frame-pointer \
		-Wlarger-than=8192 -Wstack-usage=8192 -pie -fPIE -Werror=vla 

COBJ = $(CSRC : %.c = %.o)

all : main.o 
	$(CXX) main.o -I./include -o $(TARGET)

# test : tests.o
# $(CC) tests.o -I./include -o test.x

# tests.o : src/tests.cpp
# $(CC) -O2 -I./include -c src/tests.cpp

main.o : main.cpp
	$(CXX) -std=c++20 -O2 -I./include -c main.cpp

.PHONY clear :
	rm -rf *.o *.log *.x
