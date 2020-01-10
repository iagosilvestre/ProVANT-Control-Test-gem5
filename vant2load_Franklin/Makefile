# the compiler: gcc for C program, define as g++ for C++
CC = arm-linux-gnueabi-g++ -I /usr/include/eigen3/ 
# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CFLAGS  = -g -Wall -static ${GEM5_PATH}/util/m5/m5op_arm.S

# the build target executable:
TARGET = vant2loadFranklin

all: $(TARGET)

$(TARGET): $(TARGET).cpp 
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).cpp

clean:
	$(RM) $(TARGET)




