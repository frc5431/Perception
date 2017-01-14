SHELL = /bin/sh
ARCH = x86_64

CXX = g++
CC = gcc


SRCS = $(shell echo src/*.cpp)
HEADS = $(shell echo include/*.hpp)
OBJS = $(SRCS:.cpp=.o)

LDFLAGS = -L/usr/local/lib -lboost_date_time -lboost_filesystem -lboost_iostreams \
-lboost_serialization -lboost_system -lboost_log -lboost_thread -lboost_regex \
-lboost_python -lboost_locale -lboost_wave -lboost_random -lboost_mpi \
-lboost_coroutine -lboost_chrono -lboost_atomic -lboost_graph -lfreenect2 \
-lfreenect2-openni2 -lopencv_shape -lopencv_stitching -lopencv_objdetect \
-lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d \
-lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo \
-lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core -lpthread

CFLAGS = -I/usr/local/include/opencv -I/usr/local/include

CXXFLAGS = -std=c++11 $(CFLAGS) -O3 -Wall

TARGET = perception


OBJF = obj/
SRCF = src/
BINF = bin/
RM = rm -f

.PHONY: all
all: ${TARGET}

$(TARGET): $(OBJS)
	@echo Creating $(TARGET)
	$(CXX) -o $(BINF)$@ $^ $(CXXFLAGS) $(LDFLAGS)
	mv $(OBJS) $(OBJF)
	@echo Finished building!

$(OBJF)%.0: %.cpp $(HEADS)
	@echo Compiling... $<
	$(CXX) -o $@ $< $(CXXFLAGS)

.PHONY: clean
clean:
	@echo Removing object files
	-${RM} ${OBJS}
