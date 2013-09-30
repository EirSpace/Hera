CC=gcc
CFLAGS=-Wall -Wno-trigraphs -std=gnu99 -I/usr/include/libusb-1.0 -I./
LDFLAGS=-I/usr/include/libusb-1.0 -lusb-1.0 -lm
SRC=tuner_e4k.c tuner_fc0012.c tuner_fc0013.c tuner_fc2580.c tuner_r820t.c librtlsdr.c sdr_fusex.c
OBJ=$(SRC:.c=.o)
EXEC=sdr_fusex

.PHONY: clean

all: $(EXEC)

$(EXEC): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

%.o: %.c %.h
	$(CC) -c $< -o $@ $(CFLAGS)

sdr_fusex.o: librtlsdr.o

tuner_%.o: tuner_%.c tuner_%.h

clean:
	rm -rf *.o $(EXEC)