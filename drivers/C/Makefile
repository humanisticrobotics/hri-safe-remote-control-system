#/***************************************************************************
# * Humanistic Robotics VSC Interface Library                               *
# * Version 1.1                                                             *
# * Copyright 2013, Humanistic Robotics, Inc                                *
# ***************************************************************************/
#/*
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# * POSSIBILITY OF SUCH DAMAGE.
# */

SHELL = /bin/sh
CC    = gcc
CFLAGS       = -fPIC -g -Wall -Wextra -Iinclude
LDFLAGS      = -shared
VERSION = 1.1

TARGET  = lib/hri_vsc_interface.$(VERSION).so
SOURCES = $(shell echo src/*.c)
HEADERS = $(shell echo include/*.h)
OBJECTS = $(SOURCES:.c=.o)
TUTORIAL_SRC =  $(shell echo tutorials/*.c) 

BINDIR = $(PREFIX)/bin

all: $(TARGET) $(TUTORIAL_SRC:.c=)

.c:
	$(CC) $(CFLAGS) $< $(TARGET) -o $@ -Iinclude -lrt

$(TARGET): $(OBJECTS)
	mkdir -p $(dir $@)
	$(CC) $(FLAGS) $(CFLAGS) $(DEBUGFLAGS) -o $(TARGET) $(LDFLAGS) $(OBJECTS)

clean: 
	rm $(OBJECTS) $(TARGET) $(TUTORIAL_SRC:.c=)




