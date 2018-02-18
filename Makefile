# Uncomment lines below if you have problems with $PATH
SHELL := /bin/bash
PATH := /usr/bin:$(PATH)
BIN := .pioenvs/wemos_d1_mini/firmware.bin
WEBNAME := flaschengeist_1.local
WEBPATH := firmware

all: run

webupdate: run
	curl -F "file=@$(BIN)" --user `cat ../user.cred` $(WEBNAME)/$(WEBPATH) > /dev/null

upload:
	platformio -f -c qtcreator run --target upload;
	sleep 1;

clean:
	platformio -f -c qtcreator run --target clean

program:
	platformio -f -c qtcreator run --target program

uploadfs:
	platformio -f -c qtcreator run --target uploadfs

update: 
	platformio -f -c qtcreator update

$(BIN): run

run: 
	platformio -f -c qtcreator run
