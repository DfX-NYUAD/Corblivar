#!/bin/bash

REV=commit.git
ARCHIVE=Corblivar.zip
SERVER=ganymed

# annotate current git commit
git show --pretty=oneline | head -n 1 > $REV

# create archive
zip -r $ARCHIVE $REV src/*.cpp src/*.hpp Makefile exp/*.sh exp/*.conf exp/bench/n*

# push archive
scp $ARCHIVE $SERVER:public_html/

# clean stuff
rm $REV $ARCHIVE
