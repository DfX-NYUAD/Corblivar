#!/bin/bash

REV=commit.git
ARCHIVE=Corblivar.tar.gz
SERVER=ganymed

# annotate current git commit
git show --pretty=oneline | head -n 1 > $REV

# create archive
tar -czf $ARCHIVE $REV src/*.cpp src/*.hpp Makefile exp/*.sh exp/*.conf

# push archive
scp $ARCHIVE $SERVER:public_html/
