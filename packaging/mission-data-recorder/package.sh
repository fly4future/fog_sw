#!/bin/bash

if ! go version > /dev/null 2>&1; then
  export PATH=$PATH:/usr/lib/go-1.16/bin/
fi

pushd ../../mission-data-recorder > /dev/null
params="-m $(realpath .) -v 1.0.0 -c $(git rev-parse HEAD) -g $(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1) -b dirty"
./packaging/common/package.sh $params
popd

exit 0
